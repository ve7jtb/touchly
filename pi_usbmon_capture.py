#!/usr/bin/env python3
"""
Captures full-length (untruncated 64-byte+ URB) USB traffic from /dev/usbmon1
on Raspberry Pi (<pi-address>) for the attached USB FIDO authenticator,
and writes both:
  1. A standard Wireshark/libpcap file (<prefix>.pcap, DLT_USB_LINUX_MMAPPED = 220)
  2. A Total Phase Data Center-compatible CSV file (<prefix>.csv) containing
     all SETUP / OUT / IN transactions with full hex payloads across USB power cycles.
"""

import argparse
import ctypes
import csv
import fcntl
import os
import select
import signal
import socket
import struct
import sys
import tempfile
import time

# Linux /dev/usbmonN ioctl constants (drivers/usb/mon/mon_bin.c)
MON_IOCT_RING_SIZE = 0x00009204
# _IOW(0x92, 10, struct mon_get_arg) where sizeof(struct mon_get_arg) == 24 on 64-bit
MON_IOCX_GETX = 0x4018920A

HDR_STRUCT = struct.Struct("<QBBBBHbbqiiII8siiII")
assert HDR_STRUCT.size == 64

PCAP_GLOBAL_HDR = struct.Struct("<IHHiIII")
PCAP_PKT_HDR = struct.Struct("<IIII")
DLT_USB_LINUX_MMAPPED = 220

XFER_NAMES = {
    0: "ISO",
    1: "INTR",
    2: "CTRL",
    3: "BULK",
}


class MonGetArg(ctypes.Structure):
    _fields_ = [
        ("hdr_ptr", ctypes.c_uint64),
        ("data_ptr", ctypes.c_uint64),
        ("alloc", ctypes.c_uint64),
    ]


def format_hex_bytes(raw: bytes) -> str:
    return " ".join(f"{b:02x}" for b in raw)


def format_ascii(raw: bytes) -> str:
    return "".join(chr(b) if 32 <= b <= 126 else "." for b in raw)


def lookup_sysfs_usb_identity(busnum: int, devnum: int, cache: dict[tuple[int, int], str]) -> str:
    """Resolve ` vid=<4hex> pid=<4hex> [serial=<token>]` for `(busnum, devnum)` from `/sys/bus/usb/devices`."""
    key = (busnum, devnum)
    if key in cache:
        return cache[key]
    sysfs_root = "/sys/bus/usb/devices"
    try:
        for name in os.listdir(sysfs_root):
            entry = os.path.join(sysfs_root, name)
            try:
                with open(os.path.join(entry, "busnum"), "r", encoding="utf-8") as f:
                    b_num = int(f.read().strip())
                with open(os.path.join(entry, "devnum"), "r", encoding="utf-8") as f:
                    d_num = int(f.read().strip())
            except (OSError, ValueError):
                continue
            if b_num != busnum or d_num != devnum:
                continue
            try:
                with open(os.path.join(entry, "idVendor"), "r", encoding="utf-8") as f:
                    vid = f.read().strip().lower()
                with open(os.path.join(entry, "idProduct"), "r", encoding="utf-8") as f:
                    pid = f.read().strip().lower()
            except OSError:
                break
            tokens = [f"vid={vid}", f"pid={pid}"]
            try:
                with open(os.path.join(entry, "serial"), "r", encoding="utf-8") as f:
                    raw_serial = f.read().strip()
                clean_serial = "".join(c if c.isalnum() or c in "._:-" else "_" for c in raw_serial).lower()
                if clean_serial:
                    tokens.append(f"serial={clean_serial}")
            except OSError:
                pass
            suffix = " " + " ".join(tokens)
            cache[key] = suffix
            return suffix
    except OSError:
        pass
    return ""


def decode_ctaphid_summary(ep_num: int, is_in: bool, data: bytes) -> str:
    direction = "IN" if is_in else "OUT"
    if len(data) != 64:
        return f"USB {direction} ({len(data)}B)"
    cid = data[0:4].hex()
    cmd_or_seq = data[4]
    if cmd_or_seq & 0x80:
        cmd = cmd_or_seq & 0x7F
        bant = (data[5] << 8) | data[6]
        cmd_names = {
            0x01: "PING",
            0x03: "MSG",
            0x04: "LOCK",
            0x06: "INIT",
            0x08: "WINK",
            0x10: "CBOR",
            0x11: "CANCEL",
            0x3B: "KEEPALIVE",
            0x3F: "ERROR",
        }
        cname = cmd_names.get(cmd, f"0x{cmd:02x}")
        if cmd == 0x10 and len(data) >= 8:
            ctap_byte = data[7]
            return f"CTAPHID_{cname} {direction} cid={cid} len={bant} byte0=0x{ctap_byte:02x}"
        return f"CTAPHID_{cname} {direction} cid={cid} len={bant}"
    else:
        return f"CTAPHID_CONT {direction} cid={cid} seq={cmd_or_seq}"


def open_new_file(path, flags):
    return os.open(path, flags | os.O_NOFOLLOW, 0o600)


def run_capture(
    bus: int,
    out_prefix: str,
    min_devnum: int,
    hid_only: bool = True,
    write_pcap: bool = False,
):
    dev_path = f"/dev/usbmon{bus}"
    if not os.path.exists(dev_path):
        os.system("modprobe usbmon")
    fd = os.open(dev_path, os.O_RDONLY | os.O_NONBLOCK)
    try:
        fcntl.ioctl(fd, MON_IOCT_RING_SIZE, 1024 * 1024)
    except OSError:
        pass

    pcap_path = f"{out_prefix}.pcap"
    csv_path = f"{out_prefix}.csv"

    stop_requested = False

    def handle_sig(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGTERM, handle_sig)
    signal.signal(signal.SIGINT, handle_sig)

    hdr_buf = (ctypes.c_ubyte * 64)()
    data_buf = (ctypes.c_ubyte * 4096)()
    arg = MonGetArg(
        ctypes.addressof(hdr_buf),
        ctypes.addressof(data_buf),
        ctypes.sizeof(data_buf),
    )

    t0: float | None = None
    row_idx = 0

    # Local UNIX datagram socket where touch.py daemon listens for UP_SATISFIED events
    up_sock_path = "/tmp/yubikey_up.sock"
    up_sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    up_sock.setblocking(False)

    # Unique monitor-instance ID generated at monitor startup so touch.py scopes
    # generation/phase retirement to this monitor process instance across restarts.
    mon_id = f"{os.getpid():x}-{int(time.monotonic() * 1000) & 0xFFFFFFFF:08x}"
    req_gen_seq = 0
    channel_gen: dict[tuple[int, bytes], int] = {}
    channel_phase: dict[tuple[int, bytes], int] = {}
    # Track active pending UP_NEEDED phase per (devnum, cid_bytes) channel -> (ep_addr, cur_gen, cur_phase)
    # so concurrent transactions on different devices or channels never overwrite each other's pending slot.
    up_pending_by_chan: dict[tuple[int, bytes], tuple[int, int, int]] = {}
    usb_identity_cache: dict[tuple[int, int], str] = {}

    pcap_f = open(pcap_path, "xb", opener=open_new_file) if write_pcap else None
    try:
        with open(csv_path, "x", newline="", encoding="utf-8", opener=open_new_file) as csv_f:
            if pcap_f is not None:
                # Write standard libpcap global header (microsecond resolution, DLT_USB_LINUX_MMAPPED=220)
                pcap_f.write(PCAP_GLOBAL_HDR.pack(0xA1B2C3D4, 2, 4, 0, 0, 65535, DLT_USB_LINUX_MMAPPED))
                pcap_f.flush()

            writer = csv.writer(csv_f)
            writer.writerow(["Index", "Timestamp", "Len", "Error", "Dev", "Ep", "Record", "Data", "Summary", "ASCII"])
            csv_f.flush()

            while not stop_requested:
                rlist, _, _ = select.select([fd], [], [], 0.2)
                if not rlist:
                    continue
                while not stop_requested:
                    try:
                        fcntl.ioctl(fd, MON_IOCX_GETX, arg)
                    except BlockingIOError:
                        break
                    except OSError:
                        break

                    raw_hdr = bytes(hdr_buf)
                    (
                        urb_id,
                        ev_type,
                        xfer_type,
                        epnum,
                        devnum,
                        busnum,
                        flag_setup,
                        flag_data,
                        ts_sec,
                        ts_usec,
                        status,
                        len_urb,
                        len_cap,
                        setup_bytes,
                        interval,
                        start_frame,
                        xfer_flags,
                        ndesc,
                    ) = HDR_STRUCT.unpack(raw_hdr)

                    if devnum < min_devnum:
                        continue

                    cap_len = min(len_cap, 4096)
                    is_in = bool(epnum & 0x80)
                    ep_addr = epnum & 0x0F
                    ev_char = chr(ev_type) if 32 <= ev_type <= 126 else "?"

                    if hid_only:
                        # Only capture 64-byte USB Interrupt (HID) report payloads:
                        # - OUT report on URB Submit ('S')
                        # - IN report on URB Complete ('C')
                        # Excludes USB control/enumeration, CCID bulk, CCID 2-byte slot notifications,
                        # and empty URB submit/complete events.
                        if xfer_type != 1 or cap_len != 64:
                            continue
                        if not ((not is_in and ev_char == "S") or (is_in and ev_char == "C")):
                            continue

                    payload = bytes(data_buf[:cap_len]) if cap_len > 0 else b""

                    # Track CTAPHID channel (cid), device (devnum), IN endpoint (ep_addr), and
                    # request generation (req_gen) across 64-byte USB Interrupt HID transfers.
                    if xfer_type == 1 and cap_len == 64:
                        cid_bytes = payload[0:4]
                        cmd_byte = payload[4]
                        dev_cid = (devnum, cid_bytes)
                        dev_id_suffix = lookup_sysfs_usb_identity(busnum, devnum, usb_identity_cache)

                        if not is_in and ev_char == "S" and (cmd_byte & 0x80):
                            # New CTAPHID command initialization frame (or CTAPHID_CANCEL) on (devnum, cid):
                            # retire any prior generation on this channel, advance req_gen_seq, and reset phase=0.
                            prev_gen = channel_gen.get(dev_cid)
                            if prev_gen is not None:
                                done_msg = (
                                    f"TX_DONE mon={mon_id} dev={devnum} ep=0 "
                                    f"cid={cid_bytes.hex()} gen={prev_gen}{dev_id_suffix}\n"
                                ).encode("ascii")
                                try:
                                    up_sock.sendto(done_msg, up_sock_path)
                                except OSError:
                                    pass
                            req_gen_seq += 1
                            channel_gen[dev_cid] = req_gen_seq
                            channel_phase[dev_cid] = 0
                            # Clear only this (devnum, cid) channel's pending presence phase
                            up_pending_by_chan.pop(dev_cid, None)

                        elif is_in and ev_char == "C":
                            cur_gen = channel_gen.get(dev_cid, req_gen_seq)
                            pending = up_pending_by_chan.get(dev_cid)
                            is_up_needed = (
                                (cmd_byte == 0xBB and payload[7] == 0x02)
                                or (cmd_byte == 0x83 and payload[5:9] == b"\x00\x02\x69\x85")
                            )
                            is_processing = (cmd_byte == 0xBB and payload[7] == 0x01)
                            is_terminal_resp = (
                                cmd_byte in (0x90, 0xBF)
                                or (cmd_byte == 0x83 and payload[5:9] != b"\x00\x02\x69\x85")
                            )

                            if is_up_needed:
                                # Increment presence-phase ID only when this (devnum, cid) transaction enters UP_NEEDED
                                if pending is None or pending[:2] != (ep_addr, cur_gen):
                                    cur_phase = channel_phase.get(dev_cid, 0) + 1
                                    channel_phase[dev_cid] = cur_phase
                                    up_pending_by_chan[dev_cid] = (ep_addr, cur_gen, cur_phase)
                                    usbmon_ns = time.monotonic_ns()
                                    needed_msg = (
                                        f"UP_NEEDED mon={mon_id} dev={devnum} ep={ep_addr} "
                                        f"cid={cid_bytes.hex()} gen={cur_gen} phase={cur_phase} "
                                        f"usbmon_ns={usbmon_ns}{dev_id_suffix}\n"
                                    ).encode("ascii")
                                    try:
                                        up_sock.sendto(needed_msg, up_sock_path)
                                    except OSError:
                                        pass
                            elif pending is not None and pending[:2] == (ep_addr, cur_gen):
                                # Active presence phase for this (devnum, cid) ends on PROCESSING (final=0) or terminal response (final=1)
                                if is_processing or is_terminal_resp:
                                    cur_phase = pending[2]
                                    up_pending_by_chan.pop(dev_cid, None)
                                    final_flag = 1 if is_terminal_resp else 0
                                    if is_terminal_resp:
                                        channel_gen.pop(dev_cid, None)
                                    msg = (
                                        f"UP_SATISFIED mon={mon_id} dev={devnum} ep={ep_addr} "
                                        f"cid={cid_bytes.hex()} gen={cur_gen} "
                                        f"phase={cur_phase} final={final_flag}{dev_id_suffix}\n"
                                    ).encode("ascii")
                                    try:
                                        up_sock.sendto(msg, up_sock_path)
                                    except OSError:
                                        pass
                            elif is_terminal_resp and dev_cid in channel_gen:
                                # Request completed after an earlier PROCESSING transition: retire `cur_gen`
                                channel_gen.pop(dev_cid, None)
                                done_msg = (
                                    f"TX_DONE mon={mon_id} dev={devnum} ep={ep_addr} "
                                    f"cid={cid_bytes.hex()} gen={cur_gen}{dev_id_suffix}\n"
                                ).encode("ascii")
                                try:
                                    up_sock.sendto(done_msg, up_sock_path)
                                except OSError:
                                    pass

                    if pcap_f is not None:
                        pkt_bytes = raw_hdr + payload
                        pcap_f.write(
                            PCAP_PKT_HDR.pack(
                                int(ts_sec) & 0xFFFFFFFF,
                                int(ts_usec) & 0xFFFFFFFF,
                                len(pkt_bytes),
                                64 + len_urb,
                            )
                        )
                        pcap_f.write(pkt_bytes)

                    ts_float = float(ts_sec) + float(ts_usec) * 1e-6
                    if t0 is None:
                        t0 = ts_float
                    rel_ts = ts_float - t0
                    mins = int(rel_ts // 60)
                    secs = int(rel_ts % 60)
                    millis = int((rel_ts - int(rel_ts)) * 1000)
                    micros = int((rel_ts * 1_000_000) % 1000)
                    ts_str = f"{mins:02d}:{secs:02d}.{millis:03d}.{micros:03d}"

                    err_str = "OK" if status == 0 or status == -115 else f"ERR({status})"

                    # Record SETUP packet on Submit ('S') of Control transfer (when not hid_only)
                    if not hid_only and xfer_type == 2 and ev_char == "S" and flag_setup == 0:
                        row_idx += 1
                        writer.writerow([
                            row_idx,
                            ts_str,
                            "8 B",
                            err_str,
                            f"{devnum:02d}",
                            f"{ep_addr:02d}",
                            "SETUP txn",
                            format_hex_bytes(setup_bytes),
                            f"CTRL SETUP (len_urb={len_urb})",
                            format_ascii(setup_bytes),
                        ])

                    # Record OUT payload on Submit ('S') and IN payload on Complete ('C')
                    if ((not is_in and ev_char == "S") or (is_in and ev_char == "C")) and cap_len > 0:
                        row_idx += 1
                        rec_type = "IN txn" if is_in else "OUT txn"
                        xfer_label = XFER_NAMES.get(xfer_type, "USB")
                        summary = (
                            decode_ctaphid_summary(ep_addr, is_in, payload)
                            if xfer_type == 1
                            else f"{xfer_label} {rec_type} ({cap_len}B)"
                        )
                        writer.writerow([
                            row_idx,
                            ts_str,
                            f"{cap_len} B",
                            err_str,
                            f"{devnum:02d}",
                            f"{ep_addr:02d}",
                            rec_type,
                            format_hex_bytes(payload),
                            summary,
                            format_ascii(payload),
                        ])

                if pcap_f is not None:
                    pcap_f.flush()
                csv_f.flush()
    finally:
        if pcap_f is not None:
            pcap_f.close()
        try:
            up_sock.close()
        except OSError:
            pass

    os.close(fd)


def daemonize(logfile: str = "/run/pi_usbmon_capture.log"):
    pid = os.fork()
    if pid > 0:
        os._exit(0)
    os.setsid()
    pid = os.fork()
    if pid > 0:
        os._exit(0)
    sys.stdout.flush()
    sys.stderr.flush()
    try:
        if os.path.exists(logfile):
            os.unlink(logfile)
    except OSError:
        pass
    devnull_fd = os.open(os.devnull, os.O_RDONLY)
    log_fd = os.open(logfile, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    os.dup2(devnull_fd, 0)
    os.dup2(log_fd, 1)
    os.dup2(log_fd, 2)
    if devnull_fd > 2:
        os.close(devnull_fd)
    if log_fd > 2:
        os.close(log_fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=1, help="USB bus number (default: 1)")
    parser.add_argument("-o", "--output-prefix", help="Output file prefix (default: usbmon_yubikey in a new private directory)")
    parser.add_argument("--min-devnum", type=int, default=5, help="Minimum USB device number to capture (default: 5, skipping root hub & onboard hubs/eth0)")
    parser.add_argument("--pidfile", help="PID file path (default: pi_usbmon_capture.pid in a new private directory)")
    parser.add_argument("-d", "--daemon", action="store_true", help="Detach and run in background")
    parser.add_argument("--hid-only", dest="hid_only", action="store_true", default=True, help="Capture only 64-byte USB Interrupt (HID) report payloads (default: True)")
    parser.add_argument("--all-usb", dest="hid_only", action="store_false", help="Capture all USB transfers (CTRL, BULK, INTR) instead of HID-only")
    parser.add_argument("--pcap", dest="write_pcap", action="store_true", default=False, help="Also write a .pcap file alongside .csv (default: False)")
    parser.add_argument("--no-pcap", dest="write_pcap", action="store_false", help="Skip writing .pcap file and write only .csv (default)")
    args = parser.parse_args()
    if args.daemon:
        daemonize()
    private_dir = None
    if args.output_prefix is None or args.pidfile is None:
        private_dir = tempfile.mkdtemp(prefix="touchly-", dir="/tmp")
    out_prefix = args.output_prefix
    if out_prefix is None:
        out_prefix = os.path.join(private_dir, "usbmon_yubikey")
    pidfile = args.pidfile
    if pidfile is None:
        pidfile = os.path.join(private_dir, "pi_usbmon_capture.pid")

    with open(pidfile, "x", encoding="utf-8", opener=open_new_file) as pf:
        pf.write(str(os.getpid()))
    print(f"Capture output prefix: {out_prefix}", file=sys.stderr, flush=True)
    try:
        run_capture(
            args.bus,
            out_prefix,
            args.min_devnum,
            hid_only=args.hid_only,
            write_pcap=args.pcap,
        )
    finally:
        try:
            os.unlink(pidfile)
        except FileNotFoundError:
            pass


if __name__ == "__main__":
    main()
