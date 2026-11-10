#!/usr/bin/env python3
"""Transparent serial proxy that logs all E201 traffic.

Creates a pseudo-terminal, publishes it at LINK (a symlink), and forwards every
byte between it and the real E201 device, logging both directions with
timestamps.  Point Wine's COM port at LINK (encosight.sh --sniff does this).

    e201_sniff.py /dev/ttyACM0 /tmp/e201-pty e201-traffic.log

Log format: one line per chunk, "<seconds> <dir> <bytes repr>", where dir is
"PC>" (EncoSight to E201) or "<DEV" (E201 to EncoSight).  Commands and replies
end in \\r, so the log reads as a command/response transcript.
"""

import os
import pty
import select
import signal
import sys
import termios
import time
import tty


def open_raw(path: str) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def main() -> int:
    if len(sys.argv) != 4:
        print(__doc__, file=sys.stderr)
        return 2
    device, link, logpath = sys.argv[1:]
    dev = open_raw(device)
    master, slave = pty.openpty()
    tty.setraw(slave)
    tty.setraw(master)
    if os.path.lexists(link):
        os.unlink(link)
    os.symlink(os.ttyname(slave), link)
    log = open(logpath, "a", buffering=1)
    t0 = time.monotonic()
    log.write(f"# {time.strftime('%Y-%m-%d %H:%M:%S')} sniffing {device} via {link} ({os.ttyname(slave)})\n")

    def stop(*_):
        try:
            os.unlink(link)
        except OSError:
            pass
        sys.exit(0)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    pending = b""  # partial E201 reply
    while True:
        r, _, _ = select.select([master, dev], [], [], 0.5)
        for fd in r:
            try:
                data = os.read(fd, 4096)
            except OSError:
                data = b""
            if not data:
                if fd == dev:
                    log.write(f"{time.monotonic() - t0:10.4f} !!! device closed\n")
                    stop()
                continue
            os.write(dev if fd == master else master, data)
            now = time.monotonic() - t0
            if fd == master:
                # EncoSight's commands have no terminator ("v", "R01:066"): log each write as is.
                log.write(f"{now:10.4f} PC>  {data!r}\n")
                continue
            # E201 replies end in \r: log one line per reply.
            *lines, rest = (pending + data).split(b"\r")
            for line in lines:
                log.write(f"{now:10.4f} <DEV {(line + b'\r')!r}\n")
            pending = rest

if __name__ == "__main__":
    sys.exit(main())
