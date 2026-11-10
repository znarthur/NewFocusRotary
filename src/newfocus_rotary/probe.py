"""Commissioning tool: find devices, check the encoder frame format, calibrate.

    newfocus-rotary-probe list
    newfocus-rotary-probe encoder [--port /dev/ttyACM0] [-n 20]
    newfocus-rotary-probe picomotor
    newfocus-rotary-probe calibrate -c config.yaml [--steps 2000]
    newfocus-rotary-probe move -c config.yaml 45.0
    newfocus-rotary-probe move -c config.yaml --open-loop 45.0   # ignore the encoder
    newfocus-rotary-probe steps -c config.yaml -- -500             # raw picomotor steps, no encoder
    newfocus-rotary-probe -c config.yaml encoder-status              # AksIM-2 detailed status register
    newfocus-rotary-probe -c config.yaml reset-multiturn [--value 0] # clear a multiturn error
    newfocus-rotary-probe -c config.yaml raw v 4 R04:020             # send raw E201 commands
    newfocus-rotary-probe -c config.yaml reg-read --bank 3 0x14 4    # read encoder registers
    newfocus-rotary-probe -c config.yaml reg-write [--bank 3] [--key] 0x14 0  # write one register
"""

from __future__ import annotations

import argparse
import sys
import time

from .config import load_config


def cmd_list(args) -> int:
    try:
        import usb.core

        devs = list(usb.core.find(find_all=True, idVendor=0x104D) or [])
        print(f"Newport/New Focus USB devices (VID 104d): {len(devs)}")
        for d in devs:
            try:
                sn = d.serial_number
            except Exception as exc:
                sn = f"<unreadable: {exc}; check udev permissions>"
            print(f"  {d.idVendor:04x}:{d.idProduct:04x} bus {d.bus} addr {d.address} serial {sn}")
    except Exception as exc:
        print(f"pyusb unavailable or failed: {exc}")
    try:
        import serial.tools.list_ports

        ports = [p for p in serial.tools.list_ports.comports() if p.vid == 0x0483]
        print(f"E201 candidate serial ports (VID 0483): {len(ports)}")
        for p in ports:
            print(f"  {p.device}  {p.description}  serial {p.serial_number}")
    except Exception as exc:
        print(f"pyserial failed: {exc}")
    return 0


def cmd_encoder(args) -> int:
    from .e201 import E201_VID, E201_9B, BissFrameFormat, decode_frame, guess_data_bits
    from .transports import SerialTransport, find_serial_port_by_vid

    cfg = load_config(args.config)
    port = args.port or cfg.encoder.port
    if port == "auto":
        port = find_serial_port_by_vid(E201_VID)
        if not port:
            print("No E201 found")
            return 1
    fmt = cfg.encoder.frame_format()
    enc = E201_9B(SerialTransport(port), fmt, power_on=True, clock_khz=cfg.encoder.clock_khz)
    print(f"{port}: {enc.version}")
    frames = []
    for _ in range(args.n):
        raw = enc.read_raw()
        frames.append(raw)
        r = decode_frame(raw, fmt)
        deg = r.singleturn * 360.0 / fmt.counts_per_rev
        print(
            f"  {raw.hex().upper()}  st={r.singleturn:>8d} ({deg:10.5f} deg) mt={r.multiturn} "
            f"crc={'ok' if r.crc_ok else 'BAD'} err={int(r.error)} warn={int(r.warning)}"
        )
        time.sleep(0.05)
    cands = guess_data_bits(frames, fmt.crc_bits)
    print(f"\nConfigured: {fmt.multiturn_bits} MT + {fmt.singleturn_bits} ST + {fmt.status_bits} status bits"
          f" = {fmt.data_bits} data bits")
    print(f"Data lengths consistent with CRC over all frames: {cands}")
    if fmt.data_bits not in cands:
        print("WARNING: configured frame format fails CRC; adjust encoder.singleturn_bits/multiturn_bits")
    enc.close()
    return 0


def _open_e201(cfg, port=None):
    from .e201 import E201_VID, E201_9B
    from .transports import SerialTransport, find_serial_port_by_vid

    port = port or cfg.encoder.port
    if port == "auto":
        port = find_serial_port_by_vid(E201_VID)
        if not port:
            raise SystemExit("No E201 found")
    return port, E201_9B(SerialTransport(port), cfg.encoder.frame_format(), power_on=True,
                         clock_khz=cfg.encoder.clock_khz)


def _print_reading(enc) -> None:
    r = enc.read()
    fmt = enc.fmt
    print(f"  frame {r.raw.hex().upper()}  st={r.singleturn} ({r.singleturn * 360.0 / fmt.counts_per_rev:.5f} deg)"
          f" mt={r.multiturn} crc={'ok' if r.crc_ok else 'BAD'} err={int(r.error)} warn={int(r.warning)}")


def cmd_encoder_status(args) -> int:
    cfg = load_config(args.config)
    port, enc = _open_e201(cfg, args.port)
    print(f"{port}: {enc.version}")
    _print_reading(enc)
    try:
        st = enc.read_detailed_status()
        print(f"  AksIM-2 detailed status (bank 0, 0x4A): 0x{st:04X}  bits set: "
              f"{[b for b in range(16) if st >> b & 1] or 'none'}")
        print("  (bit meanings: AksIM-2 data sheet, 'detailed status' register)")
    except Exception as exc:
        print(f"  detailed status not readable: {exc}")
    enc.close()
    return 0


def cmd_reset_multiturn(args) -> int:
    cfg = load_config(args.config)
    port, enc = _open_e201(cfg, args.port)
    print(f"{port}: {enc.version}")
    print("before:")
    _print_reading(enc)
    variant = enc.reset_multiturn(args.value)
    print(f"accepted write sequence: {variant}")
    time.sleep(0.2)
    print(f"after setting multiturn = {args.value}:")
    _print_reading(enc)
    r = enc.read()
    enc.close()
    if r.error:
        print("encoder still reports an error; check 'encoder-status' and the mounting")
        return 1
    print("encoder OK. If the IOC uses a state file, re-reference with SET mode if the dial looks wrong.")
    return 0


def cmd_raw(args) -> int:
    cfg = load_config(args.config)
    port, enc = _open_e201(cfg, args.port)
    print(f"{port}: {enc.version}")
    for c in args.commands:
        print(f"  {c!r:>14} -> {enc.raw_query(c)!r}")
    enc.close()
    return 0


def _int(x: str) -> int:
    return int(x, 0)


def cmd_reg_read(args) -> int:
    cfg = load_config(args.config)
    port, enc = _open_e201(cfg, args.port)
    print(f"{port}: {enc.version}")
    if args.bank is not None:
        print(f"  select bank {args.bank}: {enc.raw_query(f'Ws{args.bank:03d}:064')!r}")
    print(f"  R{args.length:02d}:{args.address:03d} -> {enc.raw_query(f'R{args.length:02d}:{args.address:03d}')!r}")
    enc.close()
    return 0


def cmd_reg_write(args) -> int:
    cfg = load_config(args.config)
    port, enc = _open_e201(cfg, args.port)
    print(f"{port}: {enc.version}")
    if args.key:
        print(f"  key 0xCD -> 0x48: {enc.raw_query('Ws205:072')!r}")
    if args.bank is not None:
        print(f"  select bank {args.bank}: {enc.raw_query(f'Ws{args.bank:03d}:064')!r}")
    print(f"  Ws{args.value:03d}:{args.address:03d} -> {enc.raw_query(f'Ws{args.value:03d}:{args.address:03d}')!r}")
    enc.close()
    return 0


def cmd_picomotor(args) -> int:
    cfg = load_config(args.config)
    from .config import build_hardware

    cfg.encoder.transport = "none"  # only the picomotor is needed here
    pm, _ = build_hardware(cfg)
    print(f"*IDN?: {pm.identity}")
    for ax in range(1, 5):
        print(f"  axis {ax}: motor={pm.motor_type(ax)} TP={pm.position(ax)} VA={pm.velocity(ax)} AC={pm.acceleration(ax)}")
    print(f"error: {pm.error()}")
    return 0


def _axis(cfg):
    from .axis import ClosedLoopRotaryAxis
    from .config import build_hardware

    pm, enc = build_hardware(cfg)
    axis = ClosedLoopRotaryAxis(pm, enc, cfg.axis, cfg.axis_params())
    axis.start()
    return axis


def cmd_calibrate(args) -> int:
    cfg = load_config(args.config)
    axis = _axis(cfg)
    try:
        print(f"start: dial {axis.get_status().dial:.5f} deg")
        axis.calibrate(args.steps)
        st = axis.wait_idle(600)
        print(st.message)
        print(f"Put these in the config (axis section):\n  steps_per_degree: {st.steps_per_deg_fwd:.2f}\n"
              f"  steps_per_degree_reverse: {st.steps_per_deg_rev:.2f}")
        return 0 if not st.problem else 1
    finally:
        axis.close()


def cmd_steps(args) -> int:
    """Move the picomotor a raw number of steps; the encoder is not touched."""
    from .config import build_hardware

    cfg = load_config(args.config)
    cfg.encoder.transport = "none"
    pm, _ = build_hardware(cfg)
    ax = cfg.axis.picomotor_axis
    if args.velocity:
        pm.set_velocity(ax, args.velocity)
    before = pm.position(ax)
    pm.move_relative(ax, args.steps)
    try:
        while not pm.motion_done(ax):
            print(f"\r  axis {ax} step counter {pm.position(ax):>10d}", end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pm.stop(ax)
        print("\nstopped")
        return 130
    after = pm.position(ax)
    print(f"\r  axis {ax}: {before} -> {after} ({after - before:+d} steps)")
    return 0


def cmd_move(args) -> int:
    cfg = load_config(args.config)
    if args.open_loop:
        cfg.motor.use_encoder = False
        cfg.encoder.transport = "none"  # don't even open the E201
    axis = _axis(cfg)
    try:
        t0 = time.monotonic()
        axis.move_to(args.target)
        while True:
            st = axis.get_status()
            print(f"\r  dial {st.dial:11.5f}  {st.message:60s}", end="", flush=True)
            if not st.busy:
                break
            time.sleep(0.1)
        print(f"\nfinal error {st.dial - args.target:+.5f} deg, retries {st.retries}, "
              f"{time.monotonic() - t0:.1f} s")
        return 0 if not (st.missed or st.problem) else 1
    except KeyboardInterrupt:
        axis.stop()
        return 130
    finally:
        axis.close()


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(prog="newfocus-rotary-probe", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-c", "--config", help="YAML configuration file")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="list candidate USB devices")
    p = sub.add_parser("encoder", help="read and decode raw E201 frames")
    p.add_argument("--port")
    p.add_argument("-n", type=int, default=10)
    sub.add_parser("picomotor", help="query the 8742")
    p = sub.add_parser("calibrate", help="measure steps/degree in both directions")
    p.add_argument("--steps", type=int, default=2000)
    p = sub.add_parser("move", help="closed-loop move to a dial position (no IOC)")
    p.add_argument("--open-loop", action="store_true", help="move by step count, ignore the encoder")
    p.add_argument("target", type=float)
    p = sub.add_parser("encoder-status", help="read the AksIM-2 detailed status register")
    p.add_argument("--port")
    p = sub.add_parser("reset-multiturn", help="reset the AksIM-2 multiturn counter (clears multiturn error)")
    p.add_argument("--port")
    p.add_argument("--value", type=int, default=0, help="turn count to set (default 0)")
    p = sub.add_parser("raw", help="send raw E201 commands and print the replies")
    p.add_argument("--port")
    p.add_argument("commands", nargs="+")
    p = sub.add_parser("reg-read", help="read encoder registers through the E201")
    p.add_argument("--port")
    p.add_argument("--bank", type=_int)
    p.add_argument("address", type=_int)
    p.add_argument("length", type=_int, nargs="?", default=1)
    p = sub.add_parser("reg-write", help="write one encoder register through the E201")
    p.add_argument("--port")
    p.add_argument("--bank", type=_int)
    p.add_argument("--key", action="store_true", help="write unlock key 0xCD to 0x48 first")
    p.add_argument("address", type=_int)
    p.add_argument("value", type=_int)
    p = sub.add_parser("steps", help="move the picomotor a raw number of steps (no encoder)")
    p.add_argument("--velocity", type=int, help="steps/s (1-2000)")
    p.add_argument("steps", type=int)
    args = ap.parse_args(argv)
    fn = {"list": cmd_list, "encoder": cmd_encoder, "picomotor": cmd_picomotor,
          "calibrate": cmd_calibrate, "move": cmd_move, "steps": cmd_steps,
          "encoder-status": cmd_encoder_status, "reset-multiturn": cmd_reset_multiturn,
          "raw": cmd_raw, "reg-read": cmd_reg_read, "reg-write": cmd_reg_write}[args.cmd]
    sys.exit(fn(args))


if __name__ == "__main__":  # pragma: no cover
    main()
