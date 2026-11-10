# EncoSight under Wine (Raspberry Pi / Linux)

Runs RLS **EncoSight for E201-9B** (a 32-bit Windows program) under Wine and
connects it to the E201-9B over USB. It can also log every byte EncoSight
exchanges with the E201, for example to see how it resets the AksIM-2
multiturn counter.

```bash
./setup-pi.sh ~/EncoSight_for_E201-9B_ver5.0.0.17.exe    # once
encosight                                                # run
encosight --sniff                                        # run + log E201 traffic to ~/e201-traffic-*.log
```

**Stop the EPICS IOC first.** EncoSight and the IOC can't share the E201 port;
the launcher refuses to start while the port is in use.

## What the scripts do

* **`setup-pi.sh`:**
  * checks the page size (a Raspberry Pi 5 needs `kernel=kernel8.img`, see below)
  * adds you to `dialout`
  * installs Wine: Box64 + x86-64 Wine through Pi-Apps' *Wine (x64)* on ARM,
    the distribution's `wine` on x86-64, or an existing `wine`
  * creates the prefix `~/.wine-encosight`, copies EncoSight into it, and
    installs the `encosight` launcher and a menu entry
* **`encosight.sh`:** finds the E201 (USB vendor 0483, preferring its
  `/dev/serial/by-id` name), maps it to COM5, fixes the registry (below), and
  starts EncoSight. With `--sniff` it starts `e201_sniff.py` in between.
* **`e201_sniff.py`:** a transparent pty↔serial proxy. EncoSight talks to the
  pty, the proxy forwards to the real device and logs:
  ```
      4.8591 PC>  b'R01:066'
      4.8592 <DEV b'0:00:42\r'
  ```

## Why the registry fix is needed

EncoSight lists ports from `HKLM\HARDWARE\DEVICEMAP\SERIALCOMM`, but only
accepts entries named like Windows' USB-serial driver (`\Device\USBSER000`).
That's what the ST Virtual COM Port driver (usbser.sys) creates on Windows.
Wine registers mapped ports as `\Device\SerialN` and rebuilds this key every
session, so without the fix EncoSight sits at "Searching for interface…"
forever. The launcher re-adds `\Device\USBSER000 = COM5` at each start. The
Windows driver package itself isn't needed: Linux's `cdc_acm` driver handles
the E201 (`/dev/ttyACM*`).

## Raspberry Pi notes

* **64-bit OS required.** x86 code runs through Box64 emulation. EncoSight is
  a light VCL program, but expect a slower UI than on a PC.
* **Raspberry Pi 5:** the default kernel uses 16K pages, which Wine/Box64
  can't use. Add `kernel=kernel8.img` to `/boot/firmware/config.txt`, reboot,
  and check that `getconf PAGESIZE` prints 4096.
* At start-up EncoSight offers to check for updates over the internet;
  "Skip" works fine.

## Tested

On x86-64 Linux (Wine 9.0, both 32- and 64-bit prefixes) against a simulated
E201 on a pseudo-terminal: EncoSight detects the port, runs its handshake
(`v`, `N`, `r`, `e`, `R01:066`), and the sniffer logs it. Not yet tested on a
Raspberry Pi or with real hardware. The ARM path relies on Pi-Apps'
*Wine (x64)*.
