# newfocus-rotary

An EPICS IOC that drives a **New Focus / Newport 8742** picomotor rotary actuator
**closed loop** on a **Renishaw/RLS AksIM-2** absolute encoder read through an
**E201-9B** USB BiSS-C interface. It serves a standard **EPICS motor record**
(`$(P)$(M)` with VAL, RBV, DMOV, STOP, TWF/TWR, HLM/LLM, SET, ...), so the usual
APS tooling works unchanged:

* MEDM / caQtDM `motorx.adl`, `motorx_all.adl` screens from the motor module
* `ophyd.EpicsMotor` / Bluesky (tested), SPEC, pyepics, `caget`/`caput`
* `caput -c` / put-callback: completes when the move is done, as with the real record

You set positions in **degrees**. The picomotor is open-loop by nature (its step
size changes with direction and load), so the IOC closes the loop in software
using the encoder.

```
 CA clients ──► caproto IOC (motor-record fields) ──► ClosedLoopRotaryAxis ──► 8742 (USB bulk / TCP)
 (MEDM, ophyd,                                         servo + learning    ──► E201-9B (USB CDC) ─► AksIM-2
  caput -c ...)
```

## Install

```bash
python3 -m venv /opt/newfocus-rotary/venv
/opt/newfocus-rotary/venv/bin/pip install .
sudo cp deploy/99-newfocus-rotary.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger
sudo usermod -aG plugdev,dialout $USER      # then log out and back in
```

The 8742 does **not** show up as a serial port on USB. It is a vendor-specific
device (VID 104d, PID 4000) and this package reaches it with **pyusb**, which
needs libusb-1.0 (`apt install libusb-1.0-0`) and the udev rule above. If you
prefer, the 8742's Ethernet port also works (`picomotor.transport: tcp`). The
E201 appears as a CDC virtual COM port (`/dev/ttyACM*`, VID 0483).

Try it without hardware:

```bash
newfocus-rotary-ioc --sim --prefix SIM: --motor m1
caput -c -w 60 SIM:m1 140.0 ; caget SIM:m1.RBV SIM:m1.DMOV SIM:m1:Status
```

## Commissioning

1. **Find the devices.** Run `newfocus-rotary-probe list`.
2. **Check the encoder frame format.** Copy `config/example.yaml`, set
   `encoder.singleturn_bits` / `multiturn_bits` to match your AksIM-2 part number,
   then run:
   ```bash
   newfocus-rotary-probe -c my.yaml encoder -n 20
   ```
   Each frame is printed with its CRC result. The tool also lists every frame
   length that passes CRC across all samples. With 2 status bits,
   `singleturn + multiturn = length - 2`. If the configured format fails CRC, it
   is wrong.
3. **Check the 8742.** Run `newfocus-rotary-probe -c my.yaml picomotor` and note
   which output (1–4) has the motor, then set `axis.picomotor_axis`.
4. **Calibrate.** Run `newfocus-rotary-probe -c my.yaml calibrate --steps 2000`.
   It moves +2000 then −2000 steps and measures steps/degree in each direction.
   Put the printed values in `axis.steps_per_degree[_reverse]`. If calibration
   says the encoder moved the wrong way, set `axis.encoder_sign: -1`.
5. **Test a move.** Run `newfocus-rotary-probe -c my.yaml move 10.0`. This is a
   closed-loop move to dial 10° without the IOC.
6. **Run the IOC.** Run `newfocus-rotary-ioc -c my.yaml -v`. A systemd unit is in
   `deploy/`.

Set `axis.state_file`. It keeps the SET-mode zero offset, the turn count (for
single-turn encoders) and the learned steps/degree across IOC restarts.

## Using it

```bash
medm -x -macro "P=29idX:,M=m1" motorx_all.adl     # stock motor-module screen
caput -c 29idX:m1 45.0                            # absolute move, waits for completion
caput 29idX:m1.TWV 0.5; caput 29idX:m1.TWF 1      # tweak
caput 29idX:m1.STOP 1
```

```python
from ophyd import EpicsMotor
rot = EpicsMotor("29idX:m1", name="rot")
rot.move(30.0, wait=True)
rot.set_current_position(0.0)   # SET mode; redefines the user position
```

### Motor-record fields that are implemented

| Field(s) | Behaviour |
|---|---|
| VAL, DVAL, RVAL, RLV, TWF/TWR/TWV | Closed-loop moves. User→dial uses OFF and DIR, as in the motor record. A new VAL while moving retargets the move (NTM). |
| RBV, DRBV, RRBV, REP, RMP | Readback from the encoder (deg). RRBV = DRBV/ERES. REP = unwrapped encoder counts. RMP = 8742 step counter. |
| DMOV, MOVN, MIP, TDIR, CDIR | Motion status. Put-callback on VAL/DVAL/RVAL completes on DMOV=1. |
| STOP, SPMG (Stop/Pause/Move/Go) | Same semantics as the motor record. After STOP or a jog, VAL is set to RBV. |
| JOGF/JOGR, JVEL | Continuous motion. Stops at the soft limits. |
| HOMF/HOMR | Moves to `motor.home_position` (dial). An absolute encoder needs no home search. Sets ATHM and MSTA.HOMED. |
| HLM/LLM, DHLM/DLLM, LVIO | Soft limits. DHLM = DLLM = 0 disables them. A target outside the limits is refused with LVIO=1. |
| SET/SSET/SUSE, FOFF, OFF, DIR | Calibration. SET with FOFF=Variable changes OFF; FOFF=Frozen (or DVAL in SET) redefines the dial position. |
| RDBD, RTRY, RCNT, MISS | In-position tolerance, maximum corrective moves, retries used, ran-out-of-retries flag. |
| BDST | Final approach always from the BDST side (closed loop). |
| VELO, ACCL, VMAX, S | Converted to 8742 steps/s using the learned steps/degree (8742 maximum is 2000 steps/s). |
| UEIP | Yes = closed loop (default). No = one open-loop move using the learned steps/degree. |
| CNEN | Disable stops the motor and refuses moves. |
| MRES, ERES, SREV, UREV | MRES = learned deg/step (writing it sets steps/degree). ERES = 360/2^singleturn_bits. |
| MSTA | DIRECTION, DONE, POSITION, SLIP_STALL, PRESENT, PROBLEM, MOVING, GAIN_SUPPORT, COMM_ERR, HOMED. |
| SYNC, PREC, EGU, DESC | Supported. |
| Alarms | INVALID/COMM on communication loss. MAJOR/STATE on encoder error or stall. MINOR on MISS or encoder warning. |

### Extra PVs (`$(P)$(M):...`)

| PV | |
|---|---|
| `:Status` | Text status, e.g. `in position (+0.00031 deg, 1 retries)` |
| `:StepsPerDegFwd`, `:StepsPerDegRev` | Learned step size for each direction (writable) |
| `:Calibrate` (+ `:CalibrateSteps`) | Write 1 to measure steps/degree with a ± move |
| `:ResetLearning` | Write 1 to go back to the last calibration |
| `:EncCounts`, `:EncSingleTurn`, `:StepCount` | Raw encoder and picomotor counters |
| `:EncError`, `:EncWarning`, `:CommError`, `:CRCErrors` | Diagnostics |

## How the closed loop works

Each move works like this:

1. Estimate the step count as error × steps/degree for the direction of travel.
2. Move in chunks of at most `chunk_time_s`, polling the encoder about every
   30 ms. Stop early once inside the fine zone.
3. Inside `fine_zone_deg`, command `fine_gain` × the estimate, so the target is
   approached from one side.
4. Repeat until |error| ≤ RDBD. Each correction after the first predicted
   arrival counts toward RTRY; if RTRY runs out, MISS is set.
5. After every move, update the steps/degree estimate for that direction
   (EWMA, clamped to `adapt_ratio_limits` × the calibration). This tracks load
   and direction asymmetry.

Safety checks during moves:

* **Stall:** encoder not following the steps for `stall_chunks` moves → SLIP_STALL.
* **Wrong direction:** encoder moving opposite to the commanded steps → fault (check `encoder_sign`).
* **Encoder fault:** AksIM-2 error bit or CRC failures stop motion.
* **Soft limits:** overshooting a limit by more than `limit_margin_deg` aborts the move.

**Single-turn encoders.** The IOC unwraps the encoder across the 360°→0 boundary,
so the dial is continuous over multiple turns. At startup it picks the turn
nearest the last position saved in `state_file`.

## Protocol references

* **8742:** ASCII over USB bulk endpoints or TCP/23. Commands end in `\r`,
  replies in `\r\n`. Axis-prefixed commands: `PR`, `PA`, `MV`, `ST`, `MD?`,
  `TP?`, `VA`, `AC`, `DH`, `TB?`, `*IDN?`; `n>` prefix for RS-485 slaves.
  Newport 8742 user manual, §6; cross-checked against pylablib's
  `Newport/picomotor.py` and the EPICS motorNewFocus module.
* **E201-9B:** USB CDC. `v` (version), `N`/`F` (encoder power), `4` (one
  BiSS-C frame as 16 hex chars), `Mnn` (clock). The frame is MSB aligned:
  `[MT][ST][nE][nW][~CRC6]`, CRC polynomial 0x43. This was taken from RLS's
  open-source E201 test platform (`github.com/nejcropic/e201_test_platform`),
  and the tests check this package's CRC and parser against that reference code.

## Development

```bash
pip install -e .[test]
pytest            # unit tests, simulated closed loop, and end-to-end Channel Access tests
```

`newfocus_rotary.sim` simulates the 8742 + stage + E201. The simulated stage has
direction-dependent, angle-dependent and noisy step sizes, so the closed loop
has real work to do.

## Limitations

* One axis per IOC process (one encoder per picomotor output). Run several
  processes for more axes.
* This is a Python (caproto) Channel Access server that implements the
  motor-record *interface*. It does not run the C motor record, so record
  links (DOL/OUT/RDBL/FLNK), the S/SBAS/SMAX speed fields, PID gains and
  hardware limit switches (HLS/LLS, always 0) are not active. Clients that use
  the record as a positioner do not notice the difference.
* Not yet tested on real hardware. The BiSS frame layout depends on the AksIM-2
  variant, so check it with `newfocus-rotary-probe encoder` before the first move.
