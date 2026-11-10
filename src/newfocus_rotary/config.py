"""YAML configuration and hardware construction."""

from __future__ import annotations

import dataclasses
import logging
from dataclasses import dataclass, field
from typing import Any, Optional

from .axis import AxisConfig, AxisParams
from .e201 import E201_VID, BissFrameFormat
from .sim import SimStageParams

log = logging.getLogger(__name__)


@dataclass
class IocConfig:
    prefix: str = "NFR:"
    motor: str = "m1"
    description: str = "Rotary picomotor (closed loop)"
    egu: str = "deg"
    precision: int = 4
    update_rate_hz: float = 10.0


@dataclass
class PicomotorConfig:
    transport: str = "usb"  # usb | tcp | sim
    usb_serial: Optional[str] = None
    usb_index: int = 0
    host: Optional[str] = None
    port: int = 23
    address: Optional[int] = None  # RS-485 daisy-chain address
    timeout_s: float = 2.0


@dataclass
class EncoderConfig:
    transport: str = "serial"  # serial | sim | none (no encoder: open loop only)
    port: str = "auto"  # e.g. /dev/ttyACM0, /dev/serial/by-id/..., or auto (first VID 0x0483)
    usb_serial: Optional[str] = None  # used with port: auto
    singleturn_bits: int = 20
    multiturn_bits: int = 0
    status_bits: int = 2
    crc_bits: int = 6
    clock_khz: Optional[int] = None
    power_on: bool = True
    timeout_s: float = 0.5

    def frame_format(self) -> BissFrameFormat:
        return BissFrameFormat(
            singleturn_bits=self.singleturn_bits,
            multiturn_bits=self.multiturn_bits,
            status_bits=self.status_bits,
            crc_bits=self.crc_bits,
        )


@dataclass
class MotorDefaults:
    """Initial motor-record field values (dial coordinates, degrees)."""

    velocity: float = 1.0  # VELO deg/s
    accel: float = 0.2  # ACCL s
    retry_deadband: float = 0.002  # RDBD deg
    max_retries: int = 10  # RTRY
    backlash: float = 0.0  # BDST deg
    jog_velocity: float = 0.5  # JVEL deg/s
    tweak: float = 1.0  # TWV deg
    home_position: float = 0.0  # dial position for HOMF/HOMR
    dial_high_limit: float = 0.0  # DHLM (DHLM == DLLM == 0 disables limits)
    dial_low_limit: float = 0.0  # DLLM
    user_offset: float = 0.0  # OFF
    user_direction: str = "Pos"  # DIR
    use_encoder: bool = True  # UEIP: False starts in open loop (encoder ignored)


@dataclass
class Config:
    ioc: IocConfig = field(default_factory=IocConfig)
    picomotor: PicomotorConfig = field(default_factory=PicomotorConfig)
    encoder: EncoderConfig = field(default_factory=EncoderConfig)
    axis: AxisConfig = field(default_factory=AxisConfig)
    motor: MotorDefaults = field(default_factory=MotorDefaults)
    sim: SimStageParams = field(default_factory=SimStageParams)

    def axis_params(self) -> AxisParams:
        m = self.motor
        limits = not (m.dial_high_limit == 0 and m.dial_low_limit == 0)
        return AxisParams(
            deadband_deg=m.retry_deadband,
            max_retries=m.max_retries,
            velocity_deg_s=m.velocity,
            accel_time_s=m.accel,
            backlash_deg=m.backlash,
            jog_velocity_deg_s=m.jog_velocity,
            home_position_deg=m.home_position,
            closed_loop=m.use_encoder and self.encoder.transport != "none",
            low_limit_deg=m.dial_low_limit if limits else None,
            high_limit_deg=m.dial_high_limit if limits else None,
        )


def _fill(cls, data: Optional[dict]):
    data = dict(data or {})
    names = {f.name: f for f in dataclasses.fields(cls)}
    unknown = set(data) - set(names)
    if unknown:
        raise ValueError(f"Unknown {cls.__name__} keys: {sorted(unknown)}")
    if "adapt_ratio_limits" in data:
        data["adapt_ratio_limits"] = tuple(data["adapt_ratio_limits"])
    return cls(**data)


def load_config(path: Optional[str] = None, overrides: Optional[dict[str, Any]] = None) -> Config:
    raw: dict = {}
    if path:
        import yaml

        with open(path) as f:
            raw = yaml.safe_load(f) or {}
    for k, v in (overrides or {}).items():
        section, _, key = k.partition(".")
        raw.setdefault(section, {})[key] = v
    known = {"ioc", "picomotor", "encoder", "axis", "motor", "sim"}
    unknown = set(raw) - known
    if unknown:
        raise ValueError(f"Unknown config sections: {sorted(unknown)}")
    return Config(
        ioc=_fill(IocConfig, raw.get("ioc")),
        picomotor=_fill(PicomotorConfig, raw.get("picomotor")),
        encoder=_fill(EncoderConfig, raw.get("encoder")),
        axis=_fill(AxisConfig, raw.get("axis")),
        motor=_fill(MotorDefaults, raw.get("motor")),
        sim=_fill(SimStageParams, raw.get("sim")),
    )


def build_hardware(cfg: Config):
    """Return ``(picomotor, encoder)`` driver objects according to the config."""
    pc, ec = cfg.picomotor, cfg.encoder
    stage = None
    if pc.transport == "sim" or ec.transport == "sim":
        from .sim import SimRotaryStage

        stage = SimRotaryStage(cfg.sim)

    if pc.transport == "sim":
        from .sim import SimPicomotor8742

        pm = SimPicomotor8742(stage, axis=cfg.axis.picomotor_axis)
    else:
        from .picomotor import Picomotor8742
        from .transports import TcpTransport, UsbBulkTransport

        if pc.transport == "usb":
            t = UsbBulkTransport(serial_number=pc.usb_serial, index=pc.usb_index, timeout=pc.timeout_s)
        elif pc.transport == "tcp":
            if not pc.host:
                raise ValueError("picomotor.host is required for tcp transport")
            t = TcpTransport(pc.host, pc.port, timeout=pc.timeout_s)
        else:
            raise ValueError(f"unknown picomotor transport {pc.transport!r}")
        pm = Picomotor8742(t, address=pc.address)
        log.info("8742: %s via %s", pm.identity, t.description)

    fmt = ec.frame_format()
    if ec.transport == "none":
        enc = None
    elif ec.transport == "sim":
        from .sim import SimE201

        enc = SimE201(stage, fmt)
    elif ec.transport == "serial":
        from .e201 import E201_9B
        from .transports import SerialTransport, TransportError, find_serial_port_by_vid

        port = ec.port
        if port == "auto":
            port = find_serial_port_by_vid(E201_VID, ec.usb_serial)
            if port is None:
                raise TransportError("No E201 (USB VID 0x0483) serial port found")
        enc = E201_9B(
            SerialTransport(port, timeout=ec.timeout_s),
            fmt,
            power_on=ec.power_on,
            clock_khz=ec.clock_khz,
        )
        log.info("E201: %s on %s", enc.version, port)
    else:
        raise ValueError(f"unknown encoder transport {ec.transport!r}")
    return pm, enc
