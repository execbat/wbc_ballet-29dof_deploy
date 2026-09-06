"""Wire-compatible implementation of Orbit's ballet UDP protocol (G1-29DoF).

Packet layout (little-endian float32): ``[targets(29), mask(29), velocity(3)]``
= 61 floats. Joint axis order is the robot's canonical joint order (see
``wbc_ballet.robots.g1.constants`` / the compiled MJCF joint order) --
identical to the order used by ``joint_pos``/``axis_actual_normalized`` in
the policy observation and by the action space, so an axis index means the
same joint everywhere in the stack.
"""

from __future__ import annotations

import socket
from dataclasses import dataclass
from typing import Self

import numpy as np

NUM_JOINTS = 29
PACKET_FLOATS = NUM_JOINTS * 2 + 3
PACKET_BYTES = PACKET_FLOATS * np.dtype(np.float32).itemsize


@dataclass(frozen=True, slots=True)
class BalletCommand:
    """Normalized joint targets, enable mask, and base twist command."""

    targets: np.ndarray
    mask: np.ndarray
    velocity: np.ndarray

    def __post_init__(self) -> None:
        for name, value, size in (
            ("targets", self.targets, NUM_JOINTS),
            ("mask", self.mask, NUM_JOINTS),
            ("velocity", self.velocity, 3),
        ):
            arr = np.asarray(value, dtype=np.float32)
            if arr.shape != (size,):
                raise ValueError(f"{name} must have shape ({size},), got {arr.shape}")
            object.__setattr__(self, name, arr)

    @classmethod
    def neutral(cls) -> BalletCommand:
        return cls(np.zeros(NUM_JOINTS), np.zeros(NUM_JOINTS), np.zeros(3))

    @classmethod
    def from_bytes(cls, payload: bytes) -> BalletCommand:
        if len(payload) != PACKET_BYTES:
            raise ValueError(f"expected {PACKET_BYTES} bytes, got {len(payload)}")
        values = np.frombuffer(payload, dtype="<f4", count=PACKET_FLOATS).copy()
        if not np.isfinite(values).all():
            raise ValueError("packet contains NaN or infinity")
        targets = np.clip(values[:NUM_JOINTS], -1.0, 1.0)
        mask = (values[NUM_JOINTS : 2 * NUM_JOINTS] >= 0.5).astype(np.float32)
        velocity = values[-3:]
        return cls(targets, mask, velocity)

    def to_bytes(self) -> bytes:
        return np.concatenate((self.targets, self.mask, self.velocity)).astype("<f4").tobytes()


class UdpCommandReceiver:
    """Non-blocking last-value receiver; malformed packets are ignored safely."""

    def __init__(self, host: str = "127.0.0.1", port: int = 55001) -> None:
        self.command = BalletCommand.neutral()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((host, port))
        self.socket.setblocking(False)

    def poll(self) -> BalletCommand:
        while True:
            try:
                payload, _ = self.socket.recvfrom(65535)
            except BlockingIOError:
                return self.command
            try:
                self.command = BalletCommand.from_bytes(payload)
            except ValueError:
                continue

    def close(self) -> None:
        self.socket.close()

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
