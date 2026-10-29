#!/usr/bin/env python3
"""Recover JPEG images from ESP32-C3 ground station serial output."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable
import re
import struct
import zlib


MAGIC = b"CI"
VERSION = 1
TYPE_IMAGE = ord("I")
IMAGE_PACKET_LINE = re.compile(r"(?:IMG_PACKET|radio_rx)\s+([0-9A-Fa-f]+)")

_HEADER = struct.Struct(">2sBBIIIBBBH")
HEADER_SIZE = _HEADER.size


class ImageCrcMismatchError(ValueError):
    def __init__(self, message: str, image: bytes) -> None:
        super().__init__(message)
        self.image = image


@dataclass(frozen=True)
class ImagePacket:
    file_id: int
    file_size: int
    crc32: int
    k: int
    m: int
    index: int
    block_size: int
    block: bytes

    @classmethod
    def from_hex(cls, payload_hex: str) -> "ImagePacket":
        try:
            payload = bytes.fromhex(payload_hex.strip())
        except ValueError as exc:
            raise ValueError("payload is not valid hex") from exc

        if len(payload) < HEADER_SIZE:
            raise ValueError("payload is too short")

        magic, version, packet_type, file_id, file_size, crc32, k, m, index, block_size = _HEADER.unpack(
            payload[:HEADER_SIZE]
        )
        if magic != MAGIC or version != VERSION or packet_type != TYPE_IMAGE:
            raise ValueError("payload is not an image packet")

        block = payload[HEADER_SIZE:]
        if len(block) != block_size:
            raise ValueError("payload block length does not match header")

        return cls(file_id, file_size, crc32, k, m, index, block_size, block)


@dataclass(frozen=True)
class ImageReceiveResult:
    file_id: int
    collected: int
    required: int
    total_packets: int
    received_index: int
    saved_path: Path | None = None
    error: str | None = None

    @property
    def saved(self) -> bool:
        return self.saved_path is not None


@dataclass
class ImageSession:
    file_id: int
    file_size: int
    crc32: int
    k: int
    m: int
    block_size: int
    blocks: dict[int, bytes] = field(default_factory=dict)

    @classmethod
    def from_packet(cls, packet: ImagePacket) -> "ImageSession":
        return cls(
            file_id=packet.file_id,
            file_size=packet.file_size,
            crc32=packet.crc32,
            k=packet.k,
            m=packet.m,
            block_size=packet.block_size,
        )

    def add(self, packet: ImagePacket) -> None:
        if packet.file_id != self.file_id:
            raise ValueError("packet belongs to a different image")
        if (
            packet.file_size != self.file_size
            or packet.crc32 != self.crc32
            or packet.k != self.k
            or packet.m != self.m
            or packet.block_size != self.block_size
        ):
            raise ValueError("packet metadata does not match this image session")
        self.blocks.setdefault(packet.index, packet.block)

    def can_recover(self) -> bool:
        return len(self.blocks) >= self.k

    def recover(self) -> bytes:
        if not self.can_recover():
            raise ValueError("not enough packets to recover image")

        indexes = sorted(self.blocks)[: self.k]
        encoded_blocks = [self.blocks[index] for index in indexes]
        image = b"".join(decode_fec_blocks(indexes, encoded_blocks, self.k))[: self.file_size]

        actual_crc32 = zlib.crc32(image) & 0xFFFFFFFF
        if actual_crc32 != self.crc32:
            raise ImageCrcMismatchError(
                f"image CRC32 mismatch: expected {self.crc32:08x}, got {actual_crc32:08x}",
                image,
            )
        return image


@dataclass
class ImageReceiveStore:
    output_dir: Path | str = "received_images"
    sessions: dict[int, ImageSession] = field(default_factory=dict)
    saved_file_ids: set[int] = field(default_factory=set)

    def __post_init__(self) -> None:
        self.output_dir = Path(self.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def add_line(self, line: str) -> ImageReceiveResult | None:
        match = IMAGE_PACKET_LINE.search(line)
        if match is None:
            return None
        return self.add_payload_hex(match.group(1))

    def add_payload_hex(self, payload_hex: str) -> ImageReceiveResult | None:
        try:
            packet = ImagePacket.from_hex(payload_hex)
        except ValueError:
            return None

        if packet.file_id in self.saved_file_ids:
            return ImageReceiveResult(
                file_id=packet.file_id,
                collected=packet.k,
                required=packet.k,
                total_packets=packet.m,
                received_index=packet.index,
                saved_path=self.output_dir / f"{packet.file_id:08x}.jpg",
            )

        session = self.sessions.setdefault(packet.file_id, ImageSession.from_packet(packet))
        try:
            session.add(packet)
        except ValueError as exc:
            if str(exc) == "packet metadata does not match this image session":
                session = ImageSession.from_packet(packet)
                self.sessions[packet.file_id] = session
                session.add(packet)
            else:
                return ImageReceiveResult(
                    file_id=packet.file_id,
                    collected=len(session.blocks),
                    required=session.k,
                    total_packets=session.m,
                    received_index=packet.index,
                    error=str(exc),
                )

        result = ImageReceiveResult(
            file_id=packet.file_id,
            collected=len(session.blocks),
            required=session.k,
            total_packets=session.m,
            received_index=packet.index,
        )
        if not session.can_recover():
            return result

        try:
            image = session.recover()
            error = None
        except ImageCrcMismatchError as exc:
            image = exc.image
            error = str(exc)
        except ValueError as exc:
            return ImageReceiveResult(
                file_id=packet.file_id,
                collected=len(session.blocks),
                required=session.k,
                total_packets=session.m,
                received_index=packet.index,
                error=str(exc),
            )

        output_path = self.output_dir / f"{packet.file_id:08x}.jpg"
        output_path.write_bytes(image)
        self.saved_file_ids.add(packet.file_id)
        self.sessions.pop(packet.file_id, None)
        return ImageReceiveResult(
            file_id=packet.file_id,
            collected=result.collected,
            required=result.required,
            total_packets=result.total_packets,
            received_index=result.received_index,
            saved_path=output_path,
            error=error,
        )


def decode_fec_blocks(indexes: Iterable[int], blocks: list[bytes], k: int) -> list[bytes]:
    indexes = list(indexes)
    if len(indexes) != k or len(blocks) != k:
        raise ValueError("exactly k blocks are required for decoding")

    block_size = len(blocks[0])
    if any(len(block) != block_size for block in blocks):
        raise ValueError("all encoded blocks must have the same length")

    matrix = [_vandermonde_row(index, k) for index in indexes]
    inverse = _invert_matrix(matrix)
    return [_combine_blocks(row, blocks, block_size) for row in inverse]


def _vandermonde_row(row: int, cols: int) -> list[int]:
    x = row + 1
    return [_gf_pow(x, power) for power in range(cols)]


def _combine_blocks(coefficients: list[int], blocks: list[bytes], block_size: int) -> bytes:
    output = bytearray(block_size)
    for coefficient, block in zip(coefficients, blocks):
        if coefficient == 0:
            continue
        for i, value in enumerate(block):
            output[i] ^= _gf_mul(coefficient, value)
    return bytes(output)


def _invert_matrix(matrix: list[list[int]]) -> list[list[int]]:
    n = len(matrix)
    augmented = [row[:] + [1 if i == j else 0 for j in range(n)] for i, row in enumerate(matrix)]

    for col in range(n):
        pivot = next((row for row in range(col, n) if augmented[row][col] != 0), None)
        if pivot is None:
            raise ValueError("FEC matrix is not invertible")
        if pivot != col:
            augmented[col], augmented[pivot] = augmented[pivot], augmented[col]

        inv_pivot = _gf_inv(augmented[col][col])
        augmented[col] = [_gf_mul(value, inv_pivot) for value in augmented[col]]

        for row in range(n):
            if row == col:
                continue
            factor = augmented[row][col]
            if factor == 0:
                continue
            augmented[row] = [
                value ^ _gf_mul(factor, pivot_value)
                for value, pivot_value in zip(augmented[row], augmented[col])
            ]

    return [row[n:] for row in augmented]


def _gf_mul(a: int, b: int) -> int:
    result = 0
    while b:
        if b & 1:
            result ^= a
        b >>= 1
        a <<= 1
        if a & 0x100:
            a ^= 0x11D
    return result & 0xFF


def _gf_pow(a: int, power: int) -> int:
    result = 1
    for _ in range(power):
        result = _gf_mul(result, a)
    return result


def _gf_inv(a: int) -> int:
    if a == 0:
        raise ZeroDivisionError("cannot invert zero in GF(256)")
    return _gf_pow(a, 254)
