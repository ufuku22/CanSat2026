#!/usr/bin/env python3
"""Build JPEG image packets for TLM922S P2P transmission."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import struct
import zlib


MAGIC = b"CI"
VERSION = 1
TYPE_IMAGE = ord("I")

DEFAULT_MAX_RADIO_PAYLOAD = 242
FEC_RATIO_NUMERATOR = 4
FEC_RATIO_DENOMINATOR = 3

_HEADER = struct.Struct(">2sBBIIIBBBH")
HEADER_SIZE = _HEADER.size


@dataclass(frozen=True)
class ImagePacket:
    """One encoded image block that fits in a radio payload."""

    file_id: int
    file_size: int
    crc32: int
    k: int
    m: int
    index: int
    block_size: int
    block: bytes

    def to_bytes(self) -> bytes:
        if len(self.block) != self.block_size:
            raise ValueError("block length must match block_size")

        return _HEADER.pack(
            MAGIC,
            VERSION,
            TYPE_IMAGE,
            self.file_id,
            self.file_size,
            self.crc32,
            self.k,
            self.m,
            self.index,
            self.block_size,
        ) + self.block


def build_image_packets(
    image_path: str | Path,
    *,
    max_radio_payload: int = DEFAULT_MAX_RADIO_PAYLOAD,
    redundancy_ratio: float = FEC_RATIO_NUMERATOR / FEC_RATIO_DENOMINATOR,
) -> list[ImagePacket]:
    path = Path(image_path)
    if path.suffix.lower() not in {".jpg", ".jpeg"}:
        raise ValueError("input image must be a .jpg or .jpeg file")

    image = path.read_bytes()
    if not image:
        raise ValueError("input image is empty")

    block_size = max_radio_payload - HEADER_SIZE
    if block_size <= 0:
        raise ValueError("max_radio_payload is too small")

    k = math.ceil(len(image) / block_size)
    m = math.ceil(k * redundancy_ratio)
    if not 1 <= k <= m <= 255:
        raise ValueError("FEC currently supports 1 to 255 encoded blocks")

    padded = image + b"\x00" * ((k * block_size) - len(image))
    source_blocks = [padded[i * block_size : (i + 1) * block_size] for i in range(k)]
    encoded_blocks = encode_fec_blocks(source_blocks, m)

    crc32 = zlib.crc32(image) & 0xFFFFFFFF
    file_id = crc32 ^ len(image)
    return [
        ImagePacket(
            file_id=file_id,
            file_size=len(image),
            crc32=crc32,
            k=k,
            m=m,
            index=index,
            block_size=block_size,
            block=block,
        )
        for index, block in enumerate(encoded_blocks)
    ]


def encode_fec_blocks(source_blocks: list[bytes], m: int) -> list[bytes]:
    if not source_blocks:
        raise ValueError("source_blocks must not be empty")

    k = len(source_blocks)
    block_size = len(source_blocks[0])
    if any(len(block) != block_size for block in source_blocks):
        raise ValueError("all source blocks must have the same length")

    matrix = [_vandermonde_row(row, k) for row in range(m)]
    return [_combine_blocks(row, source_blocks, block_size) for row in matrix]


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
