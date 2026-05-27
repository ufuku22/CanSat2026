#!/usr/bin/env python3
"""JPEG画像をTLM922Sの小さなP2Pパケットへ分割・復元するための処理。"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable
import math
import struct
import zlib


MAGIC = b"CI"
VERSION = 1
TYPE_IMAGE = ord("I")

# TLM922S/SX1272の実用上限に合わせる。必要なら送信時に変更できる。
DEFAULT_MAX_RADIO_PAYLOAD = 242
FEC_RATIO_NUMERATOR = 4
FEC_RATIO_DENOMINATOR = 3

_HEADER = struct.Struct(">2sBBIIIBBBH")
HEADER_SIZE = _HEADER.size


@dataclass(frozen=True)
class ImagePacket:
    """1つの無線パケットに入れる自己完結な画像断片。"""

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

    @classmethod
    def from_bytes(cls, payload: bytes) -> "ImagePacket":
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


def is_image_packet_hex(payload_hex: str) -> bool:
    """ESP32側でも同じ判定をするための、Python側の簡易判定。"""
    return payload_hex.upper().startswith("43490149")


def build_image_packets(
    image_path: str | Path,
    *,
    max_radio_payload: int = DEFAULT_MAX_RADIO_PAYLOAD,
    redundancy_ratio: float = FEC_RATIO_NUMERATOR / FEC_RATIO_DENOMINATOR,
) -> list[ImagePacket]:
    """JPEGファイルから、FEC付きの送信用パケット列を作る。"""
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


@dataclass
class ImageSession:
    """PC側で画像パケットを集め、自動復元するための受信セッション。"""

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
            raise ValueError(f"image CRC32 mismatch: expected {self.crc32:08x}, got {actual_crc32:08x}")
        return image


def session_from_packet_hex(payload_hex: str) -> ImagePacket:
    return ImagePacket.from_bytes(bytes.fromhex(payload_hex.strip()))


def encode_fec_blocks(source_blocks: list[bytes], m: int) -> list[bytes]:
    """Vandermonde行列を使った小さなReed-Solomon消失訂正。"""
    if not source_blocks:
        raise ValueError("source_blocks must not be empty")
    k = len(source_blocks)
    block_size = len(source_blocks[0])
    if any(len(block) != block_size for block in source_blocks):
        raise ValueError("all source blocks must have the same length")

    matrix = _vandermonde_matrix(m, k)
    return [_combine_blocks(row, source_blocks, block_size) for row in matrix]


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


def _vandermonde_matrix(rows: int, cols: int) -> list[list[int]]:
    return [_vandermonde_row(row, cols) for row in range(rows)]


def _vandermonde_row(row: int, cols: int) -> list[int]:
    # 0は使わず、1..255の相異なる値を評価点にする。
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
