#!/usr/bin/env python3
"""TLM922S P2PでJPEG画像をFEC付き送信する簡易スクリプト。"""

from __future__ import annotations

import argparse
from pathlib import Path

import sys


# リポジトリ直下を読み込む。
sys.path.append(str(Path(__file__).resolve().parents[1]))

from communication_manager import (
    DEFAULT_IMAGE_INTER_PACKET_DELAY,
    DEFAULT_MAX_RADIO_PAYLOAD,
    DEFAULT_RADIO_TIMEOUT,
    CommunicationManager,
)
from image_transfer import ImagePacket


BASE_TEST_JPEG = bytes.fromhex(
    # 1x1 pixel JPEG。後ろにコメントセグメントを足して、任意のテストサイズにする。
    "ffd8ffe000104a46494600010100000100010000"
    "ffdb0043000302020302020303030304030304050805050404050a070706080c0a0c0c0b0a0b0b0d0e12100d0e110e0b0b1016101113141515150c0f171816141812141514"
    "ffdb00430103040405040509050509140d0b0d141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414"
    "ffc00011080001000103012200021101031101"
    "ffc4001400010000000000000000000000000000000000000008"
    "ffc4001410010000000000000000000000000000000000000000"
    "ffc4001401010000000000000000000000000000000000000008"
    "ffc4001411010000000000000000000000000000000000000000"
    "ffda000c03010002110311003f00b2c001ffd9"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send a .jpg image with TLM922S P2P FEC packets.")
    parser.add_argument("--image", help="path to a .jpg or .jpeg file")
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=DEFAULT_RADIO_TIMEOUT, help="seconds to wait for radio_tx_ok per packet")
    parser.add_argument("--max-radio-payload", type=int, default=DEFAULT_MAX_RADIO_PAYLOAD)
    parser.add_argument("--delay", type=float, default=DEFAULT_IMAGE_INTER_PACKET_DELAY, help="seconds between packets")
    parser.add_argument(
        "--generate-test-image",
        default="test_image_5kb.jpg",
        help="create and send this JPEG when image is omitted",
    )
    parser.add_argument("--test-image-size", type=int, default=5000, help="generated JPEG size in bytes")
    return parser.parse_args()


def write_test_jpeg(path: Path, size: int) -> None:
    """外部ライブラリなしで、指定サイズに近いJPEGテスト画像を作る。"""
    if size < len(BASE_TEST_JPEG):
        raise ValueError(f"test image size must be at least {len(BASE_TEST_JPEG)} bytes")

    # JPEGのCOMセグメントは画像表示に影響しないコメント領域。
    # 1セグメントの長さ上限を避けるため、必要なら複数に分けて追加する。
    body = BASE_TEST_JPEG[:-2]
    tail = BASE_TEST_JPEG[-2:]
    extra_bytes = size - len(BASE_TEST_JPEG)
    chunks: list[bytes] = []
    pattern = b"CanSat2026 TLM922S image transfer test "

    while extra_bytes >= 4:
        data_len = min(extra_bytes - 4, 65533)
        comment = (pattern * ((data_len // len(pattern)) + 1))[:data_len]
        chunks.append(b"\xff\xfe" + (data_len + 2).to_bytes(2, "big") + comment)
        extra_bytes -= data_len + 4

    # 端数が1〜3バイトだけ残る場合は、EOI後の余剰データとして足す。
    # 通信テストではバイト列が復元できればよく、多くのJPEGデコーダはこの余剰を無視する。
    path.write_bytes(body + b"".join(chunks) + tail + (b"\x00" * extra_bytes))


def print_send_progress(packet_number: int, packet_count: int, _packet: ImagePacket, response: str) -> None:
    ok = "OK" if "radio_tx_ok" in response else "NO radio_tx_ok"
    print(
        f"packet {packet_number}/{packet_count} {ok}",
        flush=True,
    )


def main() -> int:
    args = parse_args()
    if args.image is None:
        image_path = Path(args.generate_test_image)
        write_test_jpeg(image_path, args.test_image_size)
        print(f"Generated test JPEG: {image_path} ({image_path.stat().st_size} bytes)")
    else:
        image_path = Path(args.image)

    with CommunicationManager(port=args.port, baudrate=args.baudrate, timeout=args.timeout) as comm:
        result = comm.send_image(
            image_path,
            max_radio_payload=args.max_radio_payload,
            inter_packet_delay=args.delay,
        )

    print(
        f"Sent {result.image_path} ({result.file_size} bytes): "
        f"k={result.k}, m={result.m}, block={result.block_size} bytes, file_id={result.file_id:08x}"
    )
    print(f"radio_tx_ok: {result.radio_tx_ok_count}/{len(result.responses)}")
    return 0 if result.all_radio_tx_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
