#!/usr/bin/env python3
from pathlib import Path
from datetime import datetime
import subprocess
import sys


SAVE_DIR = Path.home() / "camera_test_images"
SAVE_DIR.mkdir(exist_ok=True)

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
output_path = SAVE_DIR / f"test_{timestamp}.jpg"

cmd = [
    "libcamera-still",
    "-o",
    str(output_path),
    "--width",
    "1920",
    "--height",
    "1080",
    "--timeout",
    "2000",
    "--nopreview",
]

print("Taking photo...")
print(f"Save path: {output_path}")

try:
    subprocess.run(
        cmd,
        check=True,
        text=True,
        capture_output=True,
    )
except FileNotFoundError:
    print("ERROR: libcamera-still was not found.")
    print("Try installing it with:")
    print("  sudo apt update")
    print("  sudo apt install -y libcamera-apps")
    sys.exit(1)
except subprocess.CalledProcessError as e:
    print("ERROR: Failed to capture image.")
    if e.stderr:
        print(e.stderr)
    sys.exit(e.returncode)

if output_path.exists() and output_path.stat().st_size > 0:
    print("OK: Image was saved.")
    print(output_path)
else:
    print("ERROR: Image file was not created.")
    sys.exit(1)
