import os
import select
import termios
import time


BAUD_MAP = {
    9600: termios.B9600,
    19200: termios.B19200,
    57600: termios.B57600,
    115200: termios.B115200,
}


class Tlm922sUart:
    """TLM922SのASCIIコマンド用UARTヘルパー。

    TLM922SのコマンドはASCII文字列で、終端はCR ('\r') です。
    応答は多くの場合 '>>' から始まります。
    """

    def __init__(self, port="/dev/serial0", baudrate=115200, timeout=1.5):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.fd = None

    def __enter__(self):
        if self.baudrate not in BAUD_MAP:
            raise ValueError(f"Unsupported baudrate: {self.baudrate}")

        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)

        # 8N1、フロー制御なし、入出力はrawモード。
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[3] = 0
        attrs[4] = BAUD_MAP[self.baudrate]
        attrs[5] = BAUD_MAP[self.baudrate]
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0

        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def send_command(self, command):
        data = command.encode("ascii") + b"\r"
        written = 0
        while written < len(data):
            _, ready, _ = select.select([], [self.fd], [], self.timeout)
            if not ready:
                raise TimeoutError("Timed out waiting for UART to become writable.")

            try:
                count = os.write(self.fd, data[written:])
            except BlockingIOError:
                continue

            if count == 0:
                raise OSError("UART write returned 0 bytes.")
            written += count

        termios.tcdrain(self.fd)

    def read_for(self, seconds):
        end_time = time.monotonic() + seconds
        chunks = []

        while time.monotonic() < end_time:
            ready, _, _ = select.select([self.fd], [], [], 0.05)
            if not ready:
                continue

            try:
                data = os.read(self.fd, 4096)
            except BlockingIOError:
                continue

            if data:
                chunks.append(data)

        return b"".join(chunks).decode("ascii", errors="replace")

    def command(self, command, wait=None):
        """1つのコマンドを送り、待ち時間内に受信した文字列を返す。"""
        termios.tcflush(self.fd, termios.TCIFLUSH)
        self.send_command(command)
        return self.read_for(self.timeout if wait is None else wait)


def text_to_hex(text):
    return text.encode("utf-8").hex()


def hex_to_text(hex_text):
    return bytes.fromhex(hex_text).decode("utf-8", errors="replace")


def ok_response(text):
    return ">> Ok" in text


def print_response(text):
    cleaned = text.replace("\r", "\n").strip()
    print(cleaned if cleaned else "(no response)")
