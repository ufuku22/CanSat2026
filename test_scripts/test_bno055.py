import time

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus


BUS_NUMBER = 1
BNO055_ADDRESS = 0x28

CHIP_ID = 0x00
PAGE_ID = 0x07
OPR_MODE = 0x3D
PWR_MODE = 0x3E
SYS_TRIGGER = 0x3F
EULER_H_LSB = 0x1A
ACC_DATA_X_LSB = 0x08
GYR_DATA_X_LSB = 0x14
CALIB_STAT = 0x35

CONFIG_MODE = 0x00
NDOF_MODE = 0x0C
POWER_NORMAL = 0x00


def write(bus, register, value):
    bus.write_byte_data(BNO055_ADDRESS, register, value)
    time.sleep(0.02)


def read_i16(bus, register):
    data = bus.read_i2c_block_data(BNO055_ADDRESS, register, 2)
    value = data[0] | (data[1] << 8)
    return value - 65536 if value & 0x8000 else value


def read_vec3(bus, register, scale):
    return tuple(read_i16(bus, register + offset) / scale for offset in (0, 2, 4))


def setup_bno055(bus):
    if bus.read_byte_data(BNO055_ADDRESS, CHIP_ID) != 0xA0:
        time.sleep(0.7)
        if bus.read_byte_data(BNO055_ADDRESS, CHIP_ID) != 0xA0:
            raise RuntimeError("BNO055が見つかりません。配線、I2C有効化、アドレス0x28を確認してください。")

    write(bus, OPR_MODE, CONFIG_MODE)
    write(bus, PAGE_ID, 0)
    write(bus, PWR_MODE, POWER_NORMAL)
    write(bus, SYS_TRIGGER, 0x00)
    write(bus, OPR_MODE, NDOF_MODE)
    time.sleep(0.1)


def main():
    with SMBus(BUS_NUMBER) as bus:
        setup_bno055(bus)
        print("BNO055 test start. Stop with Ctrl+C.")

        while True:
            heading, roll, pitch = read_vec3(bus, EULER_H_LSB, 16.0)
            ax, ay, az = read_vec3(bus, ACC_DATA_X_LSB, 100.0)
            gx, gy, gz = read_vec3(bus, GYR_DATA_X_LSB, 16.0)
            calib = bus.read_byte_data(BNO055_ADDRESS, CALIB_STAT)

            print(
                f"euler[deg] H={heading:7.2f} R={roll:7.2f} P={pitch:7.2f} | "
                f"acc[m/s^2] X={ax:7.2f} Y={ay:7.2f} Z={az:7.2f} | "
                f"gyro[dps] X={gx:7.2f} Y={gy:7.2f} Z={gz:7.2f} | "
                f"calib=0x{calib:02X}"
            )
            time.sleep(0.1)


if __name__ == "__main__":
    main()
