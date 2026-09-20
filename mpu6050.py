"""
MPU6050 — 6-Axis IMU Driver for MicroPython (I2C)
Accelerometer + Gyroscope for fall detection.

I2C Address: 0x68 (AD0 tied to GND)
Datasheet: https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/
"""

from micropython import const
import struct

# Register map
_PWR_MGMT_1   = const(0x6B)
_ACCEL_XOUT_H = const(0x3B)
_GYRO_XOUT_H  = const(0x43)
_ACCEL_CONFIG  = const(0x1C)
_GYRO_CONFIG   = const(0x1B)
_WHO_AM_I      = const(0x75)

# Accelerometer sensitivity (LSB/g)
_ACCEL_SCALE = {0: 16384, 1: 8192, 2: 4096, 3: 2048}  # ±2g, ±4g, ±8g, ±16g

# Gyroscope sensitivity (LSB/°/s)
_GYRO_SCALE = {0: 131.0, 1: 65.5, 2: 32.8, 3: 16.4}   # ±250, ±500, ±1000, ±2000


class MPU6050:
    """Lightweight MPU6050 driver via raw I2C register access."""

    def __init__(self, i2c, addr=0x68, accel_range=2, gyro_range=0):
        """
        Args:
            i2c: MicroPython I2C or SoftI2C instance
            addr: I2C address (0x68 with AD0=GND, 0x69 with AD0=VCC)
            accel_range: 0=±2g, 1=±4g, 2=±8g, 3=±16g (default 2 = ±8g for fall detection)
            gyro_range: 0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s
        """
        self.i2c = i2c
        self.addr = addr
        self.accel_scale = _ACCEL_SCALE[accel_range]
        self.gyro_scale = _GYRO_SCALE[gyro_range]
        self._buf6 = bytearray(6)

        # Verify device identity (0x68=MPU6050, 0x70=MPU6500, 0x71/0x72=MPU9250, plus clones)
        try:
            who = self.i2c.readfrom_mem(self.addr, _WHO_AM_I, 1)[0]
        except OSError:
            raise RuntimeError("MPU6050 not responding at I2C address 0x{:02X}".format(self.addr))

        if who not in (0x68, 0x69, 0x70, 0x71, 0x72, 0x73, 0x75, 0x98):
            print("[WARN] MPU unexpected WHO_AM_I=0x{:02X}, attempting to proceed".format(who))

        # Wake up (clear SLEEP bit in PWR_MGMT_1)
        self.i2c.writeto_mem(self.addr, _PWR_MGMT_1, b'\x00')

        # Set accelerometer range
        self.i2c.writeto_mem(self.addr, _ACCEL_CONFIG, bytes([accel_range << 3]))

        # Set gyroscope range
        self.i2c.writeto_mem(self.addr, _GYRO_CONFIG, bytes([gyro_range << 3]))

    def _read_raw(self, reg):
        """Read 3 consecutive 16-bit big-endian signed values starting at reg."""
        self.i2c.readfrom_mem_into(self.addr, reg, self._buf6)
        return struct.unpack(">hhh", self._buf6)

    def accel(self):
        """Returns (ax, ay, az) in g units."""
        raw = self._read_raw(_ACCEL_XOUT_H)
        s = self.accel_scale
        return (raw[0] / s, raw[1] / s, raw[2] / s)

    def gyro(self):
        """Returns (gx, gy, gz) in °/s."""
        raw = self._read_raw(_GYRO_XOUT_H)
        s = self.gyro_scale
        return (raw[0] / s, raw[1] / s, raw[2] / s)

    def accel_magnitude(self):
        """Returns total acceleration magnitude in g (for fall detection)."""
        ax, ay, az = self.accel()
        return (ax * ax + ay * ay + az * az) ** 0.5
