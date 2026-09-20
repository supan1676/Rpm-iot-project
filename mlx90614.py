"""
MLX90614 — Infrared Contactless Temperature Sensor Driver for MicroPython (I2C)
Measures skin surface temperature via IR thermopile.

I2C Address: 0x5A (default)
Datasheet: https://www.melexis.com/en/product/MLX90614/

IMPORTANT: Use the 3.3V version (MLX90614-BCC), NOT the 5V version (BAA).
"""

import time
from micropython import const

# Register addresses (RAM)
_REG_AMBIENT_TEMP = const(0x06)  # Ta — ambient temperature
_REG_OBJECT_TEMP  = const(0x07)  # Tobj1 — object (skin) temperature

# Core body temperature offset (wrist skin is ~3°C below core)
CORE_TEMP_OFFSET = 3.0


class MLX90614:
    """MLX90614 IR temperature sensor driver."""

    def __init__(self, i2c, addr=0x5A):
        self.i2c = i2c
        self.addr = addr

        # Verify device is responding with valid data
        try:
            raw = self._read_temp_raw(_REG_AMBIENT_TEMP)
            if raw == 0 or raw == 0x7FFF:
                raise RuntimeError("MLX90614 bus stuck (raw 0x{:04X})".format(raw))
        except Exception:
            raise RuntimeError("MLX90614 not responding at address 0x{:02X}".format(addr))

    def _read_temp_raw(self, reg, retries=3):
        """Read a 16-bit temperature value from the given register with retry logic."""
        last_err = None
        for attempt in range(retries):
            try:
                # Attempt to read 3 bytes (Data Low, Data High, PEC)
                try:
                    data = self.i2c.readfrom_mem(self.addr, reg, 3)
                except OSError:
                    data = self.i2c.readfrom_mem(self.addr, reg, 2)

                # Check error flag (bit 15 of high byte)
                if (data[1] & 0x80) != 0:
                    raise RuntimeError("MLX90614 error flag set")

                return data[0] | ((data[1] & 0x7F) << 8)
            except Exception as e:
                last_err = e
                time.sleep_ms(5)
        raise last_err if last_err else RuntimeError("MLX90614 read failed")

    def _raw_to_celsius(self, raw):
        """Convert raw 16-bit value to degrees Celsius."""
        return (raw * 0.02) - 273.15

    def ambient_temp(self):
        """Read ambient (sensor die) temperature in °C."""
        raw = self._read_temp_raw(_REG_AMBIENT_TEMP)
        return round(self._raw_to_celsius(raw), 1)

    def object_temp(self):
        """Read object (surface/skin) temperature in °C."""
        raw = self._read_temp_raw(_REG_OBJECT_TEMP)
        return round(self._raw_to_celsius(raw), 1)

    def is_skin_detected(self):
        """True if target temperature is consistent with human skin (> 28°C)."""
        try:
            return self.object_temp() >= 28.0
        except Exception:
            return False

    def body_temp(self):
        """
        Estimated core body temperature in °C.
        Applies +3.0°C offset when skin is detected (>28°C).
        """
        t = self.object_temp()
        if t < -20 or t > 70:
            raise ValueError("Unrealistic temperature: {:.1f}C".format(t))
        if t >= 28.0:
            return round(t + CORE_TEMP_OFFSET, 1)
        return round(t, 1)
