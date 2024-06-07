# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2022 ladyada for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_mmc56x3`
================================================================================

Python MMC5603 / MMC5613 magnetometer sensor library


* Author(s): ladyada

Implementation Notes
--------------------

**Hardware:**

* `Adafruit MMC5603 Magnetometer <https://www.adafruit.com/product/5579>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
* Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

import time
from micropython import const
from adafruit_bus_device import i2c_device
from adafruit_register.i2c_struct import ROUnaryStruct, UnaryStruct
from adafruit_register.i2c_bit import RWBit

try:
    from typing import Tuple
    from busio import I2C
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_MMC56x3.git"

_MMC5603_I2CADDR_DEFAULT: int = const(0x30)  # Default I2C address
_MMC5603_CHIP_ID = const(0x10)

_MMC5603_OUT_X_L = const(0x00)  # Register that starts the mag data out
_MMC5603_OUT_TEMP = const(0x09)  # Register that contains temp reading
_MMC5603_PRODUCT_ID = const(0x39)  # Register that contains the part ID
_MMC5603_STATUS_REG = const(0x18)  # Register address for device status
_MMC5603_ODR_REG = const(0x1A)  # Output data rate register
_MMC5603_CTRL_REG0 = const(0x1B)  # Register address for control 0
_MMC5603_CTRL_REG1 = const(0x1C)  # Register address for control 1
_MMC5603_CTRL_REG2 = const(0x1D)  # Register address for control 2


class MMC5603:
    """Driver for the MMC5603 3-axis magnetometer.

    **Quickstart: Importing and using the device**

    Here is an example of using the :py:class:`MMC5603` class.
    First you will need to import the libraries to use the sensor

    .. code-block:: python

        import board
        import adafruit_mmc56x3

    Once this is done you can define your `board.I2C` object and define your sensor object

    .. code-block:: python

        i2c = board.I2C()
        sensor = adafruit_mmc56x3.MMC5603(i2c)

    Now you have access to the :attr:`magnetic` attribute

    .. code-block:: python

        mag_x, mag_y, mag_z = sensor.magnetic

    :param ~busio.I2C i2c_bus: The I2C bus the MMC5603 is connected to.
    :param int address: The I2C device address. Defaults to :const:`0x30`
    """

    _chip_id = ROUnaryStruct(_MMC5603_PRODUCT_ID, "<B")
    _ctrl0_reg = UnaryStruct(_MMC5603_CTRL_REG0, "<B")
    _ctrl1_reg = UnaryStruct(_MMC5603_CTRL_REG1, "<B")
    _ctrl2_reg = UnaryStruct(_MMC5603_CTRL_REG2, "<B")
    _status_reg = ROUnaryStruct(_MMC5603_STATUS_REG, "<B")
    _odr_reg = UnaryStruct(_MMC5603_ODR_REG, "<B")
    _raw_temp_data = ROUnaryStruct(_MMC5603_OUT_TEMP, "<B")

    _reset = RWBit(_MMC5603_CTRL_REG1, 7)
    _meas_m_done = RWBit(_MMC5603_STATUS_REG, 6)
    _meas_t_done = RWBit(_MMC5603_STATUS_REG, 7)

    def __init__(self, i2c_bus: I2C, address: int = _MMC5603_I2CADDR_DEFAULT) -> None:
        # pylint: disable=no-member
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)
        if self._chip_id != _MMC5603_CHIP_ID:
            raise RuntimeError("Failed to find MMC5603 - check your wiring!")

        self.reset()
        self._buffer = bytearray(9)
        # self.performance_mode = PerformanceMode.MODE_ULTRA

    def reset(self) -> None:
        """Reset the sensor to the default state set by the library"""
        self._ctrl1_reg = 0x80  # write only, set topmost bit
        time.sleep(0.020)
        self._odr_cache = 0
        self._ctrl2_cache = 0
        self.set_reset()

    @property
    def temperature(self) -> float:
        """The processed temperature sensor value, returned in floating point C"""
        if self.continuous_mode:
            raise RuntimeError("Can only read temperature when not in continuous mode")
        self._ctrl0_reg = 0x02  # TM_T
        while not self._meas_t_done:
            time.sleep(0.005)
        temp = self._raw_temp_data
        temp *= 0.8  # 0.8*C / LSB
        temp -= 75  # 0 value is -75
        return temp

    @property
    def magnetic(self) -> Tuple[float, float, float]:
        """The processed magnetometer sensor values.
        A 3-tuple of X, Y, Z axis values in microteslas that are signed floats.
        """
        if not self.continuous_mode:
            self._ctrl0_reg = 0x01  # TM_M

            while not self._meas_m_done:
                time.sleep(0.005)
        self._buffer[0] = _MMC5603_OUT_X_L
        with self.i2c_device as i2c:
            i2c.write_then_readinto(self._buffer, self._buffer, out_end=1)
        x = self._buffer[0] << 12 | self._buffer[1] << 4 | self._buffer[6] >> 4
        y = self._buffer[2] << 12 | self._buffer[3] << 4 | self._buffer[7] >> 4
        z = self._buffer[4] << 12 | self._buffer[5] << 4 | self._buffer[8] >> 4
        # fix center offsets
        x -= 1 << 19
        y -= 1 << 19
        z -= 1 << 19
        # scale to uT by LSB in datasheet
        x *= 0.00625
        y *= 0.00625
        z *= 0.00625
        return (x, y, z)

    @property
    def data_rate(self) -> int:
        """Output data rate, 0 for on-request data.
        1-255 or 1000 for freq of continuous-mode readings"""
        return self._odr_cache

    @data_rate.setter
    def data_rate(self, value: int) -> None:
        if not ((value == 1000) or (0 <= value <= 255)):
            raise ValueError("Data rate must be 0-255 or 1000 Hz")
        self._odr_cache = value
        if value == 1000:
            self._odr_reg = 255
            self._ctrl2_cache |= 0x80  # turn on hpower bit
            self._ctrl2_reg = self._ctrl2_cache
        else:
            self._odr_reg = value
            self._ctrl2_cache &= ~0x80  # turn off hpower bit
            self._ctrl2_reg = self._ctrl2_cache

    @property
    def continuous_mode(self) -> bool:
        """Whether or not to put the chip in continous mode - be sure
        to set the data_rate as well!
        """
        return self._ctrl2_cache & 0x10

    @continuous_mode.setter
    def continuous_mode(self, value: bool) -> None:
        if value:
            self._ctrl0_reg = 0x80  # turn on cmm_freq_en bit
            self._ctrl2_cache |= 0x10  # turn on cmm_en bit
        else:
            self._ctrl2_cache &= ~0x10  # turn off cmm_en bit
        self._ctrl2_reg = self._ctrl2_cache

    def set_reset(self) -> None:
        """Pulse large currents through the sense coils to clear any offset"""
        self._ctrl0_reg = 0x08  # turn on set bit
        time.sleep(0.001)  # 1 ms
        self._ctrl0_reg = 0x10  # turn on reset bit
        time.sleep(0.001)  # 1 ms