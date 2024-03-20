#!/usr/bin/env python
# Copyright 2024 Ralph Carl Blach III
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute,
# sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
# OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
# ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
# THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""
This is a program that runs the Freescale mm8451 3 axis accelerometer which is attached to an I2C port on the rasberry pi.

This project uses the mm8451 from `Adafruit MM8451 web page <https://www.adafruit.com/search?q=mm8451>`_

This chip uses repeated stops, and must use the gpio_i2c device driver.
Add the line below to the /boot/config/txt
dtoverlay=i2c-gpio,i2c_gpio_sda=23,i2c_gpio_scl=24,i2c_gpio_delay_us=2,bus=5


This project also uses pigpio
see `Pigio web page <https://abyz.me.uk/rpi/pigpio/index.html>`_ for the documentation
Please wire I1 to gpio 6 on the raspberry pi.

How to run the program
command line and options

Sample command line below::
    accelerometer.py -h [--gpio GPIO] [--count COUNT] [--odr ODR] [--dev_number DEV_NUMBER] [--i2c_address I2C_ADDRESS]

-h Help

--gpio number           The gpio where the interrupt pin will be connected.  The default is gpio 6.

--count number          The number of times to read the acceleration registers, 0 means continuous, with no stop.  Use control c to exit.

--odr number            From 0 to 7 with 0 being 800 hz and 7 being 1.56 hz.

--dev_number number     The i2c device number, This will be the number that is appended to the base i2c name which /dev/i2c-
                        if dev_number = 5 then the i2c device used will be /dev/i2c-5

--i2c_address number    This is the i2c address of the accelerometer.  This is 0x1d for this breakout board.

Below is the output data rates

Output Data Rate Values

    +----+-----+----+-----+---------+--------+
    |DR2 |DR1  |DR0 | VAL |ODR      |Period  |
    +====+=====+====+=====+=========+========+
    |0   |0    |0   | 0   |800 Hz   |1.25 ms |
    +----+-----+----+-----+---------+--------+
    |0   |0    |1   | 1   |400 Hz   |2.5 ms  |
    +----+-----+----+-----+---------+--------+
    |0   |1    |0   | 2   |200 Hz   |5 ms    |
    +----+-----+----+-----+---------+--------+
    |0   |1    |1   | 3   |100 Hz   |10 ms   |
    +----+-----+----+-----+---------+--------+
    |1   |0    |0   | 4   |50 Hz    |20 ms   |
    +----+-----+----+-----+---------+--------+
    |1   |0    |1   | 5   |12.5 Hz  |80 ms   |
    +----+-----+----+-----+---------+--------+
    |1   |1    |0   | 6   |6.25 Hz  |160 ms  |
    +----+-----+----+-----+---------+--------+
    |1   |1    |1   | 7   |1.56 Hz  |640 ms  |
    +----+-----+----+-----+---------+--------+






"""
import argparse
import math
import signal
import struct
import sys
from smbus import SMBus  # pylint: disable=E0611
import pigpio


_ACCELEROMETER_I2C_ADDRESS = 0x1d
_STATUS = 0X0
_OUT_X_MSB = 0X01
_OUT_X_LSB = 0X02
_OUT_Y_MSB = 0X03
_OUT_Y_LSB = 0X04
_OUT_Z_MSB = 0X05
_OUT_Z_LSB = 0X06

_F_SETUP = 0x09
_TRIG_CFG = 0X0A
_SYS_MOD = 0X0B
_INT_SOURCE = 0X0C
_WHO_AM_I = 0X0D
_XYZ_DATA_CFG = 0X0E
_HPF_OUT = 0b0001_0000

_HP_FILTER_CUTOFF = 0X0F
_PL_STATUS = 0X10
_PL_CFG = 0X11
_PL_COUNT = 0X12
_PL_BF_ZCOMP = 0X13
_P_L_THS_REG = 0X14
_FF_MT_CFG = 0X15
_FF_MT_SRC = 0X15
_FF_MT_THS = 0X16
_FF_MT_COUNT = 0X16
_TRANSIENT_CFG = 0X1D
_TRANSIENT_SRC = 0X1E
_TRANSIENT_THS = 0X1F
_TRANSIENT_COUNT = 0X20

_PULSE_CFG = 0X21
_PULSE_SRC = 0X22
_PULSE_THSX = 0X23
_PULSE_THSY = 0X24
_PULSE_THSZ = 0X25
_PULSE_TMLT = 0X26
_PULSE_LTCY = 0X27
_PULSE_WIND = 0X28

_ASLP_COUNT = 0X29

_CTRL_REG1 = 0X2A
_LOW_NOISE = 0b0000_0100
_FREAD = 0b0000_0010
_ACTIVE = 0b0000_0001

_CTRL_REG2 = 0X2B
_ST = 0b1000_0000
_RST = 0b0100_0000
_AUTO_SLEEP = 0b000_0100
_CTRL_REG3 = 0X2C
_CTRL_REG4 = 0X2D
_CTRL_REG5 = 0X2E

_OFF_X = 0X2F
_OFF_Y = 0X30
_OFF_Z = 0X31

OUTPUT_DATA_RATE_800 = 0x0
OUTPUT_DATA_RATE_400 = 0x1
OUTPUT_DATA_RATE_200 = 0x2
OUTPUT_DATA_RATE_100 = 0x3
OUTPUT_DATA_RATE_50 = 0x4
OUTPUT_DATA_RATE_12_5 = 0x5
OUTPUT_DATA_RATE_6_25 = 0x6
OUTPUT_DATA_RATE_1_56 = 0x7


class Accelerometer:
    """
    This runs the mm8451 digital 3 axit accelerometer, it configures the accelerometer for +/- 2 gs, in
    high resolution mode(154 bits for the raspberry pi
    this seems only to run on the gpio i2c because this is the only I2c that supports clock stretching.

    add the line below to the /boot/config/txt
    dtoverlay=i2c-gpio,i2c_gpio_sda=23,i2c_gpio_scl=24,i2c_gpio_delay_us=2,bus=5

    this project also uses pigpio
    see 'Pigio: https://abyz.me.uk/rpi/pigpio/index.html' for the documentation
    Please wire I1 to gpio 6 on the raspberry pi.

    """

    def __init__(self, i2c_address=0x1d, bus_number=5, output_data_rate=OUTPUT_DATA_RATE_800) -> None:
        """
        initialize the device

        :param i2c_address: the i2c address of the device
        :param bus_number: number the bus number of the i2c

        """
        self.bus_number = bus_number
        self.i2c_address = i2c_address

        try:
            self.i2cbus = SMBus(bus_number)  # Create a new I2C bus
        except FileNotFoundError as error:
            raise error

        who_am_i = self.i2cbus.read_byte_data(i2c_address, _WHO_AM_I)
        print(f'The  id = 0x{who_am_i:x}. If the id = 0x1a the this is probably an mm8451.\nIf the id is not 0x1a, '
              f'this this is not an mm8451 3 axis accelerometer')
        if who_am_i != 0x1a:
            raise ValueError(f"The i2c who am i was not 0x1a, it was 0x{who_am_i:x}")
        # write the high pass filter bit to a 1 and set to 2 g

        self.i2cbus.write_byte_data(self.i2c_address, _XYZ_DATA_CFG, 0x00)  # do not set the high pass filter.
        # This caused the output to decrease every time the value is read.
        #  The reason for this for a static which is not moving, that is a
        #  very low frequency signal, and it will get filtered out because only high frequency events can be passed.
        self.i2cbus.write_byte_data(self.i2c_address, _CTRL_REG2, 0x02)  # high resolution
        self.i2cbus.write_byte_data(self.i2c_address, _CTRL_REG4, 0x01)  # data ready interrupt enabled
        self.i2cbus.write_byte_data(self.i2c_address, _CTRL_REG5, 0x01)  # interrupt routed to pin 1
        self.i2cbus.write_byte_data(self.i2c_address, _OFF_Z, 0xf0)  # offset for my project
        self.i2cbus.write_byte_data(self.i2c_address, _OFF_X, 0x16)  # offsets for my project
        self.i2cbus.write_byte_data(self.i2c_address, _CTRL_REG1, (output_data_rate << 3) | 0x04 | 0x01)  # enable at min rate

    def read_accelerations(self) -> list[float, float, float]:
        """
        Read the accelerations from the MM8451 and return a list of floats in g's
            #. List element 0 will be the x
            #. List element 1 will be the y
            #. List element 2 will be the z

        :return: a list of ints accelerations, x, y and z in g's
        """
        accelerations = self.i2cbus.read_i2c_block_data(self.i2c_address, _OUT_X_MSB, 6)  # this returns a list of 6 bytes
        packed_struct = bytes(accelerations)    # make these a byte array
        # the acceleration list is in big endian order so the > symbol, so upack big endian
        x_accel, y_accel, z_accel = struct.unpack('>hhh', packed_struct)
        # print(f'0x{x_accel:x}, 0x{y_accel:x}, 0x{z_accel:x}')
        accel_list = [x_accel/4 * .00025, y_accel/4 * .00025, z_accel/4 * .00025]

        return accel_list

    @property
    def mm8451_status(self) -> int:
        """
        Read the status byte add address 0

        :return: the int status byte
        :rtype: int
        """
        status = self.i2cbus.read_byte_data(self.i2c_address, _STATUS)
        return status

    @staticmethod
    def bytes_to_short(msb: int, lsb: int) -> int:
        """
        take a byte amd makes a short with sign extend and divide by 4
        this is because the lower to bits of each acceleration are not used.
        the read smbus returns a list of ints which are between 0 and 255 and these are not signed
        So,

        :param msb: the most significant byte of the short
        :param lsb: the least significant byte of the short
        :return: int which is signed.
        :rtype: int
        """
        value = (msb << 8) | lsb
        sign_extend = (value & 0xffff) - 0x8000 if value & 0x8000 else value
        return sign_extend // 4

    def close_accelerometer(self) -> None:
        """
        close the by stopping the accelerometer and resetting it

        :return: None
        """
        self.i2cbus.write_byte_data(self.i2c_address, _CTRL_REG1, 0x00)  # surn off the enable
        self.i2cbus.write_byte_data(self.i2c_address, _CTRL_REG2, 0b0100_0000)  # reset bit is bit 6 0100_0000
        reset_bit = 0b0100_0000
        # loop till the reset is done
        while reset_bit:
            try:
                reset_bit = self.i2cbus.read_byte_data(self.i2c_address, _CTRL_REG2) & 0b0100_0000
            except OSError:
                reset_bit = 0b0100_0000

    @property
    def read_g_range(self):
        """
        read the range register of the accelerometer
        :return: the g range

        """
        mm8451_g_range = self.i2cbus.read_byte_data(self.i2c_address, _XYZ_DATA_CFG) & 0x3
        return mm8451_g_range


def accelerometer() -> None:
    """
    start up the accelerometer
    :return:
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--gpio', type=int, default=6, help='The GPIO pin for the input:  %(default)s)')
    parser.add_argument('--count', type=int, default=0, help='The number of accelerations readings to take. 0 is continuous:  %(default)s)')
    parser.add_argument('--odr', type=int, default=0, help='The output data rate:  %(default)s)')
    parser.add_argument('--dev_number', type=int, default=1, help='the i2c device number:  %(default)s)')
    parser.add_argument('--i2c_address', type=int, default=0x1d, help='The i2c bus address default = %(default)s)')

    args = parser.parse_args()
    device_number = args.dev_number
    i2c_address = args.i2c_address
    final_count = args.count
    odr = args.odr
    gpio_pin = args.gpio
    if not 0 <= odr <= 7:
        raise ValueError(f"The output data rate must be between 0 7 inclusive. You entered {odr}")

    gpio = pigpio.pi()
    gpio.set_mode(gpio_pin, pigpio.INPUT)

    accel = Accelerometer(i2c_address, device_number, odr)

    def signal_handler(sig, frame):    # pylint: disable=W0613
        print('You pressed control C close the accelerometer and sys.exit(0)')
        accel.close_accelerometer()
        sys.exit(0)

    # set up signal handling
    signal.signal(signal.SIGINT, signal_handler)
    counter = 0
    while True:
        gpio_level = gpio.read(gpio_pin)
        # status = accel.mm8451_status
        while gpio_level:    # loop till all status becomes 1
            gpio_level = gpio.read(gpio_pin)

        gpio_level = gpio.read(gpio_pin)
        # print(f'gpio level ={gpio_level}')
        x_accel, y_accel, z_accel = accel.read_accelerations()
        angle_sine = x_accel / (x_accel ** 2 + z_accel**2)
        angle_degrees = math.asin(angle_sine) * 180 / math.pi
        print(f"x_accel ={x_accel:.4f} g's, y_accel = {y_accel:.4f} g's, z_accel = {z_accel:.4f} g's angle = {angle_degrees:.2f} degrees")

        if final_count == 0:
            pass
        elif final_count == counter:
            break
        counter += 1

    accel.close_accelerometer()


if __name__ == '__main__':
    accelerometer()


# See PyCharm help at https://www.jetbrains.com/help/pycharm/
