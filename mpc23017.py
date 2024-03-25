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

# this is for Bank = 0
_IODIR = 0x0
_IOPOL = 0x2
_GPIINTEN = 0x4
_DEFVAL = 0X06
_INTCON = 0X08
_IOCON = 0X0A
_GPPU = 0x0C
_INTF = 0X0E
_INTCAP = 0X10
_GPIO = 0X12
_OLAT = 0X14


class Mpc23017Gpio:
    """
    this is a class the runs the Adafruit gpio hat for the Raspberry PI.  This card uses the
    'MCP23017 GPIO Expander Datasheet: <http://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf>`_  The default I2C
     i2c default address is 0x20.

     This will be run on the 'Adafruit MPC23017 Bonnet: <https://www.adafruit.com/product/4132>`_ which is attached to the i2c bus 1.

     This class does not support the bank register = 1

    """
    def __init__(self, i2c_address=0x20, bus_number=1):
        """
        Initialize the gpio device

        :param i2c_address:
        :param bus_number:
        :return:
        """
        self.bus_number = bus_number
        self.i2c_address = i2c_address

    @property
    def iocon(self, bank=0):
        """
        read the IOCON: I/O EXPANDER CONFIGURATION REGISTER

        :param bank: the bank of the gpio 0/1
        :return: The value of the io expander configuration Register
        """
        return self.i2cbus.read_byte_data(self.i2c_address, _IOCON+bank)

    @iocon.setter
    def iocon(self, iocon=0, bank=0):
        """
        set the io expander configuration register

        :param iocon: the value to set the IO configuration register
        :param bank: the bank of the gpio 0/1


        """
        self.i2cbus.write_byte_data(self.i2c_address, _IOCON+bank, iocon)

    @property
    def iodir(self, bank=0):
        """
        Return the io dir a register

        :param bank: the bank of the gpio 0/1
        :return: returns the io direction register a
        """
        return self.i2cbus.read_byte_data(self.i2c_address, _IODIR+bank)

    @iocon.setter
    def iodir(self, iodir=0, bank=0):
        """
        set the io direction register

        :param iodir: the io direction register
        :param bank: the bank of the gpio 0/1
        """
        self.i2cbus.write_byte_data(self.i2c_address, _IODIR+bank, iodir)

    @property
    def gpinten(self, bank=0):
        """
        Return the GPINTEN: INTERRUPT-ON-CHANGE PINS (

        :param bank: the bank of the gpio 0/1
        :return: returns the gpinten register
        """
        return self.i2cbus.read_byte_data(self.i2c_address, _GPIINTEN + bank)

    @gpinten.setter
    def gpinten(self, gpinten=0, bank=0):
        """
        set the GPINTEN: INTERRUPT-ON-CHANGE PINS (

        :param gpinten: the io direction register
        :param bank: the bank of the gpio 0/1
        """
        self.i2cbus.write_byte_data(self.i2c_address, _GPIINTEN + bank, gpinten)

    @property
    def defval(self, bank=0):
        """
        Return the defval: DEFAULT COMPARE REGISTER FOR INTERRUPT-ON-CHANGE

        :param bank: the bank of the gpio 0/1
        :return: returns the defval register
        """
        return self.i2cbus.read_byte_data(self.i2c_address, _DEFVAL + bank)

    @defval.setter
    def defval(self, defval=0, bank=0):
        """
        set the defval:  DEFAULT COMPARE REGISTER FOR INTERRUPT-ON-CHANGE

        :param defval: the defval register
        :param bank: the bank of the gpio 0/1
        """
        self.i2cbus.write_byte_data(self.i2c_address, _DEFVAL + bank, defval)

    @property
    def intcon(self, bank=0):
        """
        Return the intcon: INTERRUPT-ON-CHANGE CONTROL REGISTER

        :param bank: the bank of the gpio 0/1
        :return: returns the intcon register
        """
        return self.i2cbus.read_byte_data(self.i2c_address, _INTCON + bank)

    @intcon.setter
    def intcon(self, intcon=0, bank=0):
        """
        set the intcon:   INTERRUPT-ON-CHANGE CONTROL REGISTER

        :param intcon: the intcon register
        :param bank: the bank of the gpio 0/1
        """
        self.i2cbus.write_byte_data(self.i2c_address, _INTCON + bank, intcon)

    @property
    def gppu(self, bank=0):
        """
        Return the gppu: GPPU: GPIO PULL-UP RESISTOR REGISTER

        :param bank: the bank of the gpio 0/1
        :return: returns the gppua register
        """
        return self.i2cbus.read_byte_data(self.i2c_address, _GPPU + bank)

    @gppu.setter
    def gppu(self, gppu=0, bank=0):
        """
        set the gppu:  GPIO PULL-UP RESISTOR REGISTER

        :param gppu: the gppu register
        :param bank: the bank of the gpio 0/1
        """
        self.i2cbus.write_byte_data(self.i2c_address, _GPPU + bank, gppu)

    @property
    def gpio(self, bank=0):
        """
        Return the GPIO: GENERAL PURPOSE I/O PORT REGISTER (

        :param bank: the bank of the gpio 0/1
        :return: returns the gpio register
        """
        return self.i2cbus.read_byte_data(self.i2c_address, _GPIO + bank)

    @gpio.setter
    def gpio(self, gpio=0, bank=0):
        """
        set the gpio:  GPIO: GENERAL PURPOSE I/O PORT REGISTER (

        :param gpio: the gpio register
        :param bank: the bank of the gpio 0/1
        """
        self.i2cbus.write_byte_data(self.i2c_address, _GPIO + bank, gpio)

    @property
    def olat(self, bank=0):
        """
        Return the OUTPUT LATCH REGISTER

        :param bank: the bank of the olat 0/1
        :return: returns the olat register
        """
        return self.i2cbus.read_byte_data(self.i2c_address, _OLAT + bank)

    @olat.setter
    def olat(self, olat=0, bank=0):
        """
        set the olat:  OUTPUT LATCH REGISTER

        :param olat: the olat register
        :param bank: the bank of the olat 0/1
        """
        self.i2cbus.write_byte_data(self.i2c_address, _OLAT + bank, olat)

    @property
    def intf(self, bank=0):
        """
        Return the INTF: INTERRUPT FLAG REGISTER

        :param bank: the bank of the intf 0/1
        :return: returns the intf register
        """
        return self.i2cbus.read_byte_data(self.i2c_address, _INTF + bank)

    @intf.setter
    def intf(self, intf=0, bank=0):
        """
        set the INTF: INTERRUPT FLAG REGISTER

        :param intf: the intf register
        :param bank: the bank of the intf 0/1
        """
        self.i2cbus.write_byte_data(self.i2c_address, _INTF + bank, intf)

    @property
    def intcap(self, bank=0):
        """
        Return the INTERRUPT CAPTURED REGISTER

        :param bank: the bank of the intcap 0/1
        :return: returns the intcap register
        """
        return self.i2cbus.read_byte_data(self.i2c_address, _INTCAP + bank)

    @intcap.setter
    def intcap(self, intcap=0, bank=0):
        """
        set the INTERRUPT CAPTURED REGISTER

        :param intcap: the intcap register
        :param bank: the bank of the intcap 0/1
        """
        self.i2cbus.write_byte_data(self.i2c_address, _INTCAP + bank, intcap)