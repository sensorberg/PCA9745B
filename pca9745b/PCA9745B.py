# MIT License
# 
# Copyright (c) 2020 Mirko Vogt, Sensorberg GmbH (mirko.vogt@sensorberg.com)
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# This is a driver class written in (micro)python, to control the PCA9745B or
# similars LED driver connected via SPI. It currently provides support for PWM
# controlling LED matrixes of various configurations.

from micropython import const

_REG_MODE2 = const(0x01)
_REG_LEDOUT_BASE = const(0x02)
_REG_GRPPWM = const(0x06)
_REG_GRPFREQ = const(0x07)
_REG_PWM_BASE = const(0x08)
_REG_IREFALL = const(0x41)

_CMD_WRITE = 0b1
_CMD_READ = 0b0

_CFG_LEDOUT_IND = const(0b10) # not used
_CFG_LEDOUT_GROUP = const(0b11) # required for blinking, used by default
_CFG_MODE2_BLINK = const(0b00100000)
_CFG_MODE2_NORM = const(0b00000000)
_CFG_IREFALL_OFF = const(0x00)
_CFG_IREFALL_ON = const(0xff)

ALL_LEDS = const(-1)

class PCA9745B():
    '''Control PCA9745B via SPI'''

    def __init__(self, mtrx, spi, cs, verify = False, power = True):
        '''Create SPI object, clear errors and reset PCA9745B'''

        #TODO: implement verification
        if verify:
            #self.verify = verify
            raise Exception("Not implemented")

        self.matrix = tuple([ tuple([ (i*unit+idx) for idx in range(0, unit) ]) for i, unit in enumerate(mtrx) ])
        #print(self.matrix)

        cs.value(1) # CS is active-low, so make sure it's inactive

        self.spi = (spi, cs)

        self._spi_write(self._create_bitstr(_REG_MODE2, 0x10)) # clear errors
        self._spi_write(self._create_bitstr(_REG_MODE2, 0x00)) # reset

        self.cmdvals = self._cmdvals_cfg_all_leds(_CFG_LEDOUT_GROUP) # initial starting point

        #self.set_clear()
        self.set_power(power)
        self.commit()


    def _create_bitstr(self, cmd, val = 0xff): # when reading registers, the value argument needs to be 0xFF
        return cmd.to_bytes(1, 'big') + val.to_bytes(1, 'big') 

    def _set_cmd_write(self, cmd):
        return cmd << 1

    def _set_cmd_read(self, cmd):
        return cmd << 1 | 0x01 # set last bit to 1

    def _spi_write(self, bstr):
        assert(len(bstr) == 2)
        self.spi[1].value(0)
        self.spi[0].write(bstr)
        self.spi[1].value(1)

    def _spi_read(self, bitstr): # cmd bit pattern
        self.spi[1].value(0)
        self.spi[0].write(bitstr)
        self.spi[1].value(1)
        self.spi[1].value(0)
        data = self.spi[0].read(2)[1] # first byte is dummy
        self.spi[1].value(1)
        return data


    def _cmdvals_cfg_all_leds(self, val):
        val |= val << 2 | val << 4 | val << 6 # apply 2bit pattern to all 8 bits
        return [
            (
                self._set_cmd_write(_REG_LEDOUT_BASE), # LEDOUT0
                val, # set all 4 LEDs controlled by this register
            ),
            (
                self._set_cmd_write(_REG_LEDOUT_BASE+1), # LEDOUT1
                val, # set all 4 LEDs controlled by this register
            ),
            (
                self._set_cmd_write(_REG_LEDOUT_BASE+2), # LEDOUT2
                val, # set all 4 LEDs controlled by this register
            ),
            (
                self._set_cmd_write(_REG_LEDOUT_BASE+3), # LEDOUT3
                val, # set all 4 LEDs controlled by this register
            ),
        ]

    def _cmdvals_cfg_leds_pwm(self, led, val):
        return [
            #(self._set_cmd_write(_REG_MODE2),   _CFG_MODE2_NORM), #TODO: unnecessary to set for every pwm call
            (self._set_cmd_write(_REG_PWM_BASE + led), val)
        ]

    def _cmdvals_cfg_all_leds_blink(self, duty, freq):
        return [
            (self._set_cmd_write(_REG_MODE2),   _CFG_MODE2_BLINK), # TODO: unnecessary to set for every blink call
            (self._set_cmd_write(_REG_GRPFREQ), freq),
            (self._set_cmd_write(_REG_GRPPWM),  duty),
        ]


    def commit(self, uniq=True):
        cmdvals = uniq and set(self.cmdvals) or self.cmdvals # remove dups
        for cmdval in cmdvals:
            bstr = bytes(cmdval) # 2 elem tuple -> bytestring of 2 bytes
            self._spi_write(bstr)
        self.cmdvals = []
        return len(cmdvals)


    def set_clear(self):
        #self.cmdvals += self._cmdvals_cfg_all_leds(_CFG_LEDOUT_IND)
        self.cmdvals += [(self._set_cmd_write(_REG_MODE2), _CFG_MODE2_NORM)]
        for grp in self.matrix:
            for led in grp:
                self.cmdvals += self._cmdvals_cfg_leds_pwm(led, 0)


    def set_pwm(self, led, val):
        #if led is None or val is None: # if either arg is None, only set LED lines to individual PWM mode
        #    self.cmdvals += self._cmdvals_cfg_all_leds(_CFG_LEDOUT_IND)
        #    return
        if val < 0 or val > 255:
            raise ValueError("Value exceeds limits")
        if isinstance(led, tuple) and len(led) != 2:
            raise ValueError("led a tuple of other than 2 elements")

        if isinstance(led, tuple) and led[0] < 0: # address all LEDs of group defined in led[1]
            for phy, log in enumerate(self.matrix): # physical is the electronics part, logical e.g. a color of the physical
                self.cmdvals += self._cmdvals_cfg_leds_pwm(phy*len(log) + led[1], val)
        elif isinstance(led, tuple):
            # address specific LED in group # FIXME: only works if all groups are structured the same
            self.cmdvals += self._cmdvals_cfg_leds_pwm(led[0] * len(self.matrix[0]) + led[1], val)
        elif isinstance(led, int) and led == -1: # all LEDs
            for grp in self.matrix:
                for led in grp:
                    self.cmdvals += self._cmdvals_cfg_leds_pwm(led, val)
        elif isinstance(led, int): # address specific single LED
            self.cmdvals += self._cmdvals_cfg_leds_pwm(led, val)
        else:
            raise ValueError()


    def set_blink(self, freq=None, duty=None):
        #if freq is None or duty is None: # if either arg is None, only set LED lines to group blinking mode
        #    self.cmdvals += self._cmdvals_cfg_all_leds(_CFG_LEDOUT_GROUP)
        #    return
        if not freq or not duty: # if either arg evals to False, deactivate blinking
            self.cmdvals += [(self._set_cmd_write(_REG_MODE2), _CFG_MODE2_NORM)]
            return
        if freq < 0 or freq > 255 or duty < 0 or duty > 255:
            raise ValueError("Value exceeds limits")
        #self.cmdvals += self._cmdvals_cfg_all_leds(_CFG_LEDOUT_GROUP)
        self.cmdvals += self._cmdvals_cfg_all_leds_blink(duty, freq)


    def set_power(self, val):
        self.cmdvals += [(self._set_cmd_write(_REG_IREFALL), val and _CFG_IREFALL_ON or _CFG_IREFALL_OFF)]



def main():
    import time
    from machine import Pin, SPI

    #matrix = (4,4,4,4)      # e.g.  4 RGBW LEDs               (=16 ch)
    matrix = (4,4,4,4)      # e.g.  4 RGBW LEDs               (=16 ch)
    #matrix = (10*(1,))     # e.g. 10 individual LEDs         (=10 ch)
    #matrix = (3,3,3,3,3)   # e.g.  5 RGB LEDs                (=15 ch)
    #matrix = (3,3,3,3,3,1) # e.g.  5 RGB + 1 individual LEDs (=16 ch)
    W,B,G,R = 0,1,2,3
    #R,G,B = 0,1,2

    spi = SPI(1, mosi=Pin(32), miso=Pin(33), sck=Pin(25), bits=16, firstbit=SPI.MSB, polarity=0, phase=0)
    spi_cs   = Pin(4, Pin.OUT)

    led = PCA9745B(matrix, spi, spi_cs)

    led.set_clear()
    led.commit()
    time.sleep(1)
    led.set_pwm((-1, G), 10)
    led.commit()
    time.sleep(1)
    led.set_pwm((-1, B), 10)
    led.commit()
    time.sleep(1)
    led.set_blink(8, 64)
    led.commit()
    time.sleep(2)
    led.set_pwm((-1, R), 200)
    led.commit()
    time.sleep(1)
    led.set_clear()
    led.commit()
    time.sleep(1)
    led.set_pwm((-1, R), 200)
    led.commit()
    time.sleep(1)
    led.set_clear()
    led.commit()
    time.sleep(1)
    led.set_pwm((0, W), 200)
    led.commit()
    time.sleep(1)
    led.set_pwm((0, W), 20)
    led.set_pwm((1, W), 200)
    led.commit()
    time.sleep(1)
    led.set_pwm((1, W), 20)
    led.set_pwm((2, W), 200)
    led.commit()
    time.sleep(1)
    led.set_pwm((2, W), 20)
    led.set_pwm((3, W), 200)
    led.commit()


    #led.set_pwm((-1, W), 5)    # set W channel of all LED units to PWM=5
    #led.set_pwm((-1, R), 23)    # set R channel of all LED units to PWM=23
    #led.set_pwm((-1, G), 42)    # set G channel of all LED units to PWM=42
    #led.set_pwm((-1, B), 255)    # set B channel of all LED units to PWM=255
    #led.set_pwm((2, R), 127)    # set R channel of LED3 to PWM=255 (max)


if __name__ == "__main__":
    main()
