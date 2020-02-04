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

LED_PWM_ADDR_BASE = 0x10 # PWM register for first LED
LED_PWM_DEFVAL = 0
    
class PCA9745B():
    '''Control PCA9745B via SPI'''

    def __init__(self, mtrx, spi, cs, verify = True, power = True):
        '''Create SPI object, clear errors and reset PCA9745B'''

        self.verify = verify

        # we only address even registers (*2) as the 8th bit needs to be skipped, as it indicates read (=1) or write (=0)
        self.matrix = tuple([ tuple([ [ LED_PWM_ADDR_BASE+(i*unit+idx)*2, LED_PWM_DEFVAL] for idx in range(0, unit) ]) for i, unit in enumerate(mtrx) ])
        #print(self.matrix)

        cs.value(1) # CS is active-low, so make sure it's inactive

        self.spi = (spi, cs)

        self._spi_write(self._create_bitstr(0x02, 0x10)) # clear errors
        self._spi_write(self._create_bitstr(0x02, 0x00)) # reset

        if power:
            self.iref_all()

    def _create_bitstr(self, cmd, val = 0xff): # when reading registers, the value argument needs to be 0xFF
        return cmd.to_bytes(1, 'big') + val.to_bytes(1, 'big') 

    def _set_write(self, cmd):
        return cmd

    def _set_read(self, cmd):
        return cmd | 0x01 # set last bit to 1

    def _spi_write(self, bitstr):
        self.spi[1].value(0)
        self.spi[0].write(bitstr)
        self.spi[1].value(1)

    def _spi_read(self, bitstr): # cmd bit pattern
        self.spi[1].value(0)
        self.spi[0].write(bitstr)
        self.spi[1].value(1)
        self.spi[1].value(0)
        data = self.spi[0].read(2)[1] # first byte is dummy
        self.spi[1].value(1)
        return data


    def set_pwm(self, led, val):
        if val < 0 or val > 255:
            raise ValueError("Value exceeds limits")

        if led[0] == -1: # address all LEDs
            for idx, grp in enumerate(self.matrix):
                self._spi_write(self._create_bitstr(self._set_write(self.matrix[idx][led[1]][0]), val))
                if self.verify:
                    ret = self._spi_read(self._create_bitstr(self._set_read(self.matrix[idx][led[1]][0])))
                    if ret != val:
                        raise ValueError("Register {} set to {} but expected {}".format(self.matrix[led[0]][led[1]][0], ret, val))
                self.matrix[idx][led[1]][1] = val
        else:
            self._spi_write(self._create_bitstr(self._set_write(self.matrix[led[0]][led[1]][0]), val))
            if self.verify:
                ret = self._spi_read(self._create_bitstr(self._set_read(self.matrix[led[0]][led[1]][0])))
                if ret != val:
                    raise ValueError("Register {} set to {} but expected {}".format(self.matrix[led[0]][led[1]][0], ret, val))
            self.matrix[led[0]][led[1]][1] = val

    def iref_all(self):
        self._spi_write(self._create_bitstr(0x82, 0x80)) # IREFALL



if __name__ == "__main__":
    from machine import Pin, SPI

    matrix = (4,4,4,4)      # e.g.  4 RGBW LEDs               (=16 ch)
    #matrix = (10*(1,))     # e.g. 10 individual LEDs         (=10 ch)
    #matrix = (3,3,3,3,3)   # e.g.  5 RGB LEDs                (=15 ch)
    #matrix = (3,3,3,3,3,1) # e.g.  5 RGB + 1 individual LEDs (=16 ch)
    W,B,G,R = 0,1,2,3
    #R,G,B = 0,1,2

    spi = SPI(1, mosi=Pin(32), miso=Pin(33), sck=Pin(25), bits=16, firstbit=SPI.MSB, polarity=0, phase=0)
    spi_cs   = Pin(4, Pin.OUT)

    led = PCA9745B(matrix, spi, spi_cs)

    led.set_pwm((-1, W), 5)    # set W channel of all LED units to PWM=5
    #led.set_pwm((-1, R), 23)    # set R channel of all LED units to PWM=23
    #led.set_pwm((-1, G), 42)    # set G channel of all LED units to PWM=42
    #led.set_pwm((-1, B), 255)    # set B channel of all LED units to PWM=255
    #led.set_pwm((2, R), 127)    # set R channel of LED3 to PWM=255 (max)
