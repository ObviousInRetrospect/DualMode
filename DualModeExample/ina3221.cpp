/*MIT License

Copyright (c) 2022 ObviousInRetrospect

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#include "ina3221.h"
#include <wire.h>
// read an ina3221 register into val
void ina_rr(ina3221_reg_t reg, uint16_t *val) {
  Wire.beginTransmission(INA_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)INA_ADDR, (uint8_t)2);
  *val = (uint16_t)Wire.read();
  *val <<= 8;
  *val |= Wire.read();
}

// write an ina3221 register from val
void ina_wr(ina3221_reg_t reg, uint16_t *val) {
  Wire.beginTransmission(INA_ADDR);
  Wire.write(reg);
  Wire.write((*val) >> 8);
  Wire.write((*val) & 0xff);
  Wire.endTransmission();
}
