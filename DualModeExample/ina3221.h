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

#ifndef __INA3221_H__
#define __INA3221_H__
#include <stdint.h>

// constants stolen from Beastdevices_INA3221.h
// that and every other INA3221 library does a lot
// of unnecessary expensive floating point math
typedef enum {
  INA3221_ADDR40_GND = 0b1000000, // A0 pin -> GND
  INA3221_ADDR41_VCC = 0b1000001, // A0 pin -> VCC
  INA3221_ADDR42_SDA = 0b1000010, // A0 pin -> SDA
  INA3221_ADDR43_SCL = 0b1000011  // A0 pin -> SCL
} ina3221_addr_t;

// Channels
typedef enum {
  INA3221_CH1 = 0,
  INA3221_CH2,
  INA3221_CH3,
  INA3221_CH_NUM
} ina3221_ch_t;

// Registers
typedef enum {
  INA3221_REG_CONF = 0,
  INA3221_REG_CH1_SHUNTV,
  INA3221_REG_CH1_BUSV,
  INA3221_REG_CH2_SHUNTV,
  INA3221_REG_CH2_BUSV,
  INA3221_REG_CH3_SHUNTV,
  INA3221_REG_CH3_BUSV,
  INA3221_REG_CH1_CRIT_ALERT_LIM,
  INA3221_REG_CH1_WARNING_ALERT_LIM,
  INA3221_REG_CH2_CRIT_ALERT_LIM,
  INA3221_REG_CH2_WARNING_ALERT_LIM,
  INA3221_REG_CH3_CRIT_ALERT_LIM,
  INA3221_REG_CH3_WARNING_ALERT_LIM,
  INA3221_REG_SHUNTV_SUM,
  INA3221_REG_SHUNTV_SUM_LIM,
  INA3221_REG_MASK_ENABLE,
  INA3221_REG_PWR_VALID_HI_LIM,
  INA3221_REG_PWR_VALID_LO_LIM,
  INA3221_REG_MANUF_ID = 0xFE,
  INA3221_REG_DIE_ID = 0xFF
} ina3221_reg_t;

// Conversion times
typedef enum {
  INA3221_REG_CONF_CT_140US = 0,
  INA3221_REG_CONF_CT_204US,
  INA3221_REG_CONF_CT_332US,
  INA3221_REG_CONF_CT_588US,
  INA3221_REG_CONF_CT_1100US,
  INA3221_REG_CONF_CT_2116US,
  INA3221_REG_CONF_CT_4156US,
  INA3221_REG_CONF_CT_8244US
} ina3221_conv_time_t;

// Averaging modes
typedef enum {
  INA3221_REG_CONF_AVG_1 = 0,
  INA3221_REG_CONF_AVG_4,
  INA3221_REG_CONF_AVG_16,
  INA3221_REG_CONF_AVG_64,
  INA3221_REG_CONF_AVG_128,
  INA3221_REG_CONF_AVG_256,
  INA3221_REG_CONF_AVG_512,
  INA3221_REG_CONF_AVG_1024
} ina3221_avg_mode_t;

// the config regiter on the ina3221
typedef struct __attribute__((packed)) {
  uint16_t mode_shunt_en : 1;
  uint16_t mode_bus_en : 1;
  uint16_t mode_continious_en : 1;
  uint16_t shunt_conv_time : 3;
  uint16_t bus_conv_time : 3;
  uint16_t avg_mode : 3;
  uint16_t ch3_en : 1;
  uint16_t ch2_en : 1;
  uint16_t ch1_en : 1;
  uint16_t reset : 1;
} conf_reg_t;

#define INA_ADDR INA3221_ADDR40_GND

// read an ina3221 register into val
void ina_rr(ina3221_reg_t reg, uint16_t *val);
void ina_wr(ina3221_reg_t reg, uint16_t *val);
#endif
