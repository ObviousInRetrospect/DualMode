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

// Dual Mode Example demonstrates a device that acts as
// a bus master on PA3/PA2 controlling an INA3221
// (such as
// https://www.amazon.com/INA3221-Triple-Channel-Current-Voltage-Monitor/dp/B0946J726D?tag=gr32-20)
// a slave on PC2/PC3 responding to address 0x42 providing an accumulated view
// of the INA3221 readings see DualModeExampleClient

// this demonstrates:
// dual mode usage (set menu option wire 1x master and slave)
// sleepy i2c device
// PIT usage with fallback from crystal to ulp

// I2C_INA PA3/PA2 (default)
// I2C0 C2/C3 (alt)

#include "ina3221.h"
#include <Wire.h>
#include <avr/sleep.h>

#include <CRC32.h>

#define TEST_PATTERN
#define ENABLE_SCAN

#define ERR_CRYSTAL 0b00000001

// outside an example this belongs in a library shared with the client
#define EWDT2_ACC_PER_CHA 2
#define EWDT2_PWR_CHA 3
#define EWDT2_WDT_CHA 3
#define EWDT2_SVS_DIV_MAH (21897.810219)

#define CHUNK_SZ 32
// register structures definitions
// this acts like a well-behaved i2c device should.
// you send it an address and read from that address

// to make it easier to write for however a union is used to overlay the
// registers onto some structs and a macro is provided to calculate offsets more
// easily

// a current accumulator
typedef struct __attribute((packed)) {
  // the sum of all the shunt voltage readings
  int32_t sv_sum;
  // the count of readings in the sum
  // divide by this for an average current
  //(after applying whatever factor is needed)
  // the default resistors on the breakout are 0.1
  // which means each lsb is 400ua or 0.4ma
  uint16_t sv_cnt; // writing to the count resets the accumulator
} ewdt_cur_acc_t;

// an ina3221 channel
typedef struct __attribute((packed)) {
  // bus voltage
  uint16_t vbus;
  // shunt voltage
  int16_t vshunt;
  // a pair of accumulators
  ewdt_cur_acc_t acc[EWDT2_ACC_PER_CHA];
} ewdt_pwr_ch_t;

// this devices register set
typedef struct __attribute((packed)) {
  uint32_t bts; // build timestamp, not implimented
  ewdt_pwr_ch_t pwr[EWDT2_PWR_CHA];
  uint8_t err;
  uint32_t crc;
} ewdt_regs_t;

// a union for convenient access
typedef union {
  ewdt_regs_t d;                  // data structure view
  uint8_t r[sizeof(ewdt_regs_t)]; // raw bytes view
} ewdt_regs_u;

// end of inlined library

ewdt_regs_u reg;
ewdt_regs_u bak;
volatile uint8_t to_clear;

#define P_WD_RSTP PIN_PD3
#define P_LED PIN_PD4

// reset the ina3221
void ina_reset() {
  conf_reg_t conf;
  ina_rr(INA3221_REG_CONF, (uint16_t *)&conf);
  conf.reset = 1;
  ina_wr(INA3221_REG_CONF, (uint16_t *)&conf);
  delay(100);
}

// configure the ina3221 for ~410ms samples
void ina_config() {
  conf_reg_t conf_reg;
  ina_rr(INA3221_REG_CONF, (uint16_t *)&conf_reg);
  conf_reg.ch1_en = 1;
  conf_reg.ch2_en = 1;
  conf_reg.ch3_en = 1;
  conf_reg.avg_mode = INA3221_REG_CONF_AVG_16; // was 64
  conf_reg.bus_conv_time = INA3221_REG_CONF_CT_588US;
  conf_reg.shunt_conv_time = INA3221_REG_CONF_CT_8244US;
  ina_wr(INA3221_REG_CONF, (uint16_t *)&conf_reg);
}

// have a reading
volatile uint8_t ina_new = 0;

// lookup tables to allow for single loop
ina3221_reg_t cha_to_shuntv_reg[] = {
    INA3221_REG_CH1_SHUNTV, INA3221_REG_CH2_SHUNTV, INA3221_REG_CH3_SHUNTV};
ina3221_reg_t cha_to_busv_reg[] = {INA3221_REG_CH1_BUSV, INA3221_REG_CH2_BUSV,
                                   INA3221_REG_CH3_BUSV};

// check for a new sample on the ina3221
// and accumulate it if it exists
uint32_t lastrd = 0;
void rd_ina3221() {
  #ifndef TEST_PATTERN
  uint16_t reg_me;
  cli();
  // work on a temp copy of the registers.
  memcpy(bak.r, reg.r, sizeof(ewdt_regs_t));
  to_clear = 0; // we have whatever was in regs
  sei();
  ina_rr(INA3221_REG_MASK_ENABLE, &reg_me);
  if (reg_me & 0x1) { // low bit = reading ready
    // read the raw registers as the nice functions use expensive floating point
    // conversions even getShuntVoltage has an unhelpful *5.
    for (uint16_t ch = 0; ch < 3; ch++) {
      int16_t sv; // shunt voltage
      ina_rr(cha_to_shuntv_reg[ch], (uint16_t *)&sv);
      ina_rr(cha_to_busv_reg[ch], &(bak.d.pwr[ch].vbus));
      sv = sv >> 3; // per the datasheet the low 3 bits are don't care.
      bak.d.pwr[ch].vshunt = sv;
      for (int a = 0; a < EWDT2_ACC_PER_CHA; a++) {
        bak.d.pwr[ch].acc[a].sv_sum += sv;
        bak.d.pwr[ch].acc[a].sv_cnt++;
      }
    }
    ina_new = 1;
    cli();
    if (to_clear) {
      for (int ch = 0; ch < 3; ch++) {
        if (to_clear & 0b1) {
          bak.d.pwr[ch].acc[0].sv_cnt = 0;
          bak.d.pwr[ch].acc[0].sv_sum = 0;
        }
        if (to_clear & 0b10) {
          bak.d.pwr[ch].acc[1].sv_cnt = 0;
          bak.d.pwr[ch].acc[1].sv_sum = 0;
        }
        to_clear >>= 2;
      }
    }
    sei();
    bak.d.crc = CRC32::calculate(bak.r, sizeof(ewdt_regs_t) - 4);
    cli();
    // could also just have a read pointer that gets moved over here
    memcpy(reg.r, bak.r, sizeof(ewdt_regs_t));
    sei();
  }
  #else
   //test pattern data
   for(uint16_t i=0; i<sizeof(ewdt_regs_t); i++){
     reg.r[i]=i&0xFF;
   }
   reg.d.crc=CRC32::calculate(reg.r,sizeof(ewdt_regs_t)-4);
  #endif
}

// setup the watch crystal, rtc, and PIT
void RTC_init(void) {
  uint32_t ims = millis();
  // setup watch crystal XOSC32K
  _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA,
                   CLKCTRL_CSUT_1K_gc            /* 64k cycles */
                       | 1 << CLKCTRL_ENABLE_bp   /* Enable: enabled */
                       | 1 << CLKCTRL_RUNSTDBY_bp /* Run standby: enabled */
                       | 0 << CLKCTRL_SEL_bp      /* Select: disabled */
                       | 1 << CLKCTRL_LPMODE_bp /* Low-Power Mode: disabled */);

  // wait for the crystal to start
  while (0 == (CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm)) {
    if ((millis() - ims) > 2000) { // taking too long, make noise
      Serial.println(F("Waiting for XOSC32K to start..."));
      delay(1000);
      Serial.print(F("XOSC32KCTRLA: "));
      Serial.println(CLKCTRL.XOSC32KCTRLA, 2);
      Serial.print(F("MCLKSTATUS: "));
      Serial.println(CLKCTRL.MCLKSTATUS, 2);
      Serial.print(F("CLKCTRL_XOSC32KS_bm: "));
      Serial.println(CLKCTRL_XOSC32KS_bm, 2);
    }
    if ((millis() - ims) > 4000) { // give up and use OSCULP32K instead
      Serial.println(F("XOSC32K startup FAILED"));
      reg.d.err |= ERR_CRYSTAL;
      while (RTC.STATUS > 0) {
        ; /* Wait for all registers to be synchronized */
      }
      RTC.CLKSEL = RTC_CLKSEL_INT32K_gc; /* 32.768kHz Internal Ultra-Low-Power
                                      Oscillator (OSCULP32K) */

      RTC.PITINTCTRL = RTC_PI_bm; /* PIT Interrupt: enabled */

      RTC.PITCTRLA = RTC_PERIOD_CYC2048_gc /* RTC Clock Cycles 2048, resulting
                                    in 32.768kHz/2048 = 16Hz */
                     | RTC_PITEN_bm;       /* Enable PIT counter: enabled */
      Serial.println(F("RTC initialized using OSCULP32K"));
      return;
    }
  }
  reg.d.err &= ~ERR_CRYSTAL;
  Serial.println(F("XOSC32K started. Initializing RTC."));

  /* Initialize RTC: */
  while (RTC.STATUS > 0) {
    ; /* Wait for all registers to be synchronized */
  }
  RTC.CLKSEL = CLKCTRL_CLKSEL_XOSC32K_gc; /* 32.768kHz crystal */
  RTC.PITINTCTRL = RTC_PI_bm;             /* PIT Interrupt: enabled */
  RTC.PITCTRLA = RTC_PERIOD_CYC2048_gc    /* RTC Clock Cycles 2048, resulting
                                   in 32.768kHz/2048 = 16Hz */
                 | RTC_PITEN_bm;          /* Enable PIT counter: enabled */
  Serial.println(F("RTC initialized using XOSC32K"));
}

volatile uint8_t wake = 0;
volatile uint32_t ticks = 0;
// programmable interrupt timer
// fires at RTC_HZ (16)
ISR(RTC_PIT_vect) {
  wake = 1;
  RTC.PITINTFLAGS =
      RTC_PI_bm; /* Clear interrupt flag by writing '1' (required) */
  ticks++;
}

uint16_t WirePointer = 0;
void receiveHandler(int numbytes) {
  Wire.getBytesRead(); // reset count of bytes read. We don't do anything with
                       // it here, but a write is going to reset it to a new
                       // value.
  WirePointer = Wire.read();
  WirePointer <<= 8;
  WirePointer |= Wire.read();
  numbytes -= 2;
  while (numbytes) {
    uint8_t nv = Wire.read();
    uint8_t wrote = 0;
    #ifndef TEST_PATTERN
    for (int ch = 0; ch < 3; ch++) {
      for (int a = 0; a < EWDT2_ACC_PER_CHA; a++) {
        if (&(reg.r[WirePointer]) ==
                (uint8_t *)&(reg.d.pwr[ch].acc[a].sv_cnt) ||
            &(reg.r[WirePointer]) ==
                ((uint8_t *)&(reg.d.pwr[ch].acc[a].sv_cnt)) + 1) {
          reg.d.pwr[ch].acc[a].sv_cnt = 0;
          reg.d.pwr[ch].acc[a].sv_sum = 0;
          to_clear |= (a == 0 ? 0b01 : 0b10) << (2 * ch);
          wrote = 1;
        }
      }
    }
    if (!wrote) {
      reg.r[WirePointer++] = nv;
    }
    #else
      WirePointer++; //ignore write
    #endif
    if (WirePointer >= sizeof(ewdt_regs_t))
      WirePointer = 0;
    numbytes--;
  }
}

void requestHandler() {
  uint8_t bytes_read = Wire.getBytesRead();
  WirePointer = (WirePointer + (bytes_read));
  uint16_t end = WirePointer + CHUNK_SZ;
  end = min(end, sizeof(ewdt_regs_t));
  for (uint16_t i = WirePointer; i < end; i++) {
    Wire.write(reg.r[i]);
    // "write" a bunch of data - but the master might only want one byte.
    // The slave doesn't know how much data the master will want yet.
    // and won't know until it's gotten all that it wants and has generated a
    // stop condition.
  }
}

void setup() {
  memset(reg.r, 0, sizeof(ewdt_regs_t));
  memset(bak.r, 0, sizeof(ewdt_regs_t));
  TCB2.CTRLA |= 1 << TCB_RUNSTDBY_bp;
  Wire.enableDualMode(false);
  Serial.begin(115200);
  Wire.begin();
  Serial.println("boot");
  ina_reset();
  ina_config();
  pinMode(P_LED, OUTPUT);
  digitalWriteFast(P_LED, 1);
  RTC_init();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
#ifdef ENABLE_SCAN     // i2c scan
  byte error, address; // variable for error and I2C address
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
#endif
  #ifdef TEST_PATTERN
  rd_ina3221();
  #endif
  Wire.begin(0x42);
  Wire.onReceive(receiveHandler);
  Wire.onRequest(requestHandler);
}

uint32_t ll = 0;
void loop() {
  while (!wake && Wire.slaveTransactionOpen()) {
    delay(1);
  }
  while (!wake && !Wire.slaveTransactionOpen()) {
    Serial.flush();
    sleep_cpu();
    //delay(1);
  }
  wake = 0;
  cli();
  uint32_t loop_ticks = ticks;
  sei();
  if (((loop_ticks - ll) >> 4) >= 5) {
    ll = loop_ticks;
    Serial.println();
    for (uint16_t i = 0; i < sizeof(ewdt_regs_t); i++) {
      if (i && !(i & 0xF))
        Serial.println(); // newline
      Serial.printHex(reg.r[i]);
    }
    Serial.println();
    for (int ch = 0; ch < 3; ch++) {
      Serial.print("ch");
      Serial.print(ch + 1);
      Serial.print(" bv:");
      Serial.print(reg.d.pwr[ch].vbus);
      Serial.print(" sv:");
      Serial.print(reg.d.pwr[ch].vshunt);
      Serial.print(" ua:");
      Serial.print(reg.d.pwr[ch].vshunt * 400);
      for (int a = 0; a < EWDT2_ACC_PER_CHA; a++) {
        Serial.print(" acc");
        Serial.print(a);
        Serial.print(':');
        Serial.print(reg.d.pwr[ch].acc[a].sv_sum);
        Serial.print('/');
        Serial.print(reg.d.pwr[ch].acc[a].sv_cnt);
        Serial.print('=');
        Serial.print((reg.d.pwr[ch].acc[a].sv_sum) / EWDT2_SVS_DIV_MAH);
        Serial.print("mah");
      }
      Serial.println();
    }
  }
  #ifndef TEST_PATTERN
  rd_ina3221();
  #endif
}
