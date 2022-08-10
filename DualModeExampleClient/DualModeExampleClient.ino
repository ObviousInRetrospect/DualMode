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

// Dual Mode Example Client talks to the DualModeExampleSketch


// I2C_INA PA3/PA2 (default)
// I2C0 C2/C3 (alt)

#define VERBOSE_DBG

#include <Wire.h>

#include <CRC32.h>

//outside an example this belongs in a library shared with the client
#define EWDT2_ACC_PER_CHA 2
#define EWDT2_PWR_CHA 3
#define EWDT2_WDT_CHA 3
#define EWDT2_SVS_DIV_MAH (21897.810219)

#define CHUNK_SZ 32
typedef struct __attribute((packed)){
    int32_t sv_sum;
    uint16_t sv_cnt;
} ewdt_cur_acc_t;

typedef struct __attribute((packed)){
    uint16_t vbus;
    int16_t vshunt;
    ewdt_cur_acc_t acc[EWDT2_ACC_PER_CHA];
} ewdt_pwr_ch_t;

typedef struct __attribute((packed)){
    uint32_t bts;
    ewdt_pwr_ch_t pwr[EWDT2_PWR_CHA];
    uint8_t err;
    uint32_t crc;
} ewdt_regs_t;

typedef union{
  ewdt_regs_t d; //data structure view
  uint8_t r[sizeof(ewdt_regs_t)]; //raw bytes view
} ewdt_regs_u;

//end of inlined library

ewdt_regs_u rcp; //register copy, use rcp_ld_* to populate

//calculate the start address of a data element in the regs.d struct
#define REG_OFFSET(base,elem)  (((uint8_t*)(&(elem)))-((uint8_t*)(&(base))))


//read register list (8-bit registers, 16-bit addresses)
//devaddr is the i2c address
//reg is the register address
//len is the number of register (bytes) to read
//buf gets the data
void rrl8(uint8_t devaddr, uint16_t reg, uint8_t len, uint8_t* buf){
  Wire.beginTransmission(devaddr);
  Wire.write((uint8_t)(reg>>8));
  Wire.write((uint8_t)(reg&0xFF));
  Wire.endTransmission();
  Wire.requestFrom(devaddr,(size_t)len);
  for(int i=0; i<len; i++){
    buf[i]=Wire.read();
  }
}

//read a single register
uint8_t rr8(uint8_t devaddr,uint16_t reg){
  Wire.beginTransmission(devaddr);
  Wire.write((uint8_t)(reg>>8));
  Wire.write((uint8_t)(reg&0xFF));
  Wire.endTransmission();
  Wire.requestFrom(devaddr,(size_t)2);
  uint8_t ret=Wire.read();
  return(ret);
}

//write version of rrl8
void rwl8(uint8_t devaddr,uint16_t reg, uint8_t len, uint8_t* buf){
  Wire.beginTransmission(devaddr);
  Wire.write((uint8_t)(reg>>8));
  Wire.write((uint8_t)(reg&0xFF));
  for(int i=0; i<len; i++){
    Wire.write(buf[i]);
  }
  Wire.endTransmission();
}

//write version of rr8
void rw8(uint8_t devaddr,uint16_t reg, uint8_t val){
  Wire.beginTransmission(devaddr);
  Wire.write((uint8_t)(reg>>8));
  Wire.write((uint8_t)(reg&0xFF));
  Wire.write(val);
  Wire.endTransmission();
}


#define RETRY_CNT 2
//load the full register file
void rcp_ld_all(){
  uint8_t tries=RETRY_CNT;
  uint32_t myCrc;
  do{
    for(uint16_t i=0; i<=sizeof(ewdt_regs_t); i+=CHUNK_SZ){
      rrl8(0x42,i,min((unsigned int)CHUNK_SZ,sizeof(ewdt_regs_t)-i),rcp.r+i);
    }
    myCrc=CRC32::calculate(rcp.r,sizeof(ewdt_regs_t)-4);
    if(myCrc != rcp.d.crc){
#ifdef VERBOSE_DBG
      Serial.print("rcv_crc:");
        Serial.printHex(rcp.d.crc);
        Serial.print(" calc:");
        Serial.printHexln(myCrc);
        Serial.print("bad crc, will retry ");
        Serial.print(tries);
        Serial.println(" times");
        for(uint16_t i=0; i<sizeof(ewdt_regs_t); i++){
          if(i && !(i&0xF)) Serial.println(); //newline
          Serial.printHex(rcp.r[i]);
        }
        Serial.println();
#endif
        delay(100+((RETRY_CNT-tries)*20));
        tries--;
    }
  } while(myCrc != rcp.d.crc && tries);
}

//clear a channel accumulator
void clear_accum(uint8_t ch, uint8_t acc){
  uint16_t ofs=REG_OFFSET(rcp.d,rcp.d.pwr[ch].acc[acc].sv_cnt);
  rw8(0x42,ofs,0);
}


void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  //Serial.swap(); 
  Serial.begin(115200);

}

uint32_t ll=0;
void loop() {
  // put your main code here, to run repeatedly:
  if(millis()-ll>5000){
    ll=millis();
    rcp_ld_all();
    Serial.println();
    for(uint16_t i=0; i<sizeof(ewdt_regs_t); i++){
      if(i && !(i&0xF)) Serial.println(); //newline
      Serial.printHex(rcp.r[i]);
    }
    Serial.println();
    for(int ch=0; ch<3; ch++){
      clear_accum(ch,0);
      Serial.print("ch");
      Serial.print(ch+1);
      Serial.print(" bv:");
      Serial.print(rcp.d.pwr[ch].vbus);
      Serial.print(" sv:");
      Serial.print(rcp.d.pwr[ch].vshunt);
      Serial.print(" ua:");
      Serial.print(rcp.d.pwr[ch].vshunt*400);
      for(int a=0; a<EWDT2_ACC_PER_CHA; a++){
        Serial.print(" acc");
        Serial.print(a);
        Serial.print(':');
        Serial.print(rcp.d.pwr[ch].acc[a].sv_sum);
        Serial.print('/');
        Serial.print(rcp.d.pwr[ch].acc[a].sv_cnt);
        Serial.print('=');
        Serial.print((rcp.d.pwr[ch].acc[a].sv_sum)/EWDT2_SVS_DIV_MAH);
        Serial.print("mah");
      }
      Serial.println();
    }
  }
}
