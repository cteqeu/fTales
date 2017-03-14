//#include "Arduino.h"
//#include <Wire.h>
#include "PXL_MAX30102.h"

MAX30100::MAX30102(){

}

void MAX30102::PXL_MAX30102_setLEDS(uint8_t pulseWidth, uint8_t red, uint8_t ir){
  uint8_t byteRead = PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_SP02CONFIG);
  byteRead = byteRead & 0xFC;
  PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_SP02CONFIG, byteRead | pulseWidth);
}

void MAX30102::PXL_MAX30102_setSPO2(uint8_t sampleRate){
  uint8_t byteRead = PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30100_SPO2_CONFIG);
  byteRead = byteRead & 0xE3;
  I2CwriteByte(MAX30102_I2CADDR, MAX30100_SPO2_CONFIG, byteRead | (sr<<2));
  byteRead = PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30100_MODE_CONFIG);
  byteRead = byteRead & 0xf8;
  I2CwriteByte(MAX30102_I2CADDR, MAX30100_SPO2_CONFIG, byteRead | 0x03); 
}

int MAX30102::PXL_MAX30102_getNumberSamples(void){
    uint8_t wrPtr = PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_FIFOWRITEPOINTER);
    uint8_t rdPtr = PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_FIFOREADPOINTER);
    return (abs( 16 + wrPtr - rdPtr ) % 16);
}

void MAX30102::PXL_MAX30102_readSensor(void){
  uint8_t tempReading[6] = {0};
  PXL_MAX30102_I2CrBYTEs(MAX30102_I2CADDR, MAX30102_FIFODATAREGISTER, &tempReading[0], 6);
  //tempReading[0] = 0x11; tempReading[1] = 0x22; tempReading[2] = 0x33; tempReading[4] = 0x44; tempReading[5] = 0x55;tempReading[6] = 0x66;
  IRlastdatapoint =  (((long)tempReading[0] & 0x03)<<16) | (long)tempReading[1]<<8 | (long)tempReading[2];    // Combine values to get the actual number
  REDlastdatapoint = (((long)tempReading[3] & 0x03)<<16) | (long)tempReading[4]<<8 | (long)tempReading[5];   // Combine values to get the actual number
}

void MAX30102::PXL_MAX30102_shutdown(void){
  uint8_t byteRead = PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_MODECONFIG);
  PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_MODECONFIG , byteRead | 0x80);   
}

void MAX30102::PXL_MAX30102_reset(void){
  uint8_t byteRead = PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_MODECONFIG);
  PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_MODECONFIG , byteRead | 0x40);
}

void MAX30102::PXL_MAX30102_startup(void){
  uint8_t byteRead = PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_MODECONFIG);
  PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_MODECONFIG, byteRead & 0x7F);
}

int MAX30102::PXL_MAX30102_getPartID(void){
  return PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_PARTID );
}


//  void  PXL_MAX30102_begin(uint8_t pulseWidth = MAX30102_PULSEWIDTH_411, uint8_t ir = MAX30102_CURRENT_FF, uint8_t sampleRate = MAX30102_SAMPLERATE_100);
// static const uint8_t MAX30102_PULSEWIDTH_411                        =   0x03;
// static const uint8_t MAX30102_SAMPLERATE_100                        =   0x04; 
// static const uint8_t MAX30102_CURRENT_FF                            =   0xFF; 
 
void  PXL_MAX30102_begin(uint8_t pulseWidth, uint8_t ir, uint8_t sampleRate)
{
  I2CwriteByte(MAX30102_I2CADDR, MAX30102_INTERRUPTENABLE1 ,                  0xC0); // A_FULL_EN  & PPG_RDY_EN
  I2CwriteByte(MAX30102_I2CADDR, MAX30102_INTERRUPTENABLE2,                   0x00); 
  I2CwriteByte(MAX30102_I2CADDR, MAX30102_FIFOWRITEPOINTER,                   0x00); 
  I2CwriteByte(MAX30102_I2CADDR, MAX30102_FIFOOVERFLOWCOUNTER,                0x00); 
  I2CwriteByte(MAX30102_I2CADDR, MAX30102_FIFOREADPOINTER ,                   0x00); 
  I2CwriteByte(MAX30102_I2CADDR, MAX30102_FIFOCONFIGURATION,                  0x4F); // b0100 1111 SMP_AVE[2:0] = > 010 => 4 samples ; FIFO_ROLLOVER_EN = 0 ; FIFO_A_FULL[3:0] => 1111 
  I2CwriteByte(MAX30102_I2CADDR, MAX30102_MODECONFIG,                         0x03); // MODE[2:0] = > 011 => SpO2 mode => REd and IR
  I2CwriteByte(MAX30102_I2CADDR, MAX30102_SP02CONFIG,                         0x27); // b0010 0111 => SPO2_ADC_REG[1:0] = 01 LSB Size = 15.63pA , 4096nA full scale ; SPO2_SR[2:0]=001 => 100 Samples Per second ; LED_PW[1:0]= 11 => 411 us => 18 bits ADC resolution
  I2CwriteByte(MAX30102_I2CADDR, MAX30102_LED_PULSEAMP1 ,                     0x24); // b0010 0100 => mA
  I2CwriteByte(MAX30102_I2CADDR, MAX30102_LED_PULSEAMP2,                      0x24); // b0010 0100 => mA
  I2CwriteByte(MAX30102_I2CADDR, MAX30102_PROXIMITY_MODE_LED_PULSE_AMPLITUDE, 0x7F); // b0111 1111 => 
  
}

void MAX30102::PXL_MAX30102_debug(void){
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_INTERRUPTSTATUS1),                    BIN); // 0x00
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_INTERRUPTSTATUS2),                    BIN); // 0x01
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_INTERRUPTENABLE1),                    BIN); // 0x02
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_INTERRUPTENABLE2),                    BIN); // 0x03
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_FIFOWRITEPOINTER),                    BIN); // 0x04
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_FIFOOVERFLOWCOUNTER),                 BIN); // 0x05
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_FIFOREADPOINTER),                     BIN); // 0x06
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_MAX30102_FIFODATAREGISTER),           BIN); // 0x07
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_FIFOCONFIGURATION),                   BIN); // 0x08
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_MODECONFIG),                          BIN); // 0x09
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_SP02CONFIG),                          BIN); // 0x0A
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_LED_PULSEAMP1),                       BIN); // 0x0C
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_LED_PULSEAMP2),                       BIN); // 0x0D
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_PROXIMITY_MODE_LED_PULSE_AMPLITUDE),  BIN); // 0x10
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_MULTILED_MODE_CONTROL_REGISTERS1),    BIN); // 0x11
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_MULTILED_MODE_CONTROL_REGISTERS2),    BIN); // 0x12
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_DIETEMPERATURE_INTEGER),              BIN); // 0x1F
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_DIETEMPERATURE_FRACTION),             BIN); // 0x20
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_DIETEMPERATURE_CONFIG),               BIN); // 0x21
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_PROXIMITY_INTERRUPT_THRESHOLD),       BIN); // 0x30
}

// Wire.h read and write protocols
void MAX30102::PXL_MAX30102_I2CwBYTE(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t MAX30102::PXL_MAX30102_I2CrBYTE(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);         
  Wire.write(subAddress);                  
  Wire.endTransmission(false);             
  Wire.requestFrom(address, (uint8_t) 1);  
  data = Wire.read();                      
  return data;                             
}

void MAX30102::PXL_MAX30102_I2CrBYTES(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  // Next send the register to be read. OR with 0x80 to indicate multi-read.
  Wire.write(subAddress);     
  Wire.endTransmission(false);
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address
  while (Wire.available())
  {
    dest[i++] = Wire.read(); 
  }
}
