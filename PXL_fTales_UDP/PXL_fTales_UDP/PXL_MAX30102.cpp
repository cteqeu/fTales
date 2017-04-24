#include "PXL_MAX30102.h"

MAX30102::MAX30102(){

}

void MAX30102::PXL_MAX30102_setLEDS(uint8_t pulseWidth, uint8_t red, uint8_t ir){
  uint8_t byteRead = PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_SP02CONFIG);
  byteRead = byteRead & 0xFC;
  PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_SP02CONFIG, byteRead | pulseWidth);
}

void MAX30102::PXL_MAX30102_setSPO2(uint8_t sampleRate){
  uint8_t byteRead = PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_SP02CONFIG);
  byteRead = byteRead & 0xE3;
  PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_SP02CONFIG, byteRead | (sampleRate<<2));
  byteRead = PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_MODECONFIG);
  byteRead = byteRead & 0xf8;
  PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_SP02CONFIG, byteRead | 0x03); 
}

int MAX30102::PXL_MAX30102_getNumberSamples(void){
    uint8_t wrPtr = PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_FIFOWRITEPOINTER);
    uint8_t rdPtr = PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_FIFOREADPOINTER);
    return (abs( 16 + wrPtr - rdPtr ) % 16);
}

void MAX30102::PXL_MAX30102_readSensor(void){
  uint8_t tempReading[6] = {0};
  PXL_MAX30102_I2CrBYTES(MAX30102_I2CADDR, MAX30102_FIFODATAREGISTER, &tempReading[0], 6);
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
 
void  MAX30102::PXL_MAX30102_begin(uint8_t pulseWidth, uint8_t ir, uint8_t sampleRate)
{
  PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_INTERRUPTENABLE1 ,                  0xC0); // A_FULL_EN  & PPG_RDY_EN
  PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_INTERRUPTENABLE2,                   0x00); 
  PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_FIFOWRITEPOINTER,                   0x00); 
  PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_FIFOOVERFLOWCOUNTER,                0x00); 
  PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_FIFOREADPOINTER ,                   0x00); 
  PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_FIFOCONFIGURATION,                  0x4F); // b0100 1111 SMP_AVE[2:0] = > 010 => 4 samples ; FIFO_ROLLOVER_EN = 0 ; FIFO_A_FULL[3:0] => 1111 
  PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_MODECONFIG,                         0x03); // MODE[2:0] = > 011 => SpO2 mode => REd and IR
  PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_SP02CONFIG,                         0x27); // b0010 0111 => SPO2_ADC_REG[1:0] = 01 LSB Size = 15.63pA , 4096nA full scale ; SPO2_SR[2:0]=001 => 100 Samples Per second ; LED_PW[1:0]= 11 => 411 us => 18 bits ADC resolution
  PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_LED_PULSEAMP1 ,                     0x24); // b0010 0100 => mA
  PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_LED_PULSEAMP2,                      0x24); // b0010 0100 => mA
  PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_PROXIMITY_MODE_LED_PULSE_AMPLITUDE, 0x7F); // b0111 1111 => 
  
}

void MAX30102::PXL_MAX30102_debug(void){
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_INTERRUPTSTATUS1),                    BIN); // 0x00
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_INTERRUPTSTATUS2),                    BIN); // 0x01
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_INTERRUPTENABLE1),                    BIN); // 0x02
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_INTERRUPTENABLE2),                    BIN); // 0x03
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_FIFOWRITEPOINTER),                    BIN); // 0x04
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_FIFOOVERFLOWCOUNTER),                 BIN); // 0x05
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_FIFOREADPOINTER),                     BIN); // 0x06
  Serial.println(PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_FIFODATAREGISTER),                    BIN); // 0x07
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

bool MAX30102::PXL_MAX30102_rFIFO(uint32_t *pun_red_led,uint32_t *pun_ir_led){
  uint32_t un_temp;
 // uint8_t uch_temp;
  unsigned char uch_temp;
  *pun_ir_led=0;
  *pun_red_led=0;
  char tempReading[6];
 
  
  uch_temp= PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_INTERRUPTSTATUS1);
  uch_temp= PXL_MAX30102_I2CrBYTE(MAX30102_I2CADDR, MAX30102_INTERRUPTSTATUS2);

   tempReading[0]=MAX30102_FIFODATAREGISTER;
  //Wire.beginTransmission(address);  // Initialize the Tx buffer
  //Wire.write(subAddress);           // Put slave register address in Tx buffer
  //Wire.write(data);                 // Put data in Tx buffer
  //Wire.endTransmission();           // Send the Tx buffer
  
  //maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp);
  //maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp);
 
  //PXL_MAX30102_I2CwBYTE(MAX30102_I2CADDR, MAX30102_FIFODATAREGISTER,                  0xC0);

  Wire.beginTransmission(MAX30102_I2CADDR);   // Initialize the Tx buffer
  // Next send the register to be read. OR with 0x80 to indicate multi-read.
  Wire.write(tempReading[0]);     
  Wire.endTransmission(false);

  //PXL_MAX30102_I2CrBYTES(I2C_READ_ADDR, TempReading,6);
  //PXL_MAX30102_I2CrBYTES(I2C_READ_ADDR, MAX30102_FIFODATAREGISTER, &tempReading[0], 6);
  
  // Wire.beginTransmission(MAX30102_I2CADDR);   // Initialize the Tx buffer
  
  // Next send the register to be read. OR with 0x80 to indicate multi-read.
 // Wire.write(subAddress);     
  
  //Wire.endTransmission(false);
  uint8_t i = 0;
  Wire.requestFrom(MAX30102_I2CADDR, 6);  // Read bytes from slave register address
  while (Wire.available())
  {
    tempReading[i] = Wire.read();
    //Serial.print(tempReading[i]);
    i++;
  }
  
  
  
  //uint8_t i = 0;
  //Wire.requestFrom(address, count);  // Read bytes from slave register address
  //while (Wire.available())
  //{
  //  dest[i++] = Wire.read(); 
  //}


/*  if(!i2c_start(I2C_WRITE_ADDR))
    return false;
  if(!i2c_write(REG_FIFO_DATA))
    return false;
  if(!i2c_rep_start(I2C_READ_ADDR))
    return false;  
  
  un_temp=i2c_read(false);
  un_temp<<=16;
  *pun_red_led+=un_temp;
  
  un_temp=i2c_read(false);
  un_temp<<=8;
  *pun_red_led+=un_temp;
  
  un_temp=i2c_read(false);
  *pun_red_led+=un_temp;
  
  un_temp=i2c_read(false);
  un_temp<<=16;
  *pun_ir_led+=un_temp;
  
  un_temp=i2c_read(false);
  un_temp<<=8;
  *pun_ir_led+=un_temp;
  
  un_temp=i2c_read(true);
  *pun_ir_led+=un_temp;
  
  i2c_stop();*/
  
  
  
  un_temp =tempReading[0];
  un_temp<<=16;
  *pun_red_led+=un_temp;
  
  un_temp =tempReading[1];
  un_temp<<=8;
  *pun_red_led+=un_temp;
  
  un_temp =tempReading[2];
  *pun_red_led+=un_temp;

  un_temp =tempReading[3];
   un_temp<<=16;
  *pun_ir_led+=un_temp;

  un_temp =tempReading[4];
    un_temp<<=8;
  *pun_ir_led+=un_temp;
  
  un_temp =tempReading[5];
  *pun_ir_led+=un_temp;
   
  *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
  *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]
  return true;
  
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
