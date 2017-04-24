#include <Energia.h>
#include <Wire.h>

#define I2C_WRITE_ADDR 0xAE
#define I2C_READ_ADDR 0xAF

typedef unsigned char uint8_t;
//typedef unsigned int uint16_t;



// I2C Address of MAX30102
static const uint8_t MAX30102_I2CADDR                               =    0x57;

// PartID of MAX30102 => if readout should always be 0x15
static const uint8_t MAX30102_PARTID                                =    0xFF;  

// MAX30102 Status Registers
static const uint8_t MAX30102_INTERRUPTSTATUS1                      =    0x00;
static const uint8_t MAX30102_INTERRUPTSTATUS2                      =    0x01;
static const uint8_t MAX30102_INTERRUPTENABLE1                      =    0x02;
static const uint8_t MAX30102_INTERRUPTENABLE2                      =    0x03;

// MAX30102 FIFO Registers
static const uint8_t MAX30102_FIFOWRITEPOINTER                      =    0x04;
static const uint8_t MAX30102_FIFOOVERFLOWCOUNTER                   =    0x05;
static const uint8_t MAX30102_FIFOREADPOINTER                       =    0x06;
static const uint8_t MAX30102_FIFODATAREGISTER                      =    0x07;

// MAX30102 CONFIGURATION Registers
static const uint8_t MAX30102_FIFOCONFIGURATION                     =    0x08;
static const uint8_t MAX30102_MODECONFIG                            =    0x09;
static const uint8_t MAX30102_SP02CONFIG                            =    0x0A;    
static const uint8_t MAX30102_LED_PULSEAMP1                         =    0x0C;
static const uint8_t MAX30102_LED_PULSEAMP2                         =    0x0D;
static const uint8_t MAX30102_PROXIMITY_MODE_LED_PULSE_AMPLITUDE    =    0x10;
static const uint8_t MAX30102_MULTILED_MODE_CONTROL_REGISTERS1      =    0x11;
static const uint8_t MAX30102_MULTILED_MODE_CONTROL_REGISTERS2      =    0x12;

// MAX30102 Die (chip) Temperature Registers
static const uint8_t MAX30102_DIETEMPERATURE_INTEGER                =    0x1F;
static const uint8_t MAX30102_DIETEMPERATURE_FRACTION               =    0x20;
static const uint8_t MAX30102_DIETEMPERATURE_CONFIG                 =    0x21;

// MAX30102 Proximity Function Registers
static const uint8_t MAX30102_PROXIMITY_INTERRUPT_THRESHOLD         =    0x30;


static const uint8_t MAX30102_SAMPLERATE_MASK                       =   0xE3;
static const uint8_t MAX30102_SAMPLERATE_50                         =   0x00;   // 50   Hz
static const uint8_t MAX30102_SAMPLERATE_100                        =   0x04;   // 100  Hz
static const uint8_t MAX30102_SAMPLERATE_200                        =   0x08;   // 200  Hz
static const uint8_t MAX30102_SAMPLERATE_400                        =   0x0C;   // 400  Hz
static const uint8_t MAX30102_SAMPLERATE_800                        =   0x10;   // 800  Hz
static const uint8_t MAX30102_SAMPLERATE_1000                       =   0x14;   // 1000 Hz
static const uint8_t MAX30102_SAMPLERATE_1600                       =   0x18;   // 1600 Hz
static const uint8_t MAX30102_SAMPLERATE_3200                       =   0x1C;   // 3200 Hz

static const uint8_t MAX30102_PULSEWIDTH_MASK                       =   0xFC;
static const uint8_t MAX30102_PULSEWIDTH_69                         =   0x00; // 69   us
static const uint8_t MAX30102_PULSEWIDTH_118                        =   0x01; // 118  us
static const uint8_t MAX30102_PULSEWIDTH_215                        =   0x02; // 215  us
static const uint8_t MAX30102_PULSEWIDTH_411                        =   0x03; // 411  us

static const uint8_t MAX30102_CURRENT_00                            =   0x00; // 0.0 mA
static const uint8_t MAX30102_CURRENT_01                            =   0x01; // 0.2 mA
static const uint8_t MAX30102_CURRENT_02                            =   0x02; // 0.4 mA
static const uint8_t MAX30102_CURRENT_0F                            =   0x0F; // 3.1 mA
static const uint8_t MAX30102_CURRENT_1F                            =   0x1F; // 6.4 mA
static const uint8_t MAX30102_CURRENT_3F                            =   0x3F; // 12.5 mA
static const uint8_t MAX30102_CURRENT_7F                            =   0x7F; // 25.4 mA
static const uint8_t MAX30102_CURRENT_FF                            =   0xFF; // 50.0 mA

class MAX30102 {
public:
   long IRlastdatapoint   = 0;      
   long REDlastdatapoint  = 0;     

  MAX30102();
  void  PXL_MAX30102_setLEDS(uint8_t pulseWidth, uint8_t red, uint8_t ir);
  void  PXL_MAX30102_setSPO2(uint8_t sampleRate);
  int   PXL_MAX30102_getNumberSamples(void);
  void  PXL_MAX30102_readSensor(void);
  void  PXL_MAX30102_shutdown(void);
  void  PXL_MAX30102_reset(void);
  void  PXL_MAX30102_startup(void);
  int   PXL_MAX30102_getPartID(void);
  void  PXL_MAX30102_begin(uint8_t pulseWidth = MAX30102_PULSEWIDTH_411, uint8_t ir = MAX30102_CURRENT_FF, uint8_t sampleRate = MAX30102_SAMPLERATE_100);
  void  PXL_MAX30102_debug(void);
  bool  PXL_MAX30102_rFIFO(uint32_t *pun_red_led,uint32_t *pun_ir_led);

private:
  void    PXL_MAX30102_I2CwBYTE(uint8_t address, uint8_t subAddress, uint8_t data);
  uint8_t PXL_MAX30102_I2CrBYTE(uint8_t address, uint8_t subAddress);
  void    PXL_MAX30102_I2CrBYTES(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);
};

