// fTales application by Vincent Claes - Hogeschool PXL

#include "PXL_MAX30102.h"
#include "PXL_algorithm.h"
#include <Wire.h>

uint32_t aun_ir_buffer[100];
uint32_t aun_red_buffer[100];
int32_t n_ir_buffer_length;
int32_t n_spo2;
int8_t ch_spo2_valid;
int32_t n_heart_rate;
int8_t  ch_hr_valid;
uint8_t uch_dummy;

MAX30102 sensor;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  pinMode(8, INPUT_PULLUP);
  Serial.println("In Setup...");
  sensor.PXL_MAX30102_begin(MAX30102_PULSEWIDTH_411, MAX30102_CURRENT_FF, MAX30102_SAMPLERATE_100);
}

void loop() {
   Serial.println("In Loop...");
  int32_t i;
  uint32_t un_min, un_max, un_prev_data, un_brightness; 
  n_ir_buffer_length=100;  //buffer length of 100 stores 4 seconds of samples running at 25sps
  
  //read the first 100 samples, and determine the signal range
  for(i=0;i<n_ir_buffer_length;i++)
  {
    while(digitalRead(8)==1);  //wait until the interrupt pin asserts
    sensor.PXL_MAX30102_rFIFO((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
    
    if(un_min>aun_red_buffer[i])
      un_min=aun_red_buffer[i];  //update signal min
    if(un_max<aun_red_buffer[i])
      un_max=aun_red_buffer[i];  //update signal max
    Serial.print("red=");
    Serial.print(aun_red_buffer[i], DEC);
    Serial.print(", ir=");
    Serial.println(aun_ir_buffer[i], DEC);
  }
  un_prev_data=aun_red_buffer[i];
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while(1)
  {
    i=0;
    un_min=0x3FFFF;
    un_max=0;

    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for(i=25;i<100;i++)
    {
      aun_red_buffer[i-25]=aun_red_buffer[i];
      aun_ir_buffer[i-25]=aun_ir_buffer[i];

      //update the signal min and max
      if(un_min>aun_red_buffer[i])
        un_min=aun_red_buffer[i];
      if(un_max<aun_red_buffer[i])
        un_max=aun_red_buffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for(i=75;i<100;i++)
    {
      un_prev_data=aun_red_buffer[i-1];
      while(digitalRead(8)==1);
      sensor.PXL_MAX30102_rFIFO((aun_red_buffer+i), (aun_ir_buffer+i));
      //send samples and calculation result to terminal program through UART
      Serial.print("red=");
      Serial.print(aun_red_buffer[i], DEC);
      Serial.print(", ir=");
      Serial.print(aun_ir_buffer[i], DEC);
      
      Serial.print(", HR=");
      Serial.print(n_heart_rate, DEC);
      
      Serial.print(", HRvalid=");
      Serial.print(ch_hr_valid, DEC);
      
      Serial.print(", SPO2=");
      Serial.print(n_spo2, DEC);

      Serial.print(", SPO2Valid=");
      Serial.println(ch_spo2_valid, DEC);
     // sensor.PXL_MAX30102_debug();
    }
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
  }
}
