// Developed by Vincent Claes
// twitter @claesvincent
// Thanks to Maxim for MAXREFDES117
// Thanks to Texas Instruments for CC3200XL Launchpad 
// Inspired by ProtoCentral and Maxim libraries for Arduino on Github

#include "PXL_MAX30102.h"
#include <WiFi.h>
#include <Wire.h>

MAX30102 sensor;
uint8_t data_len=8;      
uint8_t DataPacketHeader[15];
volatile unsigned int IRdata,REDdata;

char ssid[]="XXXXXXXX";
char password[]="XXXXXXXX";

IPAddress DeviceIP(192 , 168 , 43, 1);
unsigned int devicePort = 2390;
unsigned int localPort  = 2390;

WiFiUDP Udp;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);
  WiFi.begin(ssid,password);
  Serial.println("Starting...");
  while( WiFi.status() != WL_CONNECTED){
    Serial.println(".");
    delay(1000);
    WiFi.begin(ssid,password);  
  }
 
 while(WiFi.localIP() == INADDR_NONE){
  Serial.println(".");
  delay(300);  
 }
  printWifiStatus();
  //void  PXL_MAX30102_begin(uint8_t pulseWidth = MAX30102_PULSEWIDTH_411, uint8_t ir = MAX30102_CURRENT_FF, uint8_t sampleRate = MAX30102_SAMPLERATE_100);
  sensor.PXL_MAX30102_begin(MAX30102_PULSEWIDTH_411, MAX30102_CURRENT_FF, MAX30102_SAMPLERATE_100);
  Udp.begin(devicePort);
}

void loop() {
    sensor.PXL_MAX30102_readSensor();
    
    IRdata=sensor.IRlastdatapoint;              
    REDdata=sensor.REDlastdatapoint;             
    
    DataPacketHeader[0] = 0x0A;
    DataPacketHeader[1] = 0xFA;
    DataPacketHeader[2] = (uint8_t) (data_len);
    DataPacketHeader[3] = (uint8_t) (data_len>>8);
    DataPacketHeader[4] = 0x02;
    
 
    DataPacketHeader[5] = REDdata;
    DataPacketHeader[6] = REDdata>>8;
    DataPacketHeader[7] = REDdata>>16;
    DataPacketHeader[8] = REDdata>>24; 

    
    DataPacketHeader[9]  = IRdata;
    DataPacketHeader[10] = IRdata>>8;
    DataPacketHeader[11] = IRdata>>16;
    DataPacketHeader[12] = IRdata>>24; 

    DataPacketHeader[13] = 0x00;
    DataPacketHeader[14] = 0x0b;
    
    Udp.beginPacket(DeviceIP,devicePort);
    for(int i=0; i<15; i++) // transmit the data
    {
      Udp.write(DataPacketHeader[i]);
     }
    Udp.endPacket();
    delay(10);
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  Serial.print("Target IP Address: ");
  Serial.println(DeviceIP);
  
  // print target WiFi port:
  //IPAddress ip = WiFi.localIP();
  Serial.print("Target port: ");
  Serial.println(devicePort);


  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
