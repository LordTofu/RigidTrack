#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

extern "C" {
  #include "user_interface.h"
  }
WiFiUDP Udp1;
  unsigned int localUdpPort = 9155;
  char incomingPacket[24];
  uint16_t CRC = 0;
  uint8_t NavCRC[26];
  uint8_t PilotCRC[22];
  int packetSize = 0;
  char myhostname[] = "ESP_Telemetry";
  
void setup()
{
  Serial.begin(115200);

  WiFi.softAP("DroneWifi", "DroneWifi");
 
  Udp1.begin(localUdpPort);
  
  NavCRC[0] = 0xFF;
  NavCRC[1] = 24;
  PilotCRC[0] = 0xEE;
  PilotCRC[1] = 20;
}

#define poly 0x11021

uint16_t calculateCRC(uint8_t *packet, uint8_t size) {
  uint32_t crc = 0;

  for (uint8_t n=0; n<size; n++) /* Step through bytes in memory */
  {
    crc = crc ^ (*packet++ << 8); /* Fetch byte from memory, XOR into CRC top byte*/
    for (uint8_t i = 0; i < 8; i++) /* Prepare to rotate 8 bits */
    {
      crc = crc << 1; /* rotate */
      if (crc & 0x10000) /* bit 15 was set (now bit 16)... */
        crc ^= poly;
      /* and ensure CRC remains 16-bit value */
    } /* Loop for 8 bits */
  } /* Loop until num=0 */
  return (crc); /* Return updated CRC */
}

void loop()
{
  packetSize = Udp1.parsePacket();
  if (packetSize == 24)  // received position tracking data
  {
    Serial.write("\n"); 
    Serial.write("n");
    Serial.write(0xFF); //id 
    Serial.write(0x18); // length, 24 bytes: 3x ned position, 3x Euler
    Udp1.read(incomingPacket, 24);
    for(int i = 0; i < 6; i++)
    {
      Serial.write(incomingPacket[i*4+3]);  
      Serial.write(incomingPacket[i*4+2]); 
      Serial.write(incomingPacket[i*4+1]); 
      Serial.write(incomingPacket[i*4]);
      NavCRC[2+i*4]   = incomingPacket[i*4+3];
      NavCRC[2+i*4+1] = incomingPacket[i*4+2];
      NavCRC[2+i*4+2] = incomingPacket[i*4+1];
      NavCRC[2+i*4+3] = incomingPacket[i*4];
    }
    CRC = calculateCRC(NavCRC, 26);
    unsigned char const * p = reinterpret_cast<unsigned char const *>(&CRC);
    Serial.write(p[0]);
    Serial.write(p[1]);
  }

  if (packetSize == 20) // Pilot command data received
  {
   
    Serial.write("\n"); 
    Serial.write("n");
    Serial.write(0xee); //id 
    Serial.write(0x14); // length, 20 bytes: stick left up down, stick left left-right stick right ud stick right lr and state command
    Udp1.read(incomingPacket, 20);
    for(int i = 0; i < 8; i++)
    {
      Serial.write(incomingPacket[i*2+1]); 
      Serial.write(incomingPacket[i*2]);
      PilotCRC[2+i*2]   = incomingPacket[i*2+1];
      PilotCRC[2+i*2+1] = incomingPacket[i*2];
    }
    for(int i = 8; i < 9; i++)
    {
      Serial.write(incomingPacket[i*4+3]);  
      Serial.write(incomingPacket[i*4+2]); 
      Serial.write(incomingPacket[i*4+1]); 
      Serial.write(incomingPacket[i*4]);
      PilotCRC[2+i*4]   = incomingPacket[i*4+3];
      PilotCRC[2+i*4+1] = incomingPacket[i*4+2];
      PilotCRC[2+i*4+2] = incomingPacket[i*4+1];
      PilotCRC[2+i*4+3] = incomingPacket[i*4];
    }
    
    CRC = calculateCRC(PilotCRC, 22);
    unsigned char const * p = reinterpret_cast<unsigned char const *>(&CRC);
    Serial.write(p[0]);
    Serial.write(p[1]);
  }
  
  Udp1.flush();
}
