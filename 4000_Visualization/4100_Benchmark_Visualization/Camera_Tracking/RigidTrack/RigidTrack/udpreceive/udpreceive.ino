#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

extern "C" {
  #include "user_interface.h"
  }
WiFiUDP Udp1;
  unsigned int localUdpPort = 9155;
  char hostDownlink[] = "192.168.4.5";
  char incomingPacket[36];
  char outgoingPacket[54];
  char myhostname[] = "ESP_Telemetry";
  
void setup()
{
  wifi_station_set_hostname(myhostname);
  Serial.begin(115200);
  WiFi.begin("DroneWifi", "DroneWifi");
  
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);  
  }
  Udp1.begin(localUdpPort);
  delay(5000);  
  // only for latency measurement
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

  int packetSize = Udp1.parsePacket();
  if (packetSize == 10) // Pilot command data received
  {
   
    Serial.write("\n"); 
    Serial.write("n");
    Serial.write(0xee); //id 
    Serial.write(0x0c); // length, 12 bytes: stick left up down, stick left left-right stick right ud stick right lr
    Udp1.read(incomingPacket, 10);
    Udp1.flush();
    for(int i = 0; i < 10; i++)
    {
      Serial.write(incomingPacket[i]);     
    }
    Serial.write(0x00);   //CRC Checksum, 0 for now
    Serial.write(0x00);
  }
  
  if (packetSize == 36)  // received position tracking data
  {
    Serial.write("\n"); 
    Serial.write("n");
    Serial.write(0xFF); //id 
    Serial.write(0x24); // length, 36 bytes: 3x wgs, 3x Velocity NED, 3x Euler
    Udp1.read(incomingPacket, 36);
    Udp1.flush();
    for(int i = 0; i < 36; i++)
    {
      Serial.write(incomingPacket[i]);     
    }
    Serial.write(0x00);   //CRC Checksum, 0 for now
    Serial.write(0x00);
  }
  
  if (Serial.available() > 0)
  {             
                Serial.readBytes(outgoingPacket, 2);
                if(outgoingPacket[0] == 0x0A && outgoingPacket[1] == 0x6E)
                {
                  Serial.readBytes(outgoingPacket, 52);
                  Udp1.beginPacket(hostDownlink, 9155);
                  Udp1.write(outgoingPacket, 54);
                  Udp1.endPacket();
                }
                else
                {
                  
                }
                
  }
}
