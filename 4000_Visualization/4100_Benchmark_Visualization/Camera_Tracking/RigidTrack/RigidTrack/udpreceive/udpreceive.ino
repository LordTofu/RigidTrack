#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
extern "C" {
  #include "user_interface.h"
  }
WiFiUDP Udp1;
  unsigned int localUdpPort = 9155;
  char incomingPacket[12];
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
}

void loop()
{
  int packetSize = Udp1.parsePacket();
  if (packetSize)
  {
    Serial.write("\n"); 
    Serial.write("n");
    Serial.write(0xFF); //id 
    Serial.write(0x24); // length, 36 bytes: 3x wgs, 3x Velocity NED, 3x Euler
    for(int i = 0; i < 20; i++) // send 0x00 for wgs and velocity x and velocity y
    {
      Serial.write(0x00);     
    }
    Udp1.read(incomingPacket, 12);
    for(int i = 0; i < 12; i++)
    {
      Serial.write(incomingPacket[i]);     
    }
    Udp1.flush();
    Serial.write(0x00);    // send 0x00 for heading 
    Serial.write(0x00);    // send 0x00 for heading
    Serial.write(0x00);    // send 0x00 for heading 
    Serial.write(0x00);    // send 0x00 for heading
    Serial.write(0x00);   //CRC Checksum, 0 for now
    Serial.write(0x00);   
  }
}
