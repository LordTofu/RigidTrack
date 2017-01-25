#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

extern "C" {
  #include "user_interface.h"
  }
WiFiUDP Udp1;
  unsigned int localUdpPort = 9155;
  unsigned int ledPin = 4;
  char hostDownlink[] = "192.168.137.205";
  char incomingPacket[16];
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
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  digitalWrite(ledPin, HIGH);
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
    for(int i = 0; i < 16; i++) // send 0x00 for wgs and velocity x and velocity y
    {
      Serial.write(0x00);     
    }
    Udp1.read(incomingPacket, 16);
    for(int i = 0; i < 16; i++)
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
    // only for latency measurement  
    digitalWrite(ledPin, LOW);   
  }
  
  if (Serial.available() > 0)
  {
                for(int i = 0; i < 54; i++) // 
                {
                  outgoingPacket[i] = Serial.read();     
                }
                Udp1.beginPacket(hostDownlink, 9155);
                Udp1.write(outgoingPacket, 54);
                Udp1.endPacket();
  }
}
