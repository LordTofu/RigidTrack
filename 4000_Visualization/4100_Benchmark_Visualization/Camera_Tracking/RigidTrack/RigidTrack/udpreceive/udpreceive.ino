#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
extern "C" {
  #include "user_interface.h"
  }
WiFiUDP Udp1;
  unsigned int localUdpPort = 9155;
  char incomingPacket[36];
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
    Serial.write(0xFF);
    Serial.write(0x24);    
    Udp1.read(incomingPacket, 36);
    for(int i = 0; i < 36; i++)
    {
      Serial.write(incomingPacket[i]);     
    }
    Udp1.flush();
    Serial.write(0x00);   
    Serial.write(0x00);   
  }
}
