#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

WiFiUDP Udp1;
  unsigned int localUdpPort = 9155;
  char incomingPacket[48];
  

void setup()
{
  Serial.begin(115200);
  Serial.println();

  WiFi.begin("Internet", "ghetto123");

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());
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
    Serial.write(0x36);    
    Udp1.read(incomingPacket, 48);
    for(int i = 0; i < 48; i++)
    {
      Serial.write(incomingPacket[i]);     
    }
    Udp1.flush();
     Serial.write(0x00);   
     Serial.write(0x00);   
  }
}
