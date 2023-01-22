/*
* ------------------------------------------------
* auther : Yi Zhang
* date   : 2021/10/1
* ------------------------------------------------
*/

#include <ros.h>
#include <WiFi.h>

//Wi-Fi configration
const char SSID[] = "Sally";
const char PASS[] = "sallywifi";
IPAddress  HOST_IP(172,20,10,5); //roscoreを実行するPCのIPアドレス//192,168,179,4 wifi IPv4 address
const uint16_t serverPort = 11411;
WiFiClient client;

class WiFiHardware {
  public:
  WiFiHardware() {};
  void init() {
    client.connect(HOST_IP, serverPort);
  }
  int read() {
    return client.read();
  }
  void write(uint8_t* data, int length) {
    for(int i=0; i<length; i++)
      client.write(data[i]);
  }
  unsigned long time() {
     return millis();
  }
};

