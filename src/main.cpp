#include <Arduino.h>
#include "SBUS.hpp"
#include "OrnibibBot.hpp"
#include <Wire.h>
#include "ina219.hpp"
#include <NativeEthernet.h>
#include <EthernetServer.h>

const byte mac[] = {0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF }; // MAC address of the Teensy
IPAddress ip(192, 168, 1, 100); // IP address of the Teensy
EthernetServer server(80); // Port number to listen on (e.g., 80 for HTTP)


void setup() {
  Ethernet.begin(mac, ip);
  server.begin();

}

void loop() {
  EthernetClient client = server.available();

  if (client)
  {
    // Handle the client connection
    // ...
    
    // Close the client connection
    client.stop();
  }
}

