#include "WiFi.h"
#include <HTTPClient.h>
#include "iot47_esp32_w5500.h"

#define W5500_CS    15
#define W5500_INT   4
#define W5500_RST   18
#define W5500_MOSI  13
#define W5500_MISO  12
#define W5500_SCLK  14


bool eth_connected=false;

void ETH_gotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.println("Ethernet Got IP Addres");
    Serial.print("ETH IP address: ");Serial.println(IPAddress(info.got_ip.ip_info.ip.addr));
    Serial.print("ETH Subnet: ");Serial.println(IPAddress(info.got_ip.ip_info.netmask.addr));
    Serial.print("ETH Gateway: ");Serial.println(IPAddress(info.got_ip.ip_info.gw.addr));
    eth_connected = true;
}
void ETH_event(WiFiEvent_t event)
{
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}

void setup() 
{
  Serial.begin(115200);

  WiFi.onEvent(ETH_event);
  WiFi.onEvent(ETH_gotIP, WiFiEvent_t::ARDUINO_EVENT_ETH_GOT_IP);
  
  // WiFi.mode(WIFI_STA);
  // WiFi.begin("wifi", "pass");
  // Serial.print("Connecting to WiFi ..");
  // while (WiFi.status() != WL_CONNECTED) {
  //   Serial.print('.');
  //   delay(1000);
  // }
  // Serial.println(WiFi.localIP());

  uint8_t base_mac_addr[6];
  esp_efuse_mac_get_default(base_mac_addr); base_mac_addr[4]++;base_mac_addr[5]++;
  w5500_set_mac(base_mac_addr);
  w5500_set_statc_ip("192.168.1.13","192.168.1.1","255.255.255.0","8.8.8.8","8.8.4.4");
  w5500_begin(W5500_MOSI,W5500_MISO,W5500_SCLK,W5500_CS,W5500_RST,W5500_INT);
}

void loop() 
{
  if(eth_connected)
  {
     HTTPClient http;
      
      // Your Domain name with URL path or IP address with path
      http.begin("http://i47tserver.top/time.php");
      
      // If you need Node-RED/server authentication, insert user and password below
      //http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");
      
      // Send HTTP GET request
      int httpResponseCode = http.GET();
      
      if (httpResponseCode>0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();

      delay(1000);
  }
}
