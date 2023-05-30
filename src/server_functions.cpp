#include "server_functions.h"
#include "file_functions.h"

void setup_wifi(String ssid, String pwd, bool set_as_hotspot) {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();

  IPAddress IP;
  if(set_as_hotspot) {
    Serial.print("Creating hotspot");
    Serial.println(ssid);
    WiFi.softAP(ssid, pwd);
    IP = WiFi.softAPIP();
    Serial.print("Hotspot created. ");
  } else {
    Serial.print("Connecting to hotspot : ");
    WiFi.mode(WIFI_STA);
    ssid.trim();
    pwd.trim();
    const char* ssid_c = ssid.c_str();
    const char* pwd_c = pwd.c_str();
    WiFi.begin(ssid_c, pwd_c);
    int i = 0;
    while (WiFi.status() != WL_CONNECTED && i < 100) {
      delay(500);
      Serial.print(".");
      i++;
    }
    IP = WiFi.localIP();
    Serial.println("WiFi connected.");
  }

  Serial.println("");
  Serial.println("IP address: ");
  Serial.println(IP);
}
