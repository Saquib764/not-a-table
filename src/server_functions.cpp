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
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    IP = WiFi.localIP();
    Serial.println("WiFi connected.");
  }

  Serial.println("");
  Serial.println("IP address: ");
  Serial.println(IP);
}

std::array<String, 2> get_wifi_login(fs::FS &fs) {
  bool is_file = fs.exists( "/wifi_login.txt" );
  if(!is_file) {
    Serial.println("File does not exist");
    return {"", ""};
  }
  File file = fs.open( "/wifi_login.txt");
  if(!file) {
    Serial.println("Failed to open login file for reading");
    return {"", ""};
  }
  String ssid = file.readStringUntil('\n');
  String pwd = file.readStringUntil('\n');
  return {ssid, pwd};
}
void save_wifi_login(fs::FS &fs, String ssid, String pwd) {
  File file = fs.open( "/wifi_login.txt" , FILE_WRITE);
  if(!file){
    Serial.println("Failed to open login file for writing");
    return;
  }
  file.println(ssid);
  file.println(pwd);
  file.close();
}