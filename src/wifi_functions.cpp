#include "server_functions.h"
#include "file_functions.h"

String SSID = "NOT_A_TABLE";
String PWD = "i_am_a_table";

std::array<String, 2> get_wifi_login(fs::FS &fs) {
  File file = read_file( fs, "/wifi_login.txt");
  String ssid = file.readStringUntil('\n');
  String pwd = file.readStringUntil('\n');
  return {ssid, pwd};
}

void save_wifi_login(fs::FS &fs, String ssid, String pwd) {
  File file = open_file(fs, "/wifi_login.txt", true);
  file.println(ssid);
  file.println(pwd);
  file.close();
}

bool check_if_network_is_available(String ssid) {
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; ++i) {
    if(WiFi.SSID(i) == ssid) {
      return true;
    }
  }
  return false;
}
bool check_if_connected_to_network() {
  return WiFi.status() == WL_CONNECTED;
}
bool check_if_connected_to_network(String ssid) {
  if(check_if_connected_to_network()) {
    return WiFi.SSID() == ssid;
  }
  return false;
}
bool disconnect_from_network() {
  return WiFi.disconnect();
}

bool connect_to_network(String ssid, String pwd, int n_try) {
  WiFi.mode(WIFI_STA);
  ssid.trim();
  pwd.trim();
  const char* ssid_c = ssid.c_str();
  const char* pwd_c = pwd.c_str();
  if(check_if_connected_to_network(ssid)) {
    return true;
  }
  if(check_if_connected_to_network()) {
    disconnect_from_network();
  }
  WiFi.begin(ssid_c, pwd_c);
  for(int i = 0; i < n_try; i++) {
    if(WiFi.status() == WL_CONNECTED) {
      Serial.print("WiFi connected: ");
      Serial.println(ssid_c);
      Serial.println(WiFi.localIP());
      return true;
    }
    delay(100);
    Serial.print(".");
  }
  Serial.println("Failed to connect to WiFi.");
  return false;
}

bool create_hotspot() {
  String ssid = SSID;
  String pwd = PWD;
  WiFi.mode(WIFI_AP);
  ssid.trim();
  pwd.trim();
  const char* ssid_c = ssid.c_str();
  const char* pwd_c = pwd.c_str();
  WiFi.softAP(ssid_c, pwd_c);
  Serial.print("Hotspot created. ");
  Serial.println(ssid);
  Serial.println(WiFi.softAPIP());
  return true;
}
bool check_if_mode_is_pairing() {
  return check_if_connected_to_network(SSID);
}
