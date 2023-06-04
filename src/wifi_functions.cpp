#include "wifi_functions.h"
#include "file_functions.h"

String SSID = "NOT_A_TABLE";
String PWD = "i_am_a_table";
String REGISTRY_URL = "http://192.168.1.104:3003/api/1/register";

// Define array of 15 possible static IP addresses
int static_ip[15][4] = {
  {192, 168, 1, 106},
  {192, 168, 1, 117},
  {192, 168, 4, 138},
  {192, 168, 190, 127},
};

std::array<String, 2> get_wifi_login(fs::FS &fs) {
  File file = read_file( fs, "/wifi_login.txt");
  String ssid = file.readStringUntil('\n');
  String pwd = file.readStringUntil('\n');
  return {ssid, pwd};
}

void save_wifi_login(fs::FS &fs, String ssid, String pwd) {
  File file = open_file(fs, "/wifi_login.txt", FILE_WRITE);
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
  return WiFi.disconnect() || WiFi.softAPdisconnect();
}

bool connect_to_network(String ssid, String pwd, int n_try) {
  WiFi.mode(WIFI_STA);
  const char* ssid_c = ssid.c_str();
  const char* pwd_c = pwd.c_str();
  if(check_if_connected_to_network(ssid)) {
    return true;
  }
  if(check_if_connected_to_network()) {
    disconnect_from_network();
  }

  // connect to network with Static IP
  for(int j = 0; j < 5; j++) {
    if(j < 4) {
      // Static IP
      WiFi.config(static_ip[j][0], static_ip[j][1], static_ip[j][2], static_ip[j][3]);
    }else{
      // Dynamic IP
      WiFi.config(0U, 0U, 0U, 0U);
    }
    WiFi.begin(ssid_c, pwd_c);
    for(int i = 0; i < n_try; i++) {
      if(WiFi.status() == WL_CONNECTED) {
        Serial.print("WiFi connected: ");
        Serial.println(ssid_c);
        Serial.println(WiFi.localIP());
        // Send IP and mac address to resistry server via a post request
        HTTPClient http;
        http.begin(REGISTRY_URL);
        http.addHeader("Content-Type", "application/json");
        String body = "{\"ip\":\"" + WiFi.localIP().toString() + "\",\"mac\":\"" + WiFi.macAddress() + "\"}";
        int httpCode = http.POST(body);
        String payload = http.getString();
        Serial.println(httpCode);
        return true;
      }
      delay(1000);
      Serial.print(".");
    }
  }
  Serial.println("Failed to connect to WiFi.");
  return false;
}

bool create_hotspot() {
  String ssid = SSID;
  String pwd = PWD;
  WiFi.mode(WIFI_AP);
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


void update_counter(fs::FS &fs) {
  File file = open_file(fs, "/restart_counter.txt", FILE_APPEND);
  file.print("1");
  file.close();
}
bool should_reset(fs::FS &fs) {
  File file = read_file(fs, "/restart_counter.txt");
  String starts = file.readStringUntil('\n');
  file.close();
  starts.trim();
  Serial.print("starts: ");
  Serial.println(starts);
  return starts == "111";
}

void clear_counter(fs::FS &fs) {
  File file = open_file(fs, "/restart_counter.txt", FILE_WRITE);
  file.print("");
  file.close();
}
