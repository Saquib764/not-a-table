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

std::array<String, 2> get_wifi_login(Preferences &preferences) {
  String ssid = preferences.getString("ssid", "");
  String pwd = preferences.getString("pwd", "");
  return {ssid, pwd};
}

void save_wifi_login(Preferences &preferences , String ssid, String pwd) {
  preferences.putString("ssid", ssid);
  preferences.putString("pwd", pwd);
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
  Serial.println("Connecting to WiFi... 50");
  WiFi.mode(WIFI_STA);
  const char* ssid_c = ssid.c_str();
  const char* pwd_c = pwd.c_str();
  if(check_if_connected_to_network(ssid)) {
  Serial.println("Connecting to WiFi... 55");
    return true;
  }
  Serial.println("Connecting to WiFi... 58");
  if(check_if_connected_to_network()) {
  Serial.println("Connecting to WiFi... 60");
    disconnect_from_network();
  }
  // Connect to network in Dynamic IP mode to get network parameter

  Serial.println("Connecting to WiFi... 65");
  WiFi.begin(ssid_c, pwd_c);
  Serial.println("Connecting to WiFi... 67");
  for(int i = 0; i < n_try; i++) {
    Serial.println("Connecting to WiFi... 69: " + String(i) + " Status: " + String(WiFi.status()));
    if(WiFi.status() == WL_CONNECTED) {
      Serial.print("WiFi connected: ");
      Serial.println(ssid_c);
      Serial.println(WiFi.localIP());
      break;
    }
    delay(1000);
    Serial.print(".");
  }
  // Get network parameters IP, gateway, subnet, DNS
  IPAddress ip = WiFi.localIP();
  IPAddress gateway = WiFi.gatewayIP();
  IPAddress subnet = WiFi.subnetMask();
  IPAddress dns = WiFi.dnsIP();
  // Disconnect from network
  WiFi.disconnect();


  // connect to network with Static IP
  for(int j = 0; j < 5; j++) {
    if(j < 4) {
      // Static IP
      IPAddress _ip(static_ip[j][0], static_ip[j][1], static_ip[j][2], static_ip[j][3]);
      if(!WiFi.config(_ip, gateway, subnet, dns)) {
        continue;
      }
    }else{
      // Dynamic IP
      if(!WiFi.config(0U, 0U, 0U)) {
        continue;
      }
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
  // Append counter to file
  File file = open_file(fs, "/restart_counter.txt", FILE_APPEND);
  file.print("1");
  file.close();
}
bool should_reset(fs::FS &fs) {
  File file = open_file(fs, "/restart_counter.txt", FILE_READ);
  String counter = file.readString();
  counter.trim();
  file.close();
  return counter == "111";
}

void clear_counter(fs::FS &fs) {
  File file = open_file(fs, "/restart_counter.txt", FILE_WRITE);
  file.print("");
  file.close();
}

void save_admin_secret(fs::FS &fs) {
  File file = open_file(fs, "/admin_secret.txt", FILE_WRITE);
  file.print(random(100000, 999999));
  file.close();
}

String get_admin_secret(fs::FS &fs) {
  File file = open_file(fs, "/admin_secret.txt", FILE_READ);
  String secret = file.readString();
  Serial.println("ADMIN: " +secret);
  secret.trim();
  file.close();
  return secret;
}

bool is_admin_secret_correct(fs::FS &fs, String secret) {
  File file = open_file(fs, "/admin_secret.txt", FILE_READ);
  String correct_secret = file.readString();
  correct_secret.trim();
  file.close();
  return secret == correct_secret;
}

void save_admin_user(fs::FS &fs, String user_id) {
  File file = open_file(fs, "/admin_user.txt", FILE_WRITE);
  file.print(user_id);
  file.close();
  // Delete all paired users
  file = open_file(fs, "/paired_users.txt", FILE_WRITE);
  file.print("");
  file.close();
}

bool is_admin_user(fs::FS &fs, String user_id) {
  File file = open_file(fs, "/admin_user.txt", FILE_READ);
  String line = file.readString();
  line.trim();
  if(line == user_id) {
    file.close();
    return true;
  }
  file.close();
  return false;
}

void save_paired_user(fs::FS &fs, String user_id) {
  File file = open_file(fs, "/paired_users.txt", FILE_APPEND);
  file.print(user_id + "\n");
  file.close();
}

bool is_paired_user(fs::FS &fs, String user_id) {
  File file = open_file(fs, "/paired_users.txt", FILE_READ);
  String line;
  while(file.available()) {
    line = file.readStringUntil('\n');
    line.trim();
    if(line == user_id) {
      file.close();
      return true;
    }
  }
  file.close();
  return false;
}

String get_paired_users(fs::FS &fs) {
  File file = open_file(fs, "/paired_users.txt");
  String users = file.readString();
  users.trim();
  file.close();
  return users;
}
