
#include "math.h"
#include <TMCStepper.h>
#include <WebServer.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <iostream>
#include<array>
#include "SD.h"

using namespace std;

#include "Player.h"
#include "SStepper.h"
#include "server_functions.h"
#include "motor_control_functions.h"
#include "led_control.h"



#define SERIAL_PORT Serial1 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f 


TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

Player player;

WebServer server(80);


int motor1DirPin = 27;
int motor1StepPin = 26;


int motor2DirPin = 13;
int motor2StepPin = 14;


String SSID = "NOT_A_TABLE";
String PWD = "i_am_a_table";


StaticJsonDocument<250> jsonDocument;
char buffer[250];

SStepper motor1(motor1DirPin, motor1StepPin);
SStepper motor2(motor2DirPin, motor2StepPin);

// Scara scara(motor1, motor2)

float Q1 = 0.0;
float Q2 = 0.0;


float ARM1 = 0.25;
float ARM2 = 0.25;

bool is_printing_design = false;
bool is_in_pairing_mode = true;
bool should_clear = false;
bool should_perform_homing = false;
bool should_play_next = false;
bool has_error = false;


void setup_sd_card() {
    if(!SD.begin(5)){
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.name(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}
void handle_status_check() {
  Serial.println("Get server status");
  
  jsonDocument.clear();  
  jsonDocument["type"] = "Running";
  jsonDocument["value"] = 200;
  jsonDocument["unit"] = true;
  serializeJson(jsonDocument, buffer);
  
  server.send(200, "application/json", buffer);
}
void handle_get_mode() {
  Serial.println("Get server mode");
  
  jsonDocument.clear();  
  if(is_in_pairing_mode) {
    jsonDocument["mode"] = "pairing";
  } else {
    jsonDocument["mode"] = "running";
  }
  serializeJson(jsonDocument, buffer);
  
  server.send(200, "application/json", buffer);
}

void handle_pairing() {
  Serial.println("Pairing");
  String body = server.arg("plain");
  Serial.println(body);
  jsonDocument.clear();
  deserializeJson(jsonDocument, body);
  String ssid = jsonDocument["ssid"];
  String pwd = jsonDocument["pwd"];
  Serial.println(ssid);
  Serial.println(pwd);
  save_wifi_login(SD, ssid, pwd);
  Serial.println("Pairing done. Restarting");
  server.send(200, "application/json", "Pairing done. Restarting");
  ESP.restart();
}

void handle_play() {
  Serial.println("Play");
  String body = server.arg("plain");
  Serial.println(body);
  jsonDocument.clear();
  deserializeJson(jsonDocument, body);

  String filename = jsonDocument["filename"];

  player.read(SD, "/" + filename + ".txt");
  is_printing_design = true;
  should_clear = true;
  should_perform_homing = true;

  server.send(200, "application/json", "Start done");
}
void handle_add_to_playlist() {
  Serial.println("Add to playlist");
  String body = server.arg("plain");
  Serial.println(body);
  jsonDocument.clear();
  deserializeJson(jsonDocument, body);

  String filename = jsonDocument["filename"];

  player.add_to_playlist(SD, "/" + filename + ".txt");

  server.send(200, "application/json", "Add to playlist done");
}


void set_routing_common(WebServer& server) {
  server.on("/", HTTP_GET, handle_status_check);  
  server.on("/mode", HTTP_GET, handle_get_mode);
}

void setup_pairing_routing(WebServer& server) {
  set_routing_common(server);
  server.on("/pair", HTTP_POST, handle_pairing);
}
void setup_paired_routing(WebServer& server) {
  set_routing_common(server);
  server.on("/play", HTTP_POST, handle_play);
  server.on("/add_to_playlist", HTTP_POST, handle_add_to_playlist);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Hello, ESP32!");

  setup_sd_card();
  listDir(SD, "/", 0);

  Serial.println("List playlist:");
  Serial.println(player.get_playlist(SD));

  std::array<String, 2> logins = get_wifi_login(SD);

  Serial.println("Wifi logins:");
  Serial.println(logins[0]);
  Serial.println(logins[1]);

  if(logins[0] != "" && logins[1] != "") {
    SSID = logins[0];
    PWD = logins[1];
    
    is_in_pairing_mode = false;

    setup_driver(driver, 32, 33, 25);
  } else {
    Serial.println("No wifi login found");
    Serial.println("Going in pairing mode");
  }

  setup_wifi(SSID, PWD, is_in_pairing_mode);
  if(is_in_pairing_mode){
    setup_pairing_routing(server);
  }else{
    setup_paired_routing(server);
    bool has_resumed = player.read(SD);
    if(has_resumed) {
      is_printing_design = true;
    }
  }
  setup_led();
  server.begin();
  Serial.println("Server started. Listening on port 80");
}

void loop() {
  if(has_error) {
    delay(5000);
    return;
  }
  test_led();
  server.handleClient();
  if(is_in_pairing_mode) {
    delay(500);
    return;
  }
  if(should_clear) {
    // Clear the table
    should_clear = false;
    return;
  }
  if(should_perform_homing) {
    // Perform homing
    should_perform_homing = false;
    return;
  }
  if(should_play_next) {
    // Play next design
    String next_design = player.get_next_design(SD);
    player.read(SD, "/" + next_design + ".txt");
    is_printing_design = true;
    should_clear = true;
    should_perform_homing = true;
    should_play_next = false;
    return;
  }

  if(is_printing_design) {
    double* points = player.next_line(SD);
    if(points[0] == 0.0) {
      is_printing_design = false;
      should_play_next = true;
      return;
    }
    double q1 = points[1];
    double q2 = points[2];
    
    move_arm(motor1, motor2, q1 - Q1, q2 - Q2);
    Q1 = q1;
    Q2 = q2;
  }
  delay(500);
}
