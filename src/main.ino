
#include "math.h"
#include <TMCStepper.h>
#include <WebServer.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <iostream>
#include<array>
#include "SD.h"

using namespace std;

#include "file_functions.h"
#include "Player.h"
#include "SStepper.h"
#include "motor_control_functions.h"
#include "led_control.h"
#include "wifi_functions.h"


#define SERIAL_PORT Serial1 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f 


TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

Player player;

WebServer server(80);

String SAVED_SSID = "";
String SAVED_PWD = "";

int motor1DirPin = 27;
int motor1StepPin = 26;


int motor2DirPin = 13;
int motor2StepPin = 14;


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


void handle_status_check() {
  Serial.println("Get server status");
  
  jsonDocument.clear();  
  jsonDocument["model_name"] = "Yume Pro V0.01";
  jsonDocument["device_id"] = "unique_id_for_each_table";
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
  ssid.trim();
  pwd.trim();
  SAVED_SSID = ssid;
  SAVED_PWD = pwd;
  Serial.println(ssid);
  Serial.println(pwd);
  save_wifi_login(SD, ssid, pwd);
  Serial.println("Pairing done. Connecting.");
  server.send(200, "application/json", "Pairing done. Restarting");
  delay(2000);
  connect_to_network(SAVED_SSID, SAVED_PWD, 5);
}

void handle_play() {
  Serial.println("Play");
  String body = server.arg("plain");
  Serial.println(body);
  jsonDocument.clear();
  deserializeJson(jsonDocument, body);

  String filename = jsonDocument["filename"];
  filename.trim();

  player.read(SD, "/" + filename);
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

  player.add_to_playlist(SD, "/" + filename + ".thr");

  server.send(200, "application/json", "Add to playlist done");
}

void setup_routing(WebServer& server) {
  server.enableCORS();
  server.on("/", HTTP_GET, handle_status_check);  
  server.on("/mode", HTTP_GET, handle_get_mode);
  server.on("/pair", HTTP_POST, handle_pairing);

  server.on("/play", HTTP_POST, handle_play);
  server.on("/add_to_playlist", HTTP_POST, handle_add_to_playlist);
}

void setup() {
  delay(50);
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Hello, ESP32!");

  setup_sd_card(SD);
  list_dir(SD, "/", 0);

  Serial.println("List playlist:");
  Serial.println(player.get_playlist(SD));
  update_counter(SD);
  is_in_pairing_mode = should_reset(SD);
  if(!is_in_pairing_mode) {
    delay(5000);
  }
  clear_counter(SD);
  
  if(!is_in_pairing_mode) {
    std::array<String, 2> logins = get_wifi_login(SD);

    Serial.println("Wifi logins:");
    Serial.println(logins[0]);
    Serial.println(logins[1]);

    if( logins[0] != "" && logins[1] != "" ) {
      // Wifi login found, connect to wifi
      SAVED_SSID = logins[0];
      SAVED_PWD = logins[1];
      SAVED_SSID.trim();
      SAVED_PWD.trim();

      connect_to_network(SAVED_SSID, SAVED_PWD, 5);
    } else {
      is_in_pairing_mode = true;
    }
  }
  if(is_in_pairing_mode){
    // No wifi login found, go in pairing mode. Creating hotspot
    create_hotspot();
    Serial.println("Going in pairing mode");
  }

  setup_driver(driver, 32, 33, 25);

  setup_routing(server);
  bool has_resumed = player.read(SD);
  if(has_resumed) {
    is_printing_design = true;
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
  move_led();
  server.handleClient();
  if(is_in_pairing_mode) {
    delay(5);
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
    player.read(SD, "/" + next_design);
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
