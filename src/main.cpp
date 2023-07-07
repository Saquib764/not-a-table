
#include <Arduino.h>
#include "led_control.h"
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
#include "motor_control_functions.h"
#include "wifi_functions.h"
#include "ota.h"
#include <Preferences.h>

const int dummy = 0;

#define SERIAL_PORT Serial2 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f
#define VERSION "1.0.0"



// TMC2209Stepper driver(&SERIAL_PORT, R_SENSE);
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

Player player;

WebServer server(80);

Preferences preferences;

String SAVED_SSID = "Zapp";
String SAVED_PWD = "Haweli@1504";

int EN_PIN = 25;
uint8_t motor1DirPin = 26;
uint8_t motor1StepPin = 27;
uint8_t motor1HomingPin = 32;


uint8_t motor2DirPin = 14;
uint8_t motor2StepPin = 13;
uint8_t motor2HomingPin = 33;

StaticJsonDocument<250> jsonDocument;
char buffer[250];

double target_q1 = 0.0;
double target_q2 = 0.0;


float ARM1 = 0.25;
float ARM2 = 0.25;

bool is_printing_design = false;
bool is_in_pairing_mode = false;
bool should_clear = false;
bool should_perform_homing = true;
bool should_play_next = false;
bool has_error = false;
int status_code = 0;
bool is_uploading = false;
bool should_use_internal_sd = false;
bool is_storage_available = false;
bool should_use_homing = true;


void handle_status_check() {
  Serial.println("Get server status");
  
  jsonDocument.clear();  
  jsonDocument["model_name"] = "Yume Pro V0.01";
  jsonDocument["id"] = "unique_id_for_each_table";
  jsonDocument["type"] = "Running";
  jsonDocument["value"] = 200;
  jsonDocument["unit"] = true;
  jsonDocument["has_error"] = has_error;
  jsonDocument["status_code"] = status_code;
  jsonDocument["SSID"] = WiFi.SSID();
  jsonDocument["mac"] = WiFi.macAddress();
  jsonDocument["ip"] = WiFi.localIP().toString();
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

void handle_file_upload() {
  Serial.println("File upload");
  is_uploading = true;
  String filename = server.arg("filename");
  filename.trim();
  Serial.println(filename);
  File file = SD.open("/" + filename, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(server.hasArg("plain")) {
    file.print(server.arg("plain"));
  } else {
    Serial.println("No file to upload");
  }
  file.close();
  is_uploading = false;
  jsonDocument.clear();  
  jsonDocument["success"] = true;
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
  save_wifi_login(preferences, ssid, pwd);
  
  jsonDocument.clear();  
  jsonDocument["success"] = true;

  serializeJson(jsonDocument, buffer);

  Serial.println("Pairing done. Connecting.");
  server.send(200, "application/json", buffer);
  delay(2000);
  ESP.restart();
}

void handle_update() {
  Serial.println("Updating..");
  String body = server.arg("plain");
  Serial.println(body);
  jsonDocument.clear();
  deserializeJson(jsonDocument, body);
  String update_url = jsonDocument["update_url"];
  update_url.trim();
  Serial.println(update_url);
  
  jsonDocument.clear();  

  bool is_success = update_firmware(update_url);
  jsonDocument["success"] = is_success;

  serializeJson(jsonDocument, buffer);

  server.send(200, "application/json", buffer);
  delay(2000);
  if(is_success) {
    ESP.restart();
  }
}

void handle_home() {
  Serial.println("Home");
  should_perform_homing = true;

  jsonDocument.clear();  
  jsonDocument["success"] = true;

  serializeJson(jsonDocument, buffer);

  server.send(200, "application/json", buffer);
}

void handle_play() {
  Serial.println("Play");
  String body = server.arg("plain");
  Serial.println(body);
  jsonDocument.clear();
  deserializeJson(jsonDocument, body);

  String filename = jsonDocument["filename"];
  filename.trim();

  player.read(SPIFFS, "/" + filename);
  is_printing_design = true;
  should_clear = true;
  // should_perform_homing = true;

  jsonDocument.clear();  
  jsonDocument["success"] = true;

  serializeJson(jsonDocument, buffer);

  server.send(200, "application/json", buffer);
}
void handle_add_to_playlist() {
  Serial.println("Add to playlist");
  String body = server.arg("plain");
  Serial.println(body);
  jsonDocument.clear();
  deserializeJson(jsonDocument, body);

  String filename = jsonDocument["filename"];

  player.add_to_playlist(SD, "/" + filename + ".thr");
  
  jsonDocument.clear();  
  jsonDocument["success"] = true;

  serializeJson(jsonDocument, buffer);

  server.send(200, "application/json", buffer);
}

void setup_routing(WebServer& server) {
  server.enableCORS();
  server.on("/", HTTP_GET, handle_status_check);  
  server.on("/mode", HTTP_GET, handle_get_mode);
  server.on("/home", HTTP_GET, handle_home);


  server.on("/update", HTTP_POST, handle_update);
  server.on("/update", HTTP_OPTIONS, handle_status_check);

  server.on("/pair", HTTP_POST, handle_pairing);
  server.on("/pair", HTTP_OPTIONS, handle_status_check);
  server.on("/design/upload", HTTP_POST, handle_file_upload);
  server.on("/design/upload", HTTP_OPTIONS, handle_status_check);

  server.on("/play", HTTP_POST, handle_play);
  server.on("/play", HTTP_OPTIONS, handle_status_check);
  server.on("/add_to_playlist", HTTP_POST, handle_add_to_playlist);
  server.on("/add_to_playlist", HTTP_OPTIONS, handle_status_check);
}

void setup() {
  preferences.begin("yume", false); 
  setup_led();
  init_led();
  delay(50);
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Version: " + String(VERSION));
  if(should_use_internal_sd) {
    if(setup_internal_card(SPIFFS)) {
      is_storage_available = true;
    }
  } else {
    if( setup_sd_card(SD)) {
      is_storage_available = true;
    }
  }

  if(is_storage_available) {
    Serial.println("Storage initialized.");
    has_error = false;
    status_code = 0;
  }else{
    Serial.println("No storage available.");
    has_error = true;
    status_code = 1;
    delay(100);
  }
  
  set_led_status(status_code);
  if(has_error) {
    return;
  }
  
  list_dir(SD, "/", 0);

  update_counter(SD);
  is_in_pairing_mode = should_reset(SD);
  if(!is_in_pairing_mode) {
    delay(5000);
  }
  clear_counter(SD);

  // Serial.println("List playlist:");
  // Serial.println(player.get_playlist(SD));
  
  if(!is_in_pairing_mode) {
    std::array<String, 2> logins = get_wifi_login(preferences);

    Serial.println("Wifi logins:");
    Serial.println(logins[0]);
    Serial.println(logins[1]);

    if( logins[0] != "" && logins[1] != "" ) {
      // Wifi login found, connect to wifi
      SAVED_SSID = logins[0];
      SAVED_PWD = logins[1];
      SAVED_SSID.trim();
      SAVED_PWD.trim();

      connect_to_network( SAVED_SSID, SAVED_PWD, 5);
    } else {
      is_in_pairing_mode = true;
    }
  }
  if(is_in_pairing_mode){
    // No wifi login found, go in pairing mode. Creating hotspot
    create_hotspot();
    status_code = 2;
    Serial.println("Going in pairing mode");
  }

  sleep(1);

  setup_driver(driver, EN_PIN);

  setup_routing(server);
  // bool has_resumed = player.read(SD);
  // if(has_resumed) {
  //   is_printing_design = false;
  // }
  server.begin();
  set_led_status(status_code);
  Serial.println("Server started. Listening on port 80");

  // Remove this
  player.read(SD, "/test_designs/spiral.thr.txt");
  is_printing_design = true;

  setup_arm(EN_PIN, motor1DirPin, motor1StepPin, motor1HomingPin, motor2DirPin, motor2StepPin, motor2HomingPin);
}

// double points[5][3] = {
//   {0., 0.0, 0.0},
//   {0., 0.1*PI, 0*PI},
//   {0., 0.2*PI, 0*PI},
//   {0., 0.3*PI, 0*PI},
//   {0., 0.4*PI, 0*PI},
// };
int current_index = 0;
void loop() {
  long current_time = micros();
  // if(has_error) {
  //   delay(10);
  //   return;
  // }
  server.handleClient();
  if(is_in_pairing_mode) {
    set_led_status(status_code);
    delay(5);
    return;
  }
  // if(should_clear) {
  //   // Clear the table
  //   should_clear = false;
  //   return;
  // }
  if(should_perform_homing) {
    // Perform homing
    Serial.println("Homing start");
    home_arm();
    target_q1 = 0.0;
    target_q2 = 0.0;
    should_perform_homing = false;
    Serial.println("Homing DONE!");
    return;
  }
  // if(should_play_next) {
  //   // Play next design
  //   String next_design = player.get_next_design(SD);
  //   player.read(SD, "/" + next_design);
  //   is_printing_design = true;
  //   should_clear = true;
  //   should_perform_homing = true;
  //   should_play_next = false;
  //   return;
  // }
  EVERY_N_MILLISECONDS(25) {
    move_led();
  }
  EVERY_N_MILLISECONDS(4) {
    if(is_printing_design) {
      bool should_read_next = follow_trajectory();
      // long int delta[2] = {0, 0};
      // bool should_read_next = move_arm(delta, target_q1, target_q2);
      if(should_read_next) {
        double* points = player.next_line(SD);
        if(points[0] == 0.0) {
          is_printing_design = false;
          add_target_to_trajectory(0.0, 0.0);
          // should_play_next = true;
          return;
        }
        target_q1 = points[1];
        target_q2 = points[2];
        add_target_to_trajectory(target_q1, target_q2);
        // target_q1 = points[current_index][1];
        // target_q2 = points[current_index][2];
        // current_index = (current_index + 1) % 5;
      }
    }
  }
  // delay(300);
  // Serial.print("Time for servo run: ");
  EVERY_N_MILLISECONDS(2070){
    Serial.println("Time: " + String(micros() - current_time));
  }
  // Serial.print("  ");
}
