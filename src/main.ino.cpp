# 1 "C:\\Users\\srava\\AppData\\Local\\Temp\\tmpq34s09_7"
#include <Arduino.h>
# 1 "C:/Users/srava/Desktop/AZ/2023/notAtable/Code/not-a-table/src/main.ino"

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
#include "SStepper.h"
#include "motor_control_functions.h"
#include "wifi_functions.h"
#include "homing_functions.h"

const int dummy = 0;

#define SERIAL_PORT Serial1
#define DRIVER_ADDRESS 0b00

#define R_SENSE 0.11f



TMC2208Stepper driver(&SERIAL_PORT, R_SENSE);


Player player;

WebServer server(80);

String SAVED_SSID = "Zapp";
String SAVED_PWD = "Haweli@1504";

int motor1DirPin = 27;
int motor1StepPin = 26;
int motor1HomingPin = 22;


int motor2DirPin = 13;
int motor2StepPin = 14;
int motor2HomingPin = 21;

StaticJsonDocument<250> jsonDocument;
char buffer[250];

SStepper motor1(motor1DirPin, motor1StepPin, motor1HomingPin);
SStepper motor2(motor2DirPin, motor2StepPin, motor2HomingPin);



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
bool should_use_internal_sd = true;
bool is_storage_available = false;
void handle_status_check();
void handle_get_mode();
void handle_file_upload();
void handle_pairing();
void handle_home();
void handle_play();
void handle_add_to_playlist();
void setup_routing(WebServer& server);
void setup();
void loop();
#line 77 "C:/Users/srava/Desktop/AZ/2023/notAtable/Code/not-a-table/src/main.ino"
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
  save_wifi_login(SD, ssid, pwd);

  jsonDocument.clear();
  jsonDocument["success"] = true;

  serializeJson(jsonDocument, buffer);

  Serial.println("Pairing done. Connecting.");
  server.send(200, "application/json", buffer);
  delay(2000);
  connect_to_network(SAVED_SSID, SAVED_PWD, 5);
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
  setup_led();
  init_led();


  Serial.begin(115200);
  Serial.println("Hello, ESP32!");
  if(should_use_internal_sd == true) {
    if(setup_sd_card(SD)) {
      is_storage_available = true;
    }
  } else {
    if(setup_internal_card(SPIFFS)) {
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

  list_dir(SPIFFS, "/", 0);
# 274 "C:/Users/srava/Desktop/AZ/2023/notAtable/Code/not-a-table/src/main.ino"
  if(!is_in_pairing_mode) {
# 288 "C:/Users/srava/Desktop/AZ/2023/notAtable/Code/not-a-table/src/main.ino"
      connect_to_network(SAVED_SSID, SAVED_PWD, 5);



  }
  if(is_in_pairing_mode){

    create_hotspot();
    status_code = 2;
    Serial.println("Going in pairing mode");
  }

  sleep(1);

  setup_driver(driver, 32, 33, 25);

  setup_routing(server);




  set_led_status(status_code);
  server.begin();
  Serial.println("Server started. Listening on port 80");



  player.read(SPIFFS, "/AngularRadiance.thr.txt");
  is_printing_design = true;
}

double points[3][2] = {
  {0.0, 0.0},
  {6.28, 0.0},
  {6280.0, 50.0}
};
int current_index = 0;
void loop() {
  long current_time = micros();
  if(has_error) {
    delay(10);
    return;
  }
  server.handleClient();
  if(is_in_pairing_mode) {
    set_led_status(status_code);
    delay(5);
    return;
  }
  if(should_clear) {

    should_clear = false;
    return;
  }
  if(should_perform_homing && false) {

    Serial.println("Homing start");
    perform_homing(motor1);
    motor1.reset();
    motor2.reset();
    target_q1 = 0.0;
    target_q2 = 0.0;

    should_perform_homing = false;
    Serial.println("Homing DONE!");
    return;
  }
# 366 "C:/Users/srava/Desktop/AZ/2023/notAtable/Code/not-a-table/src/main.ino"
    move_led();

  if(is_printing_design) {
    long int delta[2] = {0, 0};
    move_arm(delta, motor1, motor2, target_q1, target_q2);
    if(abs(delta[0]) < 3 && abs(delta[1]) < 3) {
      double* points = player.next_line(SD);
      if(points[0] == 0.0) {
        is_printing_design = false;
        should_play_next = true;
        return;
      }
      target_q1 = points[0];
      target_q2 = points[1];
      current_index++;
      current_index = current_index % 3;
    }
  }




}