
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

#include "../common/driver_setup.cpp"
#include "../common/arm_model.cpp"
#include "../common/arm_controller.cpp"
#include "file_functions.h"
#include "Player.h"
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

#define K     STEPS_PER_REV * MICROSTEPS/ (2.0*PI)
#define ARM     0.63/2

ArmModel *arm = new ArmModel(ARM, K);

ArmController *controller = new ArmController(arm);

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

StaticJsonDocument<550> jsonDocument;
char buffer[550];

double target_q1 = 0.0;
double target_q2 = 0.0;

uint8_t led_color[4] = {255, 255, 255, 255};


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


void handle_options_call() {
  jsonDocument.clear();  
  jsonDocument["status"] = "ok";
  serializeJson(jsonDocument, buffer);
  server.send(200, "application/json", buffer);
}

// 1. Get device info
void handle_info() {
  Serial.println("Get server status");

  // param user_id
  String user_id = server.arg("user_id");
  
  jsonDocument.clear();  
  jsonDocument["model_name"] = "Yume Pro V0.01";
  jsonDocument["software_version"] = "V0.01";
  jsonDocument["id"] = get_device_id(SD);
  jsonDocument["has_error"] = has_error;
  jsonDocument["status_code"] = status_code;
  jsonDocument["SSID"] = WiFi.SSID();
  jsonDocument["mac"] = WiFi.macAddress();
  jsonDocument["ip"] = WiFi.localIP().toString();
  if(is_in_pairing_mode) {
    jsonDocument["mode"] = "pairing";
  } else {
    jsonDocument["mode"] = "running";
  }
  if(is_printing_design) {
    jsonDocument["is_printing_design"] = true;
  } else {
    jsonDocument["is_printing_design"] = false;
  }
  if(should_perform_homing) {
    jsonDocument["is_performing_homing"] = true;
  } else {
    jsonDocument["is_performing_homing"] = false;
  }
  if(is_admin_user(SD, user_id)) {
    jsonDocument["is_admin"] = true;
    // admin_secret
    jsonDocument["admin_secret"] = get_admin_secret(SD);
  } else {
    jsonDocument["is_admin"] = false;
  }
  serializeJson(jsonDocument, buffer);
  
  server.send(200, "application/json", buffer);
}

// 2. Restart device
void handle_restart() {
  Serial.println("Restarting");
  led_color[0] = static_cast<uint8_t>(0);
  led_color[1] = static_cast<uint8_t>(0);
  led_color[2] = static_cast<uint8_t>(0);
  led_color[3] = static_cast<uint8_t>(0);
  jsonDocument.clear();  
  jsonDocument["success"] = true;
  is_printing_design = false;
  controller->force_stop();
  serializeJson(jsonDocument, buffer);
  server.send(200, "application/json", buffer);
  delay(2000);
  ESP.restart();
}

// 3. Update firmware
void handle_update_firmware() {
  Serial.println("Updating..");
  String body = server.arg("plain");
  jsonDocument.clear();
  deserializeJson(jsonDocument, body);
  String update_url = jsonDocument["update_url"];
  update_url.trim();
  Serial.println(update_url);
  
  jsonDocument.clear();  

  controller->force_stop();
  controller->reset();
  delay(1000);
  bool is_success = update_firmware(update_url);
  jsonDocument["success"] = is_success;

  serializeJson(jsonDocument, buffer);

  server.send(200, "application/json", buffer);
  delay(2000);
  if(is_success) {
    ESP.restart();
  }
}

// 4. Home
void handle_home() {
  Serial.println("Home");
  arm->reset_home();
  is_printing_design = false;
  should_perform_homing = true;

  jsonDocument.clear();  
  jsonDocument["success"] = true;

  serializeJson(jsonDocument, buffer);

  server.send(200, "application/json", buffer);
}

// 5. Play
void handle_play() {
  Serial.println("Play");
  String body = server.arg("plain");
  Serial.println(body);
  jsonDocument.clear();
  deserializeJson(jsonDocument, body);

  String filename = jsonDocument["filename"];
  filename.trim();

  player.play(SD, filename);
  arm->reset_home();
  is_printing_design = true;
  should_clear = true;
  should_perform_homing = true;

  jsonDocument.clear();  
  jsonDocument["success"] = true;

  serializeJson(jsonDocument, buffer);

  server.send(200, "application/json", buffer);
}

// 6. Get current playing
void handle_get_current_playing() {
  Serial.println("Get current playing");
  String current_playing = player.get_current_playing();
  current_playing.trim();

  jsonDocument.clear();  
  jsonDocument["success"] = true;
  jsonDocument["current_playing"] = current_playing;

  serializeJson(jsonDocument, buffer);

  server.send(200, "application/json", buffer);
}

// 7. Pause
void handle_pause() {
  Serial.println("Pause");
  is_printing_design = false;
  controller->force_stop();

  jsonDocument.clear();  
  jsonDocument["success"] = true;

  serializeJson(jsonDocument, buffer);

  server.send(200, "application/json", buffer);
}

// 8. Get list of tracks in device
void handle_get_tracks() {
  Serial.println("Get tracks");
  // get list of files from designs folder
  int from = 0;
  int to = 10;
  if(server.hasArg("from")) {
    from = server.arg("from").toInt();
  }
  if(server.hasArg("to")) {
    to = server.arg("to").toInt();
  }

  String files = "";
  get_files_in_dir(SD, "/designs", &files, from, to);

  jsonDocument.clear();  
  jsonDocument["success"] = true;
  jsonDocument["files"] = files;

  serializeJson(jsonDocument, buffer);

  server.send(200, "application/json", buffer);
}

// 9. Get queue
void handle_get_queue() {
  Serial.println("Get queue");
  String queue = player.get_queue(SD);
  queue.trim();

  jsonDocument.clear();  
  jsonDocument["success"] = true;
  jsonDocument["queue"] = queue;

  serializeJson(jsonDocument, buffer);

  server.send(200, "application/json", buffer);
}

// 10. Add to queue
void handle_add_to_queue() {
  Serial.println("Add to queue");
  String body = server.arg("plain");
  Serial.println(body);
  jsonDocument.clear();
  deserializeJson(jsonDocument, body);

  String filename = jsonDocument["filename"];

  player.add_to_queue(SD, filename);
  
  jsonDocument.clear();  
  jsonDocument["success"] = true;

  serializeJson(jsonDocument, buffer);

  server.send(200, "application/json", buffer);
}

// 11. Remove from queue
void handle_remove_from_queue() {
  Serial.println("Remove from queue");
  String body = server.arg("plain");
  Serial.println(body);
  jsonDocument.clear();
  deserializeJson(jsonDocument, body);

  String filename = jsonDocument["filename"];

  player.remove_from_queue(SD, filename);
  
  jsonDocument.clear();  
  jsonDocument["success"] = true;

  serializeJson(jsonDocument, buffer);

  server.send(200, "application/json", buffer);
}

// 12. Pair admin app
void handle_admin_pair() {
  Serial.println("Pairing");
  String body = server.arg("plain");
  Serial.println(body);
  jsonDocument.clear();
  deserializeJson(jsonDocument, body);
  String ssid = jsonDocument["ssid"];
  String pwd = jsonDocument["pwd"];
  String user_id = jsonDocument["user_id"];
  ssid.trim();
  pwd.trim();
  user_id.trim();
  SAVED_SSID = ssid;
  SAVED_PWD = pwd;
  Serial.println(ssid);
  Serial.println(pwd);
  save_wifi_login(preferences, ssid, pwd);

  // Create admin secret
  save_admin_secret(SD);
  String admin_secret = get_admin_secret(SD);

  // Save admin user
  save_admin_user(SD, user_id);

  // Save paired user
  save_paired_user(SD, user_id);
  
  jsonDocument.clear();  
  jsonDocument["admin_secret"] = admin_secret;
  jsonDocument["success"] = true;

  serializeJson(jsonDocument, buffer);

  Serial.println("Pairing done. Connecting.");
  server.send(200, "application/json", buffer);
  delay(2000);
  ESP.restart();
}

// 13. Pair user
void handle_user_pair() {
  Serial.println("Pairing");
  String body = server.arg("plain");
  Serial.println(body);
  jsonDocument.clear();
  deserializeJson(jsonDocument, body);
  String user_id = jsonDocument["user_id"];
  String admin_secret = jsonDocument["admin_secret"];
  user_id.trim();
  admin_secret.trim();

  Serial.println(user_id + ", " + get_admin_secret(SD));

  // Check if admin secret is correct
  if(!is_admin_secret_correct(SD, admin_secret)) {
    jsonDocument.clear();  
    jsonDocument["success"] = false;
    jsonDocument["error"] = "Admin secret is incorrect";
    serializeJson(jsonDocument, buffer);
    server.send(404, "application/json", buffer);
    return;
  }  

  // Save paired user
  if(!is_paired_user(SD, user_id)) {
    save_paired_user(SD, user_id);
  }
  
  jsonDocument.clear();  
  jsonDocument["success"] = true;

  serializeJson(jsonDocument, buffer);

  Serial.println("Pairing done. Connecting.");
  server.send(200, "application/json", buffer);
}

// 14. Get paired users
void handle_user_paired() {
  Serial.println("Get paired users");
  String paired_users = get_paired_users(SD);

  jsonDocument.clear();  
  jsonDocument["success"] = true;
  jsonDocument["paired_users"] = paired_users;

  serializeJson(jsonDocument, buffer);

  server.send(200, "application/json", buffer);
}

// 15. Download design file
void handle_file_download() {
  Serial.println("Download file from a URL");
  String body = server.arg("plain");
  Serial.println(body);
  jsonDocument.clear();
  deserializeJson(jsonDocument, body);
  String url = jsonDocument["url"];
  url.trim();
  Serial.println(url);
  String filename = jsonDocument["filename"];
  filename.trim();
  Serial.println(filename);
  String path = "/designs/" + filename;
  bool is_success = download_file(SD, url, path);
  jsonDocument.clear();
  jsonDocument["success"] = is_success;
  serializeJson(jsonDocument, buffer);
  server.send(200, "application/json", buffer);
}

void handle_led_color_update() {
  Serial.println("Changing led color");
  String body = server.arg("plain");
  Serial.println(body);
  jsonDocument.clear();
  deserializeJson(jsonDocument, body);
  String r = jsonDocument["r"];
  String g = jsonDocument["g"];
  String b = jsonDocument["b"];
  String brightness = jsonDocument["brightness"];
  r.trim();
  g.trim();
  b.trim();
  brightness.trim();

  // convert string to uint8_t
  led_color[0] = static_cast<uint8_t>(r.toInt());
  led_color[1] = static_cast<uint8_t>(g.toInt());
  led_color[2] = static_cast<uint8_t>(b.toInt());
  led_color[3] = static_cast<uint8_t>(brightness.toInt());
  
  jsonDocument.clear();  
  jsonDocument["success"] = true;
  jsonDocument["r"] = led_color[0];
  jsonDocument["g"] = led_color[1];
  jsonDocument["b"] = led_color[2];
  jsonDocument["brightness"] = led_color[3];

  serializeJson(jsonDocument, buffer);

  Serial.println("Pairing done. Connecting.");
  server.send(200, "application/json", buffer);
}


void setup_routing(WebServer& server) {
  server.enableCORS();

  // 1. Get device info
  server.on("/info", HTTP_GET, handle_info);

  // 2. Restart device
  server.on("/restart", HTTP_GET, handle_restart);

  // 3. Update firmware
  server.on("/update", HTTP_POST, handle_update_firmware);
  server.on("/update", HTTP_OPTIONS, handle_options_call);

  // 4. Home
  server.on("/home", HTTP_GET, handle_home);

  // 5. Play
  server.on("/play", HTTP_POST, handle_play);
  server.on("/play", HTTP_OPTIONS, handle_options_call);

  // 6. Get current playing
  server.on("/current-playing", HTTP_GET, handle_get_current_playing);

  // 7. Pause
  server.on("/pause", HTTP_GET, handle_pause);

  // 8. Get list of tracks in device
  server.on("/tracks", HTTP_GET, handle_get_tracks);

  // 9. Get queue
  server.on("/queue", HTTP_GET, handle_get_queue);

  // 10. Add to queue
  server.on("/add_to_queue", HTTP_POST, handle_add_to_queue);
  server.on("/add_to_queue", HTTP_OPTIONS, handle_options_call);

  // 11. Remove from queue
  server.on("/remove_from_queue", HTTP_POST, handle_remove_from_queue);
  server.on("/remove_from_queue", HTTP_OPTIONS, handle_options_call);

  // 12. Pair admin app
  server.on("/pair/admin", HTTP_POST, handle_admin_pair);
  server.on("/pair/admin", HTTP_OPTIONS, handle_options_call);

  // 13. Pair user
  server.on("/pair/add", HTTP_POST, handle_user_pair);
  server.on("/pair/add", HTTP_OPTIONS, handle_options_call);

  // 14. Get paired users
  server.on("/paired", HTTP_GET, handle_user_paired);

  // 15. Download design file
  server.on("/track/download", HTTP_POST, handle_file_download);
  server.on("/track/download", HTTP_OPTIONS, handle_options_call);

  server.on("/led/color", HTTP_POST, handle_led_color_update);
  server.on("/led/color", HTTP_OPTIONS, handle_options_call);
}

double points[3] = {0.0, 0.0, 0.0};
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

  // Serial.println("List queue:");
  // Serial.println(player.get_queue(SD));
  
  if(!is_in_pairing_mode) {
    std::array<String, 2> logins = get_wifi_login(preferences);

    Serial.println("Wifi logins:");
    // logins[0] = SAVED_SSID;
    // logins[1] = SAVED_PWD;
    // Serial.println(logins[0]);
    // Serial.println(logins[1]);

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

  arm->setup(EN_PIN, motor1DirPin, motor1StepPin, motor1HomingPin, motor2DirPin, motor2StepPin, motor2HomingPin);

  setup_routing(server);
  // bool has_resumed = player.read(SD);
  // if(has_resumed) {
  //   is_printing_design = false;
  // }
  server.begin();
  set_led_status(status_code);
  Serial.println("Server started. Listening on port 80");

  // Remove this
  player.play(SD, "/designs/cleanup_spiral.thr.txt");
  is_printing_design = true;

}

// double points[5][3] = {
//   {0., 0.0, 0.0},
//   {0., 0.1*PI, 0*PI},
//   {0., 0.2*PI, 0*PI},
//   {0., 0.3*PI, 0*PI},
//   {0., 0.4*PI, 0*PI},
// };
int current_index = 0;
int last_time = 0;
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

    if(arm->isHomed()) {
      target_q1 = -90 * PI/180.0;
      target_q2 = PI;
      should_perform_homing = false;
      controller->reset();
      arm->setSpeedInHz(1200, 1200);
      Serial.println("Arm is homed");
      delay(5000);
      return;
    }
    arm->home();
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
    set_led_color(led_color[0], led_color[1], led_color[2], led_color[3]);
    // move_led();
  }
  EVERY_N_MILLISECONDS(4) {
    if(is_printing_design) {
      // Serial.println("Time: " + String( (micros() - last_time)/1000.0 ));
      int should_read_next = controller->follow_trajectory();
      // long int delta[2] = {0, 0};
      // bool should_read_next = move_arm(delta, target_q1, target_q2);
      if(should_read_next == 1) {
        player.next_line(SD, points);
        if(points[0] != 0.0) {
          target_q1 = points[1];
          target_q2 = points[2];
        }else{
          controller->has_all_targets = true;
        }
        controller->add_point_to_trajectory(target_q1, target_q2);
        // target_q1 = points[current_index][1];
        // target_q2 = points[current_index][2];
        // current_index = (current_index + 1) % 5;
      }
      if(should_read_next == 2) {
        is_printing_design = false;
        // should_play_next = true;
        target_q1 = 0.0;
        target_q2 = 0.0;
      }
      last_time = micros();
    }
  }
  // delay(300);
  // Serial.print("Time for servo run: ");
  EVERY_N_MILLISECONDS(20070){
    // Serial.println("Time: " + String(micros() - current_time));
  }
  // Serial.print("  ");
}
