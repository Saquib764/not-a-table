
#include <Arduino.h>
#include "led_control.h"
#include "math.h"
#include <TMCStepper.h>
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

Preferences preferences;

String SAVED_SSID = "E1_1102";
String SAVED_PWD = "Roomies@2829";

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

uint8_t led_color[4] = {246, 231, 210, 255};


float ARM1 = 0.25;
float ARM2 = 0.25;

bool is_printing_design = false;
bool is_paused = false;
bool is_in_pairing_mode = false;
bool is_connected_to_wifi = false;
bool should_clear = false;
bool should_perform_homing = true;
bool should_play_next = false;
bool has_error = false;
int status_code = 0;
bool is_uploading = false;
bool should_use_internal_sd = false;
bool is_storage_available = false;
bool should_use_homing = true;
bool is_waiting_for_timer = false;
float wait_time = 10.0 * 60.0;  // 10 minutes
float wait_time_start = 0.0;


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

  // Serial.println("List queue:");
  // Serial.println(player.get_queue(SD));

  sleep(1);

  setup_driver(driver, EN_PIN);

  arm->setup(EN_PIN, motor1DirPin, motor1StepPin, motor1HomingPin, motor2DirPin, motor2StepPin, motor2HomingPin);

  // bool has_resumed = player.read(SD);
  // if(has_resumed) {
  //   is_printing_design = false;
  // }
  player.index_all_tracks(SD);

  set_led_status(status_code);
  
  Serial.println("Server started. Listening on port 80");

  // Remove this
  // player.play(SD, "/designs/cleanup_spiral.thr.txt");
  // is_printing_design = true;
  // should_play_next = true;
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
    if(is_printing_design && !is_paused) {
      // Serial.println("Time: " + String( (micros() - last_time)/1000.0 ));
      int should_read_next = controller->follow_trajectory();
      // long int delta[2] = {0, 0};
      // bool should_read_next = move_arm(delta, target_q1, target_q2);
      if(should_read_next == 1) {
        player.next_line(SD, points);
        if(points[0] != 0.0) {
          target_q1 = points[1];
          target_q2 = points[2];
          controller->add_point_to_trajectory(target_q1, target_q2);
        }else{
          controller->has_all_targets = true;
        }
        // target_q1 = points[current_index][1];
        // target_q2 = points[current_index][2];
        // current_index = (current_index + 1) % 5;
      }
      if(should_read_next == 2) {
        Serial.println("Stop design print.");
        Serial.println("Wait for " + String(wait_time) + " seconds. Then play the next design");
        is_printing_design = false;
        should_play_next = true;
        is_waiting_for_timer = true;
        wait_time_start = millis();
        // target_q1 = 0.0;
        // target_q2 = 0.0;
      }
      last_time = micros();
    }
    if(is_waiting_for_timer) {
      if(millis() - wait_time_start > wait_time * 1000.0) {
        is_waiting_for_timer = false;
      }
    }
    if(should_play_next) {
      // Play next track from queue
      Serial.println("play next");
      player.play_next_track(SD);
      is_printing_design = true;
      should_play_next = false;
    }
  }
  // delay(300);
  // Serial.print("Time for servo run: ");
  EVERY_N_MILLISECONDS(20070){
    // Serial.println("Time: " + String(micros() - current_time));
  }
  // Serial.print("  ");
}
