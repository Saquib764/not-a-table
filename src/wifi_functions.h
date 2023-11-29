#ifndef WIFI_FUNCTIONS_H
#define WIFI_FUNCTIONS_H


#include <WiFi.h>
#include <ArduinoJson.h>
#include <array>
#include <HTTPClient.h>
#include <Preferences.h>
#include "file_functions.h"
using namespace std;


bool check_if_network_is_available(String ssid);
bool check_if_connected_to_network();
bool check_if_connected_to_network(String ssid);
bool disconnect_from_network();
bool connect_to_network(String ssid, String pwd, int n_try = 3);
bool create_hotspot();
bool check_if_mode_is_pairing();

std::array<String, 2> get_wifi_login(Preferences &preferences);
void save_wifi_login(Preferences &preferences, String ssid, String pwd);

void update_counter(fs::FS &fs);
bool should_reset(fs::FS &fs);
void clear_counter(fs::FS &fs);

void save_admin_secret(fs::FS &fs);
String get_admin_secret(fs::FS &fs);
bool is_admin_secret_correct(fs::FS &fs, String secret);

void save_admin_user(fs::FS &fs, String user_id);
bool is_admin_user(fs::FS &fs, String user_id);
void save_paired_user(fs::FS &fs, String user_id);
bool is_paired_user(fs::FS &fs, String user_id);

String get_paired_users(fs::FS &fs);


#endif