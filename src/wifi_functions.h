#ifndef WIFI_FUNCTIONS_H
#define WIFI_FUNCTIONS_H


#include <WiFi.h>
#include <ArduinoJson.h>
#include <iostream>
#include<array>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <HTTPClient.h>
#include "file_functions.h"
#include <Preferences.h>
using namespace std;


bool check_if_network_is_available(String ssid);
bool check_if_connected_to_network();
bool check_if_connected_to_network(String ssid);
bool disconnect_from_network();
bool connect_to_network(String ssid, String pwd, int n_try = 3);
bool create_hotspot();
bool check_if_mode_is_pairing();

std::array<String, 2> get_wifi_login(fs::FS &fs);
void save_wifi_login(fs::FS &fs, String ssid, String pwd);

void update_counter(fs::FS &fs);
bool should_reset(fs::FS &fs);
void clear_counter(fs::FS &fs);

#endif