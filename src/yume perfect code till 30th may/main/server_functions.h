#ifndef SERVER_FUNCTIONS_H
#define SERVER_FUNCTIONS_H


#include <WiFi.h>
#include <ArduinoJson.h>
#include <iostream>
#include<array>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
using namespace std;

void setup_wifi(String ssid, String pwd, bool set_as_hotspot = true);
std::array<String, 2> get_wifi_login(fs::FS &fs);
void save_wifi_login(fs::FS &fs, String ssid, String pwd);

#endif