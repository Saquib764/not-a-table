#ifndef WIFI_FUNCTIONS_H
#define WIFI_FUNCTIONS_H


#include <WiFi.h>
#include <ArduinoJson.h>
#include <array>
#include <HTTPClient.h>
#include <Preferences.h>
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


#endif