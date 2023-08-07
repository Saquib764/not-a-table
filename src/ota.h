#ifndef OTA_H
#define OTA_H

#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>

bool update_firmware(String url);

#endif