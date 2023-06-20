
#include "ota.h"

#define OTA_SERVER_IP "192.0.0.0"

bool update_firmware(String url) {
  bool is_success = false;
  Serial.println("Starting update...");
  HTTPClient http;
  WiFiClient client;
  
  http.begin( url );
  int httpCode = http.GET();
  if(httpCode == HTTP_CODE_OK) {
    // Update ota
    int contentLength = http.getSize();
    if(contentLength > 0) {
      bool canBegin = Update.begin(contentLength);
      if(!canBegin) {
        Serial.println("Not enough space to begin OTA");
        client.flush();
      }else{
        WiFiClient stream = http.getStream();
        size_t written = Update.writeStream(stream);
        if(Update.end()) {
          is_success = true;
        }
      }
    }

  } else {
    Serial.println("Update failed.");
  }
  http.end();
  return is_success;
}