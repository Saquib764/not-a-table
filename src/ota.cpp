
#include "ota.h"


bool update_firmware(String url) {
  bool is_success = false;
  Serial.println("Starting update...");
  HTTPClient http;
  
  http.begin( url );
  int httpCode = http.GET();
  if(httpCode == HTTP_CODE_OK) {
    // Update ota
    int contentLength = http.getSize();
    Serial.println("Content length: " + String(contentLength));
    if(contentLength > 0) {
      bool canBegin = Update.begin(contentLength);
      if(!canBegin) {
        Serial.println("Not enough space to begin OTA");
      }else{
        WiFiClient stream = http.getStream();
        size_t written = Update.writeStream(stream);
        Serial.println("Written: " + String(written) + " successfully");
        if(Update.end()) {
          is_success = true;
        }
      }
    }

  } else {
    Serial.println("Update failed.");
  }
  Serial.println("Update finished.");
  http.end();
  return is_success;
}