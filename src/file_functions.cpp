#include "file_functions.h"

bool setup_internal_card(fs::SPIFFSFS &SPIFFS) {
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return false;
  }
  return true;
}

bool setup_sd_card(fs::SDFS &SD) {
  for(int i = 0; i < 20; i++) {
    if(SD.begin(5)){
      break;
    }
    Serial.println("Card Mount Failed");
    delay(300);
  }
  delay(1000);
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return false;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  // Serial.println("SPI frequency at " + String(SDSPI.getFrequency()));
  return true;
}


File open_file(fs::FS &fs, String path, const char* mode) {
  File file;
  if(mode == FILE_APPEND && !fs.exists( path )) {
    file = fs.open( path , FILE_WRITE);
  }else{
    file = fs.open( path , mode);
  }
  if(!file) {
    Serial.print("Failed to open file: ");
    Serial.println(path);
  }
  return file;
}


File read_file(fs::FS &fs, String path) {
  /*  Check if a file exists, return empty string if not. Else return file 
   */
  File file;
  bool is_file = fs.exists( path );
  if(!is_file) {
    Serial.println("File does not exist");
    return file;
  }
  return open_file(fs, path);
}

void list_dir(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        list_dir(fs, file.name(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void get_files_in_dir(fs::FS &fs, const char * dirname, String *files) {
  File root1 = fs.open("/");
  File root = fs.open(dirname);
  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      file = root.openNextFile();
      continue;
    }
    *files += "\n" + String(file.name());
    file = root.openNextFile();
  }
  return;
}

bool download_file(fs::FS &fs, String url, String path) {
  Serial.println("Downloading file from " + url);
  HTTPClient http;
  http.begin(url);
  int httpCode = http.GET();
  if(httpCode > 0) {
    if(httpCode == HTTP_CODE_OK) {
      File file = open_file(fs, path, FILE_WRITE);
      if(!file){
        Serial.println("Failed to open file for writing");
        return false;
      }
      http.writeToStream(&file);
      file.close();
    }
  } else {
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    return false;
  }
  http.end();
  return true;
}

String get_device_id(fs::FS &fs) {
  File file = open_file(fs, "/device_id.txt");
  String device_id = file.readString();
  device_id.trim();
  file.close();
  
  return device_id;
}
