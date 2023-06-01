#include "file_functions.h"

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

bool setup_sd_card(fs::SDFS &SD) {
  if(!SD.begin(5)){
    Serial.println("Card Mount Failed");
    return false;
  }
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
  return true;
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
