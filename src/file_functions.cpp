#include "file_functions.h"

void read_file(char* filename, char* buffer) {
  File file = SD_MMC.open(filename, "r");
  if(!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Reading file: ");
  Serial.println(filename);

  while(file.available()) {
    file.readBytes(buffer, file.size());
  }
  file.close();
}

void write_file(char* filename, char* buffer) {
  File file = SD_MMC.open(filename, "w");
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  if(file.print(buffer)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}
