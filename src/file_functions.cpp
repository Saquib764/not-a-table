#include "file_functions.h"

File open_file(fs::FS &fs, String path, bool write) {
  File file = fs.open( path , write ? FILE_WRITE : FILE_READ);
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

