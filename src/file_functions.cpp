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

