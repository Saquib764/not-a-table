/*
 * Read a sss file to play.
 * Format-> theta1, theta2
 * Read line by line.
 * Saves the current file and line number to a tracker file.
 * If already playing the requested file, it will ignore the request.
 * If the tracker file is the same as the current file, it will continue from the last line.
 * Otherwsie, it will start from the beginning and overwrite the tracker file.
 * Tracker file format:
 * - path
 * - line_number
 * 
 * Calling the read function without a path will read the tracker file and continue from the last position.
 * 
*/

// ensure this library description is only included once
#include "Player.h"

Player::Player() {}

void Player::add_to_playlist(fs::FS &fs, String path) {
  // Check if file exists, create if not
  if(!fs.exists("/playlist.txt")) {
    File playlist = fs.open("/playlist.txt", FILE_WRITE);
    playlist.close();
  }
  File playlist = fs.open("/playlist.txt", FILE_APPEND);
  playlist.println(path);
  playlist.close();
}
String Player::get_playlist(fs::FS &fs) {
  // Check if file exists, return empty string if not
  if(!fs.exists("/playlist.txt")) {
    return "";
  }
  File playlist = fs.open("/playlist.txt");
  return playlist.readString();
}
void Player::clear_playlist(fs::FS &fs) {
  File playlist = fs.open("/playlist.txt", FILE_WRITE);
  playlist.close();
}

String Player::get_next_design(fs::FS &fs) {
  File playlist = fs.open("/playlist.txt");
  String path;
  String first_path = this->path;
  while(playlist.available()) {
    path = playlist.readStringUntil('\n');
    if(first_path == "") {
      first_path = path;
    }
    if(path == this->path) {
      if(playlist.available()) {
        path = playlist.readStringUntil('\n');
      } else {
        path = first_path;
      }
      break;
    }
  }
  playlist.close();
  return path;
}
void Player::read(fs::FS &fs, String path) {
  // path = "/AngularRadiance.thr.txt";
  Serial.println(path);
  if(this->path == path) {
    Serial.println("Already playing this file.");
    return;
  }
  bool is_file_ = fs.exists( path);
  if(!is_file_) {
    Serial.println("File does not exist AngularRadiance");
  }else {
    Serial.println("File DOES exist AngularRadiance");
    File file1 = fs.open( path);
    if(!file1) {
      Serial.println("Failed to open login file for reading");
    }
    String ss = file1.readStringUntil('\n');
    Serial.println(ss);
  }
  File file = fs.open(path, "r");
  if(!file){
    Serial.println("Failed to open file for reading..");
    return;
  }
  this->line_number = 0;
  File tracker = fs.open("/tracker.txt");
  String tracker_file;
  if(tracker.available()) {
    tracker_file = tracker.readStringUntil('\n');
  }
  if(tracker_file == path) {
    Serial.println("Continuing from previous position");
    this->line_number = tracker.readStringUntil('\n').toInt();
    for(int i = 0; i < this->line_number; i++) {
      file.readStringUntil('\n');
    }
  }
  this->file = file;
  tracker.close();
  tracker = fs.open("/tracker.txt", FILE_WRITE);
  tracker.println(path);
  tracker.println(this->line_number);
  this->tracker = tracker;
}

bool Player::read(fs::FS &fs) {
  File tracker = fs.open("/tracker.txt");
  String tracker_file;
  if(tracker.available()) {
    tracker_file = tracker.readStringUntil('\n');
    this->read(fs, tracker_file);
    return true;
  }
  return false;
}

double * Player::next_line(fs::FS &fs) {
  double* thetas = new double[3];
  while(this->file.available()){
    this->line_number++;
    String line = file.readStringUntil('\n');
    if(line == "" || line[0] == '#') {
      continue;
    }
    int str_len = line.length() + 1; 
    
    char char_array[str_len];
    
    line.toCharArray(char_array, str_len);
    char *p = strtok (char_array, " ");
  
    double x = atof(p);
    p = strtok (NULL, " ");

    double y = atof(p);
    thetas[0] = 1.0;
    thetas[1] = x;
    thetas[2] = y;
    // Point to previous line and replace it with the current line
    this->tracker.seek(this->tracker.position() - 1);
    this->tracker.println(this->line_number);
    return thetas;
  }
  thetas[0] = 0.0;
  this->tracker.close();
  this->file.close();
  return thetas;
}
