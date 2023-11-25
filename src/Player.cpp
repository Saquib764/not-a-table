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

Player::Player() {
  is_paused = false;
  is_completed = false;
}

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
void Player::remove_from_playlist(fs::FS &fs, String path) {
  // Check if file exists, return if not
  if(!fs.exists("/playlist.txt")) {
    return;
  }
  File playlist = fs.open("/playlist.txt");
  String new_playlist = "";
  while(playlist.available()) {
    String line = playlist.readStringUntil('\n');
    if(line == path) {
      continue;
    }
    new_playlist += line + "\n";
  }
  playlist.close();
  playlist = fs.open("/playlist.txt", FILE_WRITE);
  playlist.print(new_playlist);
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
void Player::play(fs::FS &fs, String path) {
  resume();
  if(this->path == path) {
    Serial.println("Already playing this file.");
    return;
  }
  File file = fs.open(path, "r");
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }
  is_completed = false;
  this->path = path;
  this->line_number = 0;

  this->file = file;
  
  // Check if in the playlist
  File playlist = fs.open("/playlist.txt");
  bool is_in_playlist = false;
  while(playlist.available()) {
    String line = playlist.readStringUntil('\n');
    if(line == path) {
      is_in_playlist = true;
      break;
    }
  }
  playlist.close();
  if(!is_in_playlist) {
    // Add to playlist
    add_to_playlist(fs, path);
  }
}

bool Player::play(fs::FS &fs) {
  File tracker = fs.open("/tracker.txt");
  String tracker_file;
  if(tracker.available()) {
    tracker_file = tracker.readStringUntil('\n');
    this->play(fs, tracker_file);
    return true;
  }
  return false;
}

void Player::pause() {
  is_paused = true;
}
void Player::resume() {
  is_paused = false;
}

void Player::next_line(fs::FS &fs, double *thetas) {
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
    // this->tracker.seek(this->tracker.position() - 1);
    // this->tracker.println(this->line_number);
    return;
  }
  Serial.println("End of design file.");
  thetas[0] = 0.0;
  is_completed = true;
  // this->tracker.close();
  this->file.close();
}

String Player::get_current_playing() {
  return this->path;
}
