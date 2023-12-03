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

void Player::add_to_queue(fs::FS &fs, String path) {
  // Check if file exists, create if not
  if(!fs.exists("/queue.txt")) {
    File queue = fs.open("/queue.txt", FILE_WRITE);
    queue.close();
  }
  File queue = fs.open("/queue.txt", FILE_APPEND);
  queue.println(path);
  queue.close();
}
void Player::remove_from_queue(fs::FS &fs, String path) {
  path.trim();
  // Check if file exists, return if not
  if(!fs.exists("/queue.txt")) {
    return;
  }
  File queue = fs.open("/queue.txt");
  String new_queue = "";
  Serial.println("path: " + path);
  while(queue.available()) {
    String line = queue.readStringUntil('\n');
    line.trim();
    Serial.println("line: " + line);
    if(line == path) {
      continue;
    }
    new_queue += line + "\n";
  }
  queue.close();
  queue = fs.open("/queue.txt", FILE_WRITE);
  queue.print(new_queue);
  queue.close();
}
String Player::get_queue(fs::FS &fs) {
  // Check if file exists, return empty string if not
  if(!fs.exists("/queue.txt")) {
    return "";
  }
  File queue = fs.open("/queue.txt");
  return queue.readString();
}
void Player::clear_queue(fs::FS &fs) {
  File queue = fs.open("/queue.txt", FILE_WRITE);
  queue.close();
}

String Player::get_next_track_from_queue(fs::FS &fs) {
  File queue = fs.open("/queue.txt");
  String path;
  String first_path = this->path;
  while(queue.available()) {
    path = queue.readStringUntil('\n');
    if(first_path == "") {
      first_path = path;
    }
    if(path == this->path) {
      if(queue.available()) {
        path = queue.readStringUntil('\n');
      } else {
        path = first_path;
      }
      break;
    }
  }
  queue.close();
  return path;
}
void Player::play(fs::FS &fs, String path) {
  path.trim();
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
  
  // Check if in the queue
  File queue = fs.open("/queue.txt");
  bool is_in_queue = false;
  while(queue.available()) {
    String line = queue.readStringUntil('\n');
    line.trim();
    if(line == path) {
      is_in_queue = true;
      break;
    }
  }
  queue.close();
  if(!is_in_queue) {
    // Add to queue
    add_to_queue(fs, path);
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
void Player::play_next_track(fs::FS &fs) {
  String path = get_next_track_from_queue(fs);
  if(path == "") {
    Serial.println("No more tracks in queue.");
    return;
  }
  play(fs, path);
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
