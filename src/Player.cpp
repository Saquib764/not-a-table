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
  path = "";
}

void Player::play( String path) {
  path.trim();
  if(this->path == path) {
    Serial.println("Already playing this file.");
    return;
  }
  File file = SPIFFS.open(path, "r");
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }
  is_completed = false;
  this->path = path;
  this->line_number = 0;

  this->file = file;
}

void Player::next_line(double *thetas) {
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
