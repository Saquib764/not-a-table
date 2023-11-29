#ifndef FILE_FUNCTIONS_H
#define FILE_FUNCTIONS_H

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "SPIFFS.h"
#include <HTTPClient.h>

File read_file(fs::FS &fs, String path);
File open_file(fs::FS &fs, String path, const char* mode = FILE_READ);
bool setup_sd_card(fs::SDFS &SD);
bool setup_internal_card(fs::SPIFFSFS &SPIFFS);
void list_dir(fs::FS &fs, const char * dirname, uint8_t levels);
void get_files_in_dir(fs::FS &fs, const char * dirname, String *files, int start=0, int end=100);

bool download_file(fs::FS &fs, String url, String path);
String get_device_id(fs::FS &fs);


#endif
