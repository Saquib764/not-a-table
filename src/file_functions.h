#ifndef FILE_FUNCTIONS_H
#define FILE_FUNCTIONS_H

#include "FS.h"
#include "SD.h"
#include "SPI.h"

File read_file(fs::FS &fs, String path);
File open_file(fs::FS &fs, String path, const char* mode = FILE_READ);
bool setup_sd_card(fs::SDFS &SD);
void list_dir(fs::FS &fs, const char * dirname, uint8_t levels);

#endif
