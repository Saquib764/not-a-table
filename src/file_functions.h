#ifndef FILE_FUNCTIONS_H
#define FILE_FUNCTIONS_H

#include "FS.h"
#include "SD.h"
#include "SPI.h"

File read_file(fs::FS &fs, String path);
File open_file(fs::FS &fs, String path, const char* mode = FILE_READ);

#endif
