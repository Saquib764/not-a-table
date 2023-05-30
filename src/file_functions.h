#ifndef FILE_FUNCTIONS_H
#define FILE_FUNCTIONS_H

#include <SD_MMC.h>

void read_file(char* filename, char* buffer);
void write_file(char* filename, char* buffer);

#endif
