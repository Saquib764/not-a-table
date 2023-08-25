
// ensure this library description is only included once
#ifndef S_S_S_FILE_READER_h
#define S_S_S_FILE_READER_h
#include "FS.h"
#include "SD.h"
#include "SPI.h"

class Player {
  public:
    // constructors:
    Player();

    void add_to_playlist(fs::FS &fs, String path);
    String get_playlist(fs::FS &fs);
    void clear_playlist(fs::FS &fs);
    String get_next_design(fs::FS &fs);
    bool read(fs::FS &fs);
    void read(fs::FS &fs, String path);
    void next_line(fs::FS &fs, double *thetas);
    String path;
    File file;
    File tracker;
    int line_number;
};
#endif
