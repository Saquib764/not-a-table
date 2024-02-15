
// ensure this library description is only included once
#ifndef S_S_S_FILE_READER_h
#define S_S_S_FILE_READER_h

#include "SPIFFS.h"

class Player {
  public:
    // constructors:
    Player();

    void play(String path);
    void next_line(double *thetas);
    bool is_paused;
    bool is_completed;
    String path;
    File file;
    File tracker;
    int line_number;
};
#endif
