
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

    void add_to_queue(fs::FS &fs, String path);
    void remove_from_queue(fs::FS &fs, String path);
    String get_queue(fs::FS &fs);
    void clear_queue(fs::FS &fs);
    String get_next_track_from_queue(fs::FS &fs);
    bool play(fs::FS &fs);
    void play(fs::FS &fs, String path);
    void play_next_track(fs::FS &fs);
    void pause();
    void resume();
    void next_line(fs::FS &fs, double *thetas);
    String get_current_playing();

    bool is_paused;
    bool is_completed;
    String path;
    File file;
    File tracker;
    int line_number;
};
#endif
