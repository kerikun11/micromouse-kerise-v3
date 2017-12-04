#include <WiFi.h>
#include <SPIFFS.h>
#include <Preferences.h>

#include "config.h"

#include "WallDetector.h"

class DataSaver {
  public:
    DataSaver() {}
    void begin() {
      pref.begin("mouse", false);
    }
    bool restore() {
      size_t n = pref.getBytes("wd.wall_ref", &(wd.wall_ref), sizeof(WallDetector::WallValue));
      if (n == 0) {
        log_e("Restore Failed:(");
        return false;
      }
      log_i("Restore Successful");
      log_d("wd.wall_ref: %d, %d, %d, %d", wd.wall_ref.side[0], wd.wall_ref.front[0], wd.wall_ref.front[1], wd.wall_ref.side[1]);
      return true;
    }

    bool backup() {
      size_t n = pref.putBytes("wd.wall_ref", &(wd.wall_ref), sizeof(WallDetector::WallValue));
      if (n == 0) {
        log_e("Backup Failed:(");
        return false;
      }
      log_i("Backup Successful");
      return true;
    }
  private:
    Preferences pref;
};
