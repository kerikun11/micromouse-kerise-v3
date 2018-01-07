#pragma once

#include <Arduino.h>
#include "TaskBase.h"

#define BUZZER_QUEUE_SIZE   5

class Buzzer : TaskBase {
  public:
    Buzzer() {}
    bool begin(int8_t pin, int8_t ledcChannel = 0, UBaseType_t uxPriority = 0, const uint16_t usStackDepth = configMINIMAL_STACK_SIZE) {
      this->ledcChannel = ledcChannel;
      playList = xQueueCreate(BUZZER_QUEUE_SIZE, sizeof(enum Music));
      ledcSetup(ledcChannel, 880, 4);
      ledcAttachPin(pin, ledcChannel);
      return createTask("Buzzer", uxPriority, configMINIMAL_STACK_SIZE);
    }
    enum Music {
      SELECT, CANCEL, CONFIRM, SUCCESSFUL, ERROR,
      SHORT,
      BOOT,
      LOW_BATTERY,
      EMERGENCY,
      COMPLETE,
      MAZE_BACKUP,
      MAZE_RESTORE,
    };
    void play(const enum Music music) {
      xQueueSendToBack(playList, &music, 0);
    }
  private:
    int8_t ledcChannel;
    xQueueHandle playList;
    void sound(const note_t note, uint8_t octave, uint32_t time_ms) {
      ledcWriteNote(ledcChannel, note, octave);
      vTaskDelay(time_ms / portTICK_RATE_MS);
    }
    void mute(uint32_t time_ms = 400) {
      ledcWrite(ledcChannel, 0);
      vTaskDelay(time_ms / portTICK_RATE_MS);
    }
    void task() {
      while (1) {
        Music music;
        xQueueReceive(playList, &music, portMAX_DELAY);
        switch (music) {
          case SELECT:
            sound(NOTE_C, 6, 100);
            mute(100);
            break;
          case CANCEL:
            sound(NOTE_E, 6, 100);
            sound(NOTE_C, 6, 100);
            mute(100);
            break;
          case CONFIRM:
            sound(NOTE_C, 6, 100);
            sound(NOTE_E, 6, 100);
            mute(100);
            break;
          case SUCCESSFUL:
            sound(NOTE_C, 6, 100);
            sound(NOTE_E, 6, 100);
            sound(NOTE_G, 6, 100);
            mute(100);
            break;
          case ERROR:
            for (int i = 0; i < 6; i++) {
              sound(NOTE_C, 7, 100);
              sound(NOTE_E, 7, 100);
            }
            mute();
            break;
          case BOOT:
            sound(NOTE_B, 5, 200);
            sound(NOTE_E, 6, 400);
            sound(NOTE_Fs, 6, 200);
            sound(NOTE_B, 6, 600);
            mute();
            break;
          case LOW_BATTERY:
            sound(NOTE_C, 7, 400);
            mute(200);
            sound(NOTE_C, 7, 400);
            mute(200);
            sound(NOTE_C, 7, 400);
            mute(200);
            break;
          case EMERGENCY:
            sound(NOTE_C, 6, 100);
            sound(NOTE_F, 6, 100);
            mute(100);
            sound(NOTE_F, 6, 75);
            mute(25);

            sound(NOTE_F, 6, 176);
            sound(NOTE_E, 6, 176);
            sound(NOTE_D, 6, 176);
            sound(NOTE_C, 6, 200);
            mute(100);
            break;
          case COMPLETE:
            sound(NOTE_C, 6, 100);
            sound(NOTE_D, 6, 100);
            sound(NOTE_E, 6, 100);
            sound(NOTE_F, 6, 100);
            sound(NOTE_G, 6, 100);
            sound(NOTE_A, 6, 100);
            sound(NOTE_B, 6, 100);
            sound(NOTE_C, 7, 100);
            mute(100);
            break;
          case SHORT:
            sound(NOTE_C, 7, 50);
            mute(50);
            break;
          case MAZE_BACKUP:
            sound(NOTE_G, 7, 100);
            sound(NOTE_E, 7, 100);
            sound(NOTE_C, 7, 100);
            mute(100);
            break;
          case MAZE_RESTORE:
            sound(NOTE_C, 7, 100);
            sound(NOTE_E, 7, 100);
            sound(NOTE_G, 7, 100);
            mute(100);
            break;
          default:
            sound(NOTE_C, 4, 1000);
            mute();
            break;
        }
      }
    }
};

