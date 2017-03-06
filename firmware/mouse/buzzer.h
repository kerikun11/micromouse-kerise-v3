#pragma once

#include <Arduino.h>
#include "task_base.h"

#define BUZZER_PIN          21
#define LEDC_BUZZER_CH      4

class Buzzer : public TaskBase {
  public:
    Buzzer(int pin, uint8_t channel): TaskBase("Buzzer Task", 1, 512), pin(pin), channel(channel) {
      playList = xQueueCreate(3, sizeof(enum Music));
      ledcSetup(LEDC_BUZZER_CH, 880, 4);
      ledcAttachPin(BUZZER_PIN, LEDC_BUZZER_CH);
    }
    virtual ~Buzzer() {
    }
    enum Music {
      BOOT,
      LOW_BATTERY,
      SELECT,
    };
    void init() {
      create_task();
    }
    void play(const enum Music music) {
      xQueueSendToBack(playList, &music, 0);
    }
  private:
    int pin;
    uint8_t channel;
    xQueueHandle playList;
    virtual void task() {
      while (1) {
        Music music;
        xQueueReceive(playList, &music, portMAX_DELAY);
        switch (music) {
          case BOOT:
            ledcWriteNote(LEDC_BUZZER_CH, NOTE_C, 6);
            vTaskDelay(200 / portTICK_RATE_MS);
            ledcWriteNote(LEDC_BUZZER_CH, NOTE_D, 6);
            vTaskDelay(200 / portTICK_RATE_MS);
            ledcWriteNote(LEDC_BUZZER_CH, NOTE_E, 6);
            vTaskDelay(200 / portTICK_RATE_MS);
            ledcWrite(LEDC_BUZZER_CH, 0);
            break;
          case LOW_BATTERY:
            ledcWriteNote(LEDC_BUZZER_CH, NOTE_C, 7);
            vTaskDelay(800 / portTICK_RATE_MS);
            ledcWrite(LEDC_BUZZER_CH, 0);
            vTaskDelay(200 / portTICK_RATE_MS);
            ledcWriteNote(LEDC_BUZZER_CH, NOTE_C, 7);
            vTaskDelay(800 / portTICK_RATE_MS);
            ledcWrite(LEDC_BUZZER_CH, 0);
            vTaskDelay(200 / portTICK_RATE_MS);
            ledcWriteNote(LEDC_BUZZER_CH, NOTE_C, 7);
            vTaskDelay(800 / portTICK_RATE_MS);
            ledcWrite(LEDC_BUZZER_CH, 0);
            vTaskDelay(200 / portTICK_RATE_MS);
            break;
          case SELECT:
            break;
          default:
            ledcWriteNote(LEDC_BUZZER_CH, NOTE_C, 4);
            vTaskDelay(1000 / portTICK_RATE_MS);
            break;
        }
      }
    }
};

extern Buzzer bz;

