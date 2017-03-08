#pragma once

#include <Arduino.h>
#include "task_base.h"
#include "config.h"

#define BUZZER_PIN          21
#define LEDC_BUZZER_CH      2

#define BUZZER_TASK_PRIORITY    1
#define BUZZER_TASK_STACK_SIZE  512

class Buzzer : private TaskBase {
  public:
    Buzzer(int pin, uint8_t channel): TaskBase("Buzzer Task", BUZZER_TASK_PRIORITY, BUZZER_TASK_STACK_SIZE), pin(pin), channel(channel) {
      playList = xQueueCreate(3, sizeof(enum Music));
    }
    virtual ~Buzzer() {
    }
    enum Music {
      BOOT,
      LOW_BATTERY,
      SELECT,
    };
    void init() {
      ledcSetup(LEDC_BUZZER_CH, 880, 4);
      ledcAttachPin(BUZZER_PIN, LEDC_BUZZER_CH);
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
            ledcWriteNote(LEDC_BUZZER_CH, NOTE_B, 5);
            vTaskDelay(200 / portTICK_RATE_MS);
            ledcWriteNote(LEDC_BUZZER_CH, NOTE_E, 6);
            vTaskDelay(400 / portTICK_RATE_MS);
            ledcWriteNote(LEDC_BUZZER_CH, NOTE_Fs, 6);
            vTaskDelay(200 / portTICK_RATE_MS);
            ledcWriteNote(LEDC_BUZZER_CH, NOTE_B, 6);
            vTaskDelay(600 / portTICK_RATE_MS);
            ledcWrite(LEDC_BUZZER_CH, 0);
            //            ledcWriteNote(LEDC_BUZZER_CH, NOTE_Eb, 6);
            //            vTaskDelay(300 / portTICK_RATE_MS);
            //            ledcWriteNote(LEDC_BUZZER_CH, NOTE_Eb, 5);
            //            vTaskDelay(100 / portTICK_RATE_MS);
            //            ledcWriteNote(LEDC_BUZZER_CH, NOTE_Bb, 5);
            //            vTaskDelay(200 / portTICK_RATE_MS);
            //            ledcWriteNote(LEDC_BUZZER_CH, NOTE_Gs, 5);
            //            vTaskDelay(200 / portTICK_RATE_MS);
            //            ledcWriteNote(LEDC_BUZZER_CH, NOTE_Eb, 5);
            //            vTaskDelay(200 / portTICK_RATE_MS);
            //            ledcWriteNote(LEDC_BUZZER_CH, NOTE_Eb, 6);
            //            vTaskDelay(200 / portTICK_RATE_MS);
            //            ledcWriteNote(LEDC_BUZZER_CH, NOTE_Bb, 5);
            //            vTaskDelay(600 / portTICK_RATE_MS);
            //            ledcWrite(LEDC_BUZZER_CH, 0);
            break;
          case LOW_BATTERY:
            ledcWriteNote(LEDC_BUZZER_CH, NOTE_C, 7);
            vTaskDelay(400 / portTICK_RATE_MS);
            ledcWrite(LEDC_BUZZER_CH, 0);
            vTaskDelay(200 / portTICK_RATE_MS);
            ledcWriteNote(LEDC_BUZZER_CH, NOTE_C, 7);
            vTaskDelay(400 / portTICK_RATE_MS);
            ledcWrite(LEDC_BUZZER_CH, 0);
            vTaskDelay(200 / portTICK_RATE_MS);
            ledcWriteNote(LEDC_BUZZER_CH, NOTE_C, 7);
            vTaskDelay(400 / portTICK_RATE_MS);
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

