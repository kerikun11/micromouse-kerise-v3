#pragma once

#include <Arduino.h>

#define BUZZER_TASK_PRIORITY    1
#define BUZZER_TASK_STACK_SIZE  4096

#define BUZZER_QUEUE_SIZE   5

class Buzzer {
  public:
    Buzzer(int pin, uint8_t channel): pin(pin), channel(channel) {
      playList = xQueueCreate(BUZZER_QUEUE_SIZE, sizeof(enum Music));
      ledcSetup(LEDC_CH_BUZZER, 880, 4);
      ledcAttachPin(BUZZER_PIN, LEDC_CH_BUZZER);
      xTaskCreate([](void* obj) {
        static_cast<Buzzer*>(obj)->task();
      }, "Buzzer", BUZZER_TASK_STACK_SIZE, this, BUZZER_TASK_PRIORITY, NULL);
    }
    enum Music {
      BOOT,
      LOW_BATTERY,
      EMERGENCY,
      ERROR,
      SELECT,
      CONFIRM,
      CANCEL,
      COMPLETE,
      SHORT,
      MAZE_BACKUP,
      MAZE_RESTORE,
    };
    void play(const enum Music music) {
      //      xQueueSendToBack(playList, &music, 0);
    }
  private:
    int pin;
    uint8_t channel;
    xQueueHandle playList;
    void sound(const note_t note, uint8_t octave, uint32_t time_ms) {
      ledcWriteNote(LEDC_CH_BUZZER, note, octave);
      vTaskDelay(time_ms / portTICK_RATE_MS);
    }
    void mute(uint32_t time_ms = 400) {
      ledcWrite(LEDC_CH_BUZZER, 0);
      vTaskDelay(time_ms / portTICK_RATE_MS);
    }
    void task() {
      while (1) {
        Music music;
        if (xQueueReceive(playList, &music, (1000 - 100) / portTICK_RATE_MS) == pdFALSE) {
          //          sound(NOTE_C, 7, 50);
          //          mute(50);
          continue;
        }
        switch (music) {
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
          case SELECT:
            sound(NOTE_C, 6, 100);
            mute(100);
            break;
          case ERROR:
            for (int i = 0; i < 6; i++) {
              sound(NOTE_C, 7, 100);
              sound(NOTE_E, 7, 100);
            }
            mute();
            break;
          case CONFIRM:
            sound(NOTE_C, 6, 100);
            sound(NOTE_E, 6, 100);
            mute(100);
            break;
          case CANCEL:
            sound(NOTE_E, 6, 100);
            sound(NOTE_C, 6, 100);
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
            sound(NOTE_C, 6, 100);
            sound(NOTE_E, 6, 100);
            sound(NOTE_G, 6, 100);
            break;
          case MAZE_RESTORE:
            sound(NOTE_G, 6, 100);
            sound(NOTE_E, 6, 100);
            sound(NOTE_C, 6, 100);
            break;
          default:
            sound(NOTE_C, 4, 1000);
            mute();
            break;
        }
      }
    }
};

#include <vector>

class LED {
  public:
    LED(const std::vector<int> pins): pins(pins) {
      for (auto pin : pins) pinMode(pin, OUTPUT);
    }
    operator uint8_t() const {
      return value;
    }
    uint8_t operator=(uint8_t new_value) {
      value = new_value;
      for (int i = 0; i < pins.size(); i++) digitalWrite(pins[i], (value & (1 << i)) ? HIGH : LOW);
      return value;
    }
  private:
    const std::vector<int> pins;
    uint8_t value;
};

#define BUTTON_SAMPLING_MS        20

#define BUTTON_PRESS_LEVEL        1
#define BUTTON_LONG_PRESS_LEVEL_1 20
#define BUTTON_LONG_PRESS_LEVEL_2 100
#define BUTTON_LONG_PRESS_LEVEL_3 500

#define BUTTON_TASK_PRIORITY      1
#define BUTTON_STACK_SIZE         1024

class Button {
  public:
    Button(int pin) : pin(pin) {
      pinMode(pin, INPUT_PULLUP);
      flags = 0x00;
      xTaskCreate([](void* obj) {
        static_cast<Button*>(obj)->task();
      }, "Button", BUTTON_STACK_SIZE, this, BUTTON_TASK_PRIORITY, NULL);
    }
    union {
      uint8_t flags;           /**< all flags */
      struct {
        uint8_t pressed : 1;     /**< pressed */
        uint8_t long_pressed_1 : 1;  /**< long-pressed level 1 */
        uint8_t long_pressed_2 : 1;  /**< long-pressed level 2 */
        uint8_t long_pressed_3 : 1;  /**< long-pressed level 3 */
        uint8_t pressing : 1;    /**< pressing */
        uint8_t long_pressing_1 : 1; /**< long-pressing level 1 */
        uint8_t long_pressing_2 : 1; /**< long-pressing level 2 */
        uint8_t long_pressing_3 : 1; /**< long-pressing level 3 */
      };
    };
  private:
    int pin;
    int counter;

    void task() {
      portTickType xLastWakeTime = xTaskGetTickCount();
      while (1) {
        vTaskDelayUntil(&xLastWakeTime, BUTTON_SAMPLING_MS / portTICK_RATE_MS);
        if (digitalRead(pin) == LOW) {
          if (counter < BUTTON_LONG_PRESS_LEVEL_3 + 1)
            counter++;
          if (counter == BUTTON_LONG_PRESS_LEVEL_3)
            long_pressing_3 = 1;
          if (counter == BUTTON_LONG_PRESS_LEVEL_2)
            long_pressing_2 = 1;
          if (counter == BUTTON_LONG_PRESS_LEVEL_1)
            long_pressing_1 = 1;
          if (counter == BUTTON_PRESS_LEVEL)
            pressing = 1;
        } else {
          if (counter >= BUTTON_LONG_PRESS_LEVEL_3)
            long_pressed_3 = 1;
          else if (counter >= BUTTON_LONG_PRESS_LEVEL_2)
            long_pressed_2 = 1;
          else if (counter >= BUTTON_LONG_PRESS_LEVEL_1)
            long_pressed_1 = 1;
          else if (counter >= BUTTON_PRESS_LEVEL)
            pressed = 1;
          counter = 0;
          flags &= 0x0F;
        }
      }
    }
};

extern Buzzer bz;
extern Button btn;
extern LED led;

