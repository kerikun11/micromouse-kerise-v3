#pragma once

#include <Arduino.h>

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
        xLastWakeTime = xTaskGetTickCount(); vTaskDelayUntil(&xLastWakeTime, BUTTON_SAMPLING_MS / portTICK_RATE_MS);
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

