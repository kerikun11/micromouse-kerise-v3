#pragma once

#include <Arduino.h>
#include "config.h"

#define MOTOR_L_CTRL1_PIN 18
#define MOTOR_L_CTRL2_PIN 23
#define MOTOR_R_CTRL1_PIN 19
#define MOTOR_R_CTRL2_PIN 22

#define LEDC_MOTOR_L_CTRL1_CH 0
#define LEDC_MOTOR_L_CTRL2_CH 1
#define LEDC_MOTOR_R_CTRL1_CH 8
#define LEDC_MOTOR_R_CTRL2_CH 9

#define MOTOR_CTRL_FREQUENCY  25000
#define MOTOR_CTRL_BIT_NUM    10
#define MOTOR_DUTY_MAX        1023 //< 2 ^ MOTOR_CTRL_BIT_NUM - 1

class Motor {
  public:
    Motor() {
      emergency = false;
      ledcSetup(LEDC_MOTOR_L_CTRL1_CH, MOTOR_CTRL_FREQUENCY, MOTOR_CTRL_BIT_NUM);
      ledcSetup(LEDC_MOTOR_L_CTRL2_CH, MOTOR_CTRL_FREQUENCY, MOTOR_CTRL_BIT_NUM);
      ledcSetup(LEDC_MOTOR_R_CTRL1_CH, MOTOR_CTRL_FREQUENCY, MOTOR_CTRL_BIT_NUM);
      ledcSetup(LEDC_MOTOR_R_CTRL2_CH, MOTOR_CTRL_FREQUENCY, MOTOR_CTRL_BIT_NUM);
      ledcAttachPin(MOTOR_L_CTRL1_PIN, LEDC_MOTOR_L_CTRL1_CH);
      ledcAttachPin(MOTOR_L_CTRL2_PIN, LEDC_MOTOR_L_CTRL2_CH);
      ledcAttachPin(MOTOR_R_CTRL1_PIN, LEDC_MOTOR_R_CTRL1_CH);
      ledcAttachPin(MOTOR_R_CTRL2_PIN, LEDC_MOTOR_R_CTRL2_CH);
      free();
    }
    void left(int duty) {
      if (emergency) return;
      if (duty > MOTOR_DUTY_MAX) {
        ledcWrite(LEDC_MOTOR_L_CTRL1_CH, MOTOR_DUTY_MAX);
        ledcWrite(LEDC_MOTOR_L_CTRL2_CH, 0);
      } else if (duty < -MOTOR_DUTY_MAX) {
        ledcWrite(LEDC_MOTOR_L_CTRL1_CH, 0);
        ledcWrite(LEDC_MOTOR_L_CTRL2_CH, MOTOR_DUTY_MAX);
      } else if (duty > 0) {
        ledcWrite(LEDC_MOTOR_L_CTRL1_CH, MOTOR_DUTY_MAX);
        ledcWrite(LEDC_MOTOR_L_CTRL2_CH, MOTOR_DUTY_MAX - duty);
      } else {
        ledcWrite(LEDC_MOTOR_L_CTRL1_CH, MOTOR_DUTY_MAX + duty);
        ledcWrite(LEDC_MOTOR_L_CTRL2_CH, MOTOR_DUTY_MAX);
      }
    }
    void right(int duty) {
      if (emergency) return;
      if (duty > MOTOR_DUTY_MAX) {
        ledcWrite(LEDC_MOTOR_R_CTRL1_CH, MOTOR_DUTY_MAX);
        ledcWrite(LEDC_MOTOR_R_CTRL2_CH, 0);
      } else if (duty < -MOTOR_DUTY_MAX) {
        ledcWrite(LEDC_MOTOR_R_CTRL1_CH, 0);
        ledcWrite(LEDC_MOTOR_R_CTRL2_CH, MOTOR_DUTY_MAX);
      } else if (duty > 0) {
        ledcWrite(LEDC_MOTOR_R_CTRL1_CH, MOTOR_DUTY_MAX);
        ledcWrite(LEDC_MOTOR_R_CTRL2_CH, MOTOR_DUTY_MAX - duty);
      } else {
        ledcWrite(LEDC_MOTOR_R_CTRL1_CH, MOTOR_DUTY_MAX + duty);
        ledcWrite(LEDC_MOTOR_R_CTRL2_CH, MOTOR_DUTY_MAX);
      }
    }
    void drive(int16_t valueL, int16_t valueR) {
      left(valueL);
      right(valueR);
    }
    void free() {
      ledcWrite(LEDC_MOTOR_L_CTRL1_CH, 0);
      ledcWrite(LEDC_MOTOR_L_CTRL2_CH, 0);
      ledcWrite(LEDC_MOTOR_R_CTRL1_CH, 0);
      ledcWrite(LEDC_MOTOR_R_CTRL2_CH, 0);
    }
    void emergency_stop() {
      emergency = true;
      free();
    }
    void emergency_release() {
      emergency = false;
      free();
    }
    bool isEmergency() {
      return emergency;
    }
  private:
    bool emergency;
};
