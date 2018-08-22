#pragma once

#include <Arduino.h>
#include "config.h"

#define MOTOR_CTRL_FREQUENCY  250000
#define MOTOR_CTRL_BIT_NUM    10
#define MOTOR_DUTY_MAX        1023 //< 2 ^ MOTOR_CTRL_BIT_NUM - 1
#define MOTOR_DUTY_SAT        1023

class Motor {
  public:
    Motor() {
      emergency = false;
      ledcSetup(LEDC_CH_MOTOR_L_CTRL1, MOTOR_CTRL_FREQUENCY, MOTOR_CTRL_BIT_NUM);
      ledcSetup(LEDC_CH_MOTOR_L_CTRL2, MOTOR_CTRL_FREQUENCY, MOTOR_CTRL_BIT_NUM);
      ledcSetup(LEDC_CH_MOTOR_R_CTRL1, MOTOR_CTRL_FREQUENCY, MOTOR_CTRL_BIT_NUM);
      ledcSetup(LEDC_CH_MOTOR_R_CTRL2, MOTOR_CTRL_FREQUENCY, MOTOR_CTRL_BIT_NUM);
      ledcAttachPin(MOTOR_L_CTRL1_PIN, LEDC_CH_MOTOR_L_CTRL1);
      ledcAttachPin(MOTOR_L_CTRL2_PIN, LEDC_CH_MOTOR_L_CTRL2);
      ledcAttachPin(MOTOR_R_CTRL1_PIN, LEDC_CH_MOTOR_R_CTRL1);
      ledcAttachPin(MOTOR_R_CTRL2_PIN, LEDC_CH_MOTOR_R_CTRL2);
      free();
    }
    void left(int duty) {
      if (emergency) return;
      if (duty > MOTOR_DUTY_SAT) {
        ledcWrite(LEDC_CH_MOTOR_L_CTRL1, 0);
        ledcWrite(LEDC_CH_MOTOR_L_CTRL2, MOTOR_DUTY_SAT);
      } else if (duty < -MOTOR_DUTY_SAT) {
        ledcWrite(LEDC_CH_MOTOR_L_CTRL1, MOTOR_DUTY_SAT);
        ledcWrite(LEDC_CH_MOTOR_L_CTRL2, 0);
      } else if (duty > 0) {
        ledcWrite(LEDC_CH_MOTOR_L_CTRL1, MOTOR_DUTY_MAX - duty);
        ledcWrite(LEDC_CH_MOTOR_L_CTRL2, MOTOR_DUTY_MAX);
      } else {
        ledcWrite(LEDC_CH_MOTOR_L_CTRL1, MOTOR_DUTY_MAX);
        ledcWrite(LEDC_CH_MOTOR_L_CTRL2, MOTOR_DUTY_MAX + duty);
      }
    }
    void right(int duty) {
      if (emergency) return;
      if (duty > MOTOR_DUTY_SAT) {
        ledcWrite(LEDC_CH_MOTOR_R_CTRL1, 0);
        ledcWrite(LEDC_CH_MOTOR_R_CTRL2, MOTOR_DUTY_SAT);
      } else if (duty < -MOTOR_DUTY_SAT) {
        ledcWrite(LEDC_CH_MOTOR_R_CTRL1, MOTOR_DUTY_SAT);
        ledcWrite(LEDC_CH_MOTOR_R_CTRL2, 0);
      } else if (duty > 0) {
        ledcWrite(LEDC_CH_MOTOR_R_CTRL1, MOTOR_DUTY_MAX - duty);
        ledcWrite(LEDC_CH_MOTOR_R_CTRL2, MOTOR_DUTY_MAX);
      } else {
        ledcWrite(LEDC_CH_MOTOR_R_CTRL1, MOTOR_DUTY_MAX);
        ledcWrite(LEDC_CH_MOTOR_R_CTRL2, MOTOR_DUTY_MAX + duty);
      }
    }
    void drive(int16_t valueL, int16_t valueR) {
      left(valueL);
      right(valueR);
    }
    void free() {
      ledcWrite(LEDC_CH_MOTOR_L_CTRL1, 0);
      ledcWrite(LEDC_CH_MOTOR_L_CTRL2, 0);
      ledcWrite(LEDC_CH_MOTOR_R_CTRL1, 0);
      ledcWrite(LEDC_CH_MOTOR_R_CTRL2, 0);
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

