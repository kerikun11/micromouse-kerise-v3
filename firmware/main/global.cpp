/**
   @file  global.cpp
   @brief グローバル変数の実体を定義するC++ファイル．
*/
#include "config/config.h"

/* Driver */
#include "buzzer.h"
Buzzer bz(BUZZER_PIN, LEDC_CH_BUZZER);
#include "led.h"
LED led(LED_PINS);
#include "motor.h"
Motor mt;
#include "fan.h"
Fan fan(FAN_PIN, LEDC_CH_FAN);

/* Sensor */
#include "button.h"
Button btn(BUTTON_PIN);
#include "imu.h"
IMU imu;
#include "encoder.h"
Encoder enc;
#include "reflector.h"
Reflector ref(PR_TX_PINS, PR_RX_PINS);
#include "tof.h"
ToF tof(I2C_PORT_NUM_TOF);

/* Supporter */
#include "UserInterface.h"
UserInterface ui;
#include "SpeedController.h"
SpeedController sc;
#include "WallDetector.h"
WallDetector wd;
#include "Emergency.h"
Emergency em;

/* Conductor */
#include "SearchRun.h"
SearchRun sr;
#include "FastRun.h"
FastRun fr;
#include "MazeSolver.h"
MazeSolver ms;
