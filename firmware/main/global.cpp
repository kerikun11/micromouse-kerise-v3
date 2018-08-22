/**
   @file  global.cpp
   @brief グローバル変数の実体を定義するC++ファイル．
*/
#include "config.h"

/* Driver */
#include "buzzer.h"
#include "led.h"
#include "motor.h"
#include "fan.h"
Buzzer bz(BUZZER_PIN, LEDC_CH_BUZZER);
LED led(LED_PINS);
Motor mt;
Fan fan(FAN_PIN, LEDC_CH_FAN);

/* Sensor */
#include "button.h"
#include "imu.h"
#include "encoder.h"
#include "reflector.h"
#include "tof.h"
Button btn(BUTTON_PIN);
IMU imu;
Encoder enc;
Reflector ref(PR_TX_PINS, PR_RX_PINS);
ToF tof(TOF_SDA_PIN, TOF_SCL_PIN);

/* Supporter */
#include "UserInterface.h"
#include "SpeedController.h"
#include "WallDetector.h"
#include "Emergency.h"
#include "ExternalController.h"
#include "Logger.h"
UserInterface ui;
SpeedController sc;
WallDetector wd;
Emergency em;
ExternalController ec;
Logger lg;

/* Conductor */
#include "SearchRun.h"
#include "FastRun.h"
#include "MazeSolver.h"
SearchRun sr;
FastRun fr;
MazeSolver ms;

