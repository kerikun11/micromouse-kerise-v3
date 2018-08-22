/**
   @file  global.h
   @brief グローバル変数のextern宣言をするファイル．
*/
#pragma once

/* Driver */
#include "buzzer.h"
extern Buzzer bz;
#include "led.h"
extern LED led;
#include "motor.h"
extern Motor mt;
#include "fan.h"
extern Fan fan;

/* Sensor */
#include "button.h"
extern Button btn;
#include "imu.h"
extern IMU imu;
#include "encoder.h"
extern Encoder enc;
#include "reflector.h"
extern Reflector ref;
#include "tof.h"
extern ToF tof;

/* Supporter */
#include "UserInterface.h"
extern UserInterface ui;
#include "SpeedController.h"
extern SpeedController sc;
#include "WallDetector.h"
extern WallDetector wd;
#include "Emergency.h"
extern Emergency em;
#include "ExternalController.h"
extern ExternalController ec;
#include "Logger.h"
extern Logger lg;

/* Conductor */
#include "SearchRun.h"
extern SearchRun sr;
#include "FastRun.h"
extern FastRun fr;
#include "MazeSolver.h"
extern MazeSolver ms;

