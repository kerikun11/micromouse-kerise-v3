#pragma once

void straight_x(const float distance, const float v_max, const float v_end);
void turn(const float angle);
void batteryLedIndicate(const float voltage);
void batteryCheck();
int waitForSelect(int range = 16);
bool waitForCover(bool side = false);
bool waitForFix();
bool restore();
bool backup();

