#pragma once

/* KERISE_VERSION */
#define KERISE_VERSION          3

/* Hardware Mapping */
#if KERISE_VERSION == 3

#define BAT_VOL_PIN             36

#define PR_TX_PINS              {16, 17, 16, 17}
#define PR_RX_PINS              {12, 13, 32, 33}

#define BUZZER_PIN              21
#define LED_PINS                {5, 2}
#define BUTTON_PIN              0

#define MOTOR_L_CTRL1_PIN       18
#define MOTOR_L_CTRL2_PIN       23
#define MOTOR_R_CTRL1_PIN       19
#define MOTOR_R_CTRL2_PIN       22

#define FAN_PIN                 15

#define AS5145_MISO_PIN         34
#define AS5145_MOSI_PIN         38
#define AS5145_SCLK_PIN         4
#define AS5145_CS_PIN           14

#define MPU6500_MOSI_PIN        27
#define MPU6500_MISO_PIN        35
#define MPU6500_SCLK_PIN        26
#define MPU6500_CS_PIN          25

#elif KERISE_VERSION == 3.2

#define BAT_VOL_PIN             35

#define PR_TX_PINS              {12, 13, 12, 13}
#define PR_RX_PINS              {36, 38, 39, 37}

#define BUZZER_PIN              32
#define LED_PINS                {2, 4, 5, 27}
#define BUTTON_PIN              0

#define MOTOR_L_CTRL1_PIN       16
#define MOTOR_L_CTRL2_PIN       17
#define MOTOR_R_CTRL1_PIN       25
#define MOTOR_R_CTRL2_PIN       26

#define FAN_PIN                 33

#define AS5145_MISO_PIN         34
#define AS5145_MOSI_PIN         38
#define AS5145_SCLK_PIN         4
#define AS5145_CS_PIN           14

#define MPU6500_MOSI_PIN        27
#define MPU6500_MISO_PIN        35
#define MPU6500_SCLK_PIN        26
#define MPU6500_CS_PIN          25

#endif

/* LEDC Channel */
#define LEDC_CH_MOTOR_L_CTRL1   0
#define LEDC_CH_MOTOR_L_CTRL2   1
#define LEDC_CH_MOTOR_R_CTRL1   2
#define LEDC_CH_MOTOR_R_CTRL2   3

#define LEDC_CH_FAN             6

#define LEDC_CH_BUZZER          4

/* Machine Size Parameter */
#define MACHINE_ROTATION_RADIUS 16.7f
#define MACHINE_GEAR_RATIO      (13.0f/41.0f) //< 10/30
#define MACHINE_WHEEL_DIAMETER  13.5f
#define MACHINE_TAIL_LENGTH     18.4f

/* Field Size Parameter */
#define SEGMENT_WIDTH           90.0f
#define SEGMENT_DIAGONAL_WIDTH  127.2792206135786f
#define WALL_THICKNESS          6.0f

