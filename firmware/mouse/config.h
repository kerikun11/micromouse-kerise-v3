#pragma once

/* Hardware Mapping */
#define BAT_VOL_PIN             35
#define PR_TX_PINS              {12, 13, 12, 13}
#define PR_RX_PINS              {36, 38, 39, 37}
#define BUZZER_PIN              32
#define LED_PINS                {2, 4, 5, 27}
#define BUTTON_PIN              0
#define MOTOR_L_CTRL1_PIN       25 // 16
#define MOTOR_L_CTRL2_PIN       26 // 17
#define MOTOR_R_CTRL1_PIN       16 // 25
#define MOTOR_R_CTRL2_PIN       17 // 26
#define FAN_PIN                 33
#define AS5048A_MOSI_PIN        23
#define AS5048A_MISO_PIN        19
#define AS5048A_SCLK_PIN        18
#define AS5048A_CS_PIN          15
#define AS5048A_SPI_HOST        HSPI_HOST
#define AS5048A_SPI_DMA_CHAIN   1
#define ICM20602_MOSI_PIN       23
#define ICM20602_MISO_PIN       19
#define ICM20602_SCLK_PIN       18
#define ICM20602_CS_PIN         14
#define ICM20602_SPI_HOST       HSPI_HOST
#define ICM20602_SPI_DMA_CHAIN  1
#define TOF_SDA_PIN             21
#define TOF_SCL_PIN             22

/* LEDC Channel */
#define LEDC_CH_MOTOR_L_CTRL1   0
#define LEDC_CH_MOTOR_L_CTRL2   1
#define LEDC_CH_MOTOR_R_CTRL1   2
#define LEDC_CH_MOTOR_R_CTRL2   3

#define LEDC_CH_FAN             6

#define LEDC_CH_BUZZER          4

/* Machine Size Parameter */
#define MACHINE_ROTATION_RADIUS 16.75f
#define MACHINE_GEAR_RATIO      (13.0f/41.0f)
#define MACHINE_WHEEL_DIAMETER  13.604908755851063829787234042553f
#define MACHINE_TAIL_LENGTH     18.4f

/* Field Size Parameter */
#define SEGMENT_WIDTH           90.0f
#define SEGMENT_DIAGONAL_WIDTH  127.2792206135786f
#define WALL_THICKNESS          6.0f

