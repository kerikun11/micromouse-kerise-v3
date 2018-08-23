#pragma once

/* Hardware Mapping */
#define BAT_VOL_PIN 35
#define PR_TX_PINS                                                             \
  { 12, 13, 12, 13 }
#define PR_RX_PINS                                                             \
  { 36, 38, 39, 37 }
#define BUTTON_PIN GPIO_NUM_0
#define LED_PINS                                                               \
  { 2, 4, 5, 27 }

/* SPI Bus*/
#define CONFIG_SPI_MOSI_PIN GPIO_NUM_23
#define CONFIG_SPI_MISO_PIN GPIO_NUM_19
#define CONFIG_SPI_SCLK_PIN GPIO_NUM_18
#define CONFIG_SPI_HOST HSPI_HOST
#define CONFIG_SPI_DMA_CHAIN 0

/* SPI Device */
#define AS5048A_SPI_HOST CONFIG_SPI_HOST
#define AS5048A_CS_PIN GPIO_NUM_15

#define ICM20602_SPI_HOST CONFIG_SPI_HOST
#define ICM20602_CS_PIN GPIO_NUM_14

/* for pull-up*/
#define CONFIG_SPI_CS_PINS                                                     \
  { AS5048A_CS_PIN, ICM20602_CS_PIN }

/* I2C Bus*/
#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22
#define I2C_PORT_NUM I2C_NUM_0

/* I2C Device */
#define I2C_PORT_NUM_TOF I2C_PORT_NUM

/* LED Controller */
#define MOTOR_L_CTRL1_PIN 25 // 16
#define MOTOR_L_CTRL2_PIN 26 // 17
#define MOTOR_R_CTRL1_PIN 16 // 25
#define MOTOR_R_CTRL2_PIN 17 // 26

#define FAN_PIN 33

#define BUZZER_PIN 32

#define LEDC_CH_MOTOR_L_CTRL1 0
#define LEDC_CH_MOTOR_L_CTRL2 1
#define LEDC_CH_MOTOR_R_CTRL1 2
#define LEDC_CH_MOTOR_R_CTRL2 3

#define LEDC_CH_FAN 6

#define LEDC_CH_BUZZER 4

/* Machine Size Parameter */
#define MACHINE_ROTATION_RADIUS 16.75f
#define MACHINE_GEAR_RATIO (13.0f / 41.0f)
#define MACHINE_WHEEL_DIAMETER 13.807967f
#define MACHINE_TAIL_LENGTH 18.4f

/* Field Size Parameter */
#define SEGMENT_WIDTH 90.0f
#define SEGMENT_DIAGONAL_WIDTH 127.2792206135786f
#define WALL_THICKNESS 6.0f
