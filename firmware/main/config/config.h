#pragma once

/* Hardware Mapping */
#define BAT_VOL_PIN 35
#define PR_TX_PINS                                                             \
  { 12, 13, 14, 15 }
#define PR_RX_PINS                                                             \
  { 36, 37, 38, 39 }
#define BUZZER_PIN 25
#define LED_SDA_PIN 21
#define LED_SCL_PIN 22
#define BUTTON_PIN GPIO_NUM_0

#define MOTOR_L_CTRL1_PIN 32
#define MOTOR_L_CTRL2_PIN 33
#define MOTOR_R_CTRL1_PIN 18
#define MOTOR_R_CTRL2_PIN 19
#define FAN_PIN 23

#define AS5048A_MOSI_PIN 2
#define AS5048A_MISO_PIN 10
#define AS5048A_SCLK_PIN 9
#define AS5048A_CS_PIN 4
#define AS5048A_SPI_HOST HSPI_HOST
#define AS5048A_SPI_DMA_CHAIN 0

#define ICM20602_MOSI_PIN 2
#define ICM20602_MISO_PIN 10
#define ICM20602_SCLK_PIN 9
#define ICM20602_L_CS_PIN 26
#define ICM20602_R_CS_PIN 27
#define ICM20602_CS_PINS                                                       \
  { ICM20602_L_CS_PIN, ICM20602_R_CS_PIN }
#define ICM20602_SPI_HOST HSPI_HOST
#define ICM20602_SPI_DMA_CHAIN 0

#define CONFIG_SPI_CS_PINS                                                     \
  { AS5048A_CS_PIN, ICM20602_L_CS_PIN, ICM20602_R_CS_PIN }

#define TOF_SDA_PIN 21
#define TOF_SCL_PIN 22

/* Port Channel Definitions */
#define LEDC_CH_MOTOR_L_CTRL1 0
#define LEDC_CH_MOTOR_L_CTRL2 1
#define LEDC_CH_MOTOR_R_CTRL1 2
#define LEDC_CH_MOTOR_R_CTRL2 3

#define LEDC_CH_FAN 6

#define LEDC_CH_BUZZER 4

#define I2C_PORT_NUM_LED I2C_NUM_0
#define I2C_PORT_NUM_TOF I2C_NUM_0

/* Machine Size Parameter */
#define MACHINE_ROTATION_RADIUS 15.0f
#define MACHINE_GEAR_RATIO (12.0f / 37.0f)
#define MACHINE_WHEEL_DIAMETER 12.8f
#define MACHINE_TAIL_LENGTH 16.4f

/* Field Size Parameter */
#define SEGMENT_WIDTH 90.0f
#define SEGMENT_DIAGONAL_WIDTH 127.2792206135786f
#define WALL_THICKNESS 6.0f