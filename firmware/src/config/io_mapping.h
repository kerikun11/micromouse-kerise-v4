#pragma once

#include "model.h"

/* Hardware Mapping */
#define BAT_VOL_PIN 35
#define PR_TX_PINS                                                             \
  { 12, 13, 14, 15 }
#define PR_RX_PINS                                                             \
  { 36, 37, 38, 39 }
#define BUTTON_PIN GPIO_NUM_0

/* SPI Bus*/
#define CONFIG_SPI_MOSI_PIN GPIO_NUM_2
#define CONFIG_SPI_MISO_PIN GPIO_NUM_10
#define CONFIG_SPI_SCLK_PIN GPIO_NUM_9
#define CONFIG_SPI_HOST HSPI_HOST
#define CONFIG_SPI_DMA_CHAIN 0

#if KERISE_SELECT == 4 || KERISE_SELECT == 3

/* SPI Device */
#define AS5048A_SPI_HOST CONFIG_SPI_HOST
#define AS5048A_CS_PIN GPIO_NUM_4

#define ICM20602_SPI_HOST CONFIG_SPI_HOST
#define ICM20602_L_CS_PIN GPIO_NUM_26
#define ICM20602_R_CS_PIN GPIO_NUM_27
#define ICM20602_CS_PINS                                                       \
  { ICM20602_L_CS_PIN, ICM20602_R_CS_PIN }

/* for pull-up*/
#define CONFIG_SPI_CS_PINS                                                     \
  { AS5048A_CS_PIN, ICM20602_L_CS_PIN, ICM20602_R_CS_PIN }

#elif KERISE_SELECT == 5

/* SPI Device */
#define MA730_SPI_HOST CONFIG_SPI_HOST
#define MA730_L_CS_PIN GPIO_NUM_4
#define MA730_R_CS_PIN GPIO_NUM_5
#define MA730_CS_PINS                                                          \
  { MA730_L_CS_PIN, MA730_R_CS_PIN }

#define ICM20602_SPI_HOST CONFIG_SPI_HOST
#define ICM20602_CS_PIN GPIO_NUM_26

/* for pull-up */
#define CONFIG_SPI_CS_PINS                                                     \
  { MA730_L_CS_PIN, MA730_R_CS_PIN, ICM20602_CS_PIN }

#endif

/* I2C Bus*/
#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22
#define I2C_PORT_NUM I2C_NUM_0

/* I2C Device */
#define I2C_PORT_NUM_TOF I2C_PORT_NUM
#define I2C_PORT_NUM_LED I2C_PORT_NUM

/* Motor Control PWM */
#define MOTOR_L_CTRL1_PIN GPIO_NUM_32
#define MOTOR_L_CTRL2_PIN GPIO_NUM_33
#define MOTOR_R_CTRL1_PIN GPIO_NUM_18
#define MOTOR_R_CTRL2_PIN GPIO_NUM_19
#define FAN_PIN GPIO_NUM_23

/* LED Controller */
#define BUZZER_PIN 25
#define LEDC_CH_BUZZER 4
