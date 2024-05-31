/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define FC_TARGET_MCU     STM32F411

#define BOARD_NAME        JHEF411
#define MANUFACTURER_ID   JHEF

#define USE_ACCGYRO_BMI270
#define USE_ACC
#define USE_ACC_SPI_MPU6000
#define USE_GYRO
#define USE_GYRO_SPI_MPU6000
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P
#define USE_BARO_BMP280
#define USE_BARO_DPS310
// #define USE_FLASH
// #define USE_FLASH_W25Q128FV
#define USE_MAX7456

// -------------
#define USE_GPS
// #define USE_GPS_LAP_TIMER
// #define USE_GPS_NMEA
#define USE_GPS_PLUS_CODES
// #define USE_GPS_RESCUE
// #define USE_GPS_UBLOX
#define USE_DSHOT
// #define USE_DSHOT_BB
// #define USE_DSHOT_BITBAND
// #define USE_DSHOT_BITBANG
// #define USE_DSHOT_CACHE_MGMT
// #define USE_DSHOT_DMAR
// #define USE_DSHOT_TELEMETRY
// #define USE_DSHOT_TELEMETRY_STATS
#define USE_OSD
// #define USE_OSD_ADJUSTMENTS
// #define USE_OSD_HD
// #define USE_OSD_OVER_MSP_DISPLAYPORT
// #define USE_OSD_PROFILES
// #define USE_OSD_QUICK_MENU
#define USE_OSD_SD
// #define USE_OSD_STICK_OVERLAY
#define USE_VTX
// #define USE_VTX_COMMON
// #define USE_VTX_COMMON_FREQ_API
// #define USE_VTX_CONTROL
// #define USE_VTX_MSP
// #define USE_VTX_RTC6705
// #define USE_VTX_RTC6705_SOFTSPI
// #define USE_VTX_SMARTAUDIO
// #define USE_VTX_TABLE
// #define USE_VTX_TRAMP
#define USE_TELEMETRY
#define USE_TELEMETRY_CRSF
// #define USE_TELEMETRY_FRSKY_HUB
// #define USE_TELEMETRY_GHST
// #define USE_TELEMETRY_HOTT
// #define USE_TELEMETRY_IBUS
// #define USE_TELEMETRY_IBUS_EXTENDED
// #define USE_TELEMETRY_JETIEXBUS
// #define USE_TELEMETRY_LTM
// #define USE_TELEMETRY_MAVLINK
// #define USE_TELEMETRY_NRF24_LTM
// #define USE_TELEMETRY_SENSORS_DISABLED_DETAILS
// #define USE_TELEMETRY_SMARTPORT
// #define USE_TELEMETRY_SRXL
// -------------

// -------------
#define USE_RX_SPI
#define USE_RX_EXPRESSLRS
#define USE_RX_EXPRESSLRS_TELEMETRY
#define USE_RX_SX1280
#define RX_CHANNELS_AETR
#define RX_SPI_DEFAULT_PROTOCOL         RX_SPI_EXPRESSLRS
#define DEFAULT_RX_FEATURE              FEATURE_RX_SPI
#define RX_SPI_PROTOCOL                 EXPRESSLRS
#define RX_EXPRESSLRS_TIMER_INSTANCE    TIM5
#define RX_EXPRESSLRS_SPI_RESET_PIN     PB9
#define RX_EXPRESSLRS_SPI_BUSY_PIN      PB1
#define RX_SPI_CS                       PB2
#define RX_SPI_EXTI                     PA15
// #define RX_SPI_BIND                     PB2
#define RX_SPI_LED                      PC13
// -------------

#define BEEPER_PIN           PC14
#define MOTOR1_PIN           PA8
#define MOTOR2_PIN           PA9
#define MOTOR3_PIN           PA10
#define MOTOR4_PIN           PB0
#define MOTOR5_PIN           PB4
// #define RX_PPM_PIN           PA2
// #define RX_PWM1_PIN          PB1
// #define RX_PWM2_PIN          PA3
// #define LED_STRIP_PIN        PA15
#define UART1_TX_PIN         PB6
#define UART2_TX_PIN         PA2
#define UART1_RX_PIN         PB7
#define UART2_RX_PIN         PA3
// #define I2C1_SCL_PIN         PB8
// #define I2C1_SDA_PIN         PB9
// #define LED0_PIN             PC13
#define SPI1_SCK_PIN         PA5
#define SPI2_SCK_PIN         PB13
#define SPI1_SDI_PIN         PA6
#define SPI2_SDI_PIN         PB14
#define SPI1_SDO_PIN         PA7
#define SPI2_SDO_PIN         PB15
#define CAMERA_CONTROL_PIN   PB10
#define ADC_VBAT_PIN         PA0
// #define ADC_RSSI_PIN         PB1
#define ADC_CURR_PIN         PA1
#define PINIO1_PIN           PB5
// #define FLASH_CS_PIN         PB2
#define MAX7456_SPI_CS_PIN   PB12
//----
#define RX_SPI_CS_PIN        PB2
#define RX_SPI_EXTI_PIN      PA15
// #define RX_SPI_BIND_PIN      PB10
#define RX_SPI_LED_PIN       PC13
#define RX_SPI_EXPRESSLRS_RESET_PIN PB9
#define RX_SPI_EXPRESSLRS_BUSY_PIN PB1
//----
#define GYRO_1_EXTI_PIN      PB3
#define GYRO_1_CS_PIN        PA4
#define USB_DETECT_PIN       PC15

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA2 , 3, -1) \
    TIMER_PIN_MAP( 1, PA8 , 1,  1) \
    TIMER_PIN_MAP( 2, PA9 , 1,  1) \
    TIMER_PIN_MAP( 3, PA10, 1,  1) \
    TIMER_PIN_MAP( 4, PB0 , 2,  0) \
    TIMER_PIN_MAP( 5, PB4 , 1,  0)  
    // TIMER_PIN_MAP( 7, PA3 , 2,  1) 
    // TIMER_PIN_MAP( 8, PB10, 1,  0) 
    // TIMER_PIN_MAP( 9, PA15, 1,  0)
    // TIMER_PIN_MAP( 6, PB1 , 2,  0)  
 

#define ADC1_DMA_OPT        1

//---
#define RX_SPI_INSTANCE SPI2
// #define RX_SPI_LED_INVERTED
//---
#define MAG_I2C_INSTANCE (I2CDEV_1)
#define USE_BARO
#define BARO_I2C_INSTANCE (I2CDEV_1)
// #define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH
#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SCALE 170
#define BEEPER_INVERTED
#define SYSTEM_HSE_MHZ 8
#define MAX7456_SPI_INSTANCE SPI2
// #define FLASH_SPI_INSTANCE SPI2
#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_ALIGN CW180_DEG
#define GYRO_1_ALIGN_YAW 1800
#define GYRO_2_SPI_INSTANCE SPI1

//TODO #define EXPRESSLRS_DOMAIN ISM2400
 
