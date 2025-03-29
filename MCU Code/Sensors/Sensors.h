#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf_drv_spi.h"
#include "nrf_drv_gpiote.h"



// SPI Configuration
#define ADXL314_SPI_INSTANCE   0   // SPI instance for ADXL314
#define ADXL314_CS_PIN         8   // Chip Select pin for ADXL314 (P0.08)

#define MMC5983MA_SPI_INSTANCE 1   // SPI instance for MMC5983MA
#define MMC5983MA_CS_PIN       24  // Chip Select pin for MMC5983MA (P0.24)

#define ITG3701_SPI_INSTANCE   2   // SPI instance for ITG-3701 & ADIS16266
#define ITG3701_CS_PIN         20  // Chip Select pin for ITG-3701 (P0.20)
#define ADIS16266_CS_PIN       14  // Chip Select pin for ADIS16266 (P0.14)

// ADXL314 Registers
#define ADXL314_DEVID_REG      0x00  // Device ID register
#define ADXL314_POWER_CTL      0x2D  // Power control register
#define ADXL314_BW_RATE        0x2C  // Bandwidth and Output Data Rate register
#define ADXL314_INT_ENABLE     0x2E  // Interrupt enable control
#define ADXL314_INT_MAP        0x2F  // Interrupt mapping control
#define ADXL314_INT_SOURCE     0x30  // Source of interrupts
#define ADXL314_DATA_FORMAT    0x31  // Data format register
#define ADXL314_DATAX0         0x32  // X-axis data LSB
#define ADXL314_DATAY0         0x34  // Y-axis data LSB
#define ADXL314_DATAZ0         0x36  // Z-axis data LSB

// MMC5983MA Registers
#define MMC5983MA_PRODUCT_ID   0x2F  // Product ID register
#define MMC5983MA_CTRL_0       0x09  // Control register 0
#define MMC5983MA_CTRL_1       0x0A  // Control register 1
#define MMC5983MA_CTRL_2       0x0B  // Control register 2
#define MMC5983MA_CTRL_3       0x0C  // Control register 3
#define MMC5983MA_XOUT0        0x00  // X-axis output MSByte
#define MMC5983MA_YOUT0        0x02  // Y-axis output MSByte
#define MMC5983MA_ZOUT0        0x00  // Z-axis output MSByte
#define MMC5983MA_XYZOUT2      0x06  // Lower 2 bits for X, Y, Z
#define MMC5983MA_STATUS       0x08  // Status register

// ITG-3701 Registers
#define ITG3701_WHO_AM_I       0x75  // Device ID register
#define ITG3701_PWR_MGMT_1     0x6B  // Power Management Register 1
#define ITG3701_PWR_MGMT_2     0x6C  // Power Management Register 2
#define ITG3701_SMPLRT_DIV     0x19  // Sample rate divider
#define ITG3701_CONFIG         0x1A  // Configuration Register
#define ITG3701_GYRO_CONFIG    0x1B  // Gyroscope Configuration Register
#define ITG3701_INT_PIN_CFG    0x37  // Interrupt Configuration Register
#define ITG3701_INT_ENABLE     0x38  // Interrupt Enable Register
#define ITG3701_INT_STATUS     0x3A  // Interrupt Status Register
#define ITG3701_GYRO_XOUT_H    0x43  // X-axis High Byte
#define ITG3701_GYRO_XOUT_L    0x44  // X-axis Low Byte
#define ITG3701_GYRO_YOUT_H    0x45  // Y-axis High Byte
#define ITG3701_GYRO_YOUT_L    0x46  // Y-axis Low Byte
#define ITG3701_GYRO_ZOUT_H    0x47  // Z-axis High Byte
#define ITG3701_GYRO_ZOUT_L    0x48  // Z-axis Low Byte

// ADIS16266 Registers
#define ADIS16266_PROD_ID      0x56  // Device ID register
#define ADIS16266_GYRO_OUT     0x04  // Gyroscope Output Register
#define ADIS16266_GYRO_SCALE   0x16  // Scale Factor Register
#define ADIS16266_SMPL_PRD     0x36  // Sampling Period Register
#define ADIS16266_SENS_AVG     0x38  // Sensitivity & Filter Register
#define ADIS16266_MSC_CTRL     0x34  // Misc Control (Self-Test, Data Ready)
#define ADIS16266_GLOB_CMD     0x3E  // Global Commands Register
#define ADIS16266_GPIO_CTRL    0x32  // GPIO Control Register


// Function Prototypes
void configure_unconnected_pins(void);

static bool spi_write_register(const nrf_drv_spi_t *spi_instance, uint8_t cs_pin, uint8_t reg, uint8_t value);
static uint8_t spi_read_register(const nrf_drv_spi_t *spi_instance, uint8_t cs_pin, uint8_t reg);
void sensors_init(void);

bool adxl314_init(void);
bool adxl314_read_accel(int16_t *acc_values);
//void adxl314_data_ready_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
bool adxl314_data_ready(void);

bool mmc5983ma_init(void);
bool mmc5983ma_read_mag(int8_t *mag_values);
//void mmc5983ma_data_ready_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
bool mmc5983ma_data_ready(void);

bool itg3701_init(void);
bool itg3701_read_gyro(int16_t *gyro_values);
//void itg3701_data_ready_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
bool itg3701_data_ready(void);

bool adis16266_init(void);
bool adis16266_read_gyro(int16_t *gyro_values);
bool adis16266_verify_prod_id(void);
//void adis16266_data_ready_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

#endif /* SENSORS_H */
