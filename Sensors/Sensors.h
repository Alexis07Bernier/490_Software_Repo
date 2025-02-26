#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>
#include <stdint.h>

// Include necessary headers for nRF52 platform
#include "nrf_drv_twi.h"
#include "nrf_drv_spi.h"

// Define SPI and I2C instance IDs
#define TWI_INSTANCE_ID 0
#define SPI_INSTANCE_ID 1

// SPI Buffer Size
#define SPI_BUFSIZE  8

// Extern variables for SPI communication buffers
extern uint8_t spi_tx_buf[SPI_BUFSIZE];
extern uint8_t spi_rx_buf[SPI_BUFSIZE];

// Sensor Definitions
// ---------------------- ADXL314 Accelerometer ----------------------
#define ADXL314_CS_PIN         10
#define ADXL314_SPI_READ       0x80
#define ADXL314_SPI_WRITE      0x00
#define ADXL314_MULTI_BYTE     0x40
#define ADXL314_DEVID_REG      0x00
#define ADXL314_POWER_CTL_REG  0x2D
#define ADXL314_DATA_FORMAT_REG 0x31
#define ADXL314_DATAX0_REG     0x32
#define ADXL314_DATAY0_REG     0x34
#define ADXL314_DATAZ0_REG     0x36

void adxl314_init(void);
void adxl314_write_register(uint8_t reg, uint8_t value);
uint8_t adxl314_read_register(uint8_t reg);
void adxl314_read_accel(int16_t *x, int16_t *y, int16_t *z);

// ---------------------- ITG3701 Gyroscope ----------------------
#define ITG3701_ADDRESS       0x68
#define ITG3701_ADDRESS_LEN   1
#define ADDRESS_WHO_AM_I      0x75
#define ITG3701_WHO_AM_I      0x68
#define ITG3701_PWR_MGMT_1    0x6B
#define ITG3701_CONFIG        0x1A
#define ITG3701_SMPLRT_DIV    0x19
#define ITG3701_GYRO_CONFIG   0x1B
#define ITG3701_INT_PIN_CFG   0x37
#define ITG3701_INT_ENABLE    0x38
#define ITG3701_INT_STATUS    0x3A
#define ITG3701_GYRO_OUT      0x43

bool itg3701_register_write(uint8_t register_address, uint8_t value);
bool itg3701_register_read(uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes);
bool itg3701_verify_product_id(void);
bool itg3701_init(void);
bool ITG3701_ReadGyro(uint8_t *p_gyro);

// ---------------------- MMC5983MA Magnetometer ----------------------
#define MMC5983MA_ADDRESS         0x30
#define MMC5983MA_ADDRESS_LEN     1
#define MMC5983MA_PRODUCT_ID      0x2F
#define MMC5983MA_CONTROL_0       0x09
#define MMC5983MA_CONTROL_1       0x0A
#define MMC5983MA_CONTROL_2       0x0B
#define MMC5983MA_MAG_OUT         0x00
#define MBW_800Hz                 0x00
#define MODR_ONESHOT              0x00

bool mmc5983ma_register_write(uint8_t register_address, uint8_t value);
bool mmc5983ma_register_read(uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes);
bool mmc5983ma_verify_product_id(void);
bool mmc5983ma_init(void);
bool MMC5983MA_ReadMag(uint8_t *p_mag);

// ---------------------- ADIS16266 High-Rate Gyroscope ----------------------
#define ADIS16266_CS_PIN      9
#define ADIS16266_PROD_ID     0x56
#define ADIS16266_GYRO_OUT    0x06
#define ADIS16266_SMPL_PRD    0x36
#define ADIS16266_SENS_AVG    0x38
#define ADIS16266_POWER_CTRL  0x3E
#define ADIS16266_WRITE       0x80
#define ADIS16266_READ        0x00

void adis16266_init(void);
void adis16266_write_register(uint8_t reg, uint16_t value);
uint16_t adis16266_read_register(uint8_t reg);
int16_t adis16266_read_gyro(void);

// ---------------------- SPI and TWI Functions ----------------------
void twi_master_init(void);
void spi_init(void);

#endif // SENSORS_H
