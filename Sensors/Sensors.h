#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>
#include <stdint.h>

// Include necessary headers for nRF52 platform
#include "nrf_drv_spi.h"

/*
---------------------SPI Definitions----------------------
*/

// Define SPI instance IDs
#define SPI_INSTANCE_1_ID 1 //  Accelerometer: ADXL314WBCPZ
#define SPI_INSTANCE_2_ID 2 //  3-Axis Gyroscope + High Rate Z-Axis Gyroscope
#define SPI_INSTANCE_3_ID 3 //  Magnetometer: MMC5983MA

// SPI Buffer Size
#define SPI_BUFSIZE  8 // Shared Buffer Size

// SPI PINOUT
#define SPI_CS1         24 //  Accelerometer: ADXL314WBCPZ
#define SPI_CS2         44 //  3-Axis Gyroscope: ITG3701
#define SPI_CS3         36 //  High Rate Z-Axis Gyroscope: ADIS16266BCCZ
#define SPI_CS4         48 //  Magnetometer: MMC5983MA

#define SPI_MISO_A      16 //  Accelerometer: ADXL314WBCPZ
#define SPI_MISO_B      23 //  3-Axis Gyroscope + High Rate Z-Axis Gyroscope
#define SPI_MISO_C      45 //  Magnetometer: MMC5983MA

#define SPI_MOSI_A      19 //  Accelerometer: ADXL314WBCPZ
#define SPI_MOSI_B      22 //  3-Axis Gyroscope + High Rate Z-Axis Gyroscope
#define SPI_MOSI_C      43 //  Magnetometer: MMC5983MA

#define SPI_SCLK_A      42 //  Accelerometer: ADXL314WBCPZ
#define SPI_SCLK_B      18 //  3-Axis Gyroscope + High Rate Z-Axis Gyroscope
#define SPI_SCLK_C      25 //  Magnetometer: MMC5983MA

// Extern variables for SPI communication buffers
extern uint8_t spi_tx_buf[SPI_BUFSIZE];
extern uint8_t spi_rx_buf[SPI_BUFSIZE];

// SPI Functions
void spi_1_init(void); //  Accelerometer: ADXL314WBCPZ
void spi_2_init(void); //  3-Axis Gyroscope + High Rate Z-Axis Gyroscope
void spi_3_init(void); //  Magnetometer: MMC5983MA

/*
---------------------Sensors Definitions----------------------
*/

/*
----------------------ADXL314WBCPZ-RL-----------------------
-----------------------ACCELEROMETER------------------------
*/

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


/*
------------------------ITG3701-------------------------
----------------------3-AXIS GYRO-----------------------
*/

#define ITG3701_ADDRESS_LEN  1          //ITG3701
#define ITG3701_ADDRESS     (0xD0>>1)   //ITG3701 Device Address
#define ITG3701_WHO_AM_I     0x68U      //ITG3701 ID

#define ITG3701_GYRO_OUT     0x43

#define ADDRESS_WHO_AM_I    (0x75U)     // WHO_AM_I register identifies the device. Expected value is 0x68.

// ITG3701 registers addresses
#define ITG3701_XG_OFFS_TC_H     0x04
#define ITG3701_XG_OFFS_TC_L     0x05
#define ITG3701_YG_OFFS_TC_H     0x07
#define ITG3701_YG_OFFS_TC_L     0x08
#define ITG3701_ZG_OFFS_TC_H     0x0A
#define ITG3701_ZG_OFFS_TC_L     0x0B
#define ITG3701_XG_OFFS_USRH     0x13   // User-defined trim values for gyroscope
#define ITG3701_XG_OFFS_USRL     0x14
#define ITG3701_YG_OFFS_USRH     0x15
#define ITG3701_YG_OFFS_USRL     0x16
#define ITG3701_ZG_OFFS_USRH     0x17
#define ITG3701_ZG_OFFS_USRL     0x18
#define ITG3701_SMPLRT_DIV       0x19
#define ITG3701_CONFIG           0x1A
#define ITG3701_GYRO_CONFIG      0x1B
#define ITG3701_FIFO_EN          0x23
#define ITG3701_INT_PIN_CFG      0x37
#define ITG3701_INT_ENABLE       0x38
#define ITG3701_INT_STATUS       0x3A
#define ITG3701_TEMP_OUT_H       0x41
#define ITG3701_TEMP_OUT_L       0x42
#define ITG3701_GYRO_XOUT_H      0x43
#define ITG3701_GYRO_XOUT_L      0x44
#define ITG3701_GYRO_YOUT_H      0x45
#define ITG3701_GYRO_YOUT_L      0x46
#define ITG3701_GYRO_ZOUT_H      0x47
#define ITG3701_GYRO_ZOUT_L      0x48
#define ITG3701_USER_CTRL        0x6A  
#define ITG3701_PWR_MGMT_1       0x6B   // Device defaults to the SLEEP mode
#define ITG3701_PWR_MGMT_2       0x6C
#define ITG3701_FIFO_COUNTH      0x72
#define ITG3701_FIFO_COUNTL      0x73
#define ITG3701_FIFO_R_W         0x74

bool itg3701_register_write(uint8_t register_address, uint8_t value);
bool itg3701_register_read(uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes);
bool itg3701_verify_product_id(void);
bool itg3701_init(void);
bool ITG3701_ReadGyro(uint8_t *p_gyro);


/*
------------------------MMC5983MA------------------------
----------------------MAGNETOMERTER-----------------------
*/

#define MMC5983MA_ADDRESS_LEN  1          //MMC5983MA3701

#define MMC5983MA_XOUT_0        0x00
#define MMC5983MA_XOUT_1        0x01
#define MMC5983MA_YOUT_0        0x02
#define MMC5983MA_YOUT_1        0x03
#define MMC5983MA_ZOUT_0        0x04
#define MMC5983MA_ZOUT_1        0x05
#define MMC5983MA_XYZOUT_2      0x06
#define MMC5983MA_TOUT          0x07
#define MMC5983MA_STATUS        0x08
#define MMC5983MA_CONTROL_0     0x09
#define MMC5983MA_CONTROL_1     0x0A
#define MMC5983MA_CONTROL_2     0x0B
#define MMC5983MA_CONTROL_3     0x0C
#define MMC5983MA_PRODUCT_ID    0x2F // Should be 0x30

#define MMC5983MA_ADDRESS       0x30

#define MMC5983MA_MAG_OUT       0x00

// Sample rates
#define MODR_ONESHOT   0x00
#define MODR_1Hz       0x01
#define MODR_10Hz      0x02
#define MODR_20Hz      0x03
#define MODR_50Hz      0x04
#define MODR_100Hz     0x05
#define MODR_200Hz     0x06 // BW = 0x01 only
#define MODR_1000Hz    0x07 // BW = 0x11 only

//Bandwidths
#define MBW_100Hz 0x00  // 8 ms measurement time
#define MBW_200Hz 0x01  // 4 ms
#define MBW_400Hz 0x02  // 2 ms
#define MBW_800Hz 0x03  // 0.5 ms


// Set/Reset as a function of measurements
#define MSET_1     0x00 // Set/Reset each data measurement
#define MSET_25    0x01 // each 25 data measurements
#define MSET_75    0x02
#define MSET_100   0x03
#define MSET_250   0x04
#define MSET_500   0x05
#define MSET_1000  0x06
#define MSET_2000  0x07

bool mmc5983ma_register_write(uint8_t register_address, uint8_t value);
bool mmc5983ma_register_read(uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes);
bool mmc5983ma_verify_product_id(void);
bool mmc5983ma_init(void);
bool MMC5983MA_ReadMag(uint8_t *p_mag);


/*
-------------------------ADIS16266BCCZ_AF-------------------------
----------------------HIGH RATE Z-AXIS GYRO-----------------------
*/

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


#endif // SENSORS_H
