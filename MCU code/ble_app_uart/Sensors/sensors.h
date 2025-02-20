#ifndef SENSORS_H__
#define SENSORS_H__
#include "nrf_delay.h"
#include "nrf_gpio.h"

// Pins Settings
#define TWI_SCL_M       4         //I2C SCL Pin
#define TWI_SDA_M       5         //I2C SDA Pin

#define SPI_SS          15
#define SPI_MISO        2
#define SPI_MOSI        14
#define SPI_SCK         20

/*
----------------------MMC5983MA-----------------------
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

/*
----------------------ITG3701-----------------------
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


#define MAX1167_IN0              0x06
#define MAX1167_IN1              0x26
#define MAX1167_IN2              0x46
#define MAX1167_IN3              0x66

/*
----------------------Functions-----------------------
*/

void twi_master_init(void); // initialize the TWI communication
void spi_init(void);

bool mmc5983ma_init(void);    // initialize the MMC5983MA

/**
  @brief Function for writing a MMC5983MA register contents over TWI.
  @param[in]  register_address Register address to start writing to
  @param[in] value Value to write to register
  @retval true Register write succeeded
  @retval false Register write failed
*/
bool mmc5983ma_register_write(uint8_t register_address, const uint8_t value);

/**
  @brief Function for reading MMC5983MA register contents over TWI.
  Reads one or more consecutive registers.
  @param[in]  register_address Register address to start reading from
  @param[in]  number_of_bytes Number of bytes to read
  @param[out] destination Pointer to a data buffer where read data will be stored
  @retval true Register read succeeded
  @retval false Register read failed
*/
bool mmc5983ma_register_read(uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes);

/**
  @brief Function for reading and verifying MMC5983MA product ID.
  @retval true Product ID is what was expected
  @retval false Product ID was not what was expected
*/
bool mmc5983ma_verify_product_id(void);


bool MMC5983MA_ReadMag(uint8_t *p_mag);

////////////////////////////////////////////////////////////////

bool itg3701_init(void);    // initialize the ITG3701

/**
  @brief Function for writing a ITG3701 register contents over TWI.
  @param[in]  register_address Register address to start writing to
  @param[in] value Value to write to register
  @retval true Register write succeeded
  @retval false Register write failed
*/
bool itg3701_register_write(uint8_t register_address, const uint8_t value);

/**
  @brief Function for reading ITG3701 register contents over TWI.
  Reads one or more consecutive registers.
  @param[in]  register_address Register address to start reading from
  @param[in]  number_of_bytes Number of bytes to read
  @param[out] destination Pointer to a data buffer where read data will be stored
  @retval true Register read succeeded
  @retval false Register read failed
*/
bool itg3701_register_read(uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes);

/**
  @brief Function for reading and verifying ITG3701 product ID.
  @retval true Product ID is what was expected
  @retval false Product ID was not what was expected
*/
bool itg3701_verify_product_id(void);


bool ITG3701_ReadGyro(uint8_t *p_gyro);

//////////////////////////////////////////////////

bool read_adc(uint8_t input, uint16_t *p_output);

void ReadAcc(uint16_t *p_acc);
void ReadHighGyro(uint16_t *p_gyro);


#endif