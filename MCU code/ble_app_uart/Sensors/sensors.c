#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_spi.h"
#include "sensors.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// Initialize instances
#define TWI_INSTANCE_ID 0
#define SPI_INSTANCE_ID 1

#define SPI_BUFSIZE  8
uint8_t spi_tx_buf[SPI_BUFSIZE];
uint8_t spi_rx_buf[SPI_BUFSIZE];

// Flag to indicate the transfer state
static volatile bool m_xfer_done = false;
static volatile bool spi_xfer_done = false;

// Create a Handle for communications
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static const nrf_drv_spi_t m_spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE_ID);

// Event handler
void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context) {
  //Check the event to see what type of event occurred
  switch (p_event->type) {
  //If data transmission or receiving is finished
  case NRF_DRV_TWI_EVT_DONE:
    m_xfer_done = true; //Set the flag
    break;

  default:
    // Do nothing
    break;
  }
}

void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
    spi_xfer_done = true;
}

// Initialize the TWI as the Master device
void twi_master_init(void) {
  ret_code_t err_code;

  // Configure the settings for TWI communication
  const nrf_drv_twi_config_t twi_config = {
      .scl = TWI_SCL_M,                            //SCL Pin
      .sda = TWI_SDA_M,                            //SDA Pin
      .frequency = NRF_DRV_TWI_FREQ_400K,          //Communication Speed
      .interrupt_priority = 6,                     //Interrupt Priority(Note: if using Bluetooth then select priority carefully)
      .clear_bus_init = false                      //Automatically clear bus
  };

  //Function to initialize the TWI communication
  err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
  APP_ERROR_CHECK(err_code);

  //Enable the TWI Communication
  nrf_drv_twi_enable(&m_twi);
}

// Initialize the SPI
void spi_init(void)
{
  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

  spi_config.ss_pin     = SPI_SS;
  spi_config.miso_pin   = SPI_MISO;
  spi_config.mosi_pin   = SPI_MOSI;
  spi_config.sck_pin    = SPI_SCK;
  spi_config.frequency  = NRF_SPI_FREQ_4M;

  APP_ERROR_CHECK(nrf_drv_spi_init(&m_spi, &spi_config, spi_event_handler, NULL));
}

/*
----------------------MMC5983MA-----------------------
*/

// Function to write a single byte to MMC5983MA's internal register
bool mmc5983ma_register_write(uint8_t register_address, uint8_t value) {
  ret_code_t err_code;
  uint8_t tx_buf[MMC5983MA_ADDRESS_LEN + 1];

  // Write the register address and data into transmit buffer
  tx_buf[0] = register_address;
  tx_buf[1] = value;

  // Set the flag to false to show the transmission is not yet completed
  m_xfer_done = false;

  // Transmit the data over TWI Bus
  err_code = nrf_drv_twi_tx(&m_twi, MMC5983MA_ADDRESS, tx_buf, MMC5983MA_ADDRESS_LEN + 1, false);
  APP_ERROR_CHECK(err_code);

  // Wait until the transmission of the data is finished
  while (m_xfer_done == false) {
  }

  // If there is no error then return true else return false
  if (NRF_SUCCESS != err_code) {
    return false;
  }

  return true;
}

// Function to read data from the MMC5983MA
bool mmc5983ma_register_read(uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes) {
  ret_code_t err_code;

  // Set the flag to false to show receiving is not yet completed
  m_xfer_done = false;

  // Send the register address where we want to write the data
  err_code = nrf_drv_twi_tx(&m_twi, MMC5983MA_ADDRESS, &register_address, 1, true);
  APP_ERROR_CHECK(err_code);

  // Wait for the transmission to get completed
  while (m_xfer_done == false) {
  }

  // If transmission was not successful, exit the function with false as return value
  if (NRF_SUCCESS != err_code) {
    return false;
  }
  
  nrf_delay_us(1);
  // Set the flag again so that we can read data from the MMC5983MA's internal register
  m_xfer_done = false;

  // Receive the data from the MMC5983MA
  err_code = nrf_drv_twi_rx(&m_twi, MMC5983MA_ADDRESS, destination, number_of_bytes);
  APP_ERROR_CHECK(err_code);

  // Wait until the transmission is completed
  while (m_xfer_done == false) {
  }

  // If data was successfully read, return true else return false
  if (NRF_SUCCESS != err_code) {
    return false;
  }

  return true;
}

// Function to verify the product ID
bool mmc5983ma_verify_product_id(void) {
  uint8_t who_am_i; // Create a variable to hold the who_am_i value

  if (mmc5983ma_register_read(MMC5983MA_PRODUCT_ID, &who_am_i, 1)) {
    if (who_am_i != MMC5983MA_ADDRESS) {
      return false;
    } else {
      return true;
    }
  } else {
    return false;
  }
}

// Function to initialize the MMC5983MA
bool mmc5983ma_init(void) {
  bool transfer_succeeded = true;

  // Check the ID to confirm that we are communicating with the right device
  transfer_succeeded &= mmc5983ma_verify_product_id();

  // enable auto set/reset (bit 5 == 1)
  // this set/reset is a low current sensor offset measurement for normal use
  (void)mmc5983ma_register_write(MMC5983MA_CONTROL_0, 1 << 5);
  // set magnetometer bandwidth
  (void)mmc5983ma_register_write(MMC5983MA_CONTROL_1, MBW_800Hz);  
  // enable continuous measurement mode (bit 3 == 1), set sample rate
  // enable automatic Set/Reset (bit 7 == 1), set set/reset rate
  // this set/reset is a high-current "deGaussing" that should be used only to recover from 
  // high magnetic field detuning of the magnetoresistive film
  (void)mmc5983ma_register_write(MMC5983MA_CONTROL_2, 0 << 3 | MODR_ONESHOT);
  return transfer_succeeded;
}

// Read the mag values from the MMC5983MA's internal registers
bool MMC5983MA_ReadMag(uint8_t *p_mag) {
  bool ret = false;
  
  (void)mmc5983ma_register_write(MMC5983MA_CONTROL_0, 1 << 0);
  if (mmc5983ma_register_read(MMC5983MA_MAG_OUT, p_mag, 7) == true) {
    ret = true;
  }

  return ret;
}

/*
----------------------ITG3701-----------------------
*/

// Function to write a single byte to ITG3701's internal register
bool itg3701_register_write(uint8_t register_address, uint8_t value) {
  ret_code_t err_code;
  uint8_t tx_buf[ITG3701_ADDRESS_LEN + 1];

  // Write the register address and data into transmit buffer
  tx_buf[0] = register_address;
  tx_buf[1] = value;

  // Set the flag to false to show the transmission is not yet completed
  m_xfer_done = false;

  // Transmit the data over TWI Bus
  err_code = nrf_drv_twi_tx(&m_twi, ITG3701_ADDRESS, tx_buf, ITG3701_ADDRESS_LEN + 1, false);
  APP_ERROR_CHECK(err_code);

  // Wait until the transmission of the data is finished
  while (m_xfer_done == false);

  // If there is no error then return true else return false
  if (NRF_SUCCESS != err_code) {
    return false;
  }

  return true;
}

// Function to read data from the ITG3701
bool itg3701_register_read(uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes) {
  ret_code_t err_code;

  // Set the flag to false to show receiving is not yet completed
  m_xfer_done = false;

  // Send the register address where we want to write the data
  err_code = nrf_drv_twi_tx(&m_twi, ITG3701_ADDRESS, &register_address, 1, true);
  APP_ERROR_CHECK(err_code);

  // Wait for the transmission to get completed
  while (m_xfer_done == false);

  // If transmission was not successful, exit the function with false as return value
  if (NRF_SUCCESS != err_code) {
    return false;
  }
  
  // Set the flag again so that we can read data from the ITG3701's internal register
  m_xfer_done = false;

  // Receive the data from the ITG3701
  err_code = nrf_drv_twi_rx(&m_twi, ITG3701_ADDRESS, destination, number_of_bytes);
  APP_ERROR_CHECK(err_code);

  // Wait until the transmission is completed
  while (m_xfer_done == false);

  // If data was successfully read, return true else return false
  if (NRF_SUCCESS != err_code) {
    return false;
  }

  return true;
}

// Function to verify the product ID
bool itg3701_verify_product_id(void) {
  uint8_t who_am_i; // Create a variable to hold the who_am_i value

  if (itg3701_register_read(ADDRESS_WHO_AM_I, &who_am_i, 1)) {
    if (who_am_i != ITG3701_WHO_AM_I) {
      return false;
    } else {
      return true;
    }
  } else {
    return false;
  }
}

// Function to initialize the ITG3701
bool itg3701_init(void) {
  bool transfer_succeeded = true;

  // Check the ID to confirm that we are communicating with the right device
  transfer_succeeded &= itg3701_verify_product_id();

  // Set the registers with the required values
  (void)itg3701_register_write(ITG3701_PWR_MGMT_1, 0x00);
  nrf_delay_ms(100);
  (void)itg3701_register_write(ITG3701_PWR_MGMT_1, 0x01);
  nrf_delay_ms(200);

  (void)itg3701_register_write(ITG3701_CONFIG, 0x03);
  (void)itg3701_register_write(ITG3701_SMPLRT_DIV, 0x09);
  uint8_t c;
//  (void)itg3701_register_read(ITG3701_GYRO_CONFIG, &c, 1);
  while (itg3701_register_read(ITG3701_GYRO_CONFIG, &c, 1) == false) {
    nrf_delay_ms(1);
    transfer_succeeded = false;
  }
  transfer_succeeded = true;
  (void)itg3701_register_write(ITG3701_GYRO_CONFIG, c & ~0x03);
  (void)itg3701_register_write(ITG3701_GYRO_CONFIG, c & ~0x18);
  (void)itg3701_register_write(ITG3701_GYRO_CONFIG, c | 0x18);
  (void)itg3701_register_write(ITG3701_INT_PIN_CFG, 0x00);
  (void)itg3701_register_write(ITG3701_INT_ENABLE, 0x01);

  return transfer_succeeded;
}

//// Function to initialize the ITG3701
//bool itg3701_init(void) {
//  bool transfer_succeeded = true;
//
//  // Check the ID to confirm that we are communicating with the right device
//  transfer_succeeded &= itg3701_verify_product_id();
//
//  // Set the registers with the required value
//  (void)itg3701_register_write(ITG3701_PWR_MGMT_1, 0x80);
//  nrf_delay_ms(100);
//  (void)itg3701_register_write(ITG3701_PWR_MGMT_1, 0x09);
//  (void)itg3701_register_write(ITG3701_CONFIG, 0x01);
//  (void)itg3701_register_write(ITG3701_SMPLRT_DIV, 0x00);
//  (void)itg3701_register_write(ITG3701_GYRO_CONFIG, 0x18);
//
//  return transfer_succeeded;
//}

// Read the gyro values from the ITG3701's internal registers
bool ITG3701_ReadGyro(uint8_t *p_gyro) {
  uint8_t c;
  bool ret = false;
  
  (void)(itg3701_register_read(ITG3701_INT_STATUS, &c, 1));
  
  if (itg3701_register_read(ITG3701_GYRO_OUT, p_gyro, 6) == true  && c == 0x01) {
//  if (itg3701_register_read(ITG3701_GYRO_OUT, buf, 6) == true) {
    ret = true;
  }
//    NRF_LOG_INFO("%d", *p_gyro);
  return ret;
}

/*
----------------------MAX1167-----------------------
*/

bool read_adc(uint8_t input, uint16_t *p_output)
{
  spi_xfer_done = false;
  
  spi_tx_buf[0] = input;
    
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&m_spi, spi_tx_buf, 3, spi_rx_buf, 3));
  while(!spi_xfer_done);

//  NRF_LOG_INFO("%d", *spi_rx_buf);

  *p_output = (spi_rx_buf[1] << 8) | spi_rx_buf[2];
      
  return true;
}

void ReadAcc(uint16_t *p_acc)
{
      read_adc(MAX1167_IN1, p_acc++);
      read_adc(MAX1167_IN2, p_acc++);
      read_adc(MAX1167_IN3, p_acc);
}

void ReadHighGyro(uint16_t *p_gyro)
{
      read_adc(MAX1167_IN0, p_gyro);
}

