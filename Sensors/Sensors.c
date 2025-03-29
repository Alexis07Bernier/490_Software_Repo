#include "sensors.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_gpio.h"

// SPI Instances
static const nrf_drv_spi_t spi_adxl = NRF_DRV_SPI_INSTANCE(ADXL314_SPI_INSTANCE);
static const nrf_drv_spi_t spi_mmc  = NRF_DRV_SPI_INSTANCE(MMC5983MA_SPI_INSTANCE);
static const nrf_drv_spi_t spi_gyro = NRF_DRV_SPI_INSTANCE(ITG3701_SPI_INSTANCE);

static bool spi_initialized = false;

void configure_unconnected_pins(void) {
    // Configure unconnected GPIO pins as inputs and enable internal pull-up resistors
    nrf_gpio_cfg_input(3, NRF_GPIO_PIN_PULLUP);  // P0.03
    nrf_gpio_cfg_input(9, NRF_GPIO_PIN_PULLUP);  // P0.09
    nrf_gpio_cfg_input(10, NRF_GPIO_PIN_PULLUP); // P0.10
    nrf_gpio_cfg_input(11, NRF_GPIO_PIN_PULLUP); // P0.11
    nrf_gpio_cfg_input(12, NRF_GPIO_PIN_PULLUP); // P0.12
    nrf_gpio_cfg_input(28, NRF_GPIO_PIN_PULLUP); // P0.28
    nrf_gpio_cfg_input(29, NRF_GPIO_PIN_PULLUP); // P0.29
    nrf_gpio_cfg_input(30, NRF_GPIO_PIN_PULLUP); // P0.30
    nrf_gpio_cfg_input(31, NRF_GPIO_PIN_PULLUP); // P0.31
    nrf_gpio_cfg_input(32, NRF_GPIO_PIN_PULLUP);  // P1.00
    nrf_gpio_cfg_input(33, NRF_GPIO_PIN_PULLUP);  // P1.01
    nrf_gpio_cfg_input(34, NRF_GPIO_PIN_PULLUP);  // P1.02
    nrf_gpio_cfg_input(35, NRF_GPIO_PIN_PULLUP);  // P1.03
    nrf_gpio_cfg_input(36, NRF_GPIO_PIN_PULLUP);  // P1.04
    nrf_gpio_cfg_input(37, NRF_GPIO_PIN_PULLUP);  // P1.05
    nrf_gpio_cfg_input(38, NRF_GPIO_PIN_PULLUP);  // P1.06
    nrf_gpio_cfg_input(39, NRF_GPIO_PIN_PULLUP);  // P1.07
    nrf_gpio_cfg_input(41, NRF_GPIO_PIN_PULLUP);  // P1.09
    nrf_gpio_cfg_input(42, NRF_GPIO_PIN_PULLUP);  // P1.10
    nrf_gpio_cfg_input(43, NRF_GPIO_PIN_PULLUP);  // P1.11
    nrf_gpio_cfg_input(44, NRF_GPIO_PIN_PULLUP);  // P1.12
    nrf_gpio_cfg_input(45, NRF_GPIO_PIN_PULLUP);  // P1.13
    nrf_gpio_cfg_input(46, NRF_GPIO_PIN_PULLUP);  // P1.14
    nrf_gpio_cfg_input(47, NRF_GPIO_PIN_PULLUP);  // P1.15
}

void sensors_init(void) {
    if (spi_initialized) return;

    // Configure Chip Select pins as outputs and set to high before transmission
    nrf_gpio_cfg_output(ADXL314_CS_PIN);
    nrf_gpio_pin_set(ADXL314_CS_PIN);
    
    nrf_gpio_cfg_output(MMC5983MA_CS_PIN);
    nrf_gpio_pin_set(MMC5983MA_CS_PIN);

    nrf_gpio_cfg_output(ITG3701_CS_PIN);
    nrf_gpio_pin_set(ITG3701_CS_PIN);

    nrf_gpio_cfg_output(ADIS16266_CS_PIN);
    nrf_gpio_pin_set(ADIS16266_CS_PIN);

    configure_unconnected_pins();

    // Configure SPI for ADXL314 (Accelerometer)
    nrf_drv_spi_config_t spi_config_adxl = {
        .sck_pin      = 19, // P0.19  
        .mosi_pin     = 26, // P0.26  
        .miso_pin     = 27, // P0.27 
        .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED, // CS is manually controlled  
        .irq_priority = 3,
        .orc          = 0xFF, // Default over-read character  
        .frequency    = NRF_DRV_SPI_FREQ_2M, // Set SPI speed to 2 MHz (>= 2 MHz required) 
        .mode         = NRF_DRV_SPI_MODE_3, // CPOL = 1, CPHA = 1 (SPI Mode 3)  
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
    };

    // Configure SPI for MMC5983MA (Magnetometer)
    nrf_drv_spi_config_t spi_config_mmc = {
        .sck_pin      = 40, // P1.08 
        .mosi_pin     = 21, // P0.21 
        .miso_pin     = 23, // P0.23 
        .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,  
        .irq_priority = 3,
        .orc          = 0xFF,  
        .frequency    = NRF_DRV_SPI_FREQ_2M, // Set SPI speed to 2 MHz (max 10 MHz supported) 
        .mode         = NRF_DRV_SPI_MODE_0, // CPOL = 0, CPHA = 0 (SPI Mode 0)  
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
    };

    // Configure SPI for ITG-3701 & ADIS16266 (Gyroscopes)
    nrf_drv_spi_config_t spi_config_gyro = {
        .sck_pin      = 1, // P0.01 
        .mosi_pin     = 6, // P0.06
        .miso_pin     = 7, // P0.07
        .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,  
        .irq_priority = 3,
        .orc          = 0xFF,  
        .frequency    = NRF_DRV_SPI_FREQ_2M, // Set SPI speed to 2 MHz (max 2.5 MHz supported for ADIS16266)
        .mode         = NRF_DRV_SPI_MODE_3, // CPOL = 1, CPHA = 1, (SPI Mode 3)  
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
    };

    // Initialize SPI instances
    if (nrf_drv_spi_init(&spi_adxl, &spi_config_adxl, NULL, NULL) == NRF_SUCCESS) {
        NRF_LOG_INFO("SPI_A Initialized for ADXL314");
    } else {
        NRF_LOG_ERROR("SPI_A Initialization Failed");
    }

    if (nrf_drv_spi_init(&spi_mmc, &spi_config_mmc, NULL, NULL) == NRF_SUCCESS) {
        NRF_LOG_INFO("SPI_C Initialized for MMC5983MA");
    } else {
        NRF_LOG_ERROR("SPI_C Initialization Failed");
    }

    if (nrf_drv_spi_init(&spi_gyro, &spi_config_gyro, NULL, NULL) == NRF_SUCCESS) {
        NRF_LOG_INFO("SPI_B Initialized for ITG-3701 & ADIS16266");
    } else {
        NRF_LOG_ERROR("SPI_B Initialization Failed");
    }

    spi_initialized = true;
}


// SPI Read & Write Functions
static bool spi_write_register(const nrf_drv_spi_t *spi_instance, uint8_t cs_pin, uint8_t reg, uint8_t value) {
    // Write a single byte (value) to a register (reg)
    uint8_t tx_data[2] = {reg, value};
    nrf_gpio_pin_clear(cs_pin);
    bool success = (nrf_drv_spi_transfer(spi_instance, tx_data, 2, NULL, 0) == NRF_SUCCESS);
    nrf_gpio_pin_set(cs_pin);
    return success;
}

static uint8_t spi_read_register(const nrf_drv_spi_t *spi_instance, uint8_t cs_pin, uint8_t reg) {
    // Send a read command to request register data and return the received byte
    uint8_t tx_data[2] = {reg | 0x80, 0x00}; // Read command (MSB = 1)
    uint8_t rx_data[2];
    nrf_gpio_pin_clear(cs_pin);
    bool success = (nrf_drv_spi_transfer(spi_instance, tx_data, 2, rx_data, 2) == NRF_SUCCESS);
    nrf_gpio_pin_set(cs_pin);
    return success ? rx_data[1] : 0;
}


/* ---------------------------------------------- ADXL314 -------------------------------------------------------------------*/
/*
void adxl314_enable_interrupt(void) {
    // Configure INT1 (P0.17) as input for Data Ready interrupt
    nrf_gpio_cfg_input(17, NRF_GPIO_PIN_NOPULL);

    // Enable Data Ready Interrupt in ADXL314
    spi_write_register(&spi_adxl, ADXL314_CS_PIN, ADXL314_INT_MAP, 0x00);  // Map Data Ready to INT1
    spi_write_register(&spi_adxl, ADXL314_CS_PIN, ADXL314_INT_ENABLE, 0x80);  // Enable Data Ready Interrupt

    // Configure GPIOTE for interrupt handling
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    config.pull = NRF_GPIO_PIN_NOPULL;
    nrf_drv_gpiote_in_init(17, &config, adxl314_data_ready_handler);
    nrf_drv_gpiote_in_event_enable(17, true);
}
*/
//// Define ISR that runs automatically when ADXL314 signals new data is ready
//void adxl314_data_ready_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
//    int16_t acc_values[3];
//    if (adxl314_read_accel(acc_values)) {
//        NRF_LOG_INFO("Accel: X=%d Y=%d Z=%d", acc_values[0], acc_values[1], acc_values[2]);
//    }
//}

bool adxl314_data_ready(void) {
    uint8_t status = spi_read_register(&spi_adxl, ADXL314_CS_PIN, ADXL314_INT_SOURCE);
    return (status & 0x80) != 0;  // Check if DATA_READY bit (Bit 7) is set
}

// ADXL314 Accelerometer Initialization
bool adxl314_init(void) {
    // Ensure SPI is initialized
    if (!spi_initialized) sensors_init();

    // Check that ADXL314 is detected
    uint8_t dev_id = spi_read_register(&spi_adxl, ADXL314_CS_PIN, ADXL314_DEVID_REG);
    if (dev_id != 0xE5) {  // Expected ID for ADXL314
        NRF_LOG_ERROR("ADXL314 not detected! ID: 0x%X", dev_id);
        return false;
    }

    // Set ODR to 1600 Hz (BW_RATE = 0x0E)
    spi_write_register(&spi_adxl, ADXL314_CS_PIN, ADXL314_BW_RATE, 0x0E);

    // Configure data format: 4-wire SPI, Full Resolution, Right Justified
    spi_write_register(&spi_adxl, ADXL314_CS_PIN, ADXL314_DATA_FORMAT, 0x0B); 
    
    // Enable measurement mode (POWER_CTL = 0x08)
    spi_write_register(&spi_adxl, ADXL314_CS_PIN, ADXL314_POWER_CTL, 0x08);

    // Configure INT1 (Pin P0.13) as input for Data Ready interrupt
    nrf_gpio_cfg_input(13, NRF_GPIO_PIN_NOPULL);

    // Map Data Ready Interrupt to INT1
    spi_write_register(&spi_adxl, ADXL314_CS_PIN, ADXL314_INT_MAP, 0x00);  // Set bit D7 = 0

    // Enable Data Ready Interrupt
    spi_write_register(&spi_adxl, ADXL314_CS_PIN, ADXL314_INT_ENABLE, 0x80);  // Set bit D7 = 1

    //// Configure GPIOTE to handle interrupt:

    //if (!nrf_drv_gpiote_is_init()) {
    //    nrf_drv_gpiote_init();  // Initialize GPIOTE module
    //}

    //// Configure GPIO for interrupt detection on INT1 (P0.13)
    //nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true); // Active high 
    //config.pull = NRF_GPIO_PIN_NOPULL;

    //nrf_drv_gpiote_in_init(13, &config, adxl314_data_ready_handler); // Links the interrupt to data ready handler
    //nrf_drv_gpiote_in_event_enable(13, true); // Enables the interrupt event on P0.13
    
    NRF_LOG_INFO("ADXL314 Initialized with ODR 1600 Hz, 4-wire SPI");
    return true;

}

bool adxl314_read_accel(int16_t *acc_values) {
    // Wait for new data
    if (!adxl314_data_ready()) {
        return false;  // No new data available, avoid reading old data
    }

    uint8_t tx_data = ADXL314_DATAX0 | 0xC0;  // Set MB bit (0x40) for multi-byte read
    uint8_t rx_data[6];
    nrf_gpio_pin_clear(ADXL314_CS_PIN); // Select ADXL314
    bool success = (nrf_drv_spi_transfer(&spi_adxl, &tx_data, 1, rx_data, 6) == NRF_SUCCESS);
    nrf_gpio_pin_set(ADXL314_CS_PIN); // Deselect ADXL314
    if (success) {
        // Combine MSB and LSB for 16-bit acceleration values
        acc_values[0] = (rx_data[1] << 8) | rx_data[0]; // X-axis
        acc_values[1] = (rx_data[3] << 8) | rx_data[2]; // Y-axis
        acc_values[2] = (rx_data[5] << 8) | rx_data[4]; // Z-axis
    }
    return success;
}

/* ---------------------------------------------- MMC5983MA -------------------------------------------------------------------*/
// MMC5983MA Magnetometer

//// Define ISR that runs automatically when magnetometer signals new data is ready
//void mmc5983ma_data_ready_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
//    int32_t mag_values[3];
//    if (mmc5983ma_read_mag(mag_values)) {
//        NRF_LOG_INFO("Magnetometer: X=%d Y=%d Z=%d", mag_values[0], mag_values[1], mag_values[2]);
//    }
//}

bool mmc5983ma_data_ready(void) {
    uint8_t status = spi_read_register(&spi_mmc, MMC5983MA_CS_PIN, MMC5983MA_STATUS);
    return (status & 0x01) != 0;  // Bit 0 = DATA_RDY
}

bool mmc5983ma_init(void) {
    // Ensure SPI is initialized
    if (!spi_initialized) sensors_init();

    // Check that MMC5983MA is detected
    uint8_t dev_id = spi_read_register(&spi_mmc, MMC5983MA_CS_PIN, MMC5983MA_PRODUCT_ID);
    if (dev_id != 0x30) {  // Expected ID for MMC5983MA
        NRF_LOG_ERROR("MMC5983MA not detected! ID: 0x%X", dev_id);
        return false;
    }

    // Configure INT1 (Pin P0.25) as input for Data Ready interrupt
    nrf_gpio_cfg_input(25, NRF_GPIO_PIN_NOPULL);

    spi_write_register(&spi_mmc, MMC5983MA_CS_PIN, MMC5983MA_CTRL_0, 0x28); // Enable auto set/reset & enable data ready interrupt
    spi_write_register(&spi_mmc, MMC5983MA_CS_PIN, MMC5983MA_CTRL_1, 0x03); // Set BW to 800 Hz (BW = 11)
    spi_write_register(&spi_mmc, MMC5983MA_CS_PIN, MMC5983MA_CTRL_2, 0x0F); // Set ODR to 1000 Hz & enable continuous measurement mode

    //// Configure nRF GPIO for interrupt handling:

    //if (!nrf_drv_gpiote_is_init()) {
    //    nrf_drv_gpiote_init();  // Initialize GPIOTE module if not already done
    //}

    //// Configure GPIO for interrupt detection on MMC5983MA INT pin (P0.25)
    //nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true); // Active high
    //config.pull = NRF_GPIO_PIN_NOPULL;

    //nrf_drv_gpiote_in_init(25, &config, mmc5983ma_data_ready_handler); // Link the interrupt to data ready handler
    //nrf_drv_gpiote_in_event_enable(25, true); // Enable the interrupt event on P0.25
    
    NRF_LOG_INFO("MMC5983MA Initialized with ODR 1000 Hz");
    return true;
}

bool mmc5983ma_read_mag(int8_t *mag_values) {
    if (!mmc5983ma_data_ready()) return false;  // Polling method

    uint8_t tx_data = MMC5983MA_XOUT0 | 0xC0;  // Enable multi-byte read
    uint8_t rx_data[7];  // Read XOUT0 to XYZOUT2

    nrf_gpio_pin_clear(MMC5983MA_CS_PIN); // Select MMC5983MA
    bool success = (nrf_drv_spi_transfer(&spi_mmc, &tx_data, 1, rx_data, 7) == NRF_SUCCESS);
    nrf_gpio_pin_set(MMC5983MA_CS_PIN); // Deselect MMC5983MA

    if (success) {
        //// Extract 18-bit data for each axis
        //mag_values[0] = (rx_data[0] << 10) | (rx_data[1] << 2) | ((rx_data[6] >> 6) & 0x03);  // X-axis
        //mag_values[1] = (rx_data[2] << 10) | (rx_data[3] << 2) | ((rx_data[6] >> 4) & 0x03);  // Y-axis
        //mag_values[2] = (rx_data[4] << 10) | (rx_data[5] << 2) | ((rx_data[6] >> 2) & 0x03);  // Z-axis

        mag_values[0] = rx_data[0];
        mag_values[1] = rx_data[1];
        mag_values[2] = rx_data[2];
        mag_values[3] = rx_data[3];
        mag_values[4] = rx_data[4];
        mag_values[5] = rx_data[5];
        mag_values[6] = rx_data[6];
    }

    return success;
}

/* ---------------------------------------------- ITG-3701 -------------------------------------------------------------------*/
// ITG-3701 Gyroscope

//// Define the ISR that runs automatically when data ready interrupt is triggered
//void itg3701_data_ready_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
//    int16_t gyro_values[3];
//    if (itg3701_read_gyro(gyro_values)) {
//        NRF_LOG_INFO("Gyro: X=%d Y=%d Z=%d", gyro_values[0], gyro_values[1], gyro_values[2]);
//    }
//}

bool itg3701_data_ready(void) {
    uint8_t int_status = spi_read_register(&spi_gyro, ITG3701_CS_PIN, ITG3701_INT_STATUS);
    return (int_status & 0x01) != 0;  // Bit 0 = DATA_RDY
}

bool itg3701_init(void) {
    // Ensure SPI is initialized
    if (!spi_initialized) sensors_init();

    // Verify WHO_AM_I register
    uint8_t dev_id = spi_read_register(&spi_gyro, ITG3701_CS_PIN, ITG3701_WHO_AM_I);
    if (dev_id != 0x68) {  // Expected ID for ITG-3701
        NRF_LOG_ERROR("ITG-3701 not detected! ID: 0x%X", dev_id);
        return false;
    }

    // Wake up the sensor (disable sleep mode)
    spi_write_register(&spi_gyro, ITG3701_CS_PIN, ITG3701_PWR_MGMT_1, 0x01);  // Clock source = PLL

    // Set full-scale range to ±4000 DPS (FS_SEL = 3)
    spi_write_register(&spi_gyro, ITG3701_CS_PIN, ITG3701_GYRO_CONFIG, 0x18);  // FS_SEL[1:0] = 11 (±4000 DPS)

    // Configure Digital Low Pass Filter (DLPF) and sample rate
    spi_write_register(&spi_gyro, ITG3701_CS_PIN, ITG3701_CONFIG, 0x03);  // DLPF_CFG = 3 (41 Hz Bandwidth)
    spi_write_register(&spi_gyro, ITG3701_CS_PIN, ITG3701_SMPLRT_DIV, 0x03);  // Sample Rate = Gyro Output Rate (1 kHz) / (1 + 3) = 250 Hz

    // Configure INT1 (Pin P0.15) as input for Data Ready interrupt
    nrf_gpio_cfg_input(15, NRF_GPIO_PIN_NOPULL);

    spi_write_register(&spi_gyro, ITG3701_CS_PIN, ITG3701_INT_PIN_CFG, 0x00); // Configure active high interrupt & INT pin as push-pull
    spi_write_register(&spi_gyro, ITG3701_CS_PIN, ITG3701_INT_ENABLE, 0x01); // Set bit 0 to 1 to enable data ready interrupt

    //// Configure nRF GPIO for interrupt handling

    //if (!nrf_drv_gpiote_is_init()) {
    //    nrf_drv_gpiote_init();  // Initialize GPIOTE module if not already initialized
    //}

    //// Configure GPIO P0.15 to detect rising edge (INT pin of ITG-3701)
    //nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true); // Active high
    //config.pull = NRF_GPIO_PIN_NOPULL;

    //nrf_drv_gpiote_in_init(15, &config, itg3701_data_ready_handler); // Link the interrupt to data ready handler
    //nrf_drv_gpiote_in_event_enable(15, true); // Enable the interrupt event on P0.15
  
    NRF_LOG_INFO("ITG-3701 Initialized with ±4000 DPS full scale");
    return true;
}

bool itg3701_read_gyro(int16_t *gyro_values) {
    if (!itg3701_data_ready()) return false;  // Polling method

    uint8_t tx_data = ITG3701_GYRO_XOUT_H | 0x80;  // Set read bit (MSB = 1), supports auto-increment for burst read
    uint8_t rx_data[6];

    nrf_gpio_pin_clear(ITG3701_CS_PIN); // Select ITG-3701
    bool success = (nrf_drv_spi_transfer(&spi_gyro, &tx_data, 1, rx_data, 6) == NRF_SUCCESS);
    nrf_gpio_pin_set(ITG3701_CS_PIN); // Deselect ITG-3701

    if (success) {
        // Combine MSB and LSB to form 16-bit values
        gyro_values[0] = (rx_data[0] << 8) | rx_data[1];  // X-axis
        gyro_values[1] = (rx_data[2] << 8) | rx_data[3];  // Y-axis
        gyro_values[2] = (rx_data[4] << 8) | rx_data[5];  // Z-axis
    }

    return success;
}

/* ---------------------------------------------- ADIS16266 -------------------------------------------------------------------*/
// ADIS16266 High-Rate Gyroscope

//// Define ISR that runs automatically when data ready interrupt is triggered
//void adis16266_data_ready_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
//  int16_t high_gyro_values[4];
//    if (adis16266_read_gyro(high_gyro_values)) {
//        NRF_LOG_INFO("Gyro: High Z=%d", high_gyro_values[3]);
//    }
//}

bool adis16266_verify_prod_id(void) {
    // Read from PROD_ID register
    uint8_t tx_data[2] = {ADIS16266_PROD_ID, 0x00}; // Read command (MSB = 0)
    uint8_t rx_data[2];
    uint16_t dev_id;
    nrf_gpio_pin_clear(ADIS16266_CS_PIN);
    bool success = (nrf_drv_spi_transfer(&spi_gyro, tx_data, 2, rx_data, 2) == NRF_SUCCESS);
    nrf_gpio_pin_set(ADIS16266_CS_PIN);
    if (success) dev_id = (rx_data[1] << 8) | rx_data[0]; // Combine MSB and LSB to form 16-bit data

    // Verify PROD_ID = 0x3F8A
    if (dev_id != 0x3F8A) {  
        NRF_LOG_ERROR("ADIS16266 not detected! ID: 0x%X", dev_id);
        return false;
    }

    return true; // dev_id = 0x3F8A
}

bool adis16266_init(void) {
    // Ensure SPI is initialized
    if (!spi_initialized) sensors_init();

    // Exit if device not identified
    if (!adis16266_verify_prod_id()) return false;

    // 16-bit SPI command format: R/W bit (1), 7-bit register address, 8-bit data

    // spi_write_register(&spi_gyro, ADIS16266_CS_PIN, ADIS16266_GYRO_SCALE, 0x0800); // multiply gyro output by 1 (no scale factor)

    // Set Sample Rate to Maximum (2429 Hz), (SMPL_PRD[3:0] = 0000)
    spi_write_register(&spi_gyro, ADIS16266_CS_PIN, 0xB6, 0x00); // Write low byte (0x00) to address 0x36 with MSB = 1 (write)

    // Set Filter & Sensitivity: 2-tap filter (SENS_AVG[2:0] = 001), ±14,000 DPS (SENS_AVG[10:8] = 100), (SENS_AVG = 0x0401)
    spi_write_register(&spi_gyro, ADIS16266_CS_PIN, 0xB9, 0x04); // Write high byte (0x04) to address 0x39 with MSB = 1 (write)
    spi_write_register(&spi_gyro, ADIS16266_CS_PIN, 0xB8, 0x01); // Write low byte (0x01) to address 0x38 with MSB = 1 (write)

    // Configure INT1 (Pin P0.17) as input for Data Ready interrupt
    nrf_gpio_cfg_input(17, NRF_GPIO_PIN_NOPULL);

    // Configure DIO1 as output and DIO2 as output set to high (GPIO_CTRL = 0x0203)
    spi_write_register(&spi_gyro, ADIS16266_CS_PIN, 0xB3, 0x02); // Write higher byte (0x02) to address 0x33 with MSB = 1 (write)
    spi_write_register(&spi_gyro, ADIS16266_CS_PIN, 0xB2, 0x03); // Write lower byte (0x03) to address 0x32 with MSB = 1 (write)

    // Enable Data Ready Interrupt (DIO1 as Data Ready, active high) (MSC_CTRL[2:0] = 110)
    spi_write_register(&spi_gyro, ADIS16266_CS_PIN, 0xB4, 0x06); // Write lower byte (0x04) to address 0x34 with MSB = 1 (write)

    // Save Settings to Flash Memory (GLOB_CMD[3] = 1)
    spi_write_register(&spi_gyro, ADIS16266_CS_PIN, 0xBE, 0x08); // Write lower byte (0x08) to address 0x3E with MSB = 1 (write)
    nrf_delay_ms(50);  // Wait for flash write

    //// Set up a GPIOTE interrupt to handle the Data Ready signal:

    //if (!nrf_drv_gpiote_is_init()) {
    //    nrf_drv_gpiote_init();  // Initialize GPIOTE module if not already initialized
    //}

    //nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true); // Active high
    //config.pull = NRF_GPIO_PIN_NOPULL;

    //nrf_drv_gpiote_in_init(17, &config, adis16266_data_ready_handler); // Link the interrupt to data ready handler
    //nrf_drv_gpiote_in_event_enable(17, true); // Enable the interrupt event on P0.17

    NRF_LOG_INFO("ADIS16266 Initialized with ±14,000 DPS and max ODR.");
    return true;
}

bool adis16266_read_gyro(int16_t *gyro_values) {
    // Read from GYRO_OUT register
    uint8_t tx_data[2] = {ADIS16266_GYRO_OUT, 0x00}; // Read command (MSB = 0)
    uint8_t rx_data[2];
    nrf_gpio_pin_clear(ADIS16266_CS_PIN);
    bool success = (nrf_drv_spi_transfer(&spi_gyro, tx_data, 2, rx_data, 2) == NRF_SUCCESS);
    nrf_gpio_pin_set(ADIS16266_CS_PIN);
    
    uint8_t unread_data = (rx_data[1] & 0x10); // Check MSB 
    if (unread_data != 0x10) return false;  // ND bit = 0

    if (success) {
        // Combine MSB and LSB to form 16-bit values
        gyro_values[3] = ((rx_data[1] & 0x3F) << 8) | rx_data[0];  // Clear bits 15:14
    }

    return success;

}
