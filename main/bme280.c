#include "bme280.h"
#include "app_init.h"

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

#include <stdint.h>
#include <string.h>

/* =====================================================================
 * I2C address
 * ===================================================================== */
/* Default address when SDO is tied to GND.
 * If your module ties SDO to VDD, change this to 0x77. */
#define BME280_ADDRESS              0x76


/* =====================================================================
 * Register map (Bosch BME280 datasheet, section 5)
 * ===================================================================== */

/* --- Identification and reset --- */
#define BME280_REG_ID               0xD0
#define BME280_CHIP_ID              0x60
#define BME280_REG_RESET            0xE0

/* --- Control and status --- */
#define BME280_REG_CTRL_HUM         0xF2
#define BME280_REG_STATUS           0xF3
#define BME280_REG_CTRL_MEAS        0xF4
#define BME280_REG_CONFIG           0xF5

/* --- Raw data (0xF7..0xFE): press(3) + temp(3) + hum(2) --- */
#define BME280_REG_DATA             0xF7
#define BME280_DATA_LEN             8

/* --- Calibration block 1: temperature + pressure + dig_H1 ---
 * Contiguous range 0x88..0xA1 (26 bytes).
 *   0x88..0x9F : dig_T1..T3 and dig_P1..P9 (24 bytes)
 *   0xA0       : reserved
 *   0xA1       : dig_H1
 * Read in a single I2C transaction even though dig_H1 conceptually
 * belongs to the humidity calibration. */
#define BME280_REG_CALIB_TPH1       0x88
#define BME280_CALIB_TPH1_LEN       26

/* --- Calibration block 2: remaining humidity coefficients ---
 * 0xE1..0xE7 (7 bytes): dig_H2..H6. */
#define BME280_REG_CALIB_H          0xE1
#define BME280_CALIB_H_LEN          7


/* =====================================================================
 * Operating configuration
 * ===================================================================== */
/* Oversampling T x1, P x1, normal mode.
 * Note: ctrl_hum must be written BEFORE ctrl_meas for it to take effect. */
#define BME280_CTRL_MEAS_NORMAL_TP  0x27
#define BME280_CTRL_HUM_X1          0x01


/* =====================================================================
 * Driver internal state
 * ===================================================================== */
static const char *TAG = "BME280";

/* I2C device handle (lives for the entire lifetime of the program) */
static i2c_master_dev_handle_t s_bme280;

/* --- Temperature calibration coefficients --- */
static uint16_t dig_T1;
static int16_t dig_T2;
static int16_t dig_T3;

/* --- Pressure calibration coefficients --- */
static uint16_t dig_P1;
static int16_t dig_P2;
static int16_t dig_P3;
static int16_t dig_P4;
static int16_t dig_P5;
static int16_t dig_P6;
static int16_t dig_P7;
static int16_t dig_P8;
static int16_t dig_P9;

/* --- Humidity calibration coefficients ---
 * dig_H1 lives in the TPH1 block (0xA1). The remaining ones (H2..H6)
 * are read from 0xE1..0xE7 during the humidity step. */
static uint8_t dig_H1;
static int16_t dig_H2;
static uint8_t dig_H3;
static int16_t dig_H4;
static int16_t dig_H5;
static int8_t dig_H6;


/* Shared variable between the temperature, pressure and humidity
 * compensation routines. Defined as global by Bosch's algorithm. */
static int32_t t_fine;


/* =====================================================================
 * I2C helpers
 * ===================================================================== */
/**
 * @brief Read @p len consecutive bytes starting at register @p reg.
 */
static esp_err_t bme280_read_registers(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_bme280, &reg, 1, buf, len, -1);
}

/**
 * @brief Write a single byte @p value to register @p reg.
 */
static esp_err_t bme280_write_register(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = { reg, value };
    return i2c_master_transmit(s_bme280, buffer, sizeof(buffer), -1);
}


/* =====================================================================
 * Calibration readout
 * ===================================================================== */

/**
 * @brief Read the contiguous block 0x88..0xA1 (26 bytes) and unpack
 *        dig_T1..T3, dig_P1..P9 and dig_H1.
 */
static esp_err_t bme280_read_calibration_tph1(void)
{
    uint8_t calib[BME280_CALIB_TPH1_LEN];

    esp_err_t err = bme280_read_registers(BME280_REG_CALIB_TPH1,
                                          calib, sizeof(calib));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read TPH1 calibration block: %s",
                 esp_err_to_name(err));
        return err;
    }

    /* Temperature (0x88..0x8D) */
    dig_T1 = (uint16_t)((calib[1] << 8) | calib[0]);
    dig_T2 = (int16_t) ((calib[3] << 8) | calib[2]);
    dig_T3 = (int16_t) ((calib[5] << 8) | calib[4]);

    /* Pressure (0x8E..0x9F) */
    dig_P1 = (uint16_t)((calib[7]  << 8) | calib[6]);
    dig_P2 = (int16_t) ((calib[9]  << 8) | calib[8]);
    dig_P3 = (int16_t) ((calib[11] << 8) | calib[10]);
    dig_P4 = (int16_t) ((calib[13] << 8) | calib[12]);
    dig_P5 = (int16_t) ((calib[15] << 8) | calib[14]);
    dig_P6 = (int16_t) ((calib[17] << 8) | calib[16]);
    dig_P7 = (int16_t) ((calib[19] << 8) | calib[18]);
    dig_P8 = (int16_t) ((calib[21] << 8) | calib[20]);
    dig_P9 = (int16_t) ((calib[23] << 8) | calib[22]);

    /* calib[24] = reserved (0xA0) */
    /* dig_H1 (0xA1) */
    dig_H1 = calib[25];

    ESP_LOGI(TAG, "Calib T: T1=%u T2=%d T3=%d",
             dig_T1, dig_T2, dig_T3);
    ESP_LOGI(TAG, "Calib P: P1=%u P2=%d P3=%d P4=%d P5=%d "
                  "P6=%d P7=%d P8=%d P9=%d",
             dig_P1, dig_P2, dig_P3, dig_P4, dig_P5,
             dig_P6, dig_P7, dig_P8, dig_P9);
    ESP_LOGI(TAG, "Calib H1=%u", dig_H1);

    return ESP_OK;
}

/**
 * @brief Read the humidity calibration block 0xE1..0xE7 (7 bytes) and unpack dig_H2..H6.
 * 
 * dig_H4 and dig_H5 are 12-bit signed values split across two registers (0xE4/0xE5 and 0xE5/0xE6). Sign-extension from bit 11 is required.
 * 
 * @return esp_err_t 
 */
static esp_err_t bme280_read_calibration_h(void){
    
    uint8_t calib[BME280_CALIB_H_LEN];

    esp_err_t err = bme280_read_registers(BME280_REG_CALIB_H,
                                          calib, sizeof(calib));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read H calibration block: %s",
                 esp_err_to_name(err));
        return err;
    }

    dig_H2 = (int16_t)((calib[1] << 8) | calib[0]);
    dig_H3 = calib[2];
    dig_H4 = (int16_t)((calib[3] << 4) | (calib[4] & 0x0F));
    if(dig_H4 & 0x0800) {
        dig_H4 |= (int16_t)0xF000; /* sign-extension of bit 11 */
    }
    dig_H5 = (int16_t)((calib[5] << 4) | ((calib[4] >> 4) & 0x0F));
    if(dig_H5 & 0x0800) {
        dig_H5 |= (int16_t)0xF000; /* sign-extension of bit 11 */
    }
    dig_H6 = (int8_t)calib[6];

    ESP_LOGI(TAG, "Calib H: H1=%u H2=%d H3=%u H4=%d H5=%d H6=%d",
             dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6);

    return ESP_OK;
}

/* =====================================================================
 * Compensation (Bosch algorithms, datasheet section 4.2.3)
 * ===================================================================== */

/**
 * @brief Compensate the raw temperature value.
 *
 * @param adc_T  Raw 20-bit value read from 0xFA..0xFC.
 * @return Temperature in hundredths of °C (e.g. 2512 = 25.12 °C).
 *
 * @note Updates t_fine, which is required to compensate pressure
 *       and humidity.
 */
static int32_t bme280_compensate_temperature(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) *
            ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
              ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
            ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;

    return T;
}

/**
 * @brief Compensate the raw pressure value (64-bit variant).
 *
 * Uses the 64-bit form from the Bosch datasheet (section 4.2.3). This
 * version avoids the overflow workarounds required by the 32-bit
 * variant, but the return format is the same: Q24.8 (Pa with 8 fractional bits).
 *
 * @param adc_P  Raw 20-bit value read from 0xF7..0xF9.
 * @return Pressure in Q24.8. Divide by 256 to get integer Pa,
 *         then by 100 to get hPa.
 *
 * @note Requires t_fine to be already computed: call
 *       bme280_compensate_temperature() first.
 */
static uint32_t bme280_compensate_pressure(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
           ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0) {
        return 0;   /* Pressure out of range / avoid division by zero */
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

    return (uint32_t)p;
}

/**
 * @brief Compensate the raw humidity value.
 *
 * @param adc_H  Raw 16-bit value read from 0xE1..0xE2.
 * @return Humidity in Q24.8. Divide by 1024 to get integer %RH,
 *         then by 100 to get hPa.
 *
 * @note Requires t_fine to be already computed: call
 *       bme280_compensate_temperature() first.
 */
static uint32_t bme280_compensate_humidity(int32_t adc_H){
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
                  ((int32_t)16384)) >> 15) *
                (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
                     (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                   ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < (int32_t)0 ? (int32_t)0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > (int32_t)419430400 ? (int32_t)419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}

/* =====================================================================
 * Public API
 * ===================================================================== */

esp_err_t bme280_init(void)
{
    /* 1) Register the device on the I2C bus created by app_init() */
    i2c_master_bus_handle_t bus = app_get_i2c_bus();

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BME280_ADDRESS,
        .scl_speed_hz    = 100000,
    };

    esp_err_t err = i2c_master_bus_add_device(bus, &device_config, &s_bme280);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to I2C bus: %s",
                 esp_err_to_name(err));
        return err;
    }

    /* 2) Verify the chip ID */
    uint8_t chip_id = 0;
    err = bme280_read_registers(BME280_REG_ID, &chip_id, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID: %s", esp_err_to_name(err));
        return err;
    }

    if (chip_id != BME280_CHIP_ID) {
        ESP_LOGE(TAG, "Unexpected chip ID: 0x%02X (expected 0x%02X)",
                 chip_id, BME280_CHIP_ID);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "BME280 detected (chip ID 0x%02X)", chip_id);

    /* 3) Read the T + P + H1 and H2-H6 calibration blocks */
    err = bme280_read_calibration_tph1();
    if (err != ESP_OK) {
        return err;
    }

    err = bme280_read_calibration_h();
    if (err != ESP_OK) {
        return err;
    }

    /* 4) Configuration: humidity on, then ctrl_meas in normal mode.
     *    Order matters: ctrl_hum before ctrl_meas. */
    err = bme280_write_register(BME280_REG_CTRL_HUM, BME280_CTRL_HUM_X1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write ctrl_hum: %s", esp_err_to_name(err));
        return err;
    }

    err = bme280_write_register(BME280_REG_CTRL_MEAS, BME280_CTRL_MEAS_NORMAL_TP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write ctrl_meas: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "BME280 initialized (normal mode, T x1, P x1, H x1)");
    return ESP_OK;
}

esp_err_t bme280_read(bme280_data_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Read the 8 raw bytes (0xF7..0xFE) in a single transaction to
     * guarantee they come from the same measurement cycle. */
    uint8_t raw[BME280_DATA_LEN];
    esp_err_t err = bme280_read_registers(BME280_REG_DATA, raw, sizeof(raw));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read raw data: %s", esp_err_to_name(err));
        return err;
    }

    /* Unpack (datasheet 4.1) */
    int32_t adc_P = (int32_t)(((uint32_t)raw[0] << 12) |
                              ((uint32_t)raw[1] << 4)  |
                              ((uint32_t)raw[2] >> 4));

    int32_t adc_T = (int32_t)(((uint32_t)raw[3] << 12) |
                              ((uint32_t)raw[4] << 4)  |
                              ((uint32_t)raw[5] >> 4));
    
    int32_t adc_H = (int32_t)(((uint32_t)raw[6] << 8) |
                              ((uint32_t)raw[7]));

    /* Compensation: temperature first (updates t_fine), then pressure */
    int32_t  T = bme280_compensate_temperature(adc_T);
    uint32_t P = bme280_compensate_pressure(adc_P);
    uint32_t  H = bme280_compensate_humidity(adc_H);
    /* Convert to physical units */
    out->temperature = (float)T / 100.0f;              /* hundredths -> °C */
    out->pressure    = (float)P / 256.0f / 100.0f;     /* Q24.8 Pa -> hPa */
    out->humidity    = (float)H / 1024.0f;             /* Q24.8 % -> %RH */

    return ESP_OK;
}