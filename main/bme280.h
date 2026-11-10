#ifndef BME280_H
#define BME280_H

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Environmental data measured by the BME280.
 *
 * Units:
 *   - temperature: degrees Celsius (°C)
 *   - pressure:    hectopascals (hPa), equivalent to millibars (mbar)
 *   - humidity:    relative humidity in percentage (%RH)
 * 
 * @note Pressure is computed with the 64-bit Bosch algorithm and
 *       reported in hPa. For a weather-station-style reading reduced
 *       to sea level, apply the barometric formula with your local
 *       altitude.
 */
typedef struct {
    float temperature;
    float pressure;
    float humidity;
} bme280_data_t;

/**
 * @brief Initialize the BME280 on the I2C bus created by app_init().
 *
 * Performs:
 *   - Device registration on the I2C bus.
 *   - Chip ID verification.
 *   - Calibration coefficient readout (temperature, pressure, humidity).
 *   - Oversampling and operating mode configuration.
 *
 * @return
 *   - ESP_OK on success.
 *   - ESP_ERR_NOT_FOUND if the chip ID does not match the BME280.
 *   - Other errors from esp_err.h propagated from the I2C driver.
 */
esp_err_t bme280_init(void);

/**
 * @brief Read and compensate temperature, pressure and humidity values.
 *
 * Must be called after bme280_init(). Performs a single I2C transaction
 * to read the 8 raw data bytes (0xF7..0xFE) and applies Bosch's
 * compensation algorithm.
 *
 * @param[out] out Pointer to the structure where data will be written.
 *                 Must not be NULL.
 *
 * @return
 *   - ESP_OK on success.
 *   - ESP_ERR_INVALID_ARG if out is NULL.
 *   - Other errors from esp_err.h propagated from the I2C driver.
 *
 * @note Not thread-safe. If multiple tasks call this function, protect
 *       access to the I2C bus with a mutex.
 */
esp_err_t bme280_read(bme280_data_t *out);

#endif /* BME280_H */