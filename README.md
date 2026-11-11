# ESP32 Environmental Monitoring Platform

A bare-metal firmware project for the ESP32 that reads environmental data
from a **BME280** sensor over I²C, processes it through FreeRTOS tasks, and
publishes it over UART. Built with **ESP-IDF** and **C**, with no
third-party sensor libraries: the BME280 driver is written from scratch
based on the Bosch datasheet.

The goal of this project is to practice and demonstrate core embedded
software skills: register-level driver development, sensor compensation
algorithms, FreeRTOS task design, and fault-tolerant communication with
peripherals.

---

## Overview

The firmware reads temperature, pressure, and humidity from a BME280 every
second, publishes the samples to a FreeRTOS queue, and prints them over
UART from a consumer task. The architecture is designed so that new
sensors (MPU6050) and new consumers (SD card logger, OLED display) can be
added without touching the existing code.

The project currently supports:

- **BME280 driver written from scratch** (I²C, register-level).
- **Bosch compensation algorithms** for temperature, pressure, and
  humidity, implemented in 64-bit integer arithmetic.
- **FreeRTOS producer/consumer architecture** with a queue between the
  sensor task and the display task.
- **Fault detection**: consecutive I²C failures and "stuck sensor"
  detection (sensor returns identical raw values, which happens after a
  VDD drop).
- **Automatic recovery**: after repeated failures, the driver sends a soft
  reset to the sensor and re-initializes it.

---

## Hardware

| Component        | Interface | Notes                                       |
|------------------|-----------|---------------------------------------------|
| ESP32 DevKit     | —         | Target MCU (dual-core Xtensa LX6)           |
| BME280           | I²C       | Address `0x76`, SDO tied to GND             |
| MPU6050          | I²C       | Planned: accelerometer + gyroscope          |
| microSD module   | SPI       | Planned: persistent data logging            |
| SSD1306 OLED     | I²C       | Planned: live display                       |

**Current wiring (BME280 only):**

| ESP32 pin | BME280 pin |
|-----------|------------|
| GPIO 21   | SDA        |
| GPIO 22   | SCL        |
| 3V3       | VCC        |
| GND       | GND        |

---

## Software architecture

```
┌──────────────────┐
│   sensor_task    │  Reads BME280 every 1000 ms
│   priority 5     │  ──┐
└──────────────────┘    │ xQueueSend (non-blocking)
                        ▼
                 ┌─────────────┐
                 │ data_queue  │  Depth 4
                 │ bme280_data │
                 └─────┬───────┘
                       │ xQueueReceive (blocking)
                       ▼
                 ┌──────────────┐
                 │ display_task │  Prints over UART
                 │ priority 5   │
                 └──────────────┘
```

**Key design points:**

- **The sensor task never blocks** on the queue. If the queue is full,
  the sample is dropped and a warning is logged. Losing one sample is
  better than delaying the next read.
- **The display task blocks indefinitely** on `xQueueReceive`. It only
  runs when new data is available.
- **`vTaskDelayUntil`** is used instead of `vTaskDelay` to keep the
  sampling period constant regardless of how long each read takes.
- **Sensor fault handling** is split in two layers:
  - The driver (`bme280_read`) only reports I²C errors.
  - The application (`sensor_task`) decides what to do: track consecutive
    failures, attempt re-initialization, mute repeated logs.

---

## Repository structure

```
.
├── main/
│   ├── main.c          Entry point: init + start tasks
│   ├── app_init.c/h    I²C bus initialization
│   ├── app_tasks.c/h   FreeRTOS tasks and queue
│   └── bme280.c/h      BME280 driver (I²C + Bosch algorithms)
├── CMakeLists.txt      Top-level build configuration
└── README.md
```

---

## Design decisions worth highlighting

### Driver written from scratch

The BME280 driver talks to the sensor at the register level. There is no
dependency on Adafruit, Bosch, or any other vendor library. Every write
goes through `i2c_master_transmit` and every read through
`i2c_master_transmit_receive`, with the register addresses and bit
layouts taken directly from the datasheet.

### Bosch compensation algorithms

Raw sensor values are not the physical quantities. The BME280 requires a
set of compensation formulas from Bosch. Temperature is compensated in
32-bit arithmetic; pressure and humidity use the 64-bit variants to avoid
overflow and precision issues.

- **Temperature** returns hundredths of °C (Q-something implicit).
- **Pressure** returns Q24.8 (Pa with 8 fractional bits). Divide by 256
  to get Pa, then by 100 to get hPa.
- **Humidity** returns Q22.10. Divide by 1024 to get %RH.

### Sign-extension for 12-bit humidity coefficients

`dig_H4` and `dig_H5` are the only calibration coefficients that are
12-bit signed values, split across two registers. Reading them into a
16-bit integer without sign-extension causes the driver to interpret
negative values as large positive ones. The driver explicitly extends the
sign bit after unpacking.

### Stuck sensor detection

The BME280 can enter a state where I²C reads succeed but the internal ADC
is stuck, returning the same 8 raw bytes on every read. This happens most
commonly when the sensor's VDD is dropped and restored (the power-on
reset circuit does not always recover the internal state).

The driver tracks the last raw reading and counts how many times the same
8 bytes are returned in a row. When the count crosses a threshold, the
application is informed and the sensor is soft-reset. This is more robust
than comparing the final float values, which can drift due to rounding.

### Layered fault handling

The driver does not decide whether the sensor is "healthy". It only
reports I²C errors and (through `bme280_is_stuck`) whether the raw values
look frozen. All policy — how many failures to tolerate, when to
re-initialize, how loudly to log — lives in `sensor_task`. This keeps the
driver reusable in different applications with different recovery
policies.

---

## Building and flashing

### Requirements

- **ESP-IDF v5.x** (tested with v6.0.2)
- **xtensa-esp32-elf** toolchain (installed by ESP-IDF)
- USB cable and an ESP32 DevKit
- BME280 connected as shown above

### Commands

```bash
# Configure target (only once)
idf.py set-target esp32

# Build
idf.py build

# Flash and monitor (adjust the port)
idf.py -p /dev/ttyUSB0 flash monitor
# On Windows:
idf.py -p COM3 flash monitor
```

To exit the serial monitor, press `Ctrl + ]`.

### Expected output

```
I (269) APP_INIT: I2C bus initialized
I (269) BME280: BME280 detected (chip ID 0x60)
I (279) BME280: Calib T: T1=... T2=... T3=...
I (289) BME280: Calib P: ...
I (289) BME280: Calib H: ...
I (299) BME280: BME280 initialized (normal mode, T x1, P x1, H x1)
I (299) APP_TASKS: Application tasks started
I (1299) APP_TASKS: Temperature: 21.65 °C, Pressure: 785.37 hPa, Humidity: 62.33 %RH
I (2299) APP_TASKS: Temperature: 21.66 °C, Pressure: 785.36 hPa, Humidity: 62.30 %RH
...
```

---

## Project status and roadmap

- [x] I²C bus initialization
- [x] BME280 driver: chip ID, calibration, raw reads
- [x] Temperature compensation
- [x] Pressure compensation
- [x] Humidity compensation (with 12-bit sign-extension)
- [x] FreeRTOS producer/consumer architecture (queue)
- [x] Consecutive-failure handling and automatic re-initialization
- [x] Stuck-sensor detection and recovery attempt
- [ ] MPU6050 driver (accelerometer + gyroscope)
- [ ] microSD logging (SPI + FatFS)
- [ ] SSD1306 OLED live display
- [ ] Pressure reduced to sea level (barometric formula)
- [ ] Unit tests for the compensation functions

---

## Problems found and how they were solved

### Pressure reading was off by a factor of ~250

Initial pressure readings showed values around 3 hPa. The compensation
algorithm was returning Q24.8 values but the conversion to hPa was
incorrect. Fixing the conversion and switching to the 64-bit variant of
the Bosch algorithm brought the values to ~785 hPa, which is correct for
Mexico City's altitude (~2240 m).

### Sensor stuck after VDD drop

Disconnecting the sensor's VDD or GND while running left the sensor in a
state where I²C reads succeeded but the values were frozen. The soft
reset command (`0xB6` to register `0xE0`) did not always recover it.
Mitigation: detect repeated raw readings and attempt re-initialization;
document the limitation in this README.

### Floats cannot be compared with `==`

Early detection of "stuck sensor" compared `float` values with `==`,
which fails intermittently due to rounding. The fix was to compare the
raw 8-byte reading with `memcmp`, which is exact for integer data and
correct for detecting a frozen ADC.

---

## License

MIT License. See [LICENSE](LICENSE) for details.