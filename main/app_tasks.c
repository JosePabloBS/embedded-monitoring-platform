#include "app_tasks.h"
#include "bme280.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "APP_TASKS";

/* Task configuration */ 
#define SENSOR_TASK_STACK_SIZE      3072
#define SENSOR_TASK_PRIORITY        5
#define SENSOR_TASK_PERIOD_MS       1000
#define SENSOR_FAIL_THRESHOLD       3
#define SENSOR_REINIT_THRESHOLD     5

#define DISPLAY_TASK_STACK_SIZE     3072
#define DISPLAY_TASK_PRIORITY       5

#define DATA_QUEUE_LENGTH            4

/* Queue shared between the sensor task (producer) and the display task 
* (consumer). Created in app_tasks_start() and passed by parameter. */
static QueueHandle_t s_data_queue;

/**
 * @brief Producer task: read the BME280 every SENSOR_TASK_PERIOD_MS and
 * publish the sample to the data queue. 
 * 
 * Handles sensor failures gracefully: 
 *  - Tracks consecutive failures to avoid flooding the log. 
 *  - After SENSOR_REINIT_THRESHOLD failures, attemps to re-initialize the sensor. If re-init succeeds, the counter resets and normal operation resumes.
 *  - Never blocks on the queue: losing a sample is better than delaying the next read. 
 * 
 */
static void sensor_task(void *pvParameters)
{
    QueueHandle_t queue = (QueueHandle_t)pvParameters;
    bme280_data_t data;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS);

    int fail_count = 0;

    while (1) {
        esp_err_t err = bme280_read(&data);

        if (err != ESP_OK) {
            if(fail_count > 0){
                ESP_LOGI(TAG, "Sensor recovered after %d consecutive failures", fail_count);
                fail_count = 0;
            }

            if(xQueueSend(queue, &data,0) != pdTRUE){
                ESP_LOGW(TAG, "Data queue full, dropping sample");
            }
        }else{
            fail_count = 0;

            /* Escalate log level as failures accumulate, then quiet down */
            if(fail_count <= SENSOR_FAIL_THRESHOLD){
                ESP_LOGW(TAG, "Sensor read failed (%d): %s", fail_count, esp_err_to_name(err));
            }else if (fail_count == SENSOR_FAIL_THRESHOLD + 1){
                ESP_LOGE(TAG, "Sensor failure threshold reached, muting further logs");
            }

            /* Attempt to re-initialize after repeated failures */
            if(fail_count >= SENSOR_REINIT_THRESHOLD) {
                ESP_LOGW(TAG, "Attempting to re-initialize the sensor...");
                if(bme280_init() == ESP_OK){
                    ESP_LOGI(TAG, "Sensor re-initialized successfully");
                    fail_count = 0; /* Reset failure count on successful re-init */
                } else {
                    ESP_LOGE(TAG, "Sensor re-initialization failed");
                }
            }
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

/**
 * @brief Consumer task: wait for a sample from the data queue and 
 * print it over UART.
 * 
 * Blocks indefinitely on the queue. the shceduler wakes it up as soon 
 * as the sensor task publishes a new sample.
 */
static void display_task(void *pvParameters)
{
    QueueHandle_t queue = (QueueHandle_t)pvParameters;
    bme280_data_t data;

    while (1) {
        if (xQueueReceive(queue, &data, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Temperature: %.2f °C, Pressure: %.2f hPa, Humidity: %.2f %%RH\n",
                data.temperature, data.pressure, data.humidity);
        }
    }
}

/**
 * @brief Create the FreeRTOS queue and start all application tasks.
 * 
 * Must be called after app_init() and bme280_init(). Never returns.
 * 
 */
void app_tasks_start(void)
{
    /* Create the queue. Length = DATA_QUEUE_LENGTH items, each one
    * sizeof(bme280_data_t) bytes */
    s_data_queue = xQueueCreate(DATA_QUEUE_LENGTH, sizeof(bme280_data_t));
    if (s_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create data queue");
        return;
    }

    /* Create the sensor task */
    BaseType_t ret = xTaskCreate(
        sensor_task,
        "Sensor",
        SENSOR_TASK_STACK_SIZE,
        s_data_queue, /* passed as pvParameters */
        SENSOR_TASK_PRIORITY,
        NULL /* It will not be used */
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
        return;
    }

    ret = xTaskCreate(
        display_task,
        "Display",
        DISPLAY_TASK_STACK_SIZE,
        s_data_queue, 
        DISPLAY_TASK_PRIORITY,
        NULL 
    );

    ESP_LOGI(TAG, "Application tasks started");
}