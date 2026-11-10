#ifndef APP_TASKS_H
#define APP_TASKS_H

/**
 * @brief Create the FreeRTOS queue and start all application tasks. 
 * 
 * Must be called after app_init() and bme280_init(). Never returns
 * control to the caller in the current design (tasks run forever).
 * 
 */
void app_tasks_start(void);

#endif // APP_TASKS_H

