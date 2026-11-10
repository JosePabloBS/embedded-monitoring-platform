#include <stdio.h>
#include "app_init.h"
#include "app_tasks.h"
#include "bme280.h"

void app_main(void)
{
    app_init();
    ESP_ERROR_CHECK(bme280_init());
    app_tasks_start();
}
