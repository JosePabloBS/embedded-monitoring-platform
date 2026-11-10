#ifndef APP_INIT_H
#define APP_INIT_H

#include "driver/i2c_master.h"

void app_init(void);

i2c_master_bus_handle_t app_get_i2c_bus(void);

#endif