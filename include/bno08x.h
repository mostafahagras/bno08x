#ifndef __BNO08X_H
#define __BNO08X_H

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "hardware/i2c.h"

#define min(a, b) ((a) < (b) ? (a) : (b))

#define BNO_ADDR 0x4A
#define BUFFER_SIZE 32

bool bno_begin_i2c(i2c_inst_t *i2c_inst, uint8_t addr, int8_t int_pin, int8_t reset_pin);

static void bno_hardware_reset(void);
bool bno_was_reset(void);

bool bno_enable_report(sh2_SensorId_t sensor, uint32_t interval_us);
bool bno_get_sensor_event(sh2_SensorValue_t *value);

#endif // __BNO08X_H
