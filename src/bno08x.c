#include "bno08x.h"
#include <hardware/timer.h>
#include <hardware/gpio.h>
#include <stdio.h>
#include <string.h>
#include "sh2_err.h"

sh2_Hal_t _HAL;
sh2_ProductIds_t prodIds; // Move this from header to here
static sh2_SensorValue_t *_sensor_value = NULL;
static bool _reset_occurred = false;
static int8_t _int_pin, _reset_pin;
static uint8_t _addr;
static i2c_inst_t *_i2c_inst = NULL;

static int i2chal_write(sh2_Hal_t *self, uint8_t *buffer, unsigned len);
static int i2chal_read(sh2_Hal_t *self, uint8_t *buffer, unsigned len, uint32_t *t_us);
static void i2chal_close(sh2_Hal_t *self);
static int i2chal_open(sh2_Hal_t *self);
static uint32_t hal_get_time_us(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *event);
static void sensor_handler(void *cookie, sh2_SensorEvent_t *event);
static void bno_hardware_reset(void);

// Fixed: Proper error checking for I2C operations
static inline bool i2c_read(uint8_t *buffer, unsigned len)
{
    int result = i2c_read_blocking(_i2c_inst, _addr, buffer, len, false);
    return (result == (int)len); // Success if we read the expected number of bytes
}

static inline bool i2c_write(uint8_t *buffer, unsigned len)
{
    int result = i2c_write_blocking(_i2c_inst, _addr, buffer, len, false);
    return (result == (int)len); // Success if we wrote the expected number of bytes
}

bool _init();

bool bno_begin_i2c(i2c_inst_t *i2c_inst, uint8_t addr, int8_t int_pin, int8_t reset_pin)
{
    printf("bno_begin_i2c\n");
    _HAL.open = i2chal_open;
    _HAL.close = i2chal_close;
    _HAL.read = i2chal_read;
    _HAL.write = i2chal_write;
    _HAL.getTimeUs = hal_get_time_us;

    _i2c_inst = i2c_inst;
    _int_pin = int_pin;
    _reset_pin = reset_pin;
    _addr = addr;
    
    return _init();
}

bool _init()
{
    printf("_init\n");
    int status;

    bno_hardware_reset();

    status = sh2_open(&_HAL, hal_callback, NULL);
    if (status != 0)
    {
        printf("sh2_open failed: %d\n", status);
        return false;
    }

    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);
    if (status != 0)
    {
        printf("sh2_getProdIds failed: %d\n", status);
        return false;
    }

    printf("Product ID: %08x %08x %08x %08x\n", 
           prodIds.entry[0], prodIds.entry[1], prodIds.entry[2], prodIds.entry[3]);

    sh2_setSensorCallback(sensor_handler, NULL);
    printf("_init done\n");
    return true;
}

bool bno_was_reset(void)
{
    bool x = _reset_occurred;
    _reset_occurred = false;
    return x;
}

bool bno_get_sensor_event(sh2_SensorValue_t *value)
{
    _sensor_value = value;

    value->timestamp = 0;

    sh2_service();

    if (value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV)
    {
        return false;
    }

    return true;
}

bool bno_enable_report(sh2_SensorId_t sensor_id, uint32_t interval_us)
{
    static sh2_SensorConfig_t cfg;

    cfg.changeSensitivityEnabled = false;
    cfg.wakeupEnabled = false;
    cfg.changeSensitivityRelative = false;
    cfg.alwaysOnEnabled = false;
    cfg.changeSensitivity = 0;
    cfg.batchInterval_us = 0;
    cfg.sensorSpecific = 0;
    cfg.reportInterval_us = interval_us;

    // Fixed: Actually enable the sensor by calling setSensorConfig
    int status = sh2_setSensorConfig(sensor_id, &cfg);

    if (status != 0)
    {
        printf("Failed to enable sensor %d: %d\n", sensor_id, status);
        return false;
    }

    return true;
}

static int i2chal_open(sh2_Hal_t *self)
{
    printf("i2chal_open\n");
    // Fixed: Correct soft reset packet for BNO08X
    uint8_t softreset_packet[] = {0x01, 0x00, 0x01, 0x00, 0x00};
    bool success = false;
    
    for (uint8_t attempts = 0; attempts < 5; attempts++)
    {
        printf("Soft reset attempt %d\n", attempts + 1);
        if (i2c_write(softreset_packet, 5))
        {
            success = true;
            break;
        }
        sleep_ms(50); // Increased delay between attempts
    }
    
    if (!success)
    {
        printf("Soft reset failed\n");
        return -1;
    }
    
    sleep_ms(500); // Increased delay after reset
    printf("Soft reset successful\n");
    return 0;
}

static void i2chal_close(sh2_Hal_t *self)
{
    printf("i2chal_close\n");
}

static int i2chal_read(sh2_Hal_t *self, uint8_t *buffer, unsigned len, uint32_t *t_us)
{
    // printf("i2chal_read (len=%d)\n", len);
    
    // Set timestamp when read begins
    if (t_us) {
        *t_us = time_us_32();
    }
    
    // printf("\tReading header\n");
    uint8_t header[4];
    if (!i2c_read(header, 4))
    {
        // printf("\tFailed to read header\n");
        return 0;
    }
    
    // printf("\tHeader: %02x %02x %02x %02x\n", header[0], header[1], header[2], header[3]);
    
    uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
    packet_size &= ~0x8000; // Clear continue bit
    
    // printf("\tPacket size: %d, buffer size: %d\n", packet_size, len);

    if (packet_size == 0) {
        // printf("\tEmpty packet\n");
        return 0;
    }

    if (packet_size > len)
    {
        // printf("\tPacket too large for buffer\n");
        return 0;
    }

    if (packet_size == 0) {
        // printf("\tEmpty packet\n");
        return 0;
    }

    uint16_t cargo_remaining = packet_size;
    uint8_t i2c_buffer[BUFFER_SIZE];
    uint16_t read_size;
    uint16_t cargo_read_amount = 0;
    bool first_read = true;
    
    while (cargo_remaining > 0)
    {
        if (first_read)
        {
            read_size = min(BUFFER_SIZE, (size_t)cargo_remaining);
        }
        else
        {
            read_size = min(BUFFER_SIZE, (size_t)cargo_remaining + 4);
        }
        
        // printf("\tReading from I2C: %d, remaining: %d\n", read_size, cargo_remaining);

        if (!i2c_read(i2c_buffer, read_size))
        {
            // printf("\tI2C read failed\n");
            return 0;
        }

        if (first_read)
        {
            cargo_read_amount = read_size;
            memcpy(buffer, i2c_buffer, cargo_read_amount);
            first_read = false;
        }
        else
        {
            cargo_read_amount = read_size - 4;
            memcpy(buffer, i2c_buffer + 4, cargo_read_amount);
        }
        
        buffer += cargo_read_amount;
        cargo_remaining -= cargo_read_amount;
    }
    
    // printf("i2chal_read done (returned %d bytes)\n", packet_size);
    return packet_size;
}

static int i2chal_write(sh2_Hal_t *self, uint8_t *buffer, unsigned len)
{
    uint16_t write_size = min(BUFFER_SIZE, len);

    // printf("i2c_hal write packet size: %d, max buffer size: %d\n", len, BUFFER_SIZE);

    if (!i2c_write(buffer, write_size))
    {
        // printf("I2C write failed\n");
        return 0;
    }

    return write_size;
}

static void bno_hardware_reset(void)
{
    printf("bno_hardware_reset\n");
    if (_reset_pin != -1)
    {
        gpio_init(_reset_pin);
        gpio_set_dir(_reset_pin, GPIO_OUT);
        
        // Fixed: Proper reset sequence with longer delays
        gpio_put(_reset_pin, 1);
        sleep_ms(50);  // Increased delay
        gpio_put(_reset_pin, 0);
        sleep_ms(50);  // Increased delay
        gpio_put(_reset_pin, 1);
        sleep_ms(200); // Wait for device to fully start
    }
    else
    {
        printf("No reset pin configured\n");
    }
}

static uint32_t hal_get_time_us(sh2_Hal_t *self)
{
    uint32_t t = time_us_32();
    return t; // Removed excessive printf to reduce noise
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *event)
{
    // printf("hal_callback eventId=%d\n", event->eventId);
    if (event->eventId == SH2_RESET)
    {
        printf("Reset detected!\n");
        _reset_occurred = true;
    }
}

static void sensor_handler(void *cookie, sh2_SensorEvent_t *event)
{
    int rc;
    // printf("Got sensor event\n");
    rc = sh2_decodeSensorEvent(_sensor_value, event);
    if (rc != 0)
    {
        printf("BNO08X - Error decoding sensor event: %d\n", rc);
        _sensor_value->timestamp = 0;
        return;
    }
}