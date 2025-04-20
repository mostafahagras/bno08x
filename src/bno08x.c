#include "bno08x.h"

sh2_Hal_t _HAL;
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

static inline int i2c_read(uint8_t *buffer, unsigned len)
{
    return i2c_read_blocking(_i2c_inst, _addr, buffer, len, false);
}

static inline int i2c_write(uint8_t *buffer, unsigned len)
{
    return i2c_write_blocking(_i2c_inst, _addr, buffer, len, false);
}

bool _init();

bool bno_begin_i2c(i2c_inst_t *i2c_inst, uint8_t addr, int8_t int_pin, int8_t reset_pin)
{
    printf("bno_begin_i2c");
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
    if (status != SH2_OK)
    {
        return false;
    }

    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);
    if (status != SH2_OK)
    {
        return false;
    }

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

    int status = sh2_getSensorConfig(sensor_id, &cfg);

    if (status != SH2_OK)
    {
        return false;
    }

    return true;
}

static int i2chal_open(sh2_Hal_t *self)
{
    printf("i2chal_open\n");
    uint8_t softreset_packet[] = {5, 0, 1, 0, 1};
    bool success = false;
    for (uint8_t attempts = 0; attempts < 5; attempts++)
    {
        if (i2c_write(softreset_packet, 5))
        {
            success = true;
            break;
        }
        sleep_ms(30);
    }
    if (!success)
        return -1;
    sleep_ms(300);
    return 0;
}

static void i2chal_close(sh2_Hal_t *self)
{
    printf("i2chal_close\n");
}

static int i2chal_read(sh2_Hal_t *self, uint8_t *buffer, unsigned len, uint32_t *t_us)
{
    printf("i2chal_read\n");
    printf("\tReading header\n");
    uint8_t header[4];
    if (!i2c_read(header, 4))
    {
        return 0;
    }
    printf("\tRead header\n");
    uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
    packet_size &= ~0x8000; // 0b0111111111111111 (unset continue bit)
    printf("\tPacket size: %d, buffer size: %d", packet_size, len);

    if (packet_size > len)
    {
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
        printf("\tReading from I2C: %d, remaining: %d", read_size, cargo_remaining);

        if (!i2c_read(i2c_buffer, read_size))
        {
            return 0;
        }

        if (first_read)
        {
            cargo_read_amount = read_size;
            memcpy(buffer, i2c_buffer, cargo_read_amount);
        }
        else
        {
            cargo_read_amount = read_size - 4;
            memcpy(buffer, i2c_buffer + 4, cargo_read_amount);
        }
        buffer += cargo_read_amount;
        cargo_remaining -= cargo_read_amount;
    }
    printf("i2chal_read done");
    return packet_size;
}

static int i2chal_write(sh2_Hal_t *self, uint8_t *buffer, unsigned len)
{
    uint16_t write_size = min(BUFFER_SIZE, len);

    printf("i2c_hal write packet size: %d, max buffer size: %d", len, BUFFER_SIZE);

    if (!i2c_write(buffer, write_size))
    {
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
        gpio_put(_reset_pin, 1);
        sleep_ms(10);
        gpio_put(_reset_pin, 0);
        sleep_ms(10);
        gpio_put(_reset_pin, 1);
    }
}

static uint32_t hal_get_time_us(sh2_Hal_t *self)
{
    uint32_t t = time_us_32();
    printf("hal_get_time_us %d", t);
    return t;
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *event)
{
    printf("hal_callback %d", event->eventId);
    if (event->eventId == SH2_RESET)
    {
        printf("Reset!");
        _reset_occurred = true;
    }
}

static void sensor_handler(void *cookie, sh2_SensorEvent_t *event)
{
    int rc;
    printf("Got an event\n");
    rc = sh2_decodeSensorEvent(_sensor_value, event);
    if (rc != SH2_OK)
    {
        printf("BNO08X - Error decoding sensor event\n");
        _sensor_value->timestamp = 0;
        return;
    }
}