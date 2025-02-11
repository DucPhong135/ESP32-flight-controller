#ifndef COMMON_DEFINE_H
#define COMMON_DEFINE_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "math.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"

#include "esp_wifi.h"
#include "nvs_flash.h"

#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "string.h"


extern QueueHandle_t Control_data_queue;
extern QueueHandle_t ESC_data_queue;

extern TaskHandle_t DataProcessingHandle;
extern TaskHandle_t MotorControlHandle;
extern TaskHandle_t ReceiveDataHandle;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} axis_data_t;

// Define the parent struct with gyro and acceleration
typedef struct
{
    axis_data_t gyro;         // Nested struct for gyroscope data
    axis_data_t acceleration; // Nested struct for acceleration data
} imu_data_t;

typedef struct
{
    float pitch;
    float roll;
    float yaw;
    float throttle;
} PRYT_data_t;


typedef struct
{
    uint16_t ESC1_out;
    uint16_t ESC2_out;
    uint16_t ESC3_out;
    uint16_t ESC4_out;
} ESC_out_t;

#endif // COMMON_STRUCT_H