#ifndef DATA_MODEL_H
#define DATA_MODEL_H

#include <stdint.h>
extern SemaphoreHandle_t sensor_data_mutex;
typedef struct {
    uint64_t timestamp;
    int label; 
    uint32_t pedometer_count;
    float speed;
    float distance;
    float acc[3];  // x, y, z
    float gyro[3]; // x, y, z
} sensor_data_t;

// Global instance (or make it static and expose accessors)
extern sensor_data_t shared_sensor_data;

// Optional: Mutex for thread-safe access
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#endif // DATA_MODEL_H
