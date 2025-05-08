// qmi8658.h

#ifndef QMI8658_H
#define QMI8658_H

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define QMI8658_I2C_ADDR         0x6B
#define I2C_MASTER_TIMEOUT_MS    100
#define QMI8658_SLAVE_ADDR_L     0x6A
#define QMI8658_SLAVE_ADDR_H     0x6B
#define QMI8658_UINT_MG_DPS     // Uncomment to output acc in mg and gyro in dps

#define QMI8658Register_Timestamp_L  0x30
#define QMI8658Register_Ax_L         0x35

#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2

#define ONE_G 9.80665f

typedef struct {
    float last_accel;
    uint64_t last_time_ms;
    int step_count;
    float distance_m;    // Total distance in meters
    float speed_mps;     // Instantaneous speed in meters/second
} pedometer_state_t;


extern float acc_lsb_div;
extern float gyro_lsb_div;
extern unsigned int imu_timestamp;

extern i2c_master_bus_handle_t i2c_master_handle;
extern i2c_master_dev_handle_t dev_handle_imu;

/**
 * @brief Write a single byte to a register over I2C
 */
esp_err_t i2c_write_byte(uint8_t reg, uint8_t data, i2c_master_dev_handle_t dev_handle);

/**
 * @brief Read multiple bytes from a register over I2C
 */
esp_err_t i2c_read_bytes(uint8_t reg, uint8_t *data, size_t len, i2c_master_dev_handle_t dev_handle);

/**
 * @brief Write to a QMI8658 register
 */
void QMI8658_write_reg(unsigned char reg, unsigned char value, i2c_master_dev_handle_t dev_handle);

/**
 * @brief Read from a QMI8658 register
 */
void QMI8658_read_reg(unsigned char reg, unsigned char *data, unsigned short len, i2c_master_dev_handle_t dev_handle);

/**
 * @brief Read accelerometer, gyroscope, and timestamp data
 */
void QMI8658_read_xyz(float acc[3], float gyro[3], unsigned int *tim_count, i2c_master_dev_handle_t dev_handle);


#ifdef __cplusplus
}
#endif

#endif // QMI8658_H
