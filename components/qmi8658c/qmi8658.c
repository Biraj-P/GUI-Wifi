#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "qmi8658.h"
#include <math.h>

float acc_lsb_div = 4096.0f;   // depends on config: 4G → 4096 LSB/g
float gyro_lsb_div = 32.8f;    // depends on config: 2000 dps → 32.8 LSB/dps

unsigned int imu_timestamp = 0;

//i2c_master_bus_handle_t i2c_master_handle;
//i2c_master_dev_handle_t dev_handle_imu;

esp_err_t i2c_write_byte(uint8_t reg, uint8_t data, i2c_master_dev_handle_t dev_handle) {
    uint8_t write_buf[2] = {reg, data};    
    esp_err_t ret = i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1);
    if (ret != ESP_OK) {
        ESP_LOGE("QMI8658", "I2C write failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t i2c_read_bytes(uint8_t reg, uint8_t *data, size_t len, i2c_master_dev_handle_t dev_handle) {    
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE("QMI8658", "I2C read failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

void QMI8658_write_reg(unsigned char reg, unsigned char value, i2c_master_dev_handle_t dev_handle) {
    i2c_write_byte(reg, value, dev_handle);
}

void QMI8658_read_reg(unsigned char reg, unsigned char *data, unsigned short len, i2c_master_dev_handle_t dev_handle) {
    i2c_read_bytes(reg, data, len, dev_handle);
}

void QMI8658_read_xyz(float acc[3], float gyro[3], unsigned int *tim_count, i2c_master_dev_handle_t dev_handle)
{
	unsigned char buf_reg[12];
	short raw_acc_xyz[3];
	short raw_gyro_xyz[3];
	//	float acc_t[3];
	//	float gyro_t[3];

	if (tim_count)
	{
		unsigned char buf[3];
		unsigned int timestamp;
		QMI8658_read_reg(QMI8658Register_Timestamp_L, buf, 3, dev_handle); // 0x18	24
		timestamp = (unsigned int)(((unsigned int)buf[2] << 16) | ((unsigned int)buf[1] << 8) | buf[0]);
		if (timestamp > imu_timestamp)
			imu_timestamp = timestamp;
		else
			imu_timestamp = (timestamp + 0x1000000 - imu_timestamp);

		*tim_count = imu_timestamp;
	}

	QMI8658_read_reg(QMI8658Register_Ax_L, buf_reg, 12, dev_handle); // 0x19, 25
	raw_acc_xyz[0] = (short)((unsigned short)(buf_reg[1] << 8) | (buf_reg[0]));
	raw_acc_xyz[1] = (short)((unsigned short)(buf_reg[3] << 8) | (buf_reg[2]));
	raw_acc_xyz[2] = (short)((unsigned short)(buf_reg[5] << 8) | (buf_reg[4]));

	raw_gyro_xyz[0] = (short)((unsigned short)(buf_reg[7] << 8) | (buf_reg[6]));
	raw_gyro_xyz[1] = (short)((unsigned short)(buf_reg[9] << 8) | (buf_reg[8]));
	raw_gyro_xyz[2] = (short)((unsigned short)(buf_reg[11] << 8) | (buf_reg[10]));

#if defined(QMI8658_UINT_MG_DPS)
	// mg
	acc[AXIS_X] = (float)(raw_acc_xyz[AXIS_X] * 1000.0f) / acc_lsb_div;
	acc[AXIS_Y] = (float)(raw_acc_xyz[AXIS_Y] * 1000.0f) / acc_lsb_div;
	acc[AXIS_Z] = (float)(raw_acc_xyz[AXIS_Z] * 1000.0f) / acc_lsb_div;
#else
	// m/s2
	acc[AXIS_X] = (float)(raw_acc_xyz[AXIS_X] * ONE_G) / acc_lsb_div;
	acc[AXIS_Y] = (float)(raw_acc_xyz[AXIS_Y] * ONE_G) / acc_lsb_div;
	acc[AXIS_Z] = (float)(raw_acc_xyz[AXIS_Z] * ONE_G) / acc_lsb_div;
#endif
	//	acc[AXIS_X] = imu_map.sign[AXIS_X]*acc_t[imu_map.map[AXIS_X]];
	//	acc[AXIS_Y] = imu_map.sign[AXIS_Y]*acc_t[imu_map.map[AXIS_Y]];
	//	acc[AXIS_Z] = imu_map.sign[AXIS_Z]*acc_t[imu_map.map[AXIS_Z]];

#if defined(QMI8658_UINT_MG_DPS)
	// dps
	gyro[0] = (float)(raw_gyro_xyz[0] * 1.0f) / gyro_lsb_div;
	gyro[1] = (float)(raw_gyro_xyz[1] * 1.0f) / gyro_lsb_div;
	gyro[2] = (float)(raw_gyro_xyz[2] * 1.0f) / gyro_lsb_div;
#else
	// rad/s
	gyro[AXIS_X] = (float)(raw_gyro_xyz[AXIS_X] * 0.01745f) / gyro_lsb_div; // *pi/180
	gyro[AXIS_Y] = (float)(raw_gyro_xyz[AXIS_Y] * 0.01745f) / gyro_lsb_div;
	gyro[AXIS_Z] = (float)(raw_gyro_xyz[AXIS_Z] * 0.01745f) / gyro_lsb_div;
#endif
	//	gyro[AXIS_X] = imu_map.sign[AXIS_X]*gyro_t[imu_map.map[AXIS_X]];
	//	gyro[AXIS_Y] = imu_map.sign[AXIS_Y]*gyro_t[imu_map.map[AXIS_Y]];
	//	gyro[AXIS_Z] = imu_map.sign[AXIS_Z]*gyro_t[imu_map.map[AXIS_Z]];
}



