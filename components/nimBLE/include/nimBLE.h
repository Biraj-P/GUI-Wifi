#ifndef NIMBLE_H
#define NIMBLE_H

#include "host/ble_gatt.h"
#include "host/ble_gap.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "modlog/modlog.h"
#include "console/console.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const char *TAG;

// External variables
extern TimerHandle_t bleimu_tx_timer;
extern bool notify_state;
extern uint16_t conn_handle;
extern const char *device_name;
extern uint8_t bleimu_addr_type;
extern uint16_t imu_handle;

// UUID declarations
extern const ble_uuid128_t GATT_IMU_UUID;
extern const ble_uuid128_t GATT_IMU_MEASUREMENT_UUID;
extern const ble_uuid128_t GATT_DEVICE_INFO_UUID;
extern const ble_uuid128_t GATT_MANUFACTURER_NAME_UUID;
extern const ble_uuid128_t GATT_MODEL_NUMBER_UUID;

// Function declarations
int gatt_svr_chr_access_device_info(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg);

int gatt_svr_chr_access_imu(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg);

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);

int gatt_svr_init(void);

int bleimu_gap_event(struct ble_gap_event *event, void *arg);

void bleimu_advertise(void);

void bleimu_on_sync(void);

void bleimu_on_reset(int reason);

void bleimu_host_task(void *param);

#ifdef __cplusplus
}
#endif

#endif // NIMBLE_H
