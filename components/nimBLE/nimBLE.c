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
#include "nimBLE.h"
#include "lvgl_display.h"

//const char *TAG = "BLE_IMU_SENSOR";

//
// BLE IMU UUIDs (Custom 128-bit UUIDs) as HEX
//
const ble_uuid128_t GATT_IMU_UUID =
    BLE_UUID128_INIT(0x4b, 0x91, 0x31, 0xc3, 0xc9, 0xc5, 0xcc, 0x8f, 
                    0x9e, 0x45, 0xb5, 0x1f, 0x01, 0xc2, 0xaf, 0x4f);

const ble_uuid128_t GATT_IMU_MEASUREMENT_UUID =
    BLE_UUID128_INIT(0xa8, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7, 
                    0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe);

const ble_uuid128_t GATT_DEVICE_INFO_UUID =
    BLE_UUID128_INIT(0x70, 0x36, 0x82, 0xd2, 0xb0, 0xd6, 0x45, 0xdf,
                     0xa0, 0xbc, 0xe4, 0xcd, 0xee, 0x3a, 0xe3, 0xaa);

const ble_uuid128_t GATT_MANUFACTURER_NAME_UUID =
    BLE_UUID128_INIT(0x02, 0xc6, 0xe1, 0x49, 0xcd, 0x7f, 0x4f, 0x74,
                     0xad, 0xa2, 0x76, 0x5b, 0x45, 0xf6, 0x6c, 0x41);

const ble_uuid128_t GATT_MODEL_NUMBER_UUID =
    BLE_UUID128_INIT(0xfa, 0x67, 0x61, 0x7b, 0x54, 0x4e, 0x43, 0x54,
                     0xa3, 0x3f, 0x25, 0xf9, 0x80, 0xbc, 0x74, 0x5c);


TimerHandle_t bleimu_tx_timer;
bool notify_state;
uint16_t conn_handle;
const char *device_name = "IMUdataNodeV1";
uint8_t bleimu_addr_type;
uint16_t imu_handle;

//
// GATT Service Definitions
//
const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &GATT_IMU_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &GATT_IMU_MEASUREMENT_UUID.u,
                .access_cb = gatt_svr_chr_access_imu,
                .val_handle = &imu_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {0},
        },
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &GATT_DEVICE_INFO_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &GATT_MANUFACTURER_NAME_UUID.u,
                .access_cb = gatt_svr_chr_access_device_info,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = &GATT_MODEL_NUMBER_UUID.u,
                .access_cb = gatt_svr_chr_access_device_info,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {0},
        },
    },
    {0},
};

//
// GATT Characteristic Access Callbacks
//
int gatt_svr_chr_access_device_info(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    const char *manuf_name = "NIT DURGAPUR";
    const char *model_num = "QMI8658C";

    if (ble_uuid_cmp(ctxt->chr->uuid, &GATT_MANUFACTURER_NAME_UUID.u) == 0) {
        return os_mbuf_append(ctxt->om, manuf_name, strlen(manuf_name)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (ble_uuid_cmp(ctxt->chr->uuid, &GATT_MODEL_NUMBER_UUID.u) == 0) {
        return os_mbuf_append(ctxt->om, model_num, strlen(model_num)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

int gatt_svr_chr_access_imu(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    char buf[BLE_UUID_STR_LEN];
    switch (ctxt->op) {
        case BLE_GATT_REGISTER_OP_SVC:
            MODLOG_DFLT(DEBUG, "Registered service %s with handle=%d\n",
                        ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                        ctxt->svc.handle);
            break;
        case BLE_GATT_REGISTER_OP_CHR:
            MODLOG_DFLT(DEBUG, "Registered characteristic %s with def_handle=%d val_handle=%d\n",
                        ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                        ctxt->chr.def_handle, ctxt->chr.val_handle);
            break;
        case BLE_GATT_REGISTER_OP_DSC:
            MODLOG_DFLT(DEBUG, "Registered descriptor %s with handle=%d\n",
                        ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                        ctxt->dsc.handle);
            break;
        default:
            assert(0);
            break;
    }
}

//
// GATT Server Init
//
int gatt_svr_init(void) {
    int rc;
    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }
    return 0;
}

//
// BLE GAP Event Handling
//
int bleimu_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "Connection %s; status=%d",
                     event->connect.status == 0 ? "established" : "failed",
                     event->connect.status);
            if (event->connect.status != 0) {
                bleimu_advertise();
            }
            conn_handle = event->connect.conn_handle;
            _lock_acquire(&lvgl_api_lock);
            update_bt_icon(true);
            _lock_release(&lvgl_api_lock);
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnect; reason=%d", event->disconnect.reason);
            bleimu_advertise();
            _lock_acquire(&lvgl_api_lock);
            update_bt_icon(false);
            _lock_release(&lvgl_api_lock);
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Advertisement complete");
            bleimu_advertise();
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "Subscribe event; cur_notify=%d, handle=%d",
                     event->subscribe.cur_notify, imu_handle);
            if (event->subscribe.attr_handle == imu_handle) {
                notify_state = event->subscribe.cur_notify;
                if (notify_state) {
                    xTimerStart(bleimu_tx_timer, portMAX_DELAY);
                } else {
                    xTimerStop(bleimu_tx_timer, portMAX_DELAY);
                }
            }
            break;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU update; conn_handle=%d mtu=%d",
                     event->mtu.conn_handle, event->mtu.value);
            break;
    }
    return 0;
}

//
// Advertisement Setup
//
void bleimu_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertisement data; rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(bleimu_addr_type, NULL, BLE_HS_FOREVER, &adv_params, bleimu_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising; rc=%d", rc);
    }
}

//
// BLE Sync + Reset Callbacks
//
void bleimu_on_sync(void) {
    int rc = ble_hs_id_infer_auto(0, &bleimu_addr_type);
    assert(rc == 0);

    uint8_t addr_val[6];
    rc = ble_hs_id_copy_addr(bleimu_addr_type, addr_val, NULL);

    ESP_LOGI(TAG, "Device Address: %02X:%02X:%02X:%02X:%02X:%02X",
             addr_val[5], addr_val[4], addr_val[3], addr_val[2], addr_val[1], addr_val[0]);

    bleimu_advertise();
}

void bleimu_on_reset(int reason) {
    ESP_LOGE(TAG, "Resetting state; reason=%d", reason);
}

//
// BLE Host Task
//
void bleimu_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}
