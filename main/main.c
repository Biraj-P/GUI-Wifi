#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ESP-IDF
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"

// LCD / Touch
#include "esp_lcd_gc9a01.h"
#include "esp_lcd_touch_cst816s.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

// LVGL
#include "lvgl.h"
#include "lvgl_display.h"    // your example_lvgl_* callbacks

// Drivers
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"

// IMU
#include "qmi8658.h"

// MQTT
#include "mqtt_client.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "mqtt_client.h"

#include "freertos/semphr.h"

#include "data_model.h" 
extern int selected_activity_label;

// -----------------------------------------------------------------------------
// Pin / Display / LVGL config (as before)
// -----------------------------------------------------------------------------
// #define PIN_NUM_BK_LIGHT     2
// #define PIN_NUM_SCLK         10
// #define PIN_NUM_MOSI         11
// #define PIN_NUM_MISO         12
// #define PIN_NUM_LCD_DC       8
// #define PIN_NUM_LCD_CS       9
// #define PIN_NUM_LCD_RST      14
// #define LCD_HOST             SPI2_HOST
// #define LCD_PIXEL_CLOCK_HZ   (10*1000*1000)
// #define LCD_CMD_BITS         8
// #define LCD_PARAM_BITS       8
// #define LCD_RGB_ELEMENT_ORDER_BGR true
// #define LCD_BK_LIGHT_ON_LEVEL 1
// #define LCD_H_RES            240
// #define LCD_V_RES            240

// #define TOUCH_I2C_NUM        I2C_NUM_0
// #define TOUCH_I2C_SCL        22
// #define TOUCH_I2C_SDA        21
// #define TOUCH_GPIO_RST       PIN_NUM_LCD_RST
// #define TOUCH_GPIO_INT       32

// #define LVGL_DRAW_BUF_LINES  40
// #define LVGL_TICK_PERIOD_MS  2
// #define LVGL_TASK_STACK_SIZE 4096
// #define LVGL_TASK_PRIORITY   1


// Wi-Fi / MQTT settings
//#define WIFI_SSID               "Redmi"
//#define WIFI_PASSWORD               "birajpaul"
//#define MQTT_BROKER         "mqtt://192.168.160.67:1883"
#define WIFI_SSID               "TP-Link_23EC"
#define WIFI_PASSWORD               "Subrata@nandiwifi"
#define MQTT_BROKER         "mqtt://192.168.0.141:1883"
//#define MQTT_BROKER         "ws://192.168.0.141:9001"
#define MQTT_TOPIC_PUB          "sensor/rasp"
//#define SENSOR_PUB_INTERVAL_MS  33 //26ms + 7
//#define SENSOR_PUB_INTERVAL_MS  27 //20ms + 7
#define SENSOR_PUB_INTERVAL_MS  30 //25ms + 7

const char *TAG = "GC9A01-CST816";
i2c_master_dev_handle_t touch_dev_handle;
i2c_master_dev_handle_t dev_handle_imu;
SemaphoreHandle_t sensor_data_mutex;
sensor_data_t shared_sensor_data; 

//static const char *TAG = "GC9A01-CST816";

// Globals for I²C mutex and MQTT client

//static SemaphoreHandle_t i2c_mutex;
static esp_mqtt_client_handle_t mqtt_client;

// Wi-Fi event handler
static void wifi_event_handler(void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
esp_wifi_connect();
} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
esp_wifi_connect();
} else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
ip_event_got_ip_t *e = event_data;
ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
}
}

static void mqtt_event_handler(void *handler_args,
    esp_event_base_t base,
    int32_t event_id,
    void *event_data)
{
esp_mqtt_event_handle_t evt = event_data;
if (evt->event_id == MQTT_EVENT_CONNECTED) {
ESP_LOGI(TAG, "MQTT connected");
} else if (evt->event_id == MQTT_EVENT_DISCONNECTED) {
ESP_LOGI(TAG, "MQTT disconnected");
}
}

// static void mqtt_sensor_task(void *arg)
// {
// i2c_master_dev_handle_t imu = arg;
// char buf[128];
// while (1) {
// float acc[3], gyro[3];
// unsigned int ts = 0;
// if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100))) {
// QMI8658_read_xyz(acc, gyro, &ts, imu);
// xSemaphoreGive(i2c_mutex);
// }
// int len = snprintf(buf, sizeof(buf),
// "{\"ts\":%u,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
// "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f}",
// ts, acc[0],acc[1],acc[2], gyro[0],gyro[1],gyro[2]
// );
// if (len>0) {
// esp_mqtt_client_publish(mqtt_client, "sensor/rasp", buf, 0, 1, 0);
// }
// vTaskDelay(pdMS_TO_TICKS(SENSOR_PUB_INTERVAL_MS));
// }
// }

static void mqtt_sensor_task(void *arg)
{
    i2c_master_dev_handle_t imu = (i2c_master_dev_handle_t)arg;
    const float threshold       = 1417.0f;
    const uint64_t debounce_ms  = 200;
    static uint32_t step_count  = 0;
    static float distance_m     = 0;
    static float current_speed  = 0;
    static float last_accel_mag = 0;
    static uint64_t last_step_time = 0;

    char payload[256];

    while (1) {
        float acc[3], gyro[3];
        unsigned int ts = 0;
        uint64_t now_ms = esp_timer_get_time() / 1000;

        // 1) Read IMU
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100))) {
            QMI8658_read_xyz(acc, gyro, &ts, imu);
            xSemaphoreGive(i2c_mutex);
        }

        // 2) Step detection
        float mag = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
        if (mag > threshold && last_accel_mag <= threshold && (now_ms - last_step_time) > debounce_ms) {
            step_count++;
            distance_m = step_count * 0.78f;  // 0.78 m per step
            if (last_step_time) {
                float dt_s = (now_ms - last_step_time) / 1000.0f;
                current_speed = (dt_s>0) ? (0.78f / dt_s) : 0;
            }
            last_step_time = now_ms;
        }
        last_accel_mag = mag;

        // 3) Update shared_sensor_data under its mutex
        if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY)) {
            shared_sensor_data.timestamp        = ts;
            shared_sensor_data.label            = selected_activity_label;
            shared_sensor_data.pedometer_count  = step_count;
            shared_sensor_data.distance         = distance_m;
            shared_sensor_data.speed            = current_speed;
            shared_sensor_data.acc[0]           = acc[0];
            shared_sensor_data.acc[1]           = acc[1];
            shared_sensor_data.acc[2]           = acc[2];
            shared_sensor_data.gyro[0]          = gyro[0];
            shared_sensor_data.gyro[1]          = gyro[1];
            shared_sensor_data.gyro[2]          = gyro[2];

            // Reset on “label == 0” if you like (same as before):
            if (selected_activity_label == 0) {
                step_count = 0;
                distance_m = 0;
                current_speed = 0;
                last_step_time = 0;
                shared_sensor_data.pedometer_count = 0;
                shared_sensor_data.distance = 0;
                shared_sensor_data.speed = 0;
            }
            xSemaphoreGive(sensor_data_mutex);
        }

        // 4) Format JSON with all fields
        int len = snprintf(payload, sizeof(payload),
            "{"
              "\"ts\":%u,"
              "\"label\":%d,"
              "\"steps\":%lu,"
              "\"dist\":%.2f,"
              "\"spd\":%.2f,"
              "\"acc_X\":%.2f,"
              "\"acc_Y\":%.2f,"
              "\"acc_Z\":%.2f,"
              "\"gyro_X\":%.2f,"
              "\"gyro_Y\":%.2f,"
              "\"gyro_Z\":%.2f"
            "}",
            ts,
            selected_activity_label,
            step_count,
            distance_m,
            current_speed,
            acc[0], acc[1], acc[2],
            gyro[0], gyro[1], gyro[2]
        );

        // 5) Publish
        if (len > 0) {
            esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_PUB, payload, 0, 0, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_PUB_INTERVAL_MS));
    }
}


void app_main(void){
    touch_mux = xSemaphoreCreateBinary();
    sensor_data_mutex = xSemaphoreCreateMutex();

    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_LCD_DC,
        .cs_gpio_num = PIN_NUM_LCD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };

    ESP_LOGI(TAG, "Install GC9A01 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    // create a lvgl display
    lv_display_t *display = lv_display_create(LCD_H_RES, LCD_V_RES);

    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    size_t draw_buffer_sz = LCD_H_RES * LVGL_DRAW_BUF_LINES * sizeof(lv_color16_t);

    void *buf1 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf1);
    void *buf2 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_display_set_buffers(display, buf1, buf2, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);
    // associate the mipi panel handle to the display
    lv_display_set_user_data(display, panel_handle);
    // set color depth
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);
    // set the callback which can copy the rendered image to an area of the display
    lv_display_set_flush_cb(display, example_lvgl_flush_cb);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "Register io panel event callback for LVGL flush ready notification");
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = example_notify_lvgl_flush_ready,
    };
    /* Register done callback */
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display));

    /* Initialize I2C Master Bus using new API */
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = TOUCH_I2C_NUM,
        .scl_io_num = TOUCH_I2C_SCL,
        .sda_io_num = TOUCH_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    /* Add touch device to the I2C bus */
    i2c_device_config_t dev_cfg_touch = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x15, // Check datasheet for correct address
        .scl_speed_hz = TOUCH_I2C_CLK_HZ,
    };

    i2c_device_config_t imu_dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = QMI8658_I2C_ADDR,
        .scl_speed_hz = 400000
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_touch, &touch_dev_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &imu_dev_conf, &dev_handle_imu));


    /* Initialize touch HW */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = TOUCH_GPIO_RST, // Shared with LCD reset
        .int_gpio_num = TOUCH_GPIO_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .interrupt_callback = touch_callback,

    };

    esp_lcd_touch_handle_t touch_handle;
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;

    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();

    /* Ensure correct API usage for the new driver */
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(bus_handle, &tp_io_config, &tp_io_handle));
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &touch_handle));

    static lv_indev_t *indev;
    indev = lv_indev_create(); // Input device driver (Touch)
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(indev, display);
    lv_indev_set_user_data(indev, touch_handle);
    lv_indev_set_read_cb(indev, example_lvgl_touch_cb);

    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreatePinnedToCore(example_lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL, 1);

    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        return;
    }
    ESP_LOGI(TAG, "Display LVGL Meter Widget");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    _lock_acquire(&lvgl_api_lock);
    example_lvgl_demo_ui(display);
    _lock_release(&lvgl_api_lock);

    uint8_t who_am_i = 0;
    uint8_t revision_id = 0;
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100))) {
    // Read WHO_AM_I register
    esp_err_t err = i2c_read_bytes(0x00, &who_am_i, 1, dev_handle_imu);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", who_am_i);
    }

    // Read Revision ID register
    err = i2c_read_bytes(0x01, &revision_id, 1, dev_handle_imu);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Revision ID = 0x%02X", revision_id);
    }

    // Write to control registers
    i2c_write_byte(0x02, 0x60, dev_handle_imu); // CTRL1
    i2c_write_byte(0x03, 0x23, dev_handle_imu); // CTRL2
    i2c_write_byte(0x04, 0x43, dev_handle_imu); // CTRL3
    i2c_write_byte(0x06, 0x00, dev_handle_imu); // CTRL5
    i2c_write_byte(0x08, 0x03, dev_handle_imu); // CTRL7

    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for sensor to stabilize

    unsigned char read_data = 0x00;
	QMI8658_read_reg(0x02, &read_data, 1, dev_handle_imu);
	ESP_LOGI(TAG, "QMI8658Register_Ctrl1=0x%x \n", read_data);
	QMI8658_read_reg(0x03, &read_data, 1, dev_handle_imu);
	ESP_LOGI(TAG, "QMI8658Register_Ctrl2=0x%x \n", read_data);
	QMI8658_read_reg(0x04, &read_data, 1, dev_handle_imu);
	ESP_LOGI(TAG, "QMI8658Register_Ctrl3=0x%x \n", read_data);
	QMI8658_read_reg(0x06, &read_data, 1, dev_handle_imu);
	ESP_LOGI(TAG, "QMI8658Register_Ctrl5=0x%x \n", read_data);
	QMI8658_read_reg(0x08, &read_data, 1, dev_handle_imu);
	ESP_LOGI(TAG, "QMI8658Register_Ctrl7=0x%x \n", read_data);
    xSemaphoreGive(i2c_mutex);
    } else {
        ESP_LOGW(TAG, "Failed to take I2C mutex for startup IMU read");
    }

    // Read one sample to confirm it's working
    float acc[3], gyro[3];
    unsigned int timestamp = 0;
    
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100))) {
        QMI8658_read_xyz(acc, gyro, &timestamp, dev_handle_imu);
        xSemaphoreGive(i2c_mutex);
    } else {
        ESP_LOGW(TAG, "Failed to take I2C mutex for startup IMU read");
    }
        
    ESP_LOGI(TAG, "First Read - Timestamp: %u", timestamp);
    ESP_LOGI(TAG, "Accel [m/s^2] => X: %.3f, Y: %.3f, Z: %.3f", acc[0], acc[1], acc[2]);
    ESP_LOGI(TAG, "Gyro  [rad/s] => X: %.3f, Y: %.3f, Z: %.3f", gyro[0], gyro[1], gyro[2]);

    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ret = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK(ret);

    // 1) NVS for Wi-Fi
ESP_ERROR_CHECK(nvs_flash_init());

// 2) Wi-Fi init
ESP_LOGI(TAG, "Starting Wi-Fi");
ESP_ERROR_CHECK(esp_netif_init());
ESP_ERROR_CHECK(esp_event_loop_create_default());
esp_netif_create_default_wifi_sta();
wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
wifi_config_t sta_cfg = {
    .sta = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASSWORD,
        .threshold.authmode = WIFI_AUTH_WPA2_PSK,
    },
};
ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
ESP_ERROR_CHECK(esp_wifi_start());

// 3) MQTT init
ESP_LOGI(TAG, "Starting MQTT");
esp_mqtt_client_config_t mqtt_cfg = {
    .broker.address.uri = MQTT_BROKER,
    //.buffer.out_size = 10240,
};
mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
esp_mqtt_client_start(mqtt_client);

// 4) Launch sensor→MQTT task
xTaskCreate(mqtt_sensor_task,
            "mqtt_sensor",
            4096,
            dev_handle_imu,    // your IMU I2C handle
            5,
            NULL);
}