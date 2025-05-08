#ifndef LVGL_DISPLAY_H
#define LVGL_DISPLAY_H

#include <string.h>
#include "lvgl.h"
#include "esp_lcd_types.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_panel_io.h"

extern const char *TAG;

void update_bt_icon(bool connected);

// Display and touch constants
#define TOUCH_I2C_NUM          (1)
#define LCD_HOST               SPI2_HOST
#define TOUCH_I2C_CLK_HZ       (400000)
#define LCD_H_RES              (240)
#define LCD_V_RES              (240)

#define TOUCH_I2C_SCL          (GPIO_NUM_7)
#define TOUCH_I2C_SDA          (GPIO_NUM_6)
#define TOUCH_GPIO_INT         (GPIO_NUM_5)
#define TOUCH_GPIO_RST         (GPIO_NUM_13)

#define LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)
#define LCD_BK_LIGHT_ON_LEVEL  1
#define LCD_BK_LIGHT_OFF_LEVEL (!LCD_BK_LIGHT_ON_LEVEL)
#define PIN_NUM_SCLK           (GPIO_NUM_10)
#define PIN_NUM_MOSI           (GPIO_NUM_11)
#define PIN_NUM_MISO           (GPIO_NUM_12)
#define PIN_NUM_LCD_DC         (GPIO_NUM_8)
#define PIN_NUM_LCD_RST        (GPIO_NUM_14)
#define PIN_NUM_LCD_CS         (GPIO_NUM_9)
#define PIN_NUM_BK_LIGHT       (GPIO_NUM_2)

#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8

#define LVGL_DRAW_BUF_LINES    20
#define LVGL_TICK_PERIOD_MS    2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STACK_SIZE   (8 * 1024)
#define LVGL_TASK_PRIORITY     2
#define ALPHA                  0.8f

#ifdef __cplusplus
extern "C" {
#endif

extern SemaphoreHandle_t touch_mux;
extern SemaphoreHandle_t i2c_mutex;
extern _lock_t lvgl_api_lock;
extern lv_display_rotation_t rotation;
extern lv_obj_t *btn;
extern int selected_activity_label;

void touch_callback(esp_lcd_touch_handle_t tp);
bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
void example_lvgl_port_update_callback(lv_display_t *disp);
void example_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);
void example_lvgl_touch_cb(lv_indev_t *indev, lv_indev_data_t *data);
void example_increase_lvgl_tick(void *arg);
void example_lvgl_port_task(void *arg);
void btn_cb(lv_event_t * e);
void set_angle(void * obj, int32_t v);
void example_lvgl_demo_ui(lv_display_t *disp);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // LVGL_DISPLAY_H
