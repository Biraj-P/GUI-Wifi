#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>

// FreeRTOS headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ESP-IDF headers
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

// LCD and Touch headers
#include "esp_lcd_gc9a01.h"
#include "esp_lcd_touch_cst816s.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

// Driver headers
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"

// LVGL header
#include "lvgl.h"

#include "lvgl_display.h"

SemaphoreHandle_t touch_mux = NULL;
SemaphoreHandle_t i2c_mutex;

lv_obj_t *bt_icon_label;

int selected_activity_label = 0; // 0 = none

// LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
_lock_t lvgl_api_lock;

void touch_callback(esp_lcd_touch_handle_t tp)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(touch_mux, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}


bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
void example_lvgl_port_update_callback(lv_display_t *disp)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    lv_display_rotation_t rotation = lv_display_get_rotation(disp);

    switch (rotation) {
    case LV_DISPLAY_ROTATION_0:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    case LV_DISPLAY_ROTATION_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISPLAY_ROTATION_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISPLAY_ROTATION_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    }
}

void example_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    example_lvgl_port_update_callback(disp);
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // because SPI LCD is big-endian, we need to swap the RGB bytes order
    lv_draw_sw_rgb565_swap(px_map, (offsetx2 + 1 - offsetx1) * (offsety2 + 1 - offsety1));
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
}

void example_lvgl_touch_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    esp_lcd_touch_handle_t touch_pad = lv_indev_get_user_data(indev);

    if (xSemaphoreTake(touch_mux, 0) == pdTRUE) {
        esp_lcd_touch_read_data(touch_pad); // read only when ISR was triggled
    }

    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(touch_pad, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        uint16_t x = touchpad_x[0];
        uint16_t y = touchpad_y[0];

        // Center of the screen
        uint16_t center_x = LCD_H_RES / 2;
        uint16_t center_y = LCD_V_RES / 2;
        uint16_t radius = MIN(center_x, center_y);

        // Calculate distance from center
        int dx = x - center_x;
        int dy = y - center_y;
        int dist = sqrt(dx * dx + dy * dy);

        // If the point is inside the circle, map it directly
        if (dist <= radius) {
            data->point.x = x;
            data->point.y = y;
            data->state = LV_INDEV_STATE_PRESSED;

            //ESP_LOGI(TAG, "Touch coordinates: X = %d, Y = %d", x, y);

        } else {
            // Optionally, ignore points outside the circle or map them to the edge
            data->state = LV_INDEV_STATE_RELEASED;
        }
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    uint32_t time_threshold_ms = 1000 / CONFIG_FREERTOS_HZ;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        // in case of triggering a task watch dog time out
        time_till_next_ms = MAX(time_till_next_ms, time_threshold_ms);
        usleep(1000 * time_till_next_ms);
    }
}

lv_obj_t * btn;
lv_display_rotation_t rotation = LV_DISP_ROTATION_0;

void btn_cb(lv_event_t * e)
{
    lv_display_t *disp = lv_event_get_user_data(e);
    rotation++;
    if (rotation > LV_DISP_ROTATION_270) {
        rotation = LV_DISP_ROTATION_0;
    }
    lv_disp_set_rotation(disp, rotation);
}
void set_angle(void * obj, int32_t v)
{
    lv_arc_set_value(obj, v);
}

static void scroll_event_cb(lv_event_t * e)
{
    lv_obj_t * cont = lv_event_get_target(e);

    lv_area_t cont_a;
    lv_obj_get_coords(cont, &cont_a);
    int32_t cont_y_center = cont_a.y1 + lv_area_get_height(&cont_a) / 2;

    int32_t r = lv_obj_get_height(cont) * 7 / 10;
    int32_t i;
    int32_t child_cnt = (int32_t)lv_obj_get_child_count(cont);
    for(i = 0; i < child_cnt; i++) {
        lv_obj_t * child = lv_obj_get_child(cont, i);
        lv_area_t child_a;
        lv_obj_get_coords(child, &child_a);

        int32_t child_y_center = child_a.y1 + lv_area_get_height(&child_a) / 2;
        int32_t diff_y = LV_ABS(child_y_center - cont_y_center);

        int32_t x;
        if(diff_y >= r) {
            x = r;
        } else {
            uint32_t x_sqr = r * r - diff_y * diff_y;
            lv_sqrt_res_t res;
            lv_sqrt(x_sqr, &res, 0x8000);
            x = r - res.i;
        }

        lv_obj_set_style_translate_x(child, x, 0);
        lv_opa_t opa = (lv_opa_t)lv_map(x, 0, r, LV_OPA_TRANSP, LV_OPA_COVER);
        lv_obj_set_style_opa(child, LV_OPA_COVER - opa, 0);
    }
}

static lv_obj_t *active_btn = NULL;

static void activity_btn_event_cb(lv_event_t * e)
{
    lv_obj_t * btn = lv_event_get_target(e);
    lv_obj_t * label = lv_obj_get_child(btn, 0);
    const char * txt = lv_label_get_text(label);

    // Toggle behavior
    if (active_btn == btn) {
        // Deactivate
        active_btn = NULL;
        selected_activity_label = 0;
        lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_GREEN), 0);
        LV_LOG_USER("Activity Deselected: %s", txt);
    } else {
        // Deactivate previous if any
        if (active_btn != NULL) {
            lv_obj_set_style_bg_color(active_btn, lv_palette_main(LV_PALETTE_GREEN), 0);
        }

        // Activate new
        active_btn = btn;
        lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_RED), 0);

        // Update activity label
        if      (strcmp(txt, "Walking")  == 0) selected_activity_label = 1;
        else if (strcmp(txt, "Running")  == 0) selected_activity_label = 2;
        else if (strcmp(txt, "Cycling")  == 0) selected_activity_label = 3;
        else if (strcmp(txt, "Sitting")  == 0) selected_activity_label = 4;
        else if (strcmp(txt, "Stair Dwn") == 0) selected_activity_label = 5;
        else if (strcmp(txt, "Jumping")  == 0) selected_activity_label = 6;
        else if (strcmp(txt, "Stair Up") == 0) selected_activity_label = 7;
        else if (strcmp(txt, "Sleeping")  == 0) selected_activity_label = 8;
        else selected_activity_label = 0;

        LV_LOG_USER("Activity Selected: %s -> Label %d", txt, selected_activity_label);
    }
}

void update_bt_icon(bool connected)
{
    if (bt_icon_label == NULL) return;

    lv_color_t color = connected ? lv_palette_main(LV_PALETTE_GREEN)
                                 : lv_palette_main(LV_PALETTE_RED);

    lv_obj_set_style_text_color(bt_icon_label, color, 0);
}

void example_lvgl_demo_ui(lv_display_t *disp)
{
    
    lv_obj_t * cont = lv_obj_create(lv_screen_active());
    lv_obj_set_size(cont, 220, 220);
    lv_obj_center(cont);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scroll_dir(cont, LV_DIR_VER);
    lv_obj_set_scroll_snap_y(cont, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(cont, lv_color_hex(0x000000), 0);  // Dark background
    lv_obj_set_style_radius(cont, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_clip_corner(cont, true, 0);
    lv_obj_add_event_cb(cont, scroll_event_cb, LV_EVENT_SCROLL, NULL);

    const char * activities[] = {
        "Walking", "Running", "Cycling", "Sitting",
        "Stair Dwn", "Jumping", "Stair Up", "Sleeping"
    };

    for (uint32_t i = 0; i < sizeof(activities)/sizeof(activities[0]); i++) {
        lv_obj_t * btn = lv_button_create(cont);
        lv_obj_set_width(btn, lv_pct(100));
        lv_obj_set_height(btn, 50);
        lv_obj_set_style_radius(btn, 12, 0);
        lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_GREEN), 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_90, 0);
        lv_obj_add_event_cb(btn, activity_btn_event_cb, LV_EVENT_CLICKED, NULL);

        lv_obj_t * label = lv_label_create(btn);
        lv_label_set_text(label, activities[i]);
        lv_obj_center(label);
    }

    lv_obj_send_event(cont, LV_EVENT_SCROLL, NULL);
    lv_obj_scroll_to_view(lv_obj_get_child(cont, 0), LV_ANIM_OFF);

    // Top status bar container
    lv_obj_t *status_bar = lv_obj_create(lv_screen_active());
    lv_obj_set_size(status_bar, lv_pct(100), 30);
    lv_obj_align(status_bar, LV_ALIGN_TOP_MID, 0, -5);
    lv_obj_set_style_bg_color(status_bar, lv_color_hex(0x000000), 0);  // Dark background
    lv_obj_clear_flag(status_bar, LV_OBJ_FLAG_CLICKABLE); // Make it non-clickable

    // Bluetooth icon label
    bt_icon_label = lv_label_create(status_bar);
    lv_label_set_text(bt_icon_label, LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_font(bt_icon_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(bt_icon_label, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_align(bt_icon_label, LV_ALIGN_TOP_MID, 0, -7);


}
