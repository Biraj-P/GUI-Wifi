
idf_component_register(
    SRCS "lvgl_display.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer esp_lcd_gc9a01 esp_lcd_touch_cst816s lvgl
)