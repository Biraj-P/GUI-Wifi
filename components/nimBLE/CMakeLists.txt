idf_component_register(
    SRCS "nimBLE.c"
    INCLUDE_DIRS "include"
    REQUIRES bt nvs_flash lvgl_display
)