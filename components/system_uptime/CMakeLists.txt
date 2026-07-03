idf_component_register(
    SRCS "system_uptime.c"
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash freertos
)
