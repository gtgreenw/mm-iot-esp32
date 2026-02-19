idf_component_register(
    SRCS "ds18b20.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_rom log
)
