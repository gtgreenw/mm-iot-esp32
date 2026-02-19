/*
 * Override the WiFi driver's default stack size for the "wifi" task.
 * The binary libpp.a provides config_get_wifi_task_stack_size(); we use
 * linker --wrap so __wrap_config_get_wifi_task_stack_size is called instead.
 * Avoids "stack overflow in task wifi" when running AP+STA+NAPT (beacon,
 * power save, etc.). Stack size in bytes (FreeRTOS uses this for the task).
 * 8192 was insufficient on ESP32-S3; 16384 gives headroom.
 */
#include <stdint.h>

uint32_t __wrap_config_get_wifi_task_stack_size(void)
{
    return 16384;
}
