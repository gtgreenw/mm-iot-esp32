/**
 * DS18B20 1-Wire temperature sensor driver (bit-bang on one GPIO).
 * Single sensor, Skip ROM. Use 4.7k pull-up between DATA and VCC.
 * Wiring: Red=VCC (3–5.5V), Yellow=DATA, Black=GND.
 */
#ifndef DS18B20_H
#define DS18B20_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize DS18B20 on the given GPIO (1-Wire data line).
 * Call once at startup. Does a presence check.
 * \param gpio_num GPIO number for 1-Wire data (pull-up 4.7k to VCC)
 * \return true if a device responded (presence pulse), false otherwise
 */
bool ds18b20_init(int gpio_num);

/**
 * Read temperature in degrees Celsius.
 * Triggers conversion, waits 750 ms (12-bit), then reads scratchpad.
 * \return temperature in °C, or -127.0f on error / no sensor
 */
float ds18b20_read_temp_c(void);

/**
 * Check if the driver was initialized and a device was detected.
 */
bool ds18b20_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* DS18B20_H */
