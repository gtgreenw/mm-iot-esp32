#ifndef CAMERA_UNIT_SETTINGS_H
#define CAMERA_UNIT_SETTINGS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Stream quality: 0=low, 1=medium, 2=high. Default 2 (high). */
#define CAMERA_QUALITY_LOW    0
#define CAMERA_QUALITY_MED   1
#define CAMERA_QUALITY_HIGH  2

/** Get default stream quality (0/1/2). Returns true if stored, else *out = 2 (high). */
bool camera_unit_settings_get_quality(uint8_t *out);

/** Set default stream quality (0/1/2). Returns true on success. */
bool camera_unit_settings_set_quality(uint8_t q);

/** Get mirror image / horizontal flip (default false). */
bool camera_unit_settings_get_mirror(bool *out);

/** Set mirror image. Takes effect on next frame when streaming. */
bool camera_unit_settings_set_mirror(bool on);

/** Camera orientation: 0=normal, 1=180°, 2=90° CW, 3=90° CCW. 180° is hardware; 90°/270° can use view CSS. */
bool camera_unit_settings_get_orientation(uint8_t *out);

/** Set camera orientation (0–3). Takes effect on next frame when streaming. */
bool camera_unit_settings_set_orientation(uint8_t orient);

/** Get ESP-NOW enabled (default true). */
bool camera_unit_settings_get_espnow(bool *out);

/** Set ESP-NOW enabled. Applies immediately. */
bool camera_unit_settings_set_espnow(bool on);

/** Get HaLow link LED enabled (default true). Takes effect after reboot. */
bool camera_unit_settings_get_led_enabled(bool *out);

/** Set HaLow link LED enabled. Takes effect after reboot. */
bool camera_unit_settings_set_led_enabled(bool on);

/** Get microphone gain 0–255 (100 = unity, 255 = max sensitivity ~2.55×). */
bool camera_unit_settings_get_mic_gain(uint8_t *out);

/** Set microphone gain 0–255. */
bool camera_unit_settings_set_mic_gain(uint8_t gain);

/** Get mic sample rate: 0=16kHz, 1=8kHz, 2=3kHz. Takes effect after reboot. */
bool camera_unit_settings_get_mic_sample_rate(uint8_t *out);

/** Set mic sample rate 0–2. Takes effect after reboot. */
bool camera_unit_settings_set_mic_sample_rate(uint8_t rate);

#ifdef __cplusplus
}
#endif

#endif
