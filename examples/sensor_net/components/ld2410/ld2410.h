/**
 * LD2410 / 24GHz mmWave for XIAO (Seeed) – UART driver.
 * Protocol: Seeed 24GHz mmWave for XIAO User Protocol Manual (frame F4 F3 F2 F1, target state 0–3).
 */
#ifndef LD2410_H
#define LD2410_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Target state from radar (Table 12 / 13 in User Manual). */
#define LD2410_STATE_NONE        0x00
#define LD2410_STATE_MOVING      0x01
#define LD2410_STATE_STATIONARY  0x02
#define LD2410_STATE_MOVING_AND_STATIONARY 0x03

/** Last reported target info (updated by UART task). */
typedef struct {
	uint8_t  state;           /**< LD2410_STATE_* */
	uint16_t moving_dist_cm;   /**< Movement target distance (cm). */
	uint8_t  moving_energy;   /**< Movement target energy 0–100. */
	uint16_t stationary_dist_cm;
	uint8_t  stationary_energy;
	uint16_t detection_dist_cm;
	bool     has_data;        /**< At least one valid frame received. */
} ld2410_report_t;

/**
 * Initialize LD2410 UART and start receive task.
 * Default baud for Seeed 24GHz mmWave for XIAO is 256000.
 * \return true on success.
 */
bool ld2410_init(int uart_num, int tx_gpio, int rx_gpio, uint32_t baud);

/** Stop UART and task. */
void ld2410_deinit(void);

/**
 * Return current motion level for sensor_packet_t.motion:
 * 0 = no target, 1 = any presence (moving and/or stationary).
 */
uint8_t ld2410_get_motion_level(void);

/**
 * Copy last report (state, distances, energies). Safe to call from any task.
 */
void ld2410_get_report(ld2410_report_t *out);

#ifdef __cplusplus
}
#endif

#endif /* LD2410_H */
