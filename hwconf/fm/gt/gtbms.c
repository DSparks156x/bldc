#include "gtbms.h"
#include "ch.h"
#include "hal.h"
#include "comm_can.h"
#include "utils.h"
#include "mc_interface.h"
#include "hw.h"
#include "stm32f4xx_cryp_manual.h"
#include "crc.h"
#include "bms.h"
#include <string.h>

// Settings
#define GTBMS_THREAD_STACK_SIZE 2048
#define BMS_BUF_SIZE 128

// Protocol constants
static const uint8_t bms_magic[] = {0xff, 0x55, 0xbb}; // 0xbb for crypto
static const uint8_t bms_key[16] = {
	94, 45, 98, 103, 223, 146, 201, 182, 
	151, 62, 96, 131, 74, 47, 82, 236
};
static uint8_t bms_counter[16] = {
	0, 0, 237, 37, 247, 65, 211, 7, 
	15, 171, 199, 157, 246, 35, 116, 72
};

// Variables
static volatile bool gtbms_running = false;
static float bms_v_tot = 0.0;
static float bms_soc = 0.0;
static float bms_i_in_ic = 0.0;
static float bms_temp_ic = 0.0;
static float bms_v_cell_min = 0.0;
static float bms_v_cell_max = 0.0;
static float bms_v_cell[24];
static int bms_cell_num = 0;
static uint8_t bms_status_val = 0;
static uint16_t bms_serial = 0;
static uint8_t bms_battery_type = 0;
static uint16_t bms_battery_cycles = 0;
static float bms_soh = 0.0;
static uint8_t bms_charge_state = 0;

static uint8_t bms_rx_buf[BMS_BUF_SIZE];
static THD_WORKING_AREA(gtbms_thread_wa, GTBMS_THREAD_STACK_SIZE);
static THD_FUNCTION(gtbms_thread, arg);

// Helper functions
static uint16_t bms_checksum(const uint8_t *data, int len) {
	uint16_t sum = 0;
	for (int i = 0; i < len; i++) {
		sum += data[i];
	}
	return sum;
}

static void bms_crypt(uint8_t nonce_high, uint8_t nonce_low, uint8_t *data, int len) {
	uint8_t iv[16];
	memcpy(iv, bms_counter, 16);
	iv[0] = nonce_high;
	iv[1] = nonce_low;
	
	cryp_manual_aes_ctr_crypt((uint8_t*)bms_key, iv, data, len);
}

static void bms_set_dere(bool high) {
	if (high) {
		palSetPad(HW_BMS_DERE_PORT, HW_BMS_DERE_PIN);
	} else {
		palClearPad(HW_BMS_DERE_PORT, HW_BMS_DERE_PIN);
	}
}

void gtbms_init(void) {
	if (!gtbms_running) {
		cryp_manual_init();
		chThdCreateStatic(gtbms_thread_wa, sizeof(gtbms_thread_wa),
				NORMALPRIO, gtbms_thread, NULL);
		gtbms_running = true;
	}
}

bool gtbms_is_running(void) {
	return gtbms_running;
}

static void process_packet(uint8_t *data, int len) {
	if (len < 7) return;
	
	// Check magic
	if (data[0] != bms_magic[0] || data[1] != bms_magic[1] || data[2] != bms_magic[2]) return;
	
	// Check checksum
	uint16_t rx_sum = (data[len-2] << 8) | data[len-1];
	if (rx_sum != bms_checksum(data, len - 2)) return;
	
	uint8_t nonce_high = data[3];
	uint8_t nonce_low = data[4];
	
	// Decrypt payload (includes command byte)
	// Important: This modifies the buffer in place
	bms_crypt(nonce_high, nonce_low, &data[5], len - 7 + 1);
	
	uint8_t cmd = data[5];
	uint8_t *payload = &data[6];
	int payload_len = len - 7;
	
	volatile bms_values *val = bms_get_values();

	switch (cmd) {
		case 0x00: // Status
			bms_status_val = payload[0];
			val->is_charging = (bms_status_val & 0x20) != 0;
			// bit 0x02 status bits from lisp: is-battery-temp-out-of-range (0x03), is-battery-empty (0x04), is-battery-overcharged (0x08)
			// we can set a status string if we want, or map to other flags.
			break;
			
		case 0x02: // Cell Voltage
			bms_cell_num = payload_len / 2;
			val->cell_num = bms_cell_num;
			bms_v_tot = 0;
			bms_v_cell_min = 5.0;
			bms_v_cell_max = 0.0;
			for (int i = 0; i < bms_cell_num && i < BMS_MAX_CELLS; i++) {
				bms_v_cell[i] = (float)((payload[i*2] << 8) | payload[i*2+1]) / 10000.0;
				val->v_cell[i] = bms_v_cell[i];
				bms_v_tot += bms_v_cell[i];
				if (bms_v_cell[i] < bms_v_cell_min) bms_v_cell_min = bms_v_cell[i];
				if (bms_v_cell[i] > bms_v_cell_max) bms_v_cell_max = bms_v_cell[i];
			}
			val->v_tot = bms_v_tot;
			val->v_cell_min = bms_v_cell_min;
			val->v_cell_max = bms_v_cell_max;
			break;
			
		case 0x03: // SOC
			bms_soc = (float)payload[0] / 100.0;
			val->soc = bms_soc;
			break;
			
		case 0x04: // Temp
		{
			val->temp_ic = (int8_t)data[len - 4];
			float t_max = -100.0;
			int num_temps = payload_len - 1; // last byte is temp_ic (already handled)
			if (num_temps > BMS_MAX_TEMPS) num_temps = BMS_MAX_TEMPS;
			val->temp_adc_num = num_temps;
			for (int i = 0; i < num_temps; i++) {
				float t = (int8_t)payload[i];
				val->temps_adc[i] = t;
				if (t > t_max) t_max = t;
			}
			val->temp_max_cell = t_max;
		}
			break;
			
		case 0x05: // Current
			bms_i_in_ic = (float)((int16_t)((payload[0] << 8) | payload[1])) * 0.0366;
			val->i_in_ic = bms_i_in_ic;
			break;
			
		case 0x06: // Serial
			bms_serial = (payload[0] << 8) | payload[1];
			break;
			
		case 0x08: // Battery Type
			bms_battery_type = payload[0];
			break;
			
		case 0x0D: // Cycles & Health
			bms_battery_cycles = (payload[0] << 8) | payload[1];
			bms_soh = (float)payload[2] / 100.0;
			val->soh = bms_soh;
			break;
			
		case 0x15: // Charger
			bms_charge_state = payload[0];
			break;
			
		default:
			break;
	}
	
	val->update_time = chVTGetSystemTimeX();
}

static THD_FUNCTION(gtbms_thread, arg) {
	(void)arg;
	chRegSetThreadName("GT BMS Comms");

	for(;;) {
		// Read from UART one byte at a time to find magic
		uint8_t b;
		if (sdReadTimeout(&HW_UART_DEV, &b, 1, MS2ST(10)) == 1) {
			if (b == bms_magic[0]) {
				bms_rx_buf[0] = b;
				if (sdReadTimeout(&HW_UART_DEV, &bms_rx_buf[1], 2, MS2ST(10)) == 2) {
					if (bms_rx_buf[1] == bms_magic[1] && bms_rx_buf[2] == bms_magic[2]) {
						// Found magic, read nonce and cmd (3 bytes) + at least 2 bytes checksum
						// We need to know payload length. For now, read up to a timeout or next magic.
						// The protocol seems to have variable length. 
						// Let's read until we get a pause or enough bytes.
						int idx = 3;
						while (idx < BMS_BUF_SIZE - 2) {
							if (sdReadTimeout(&HW_UART_DEV, &bms_rx_buf[idx], 1, MS2ST(5)) == 1) {
								idx++;
							} else {
								break;
							}
						}
						if (idx > 7) {
							process_packet(bms_rx_buf, idx);
						}
					}
				}
			}
		}
		
		chThdSleepMilliseconds(1);
	}
}

float gtbms_get_voltage(void) { return bms_v_tot; }
float gtbms_get_soc(void) { return bms_soc; }
float gtbms_get_current(void) { return bms_i_in_ic; }
float gtbms_get_temp(void) { return bms_temp_ic; }
float gtbms_get_cell_voltage(int cell) {
	if (cell >= 0 && cell < 24) return bms_v_cell[cell];
	return 0.0;
}
int gtbms_get_cell_num(void) { return bms_cell_num; }
