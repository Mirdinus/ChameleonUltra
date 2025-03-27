#ifndef LF_HIDPROX_READER_H
#define LF_HIDPROX_READER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "bsp_time.h"
#include "bsp_delay.h"
#include "lf_reader_data.h"
#include "data_utils.h"
#include "lf_125khz_radio.h"
#include "parity.h"

// HID Prox card data structure
typedef struct {
    uint16_t company_code;    // 12-bit company code
    uint32_t card_number;     // 19-bit card number
    uint8_t raw_data[5];      // Raw 35-bit data including parity
} hidprox_card_data_t;

// HID Prox specific buffer size
#define HIDPROX_BUF_SIZE 48   // Maximum record buffer for HID Prox waveform data
#define HID_BIT_SIZE 35       // HID Corporate 1000 uses 35 bits

// Raw buffer for HID waveform data 
typedef struct {
    uint8_t rawa[HIDPROX_BUF_SIZE];    // The time difference between recording changes
    uint8_t rawb[HIDPROX_BUF_SIZE];    // The time difference between recording changes
    uint8_t bits[HID_BIT_SIZE];        // Extracted bits (excluding parity)
    uint8_t startbit;                  // Starting bit position in buffer
} hidprox_raw_buffer_t;

// Function declarations
void register_rio_callback(void (*callback)(void));
uint32_t get_lf_counter_value(void);
void clear_lf_counter_value(void);
void start_lf_125khz_radio(void);
void stop_lf_125khz_radio(void);

// Public API functions
void init_hidprox_hw(void);
uint8_t hidprox_read(hidprox_card_data_t *card_data, uint32_t timeout_ms);

#endif // LF_HIDPROX_READER_H 