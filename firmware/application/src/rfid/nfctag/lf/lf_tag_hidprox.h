#ifndef __LF_TAG_HIDPROX_H
#define __LF_TAG_HIDPROX_H

#include <stdbool.h>
#include "rfid_main.h"
#include "tag_emulation.h"

/**
 * Low frequency HID Prox card emulation parameters
 */
#define LF_125KHZ_HID_BIT_SIZE      35      // 35-bit format
#define LF_125KHZ_HID_BIT_CLOCK     256     // Bit clock timing for HID modulation
#define LF_HIDPROX_TAG_DATA_SIZE    4       // 4 bytes needed to store company code and card ID

// Function declarations
void lf_tag_hidprox_sense_switch(bool enable);
int lf_tag_hidprox_data_loadcb(tag_specific_type_t type, tag_data_buffer_t *buffer);
int lf_tag_hidprox_data_savecb(tag_specific_type_t type, tag_data_buffer_t *buffer);
bool lf_tag_hidprox_data_factory(uint8_t slot, tag_specific_type_t tag_type);

#endif