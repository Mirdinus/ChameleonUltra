#ifndef LF_HIDPROX_DATA_H
#define LF_HIDPROX_DATA_H

#include <stdint.h>

// Initialize hardware for HID Prox reader
void init_hid_prox_hw(void);

/**
* Read the HID ProxCard II tag within the specified timeout
* @param uid: Output buffer for card ID (5 bytes)
* @param timeout_ms: Maximum time to wait for a card
* @return 1 if successful, 0 if timeout
*/
uint8_t hid_prox_read(uint8_t *uid, uint32_t timeout_ms);

/**
* Format and print HID ProxCard II information
* @param uid: Card data (5 bytes)
*/
void hid_prox_print_info(uint8_t *uid);

/**
* Encode HID ProxCard II 35-bit format
* @param: pData card number - ID format: [FC-high, FC-low|CN-high, CN-mid, CN-low, 0]
* @param: pOut Output buffer, fixed 8-length byte
*/
void hid_prox_encoder(uint8_t *pData, uint8_t *pOut);
#endif /* LF_HIDPROX_DATA_H */