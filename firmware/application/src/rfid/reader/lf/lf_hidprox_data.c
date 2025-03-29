#ifdef debug_hid_prox
#include <stdio.h>
#endif

#include "bsp_time.h"
#include "bsp_delay.h"
#include "lf_reader_main.h"
#include "lf_reader_data.h"
#include "lf_125khz_radio.h"
#include "lf_hidprox_data.h"

#define NRF_LOG_MODULE_NAME hid_prox
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

static RAWBUF_TYPE_S carddata;
static volatile uint8_t dataindex = 0;          // Record position in the raw buffer
uint8_t cardHIDbufbyte[5];                         // Processed card data buffer (5 bytes)

#ifdef debug_hid_prox
uint8_t datatest[256] = { 0x00 };
#endif

// Manchester decode HID Prox waveform data
// Returns 1 if successful, 0 if error
static uint8_t manchester_decode(RAWBUF_TYPE_S *Pdata) {
    uint8_t cardindex = 0;  // Tracks bit position in the output buffer
    
    // Find preamble (typically 16-20 bits of alternating 0/1)
    // Preamble serves as synchronization pattern
    int preamble_start = -1;
    int preamble_count = 0;
    uint8_t last_bit = 0xFF; // Invalid initial value to force first comparison to fail
    
    // Look for consistent alternating pattern (0,1,0,1...) for at least 10 bits
    for (int i = Pdata->startbit; i < Pdata->startbit + 50; i++) {
        uint8_t thisbit = readbit(Pdata->rawa, Pdata->rawb, i);
        if (thisbit > 2) return 0; // Invalid timing value
        
        if (thisbit != last_bit && last_bit != 0xFF) {
            preamble_count++;
            if (preamble_count >= 10 && preamble_start == -1) {
                preamble_start = i - preamble_count + 1;
            }
        } else {
            // Reset counter if pattern breaks
            if (last_bit != 0xFF) {
                preamble_count = 0;
            }
        }
        last_bit = thisbit;
    }
    
    if (preamble_start == -1) {
        return 0; // No valid preamble found
    }
    
    // Skip the preamble and start decoding the actual data
    // HID Prox uses Manchester encoding:
    // 01 = 0, 10 = 1, others are errors
    for (int i = preamble_start + preamble_count; i < RAW_BUF_SIZE * 8 - 1; i += 2) {
        uint8_t bit1 = readbit(Pdata->rawa, Pdata->rawb, i);
        uint8_t bit2 = readbit(Pdata->rawa, Pdata->rawb, i+1);
        
        // Manchester decode: 01 = 0, 10 = 1
        if (bit1 == 0 && bit2 == 1) {
            writebit(Pdata->hexbuf, Pdata->hexbuf, cardindex, 0);
            cardindex++;
        } else if (bit1 == 1 && bit2 == 0) {
            writebit(Pdata->hexbuf, Pdata->hexbuf, cardindex, 1);
            cardindex++;
        } else {
            // Handle potential EOF or invalid data
            if (cardindex >= 35) { // We've got enough bits for a 35-bit card
                break;
            }
            return 0; // Invalid Manchester code
        }
        
        if (cardindex >= CARD_BUF_SIZE * 8)
            break;
    }
    
    return (cardindex >= 35) ? 1 : 0; // Success if we got at least 35 bits
}

// Process HID Prox card, verify format and parity bits
static uint8_t hid_prox_decoder(uint8_t *pData, uint8_t size, uint8_t *pOut) {
    if (size != 8) {
        return 0;
    }
    
    // HID 35-bit structure:
    // 2 bit header (always 10)
    // 1 bit even parity for bits 2-14
    // 12 bits for facility code (bits 3-14)
    // 1 bit odd parity for bits 15-26
    // 16 bits for card number (bits 15-30)
    // 1 bit even parity for bits 31-34
    // 2 bit trailer (always 01)
    
    // 1. Check header (10)
    if (!(getbit(pData[0], 0) == 1 && getbit(pData[0], 1) == 0)) {
        return 0;
    }
    
    // 2. Check trailer (01)
    if (!(getbit(pData[4], 3) == 0 && getbit(pData[4], 4) == 1)) {
        return 0;
    }
    
    // 3. Extract and verify even parity for bits 2-14
    uint8_t parity_even_1 = getbit(pData[0], 2);
    uint8_t count_1 = 0;
    for (int i = 3; i <= 14; i++) {
        if (getbit(pData[i/8], i%8)) {
            count_1++;
        }
    }
    if ((count_1 % 2) != parity_even_1) {
        return 0; // Parity check failed
    }
    
    // 4. Extract and verify odd parity for bits 15-26
    uint8_t parity_odd = getbit(pData[1], 7);
    uint8_t count_2 = 0;
    for (int i = 15; i <= 26; i++) {
        if (getbit(pData[i/8], i%8)) {
            count_2++;
        }
    }
    if ((count_2 % 2) == parity_odd) {
        return 0; // Odd parity check failed
    }
    
    // 5. Extract and verify even parity for bits 31-34
    uint8_t parity_even_2 = getbit(pData[3], 7);
    uint8_t count_3 = 0;
    for (int i = 31; i <= 34; i++) {
        if (getbit(pData[i/8], i%8)) {
            count_3++;
        }
    }
    if ((count_3 % 2) != parity_even_2) {
        return 0; // Parity check failed
    }
    
    // 6. Extract data (facility code and card number)
    
    // Facility code (12 bits, positions 3-14)
    uint16_t facility_code = 0;
    for (int i = 3; i <= 14; i++) {
        facility_code = (facility_code << 1) | getbit(pData[i/8], i%8);
    }
    
    // Card number (16 bits, positions 15-30)
    uint32_t card_number = 0;
    for (int i = 15; i <= 30; i++) {
        card_number = (card_number << 1) | getbit(pData[i/8], i%8);
    }
    
    // Store decoded data in output buffer
    pOut[0] = (facility_code >> 4) & 0xFF;   // Facility code high byte
    pOut[1] = facility_code & 0x0F;          // Facility code low nibble
    pOut[1] |= (card_number >> 12) & 0xF0;   // Card number highest nibble
    pOut[2] = (card_number >> 4) & 0xFF;     // Card number middle byte
    pOut[3] = card_number & 0x0F;            // Card number lowest nibble
    pOut[4] = 0;                             // Reserved/padding
    
    return 1; // Decoding successful
}

/**
* Encode HID ProxCard II 35-bit format
* @param: pData card number - ID format: [FC-high, FC-low|CN-high, CN-mid, CN-low, 0]
* @param: pOut Output buffer, fixed 8-length byte
*/
void hid_prox_encoder(uint8_t *pData, uint8_t *pOut) {
    // Extract facility code (12 bits)
    uint16_t facility_code = ((uint16_t)pData[0] << 4) | (pData[1] >> 4);
    
    // Extract card number (16 bits)
    uint32_t card_number = ((uint32_t)(pData[1] & 0x0F) << 12) | 
                          ((uint32_t)pData[2] << 4) | 
                           (pData[3] & 0x0F);
    
    // Initialize output buffer
    for (int i = 0; i < 8; i++) {
        pOut[i] = 0x00;
    }
    
    // Set header (10)
    writebit(pOut, pOut, 0, 1);
    writebit(pOut, pOut, 1, 0);
    
    // Calculate and set even parity for bits 2-14
    uint8_t count_1 = 0;
    for (int i = 0; i < 12; i++) {
        uint8_t bit = (facility_code >> (11 - i)) & 0x01;
        count_1 += bit;
    }
    writebit(pOut, pOut, 2, (count_1 % 2)); // Even parity
    
    // Write facility code (12 bits, positions 3-14)
    for (int i = 0; i < 12; i++) {
        uint8_t bit = (facility_code >> (11 - i)) & 0x01;
        writebit(pOut, pOut, 3 + i, bit);
    }
    
    // Calculate and set odd parity for bits 15-26
    uint8_t count_2 = 0;
    for (int i = 0; i < 12; i++) {
        uint8_t bit = (card_number >> (15 - i)) & 0x01;
        count_2 += bit;
    }
    writebit(pOut, pOut, 15, !(count_2 % 2)); // Odd parity
    
    // Write card number (16 bits, positions 16-31)
    for (int i = 0; i < 16; i++) {
        uint8_t bit = (card_number >> (15 - i)) & 0x01;
        writebit(pOut, pOut, 16 + i, bit);
    }
    
    // Calculate and set even parity for bits 32-34
    uint8_t count_3 = 0;
    // The last 3 bits of card number (already written)
    for (int i = 31; i >= 29; i--) {
        if (getbit(pOut[i/8], i%8)) {
            count_3++;
        }
    }
    writebit(pOut, pOut, 32, (count_3 % 2)); // Even parity
    
    // Set trailer (01)
    writebit(pOut, pOut, 33, 0);
    writebit(pOut, pOut, 34, 1);
}

// Reading the card function, you need to stop calling, return 0 to continue reading, 1 if card detected
static uint8_t hid_prox_acquire(void) {
    if (dataindex >= RAW_BUF_SIZE * 8) {
#ifdef debug_hid_prox
        {
            for (int i = 0; i < RAW_BUF_SIZE * 8; i++) {
                NRF_LOG_INFO("%d ", readbit(carddata.rawa, carddata.rawb, i));
            }
            NRF_LOG_INFO("///raw data\r\n");
            for (int i = 0; i < RAW_BUF_SIZE * 8; i++) {
                NRF_LOG_INFO("%d ", datatest[i]);
            }
            NRF_LOG_INFO("///time data\r\n");
        }
#endif
        // Look for preamble pattern
        carddata.startbit = 0;
        
        // If you find data to process
        if (carddata.startbit < (RAW_BUF_SIZE * 8) - 100) { // Ensure enough data for 35-bit card plus preamble
            if (manchester_decode(&carddata) == 1) {
                // Card successfully decoded using Manchester decoding
#ifdef debug_hid_prox
                {
                    for (int i = 0; i < CARD_BUF_SIZE; i++) {
                        NRF_LOG_INFO("%02X", carddata.hexbuf[i]);
                    }
                    NRF_LOG_INFO("///card data\r\n");
                }
#endif
                if (hid_prox_decoder(carddata.hexbuf, CARD_BUF_SIZE, cardHIDbufbyte)) {
                    // Card data check passes
#ifdef debug_hid_prox
                    NRF_LOG_INFO("FC: %d, CN: %d", 
                        ((uint16_t)cardHIDbufbyte[0] << 4) | (cardHIDbufbyte[1] >> 4),
                        ((uint32_t)(cardHIDbufbyte[1] & 0x0F) << 12) | 
                        ((uint32_t)cardHIDbufbyte[2] << 4) | 
                        (cardHIDbufbyte[3] & 0x0F));
                    NRF_LOG_INFO("///HID card data\r\n");
#endif
                    dataindex = 0;
                    return 1;
                }
            }
        }
        // Start a new cycle
        dataindex = 0;
    }
    return 0;
}

// GPIO interrupt handler for detecting edges in the waveform
static void GPIO_INT0_callback(void) {
    static uint32_t thistimelen = 0;
    thistimelen = get_lf_counter_value();
    
    // Typical HID Prox timing:
    // Short (Manchester 0): 60-70 cycles
    // Long (Manchester 1): 100-110 cycles
    if (thistimelen > 47) {
        static uint8_t cons_temp = 0;
        if (dataindex < RAW_BUF_SIZE * 8) {
            if (48 <= thistimelen && thistimelen <= 80) {
                cons_temp = 0; // Short pulse
            } else if (81 <= thistimelen && thistimelen <= 112) {
                cons_temp = 1; // Long pulse
            } else {
                cons_temp = 3; // Invalid timing
            }
            writebit(carddata.rawa, carddata.rawb, dataindex, cons_temp);
#ifdef debug_hid_prox
            datatest[dataindex] = thistimelen;
#endif
            dataindex++;
        }
        clear_lf_counter_value();
    }

    uint16_t counter = 0;
    do {
        __NOP();
    } while (counter++ > 1000);
}

// Start the timer and initialize related peripherals
void init_hid_prox_hw(void) {
    // Register card reader IO interrupt handler
    register_rio_callback(GPIO_INT0_callback);
}

/**
* Read the HID ProxCard II tag within the specified timeout
* @param uid: Output buffer for card ID (5 bytes)
* @param timeout_ms: Maximum time to wait for a card
* @return 1 if successful, 0 if timeout
*/
uint8_t hid_prox_read(uint8_t *uid, uint32_t timeout_ms) {
    uint8_t ret = 0;

    init_hid_prox_hw();          // Initialize edge detection
    start_lf_125khz_radio();     // Start 125kHz modulation

    // Reading the card during timeout
    autotimer *p_at = bsp_obtain_timer(0);
    while (NO_TIMEOUT_1MS(p_at, timeout_ms)) {
        // Check for card, exit if found
        if (hid_prox_acquire()) {
            stop_lf_125khz_radio();
            
            // Copy card data to output buffer
            for (int i = 0; i < 5; i++) {
                uid[i] = cardHIDbufbyte[i];
            }
            
            ret = 1;
            break;
        }
    }

    if (ret != 1) {  // If card not found, manually stop reader
        stop_lf_125khz_radio();
    }

    dataindex = 0;  // Reset data index after completion

    bsp_return_timer(p_at);
    p_at = NULL;

    return ret;
}

/**
* Format and print HID ProxCard II information
* @param uid: Card data (5 bytes)
*/
void hid_prox_print_info(uint8_t *uid) {
    // Extract facility code (12 bits)
    uint16_t facility_code = ((uint16_t)uid[0] << 4) | (uid[1] >> 4);
    
    // Extract card number (16 bits)
    uint32_t card_number = ((uint32_t)(uid[1] & 0x0F) << 12) | 
                          ((uint32_t)uid[2] << 4) | 
                           (uid[3] & 0x0F);
    
    NRF_LOG_INFO("HID Prox II 35-bit");
    NRF_LOG_INFO("Facility Code: %d", facility_code);
    NRF_LOG_INFO("Card Number: %d", card_number);
}