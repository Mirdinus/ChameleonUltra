#include <stdint.h>

#include "lf_tag_hidprox.h"
#include "syssleep.h"
#include "data_utils.h"
#include "tag_emulation.h"
#include "fds_util.h"
#include "tag_persistence.h"
#include "bsp_delay.h"
#include "parity.h"

#include "nrf_gpio.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_lpcomp.h"

#define NRF_LOG_MODULE_NAME tag_hidprox
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();


// Get the specified position bit
#define GETBIT(v, bit) ((v >> bit) & 0x01)

// Antenna control
#define ANT_TO_MOD()   nrf_gpio_pin_set(LF_MOD)
#define ANT_NO_MOD()   nrf_gpio_pin_clear(LF_MOD)

// Whether the USB light effect is allowed to enable
extern bool g_usb_led_marquee_enable;

// The bit position of the card ID currently sent
static uint8_t m_bit_send_position;
// Whether to send the first edge
static bool m_is_send_first_edge;
// The current broadcast ID number is 33ms every few times
static uint8_t m_send_id_count;
// Whether it is currently in the low-frequency card number of broadcasting
static volatile bool m_is_lf_emulating = false;
// The timer of the delivery card number, we use the timer 3
const nrfx_timer_t m_hidprox_timer_send_id = NRFX_TIMER_INSTANCE(3);
// Cache label type
static tag_specific_type_t m_tag_type = TAG_TYPE_UNDEFINED;

// Card data buffers
static uint64_t m_id_bit_data = 0;
static uint8_t m_hidprox_data[LF_HIDPROX_TAG_DATA_SIZE] = {0};

/**
 * @brief Convert HID Corporate 1000 35-bit data to a Manchester encoded bit stream
 * 
 * Format: PPPPPPPPCCCCCCCCCCCCSSSSSSSSSSSSSSSSSSSSP
 * P = parity bits
 * C = 12-bit company code (facility code)
 * S = 19-bit card number
 */
uint64_t hidprox_id_to_memory64(uint8_t *id_data) {
    uint16_t company_code = ((uint16_t)id_data[0] << 4) | ((id_data[1] & 0xF0) >> 4);
    uint32_t card_number = ((uint32_t)(id_data[1] & 0x0F) << 16) | ((uint32_t)id_data[2] << 8) | id_data[3];
    
    // Union, what you see is obtained
    union {
        uint64_t u64;
        struct {
            // 2-bit header parity (even)
            uint8_t p0: 1;
            uint8_t p1: 1;
            
            // 12-bit company code
            uint8_t c00: 1;
            uint8_t c01: 1;
            uint8_t c02: 1;
            uint8_t c03: 1;
            uint8_t c04: 1;
            uint8_t c05: 1;
            uint8_t c06: 1;
            uint8_t c07: 1;
            uint8_t c08: 1;
            uint8_t c09: 1;
            uint8_t c10: 1;
            uint8_t c11: 1;
            
            // 19-bit card number
            uint8_t s00: 1;
            uint8_t s01: 1;
            uint8_t s02: 1;
            uint8_t s03: 1;
            uint8_t s04: 1;
            uint8_t s05: 1;
            uint8_t s06: 1;
            uint8_t s07: 1;
            uint8_t s08: 1;
            uint8_t s09: 1;
            uint8_t s10: 1;
            uint8_t s11: 1;
            uint8_t s12: 1;
            uint8_t s13: 1;
            uint8_t s14: 1;
            uint8_t s15: 1;
            uint8_t s16: 1;
            uint8_t s17: 1;
            uint8_t s18: 1;
            
            // 2-bit trailing parity (odd)
            uint8_t p2: 1;
            uint8_t p3: 1;
        } bit;
    } memory;
    
    // First assign all bits
    memset(&memory, 0, sizeof(memory));
    
    // Header parity bits - calculate later
    memory.bit.p0 = 0;
    memory.bit.p1 = 0;
    
    // Assign company code - 12 bits
    memory.bit.c00 = (company_code >> 11) & 0x01;
    memory.bit.c01 = (company_code >> 10) & 0x01;
    memory.bit.c02 = (company_code >> 9) & 0x01;
    memory.bit.c03 = (company_code >> 8) & 0x01;
    memory.bit.c04 = (company_code >> 7) & 0x01;
    memory.bit.c05 = (company_code >> 6) & 0x01;
    memory.bit.c06 = (company_code >> 5) & 0x01;
    memory.bit.c07 = (company_code >> 4) & 0x01;
    memory.bit.c08 = (company_code >> 3) & 0x01;
    memory.bit.c09 = (company_code >> 2) & 0x01;
    memory.bit.c10 = (company_code >> 1) & 0x01;
    memory.bit.c11 = company_code & 0x01;
    
    // Assign card number - 19 bits
    memory.bit.s00 = (card_number >> 18) & 0x01;
    memory.bit.s01 = (card_number >> 17) & 0x01;
    memory.bit.s02 = (card_number >> 16) & 0x01;
    memory.bit.s03 = (card_number >> 15) & 0x01;
    memory.bit.s04 = (card_number >> 14) & 0x01;
    memory.bit.s05 = (card_number >> 13) & 0x01;
    memory.bit.s06 = (card_number >> 12) & 0x01;
    memory.bit.s07 = (card_number >> 11) & 0x01;
    memory.bit.s08 = (card_number >> 10) & 0x01;
    memory.bit.s09 = (card_number >> 9) & 0x01;
    memory.bit.s10 = (card_number >> 8) & 0x01;
    memory.bit.s11 = (card_number >> 7) & 0x01;
    memory.bit.s12 = (card_number >> 6) & 0x01;
    memory.bit.s13 = (card_number >> 5) & 0x01;
    memory.bit.s14 = (card_number >> 4) & 0x01;
    memory.bit.s15 = (card_number >> 3) & 0x01;
    memory.bit.s16 = (card_number >> 2) & 0x01;
    memory.bit.s17 = (card_number >> 1) & 0x01;
    memory.bit.s18 = card_number & 0x01;
    
    // Trailing parity bits - calculate later
    memory.bit.p2 = 0;
    memory.bit.p3 = 0;
    
    // Calculate even parity for first half (p0)
    uint8_t p0 = 0;
    p0 ^= memory.bit.c00 ^ memory.bit.c01 ^ memory.bit.c02 ^ memory.bit.c03;
    p0 ^= memory.bit.c04 ^ memory.bit.c05 ^ memory.bit.c06 ^ memory.bit.c07;
    p0 ^= memory.bit.c08 ^ memory.bit.c09 ^ memory.bit.c10 ^ memory.bit.c11;
    p0 ^= memory.bit.s00 ^ memory.bit.s01 ^ memory.bit.s02 ^ memory.bit.s03;
    p0 ^= memory.bit.s04 ^ memory.bit.s05 ^ memory.bit.s06 ^ memory.bit.s07;
    p0 ^= memory.bit.s08;
    memory.bit.p0 = p0 ? 0 : 1; // Even parity (make sure we have even number of 1s)
    
    // Calculate even parity for second half (p1)
    uint8_t p1 = 0;
    p1 ^= memory.bit.s09 ^ memory.bit.s10 ^ memory.bit.s11 ^ memory.bit.s12;
    p1 ^= memory.bit.s13 ^ memory.bit.s14 ^ memory.bit.s15 ^ memory.bit.s16;
    p1 ^= memory.bit.s17 ^ memory.bit.s18;
    memory.bit.p1 = p1 ? 0 : 1; // Even parity
    
    // Calculate odd parity for columns (p2, p3)
    uint8_t col_parity[2] = {0, 0};
    
    // First column parity (p2) - columns 0, 1, 2
    col_parity[0] ^= memory.bit.p0;
    col_parity[0] ^= memory.bit.c00 ^ memory.bit.c04 ^ memory.bit.c08;
    col_parity[0] ^= memory.bit.s00 ^ memory.bit.s04 ^ memory.bit.s08 ^ memory.bit.s12 ^ memory.bit.s16;
    memory.bit.p2 = col_parity[0] ? 1 : 0; // Odd parity (make sure we have odd number of 1s)
    
    // Second column parity (p3) - columns 3, 4, 5
    col_parity[1] ^= memory.bit.p1;
    col_parity[1] ^= memory.bit.c01 ^ memory.bit.c05 ^ memory.bit.c09;
    col_parity[1] ^= memory.bit.s01 ^ memory.bit.s05 ^ memory.bit.s09 ^ memory.bit.s13 ^ memory.bit.s17;
    memory.bit.p3 = col_parity[1] ? 1 : 0; // Odd parity
    
    // Return the binary encoding
    return memory.u64;
}

/**
 * @brief Judgment field status
 */
bool hidprox_is_field_exists(void) {
    nrf_drv_lpcomp_enable();
    bsp_delay_us(30);                                   // Display for a period of time and sampling to avoid misjudgment
    nrf_lpcomp_task_trigger(NRF_LPCOMP_TASK_SAMPLE);    //Trigger a sampling
    return nrf_lpcomp_result_get() == 1;                //Determine the sampling results of the LF field status
}

/**
 * @brief Timer handler for sending bit patterns
 */
void hidprox_timer_ce_handler(nrf_timer_event_t event_type, void *p_context) {
    bool mod;
    switch (event_type) {
        // Because we are configured using the CC channel 2, the event recovers
        // Detect nrf_timer_event_compare0 event in the function
        case NRF_TIMER_EVENT_COMPARE2: {
            if (m_is_send_first_edge) {
                if (GETBIT(m_id_bit_data, m_bit_send_position)) {
                    // The first edge of the send 1
                    ANT_TO_MOD();
                    mod = true;
                } else {
                    // The first edge of the send 0
                    ANT_NO_MOD();
                    mod = false;
                }
                m_is_send_first_edge = false;   //The second edge is sent next time
            } else {
                if (GETBIT(m_id_bit_data, m_bit_send_position)) {
                    // Send the second edge of 1
                    ANT_NO_MOD();
                    mod = false;
                } else {
                    //The second edge of the send 0
                    ANT_TO_MOD();
                    mod = true;
                }
                m_is_send_first_edge = true;    //The first edge of the next sends next time
            }

            // measure field only during no-mod half of last bit of last broadcast
            if ((! mod) &&
                    (m_bit_send_position + 1 >= LF_125KHZ_HID_BIT_SIZE) &&
                    (m_send_id_count + 1 >= LF_125KHZ_BROADCAST_MAX)) {
                nrfx_timer_disable(&m_hidprox_timer_send_id);                       // Close the timer of the broadcast venue
                // We don't need any events, but only need to detect the state of the field
                NRF_LPCOMP->INTENCLR = LPCOMP_INTENCLR_CROSS_Msk | LPCOMP_INTENCLR_UP_Msk | LPCOMP_INTENCLR_DOWN_Msk | LPCOMP_INTENCLR_READY_Msk;
                if (hidprox_is_field_exists()) {
                    nrf_drv_lpcomp_disable();
                    nrfx_timer_enable(&m_hidprox_timer_send_id);                    // Open the timer of the broadcaster and continue to simulate
                } else {
                    // Open the incident interruption, so that the next event can be in and out normally
                    g_is_tag_emulating = false;                             // Reset the flag in the simulation
                    m_is_lf_emulating = false;
                    TAG_FIELD_LED_OFF()                                     // Make sure the indicator light of the LF field status
                    NRF_LPCOMP->INTENSET = LPCOMP_INTENCLR_CROSS_Msk | LPCOMP_INTENCLR_UP_Msk | LPCOMP_INTENCLR_DOWN_Msk | LPCOMP_INTENCLR_READY_Msk;
                    // call sleep_timer_start *after* unsetting g_is_tag_emulating
                    sleep_timer_start(SLEEP_DELAY_MS_FIELD_125KHZ_LOST);    // Start the timer to enter the sleep
                    NRF_LOG_INFO("LF FIELD LOST");
                }
            }

            if (m_is_send_first_edge == true) { // The first edge of the next sends next time
                if (++m_bit_send_position >= LF_125KHZ_HID_BIT_SIZE) {
                    m_bit_send_position = 0;    // The broadcast is successful once, and the BIT position is zero
                    if(!hidprox_is_field_exists()){  // To avoid stopping sending when the reader field is present
                        m_send_id_count++;
                    }
                    if (m_send_id_count >= LF_125KHZ_BROADCAST_MAX) {
                        m_send_id_count = 0;                                        //The number of broadcasts reaches the upper limit, re-identifies the status of the field and re-statistically count the number of broadcast times
                    }
                }
            }
            break;
        }
        default: {
            // Nothing to do.
            break;
        }
    }
}

/**
 * @brief LPCOMP event handler is called when LPCOMP detects voltage drop.
 *
 * This function is called from interrupt context so it is very important
 * to return quickly. Don't put busy loops or any other CPU intensive actions here.
 * It is also not allowed to call soft device functions from it (if LPCOMP IRQ
 * priority is set to APP_IRQ_PRIORITY_HIGH).
 */
static void lpcomp_event_handler(nrf_lpcomp_event_t event) {
    // Only when the low-frequency simulation is not launched, and the analog card is started
    if (!m_is_lf_emulating && event == NRF_LPCOMP_EVENT_UP) {
        // Turn off dormant delay
        sleep_timer_stop();
        // Close the comparator
        nrf_drv_lpcomp_disable();

        // Set the simulation status logo bit
        m_is_lf_emulating = true;
        g_is_tag_emulating = true;

        // Simulation card status should be turned off the USB light effect
        g_usb_led_marquee_enable = false;

        // LED status update
        set_slot_light_color(RGB_BLUE);
        TAG_FIELD_LED_ON()

        //In any case, every time the state finds changes, you need to reset the BIT location of the sending
        m_send_id_count = 0;
        m_bit_send_position = 0;
        m_is_send_first_edge = true;

        // openThePreciseHardwareTimerToTheBroadcastCardNumber
        nrfx_timer_enable(&m_hidprox_timer_send_id);

        NRF_LOG_INFO("LF FIELD DETECTED");
    }
}

static void lf_hidprox_sense_enable(void) {
    ret_code_t err_code;

    nrf_drv_lpcomp_config_t config = NRF_DRV_LPCOMP_DEFAULT_CONFIG;
    config.hal.reference = NRF_LPCOMP_REF_SUPPLY_1_16;
    config.input = LF_RSSI;
    config.hal.detection = NRF_LPCOMP_DETECT_UP;
    config.hal.hyst = NRF_LPCOMP_HYST_50mV;

    err_code = nrf_drv_lpcomp_init(&config, lpcomp_event_handler);
    APP_ERROR_CHECK(err_code);

    // TAG id broadcast
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    err_code = nrfx_timer_init(&m_hidprox_timer_send_id, &timer_cfg, hidprox_timer_ce_handler);
    APP_ERROR_CHECK(err_code);
    nrfx_timer_extended_compare(&m_hidprox_timer_send_id, NRF_TIMER_CC_CHANNEL2, nrfx_timer_us_to_ticks(&m_hidprox_timer_send_id, LF_125KHZ_HID_BIT_CLOCK), NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK, true);

    if (hidprox_is_field_exists() && !m_is_lf_emulating) {
        lpcomp_event_handler(NRF_LPCOMP_EVENT_UP);
    }
}

static void lf_hidprox_sense_disable(void) {
    nrfx_timer_uninit(&m_hidprox_timer_send_id);    //counterInitializationTimer
    nrfx_lpcomp_uninit();                   //antiInitializationComparator
    m_is_lf_emulating = false;              //setAsNonSimulatedState
}

static enum  {
    LF_SENSE_STATE_NONE,
    LF_SENSE_STATE_DISABLE,
    LF_SENSE_STATE_ENABLE,
} m_lf_sense_state = LF_SENSE_STATE_NONE;

/**
 * @brief switchLfFieldInductionToEnableTheState
 */
void lf_tag_hidprox_sense_switch(bool enable) {
    // initializationModulationFootIsOutput
    nrf_gpio_cfg_output(LF_MOD);
    //theDefaultIsNotShortCircuitAntenna (shortCircuitWillCauseRssiToBeUnableToJudge)
    ANT_NO_MOD();

    //forTheFirstTimeOrDisabled,OnlyInitializationIsAllowed
    if (m_lf_sense_state == LF_SENSE_STATE_NONE || m_lf_sense_state == LF_SENSE_STATE_DISABLE) {
        if (enable) {
            m_lf_sense_state = LF_SENSE_STATE_ENABLE;
            lf_hidprox_sense_enable();
        }
    } else {    // inOtherCases,OnlyAntiInitializationIsAllowed
        if (!enable) {
            m_lf_sense_state = LF_SENSE_STATE_DISABLE;
            lf_hidprox_sense_disable();
        }
    }
}

/** @brief HID Prox load data
 * @param type     Refined label type
 * @param buffer   Data buffer
 */
int lf_tag_hidprox_data_loadcb(tag_specific_type_t type, tag_data_buffer_t *buffer) {
    //Make sure that external capacity is enough to convert to an information structure
    if (buffer->length >= LF_HIDPROX_TAG_DATA_SIZE) {
        // The ID card number is directly converted here as the corresponding BIT data stream
        m_tag_type = type;
        memcpy(m_hidprox_data, buffer->buffer, LF_HIDPROX_TAG_DATA_SIZE);
        m_id_bit_data = hidprox_id_to_memory64(m_hidprox_data);
        NRF_LOG_INFO("LF HID Prox data load finish.");
    } else {
        NRF_LOG_ERROR("LF_HIDPROX_TAG_DATA_SIZE too big.");
    }
    return LF_HIDPROX_TAG_DATA_SIZE;
}

/** @brief Id card deposit card number before callback
 * @param type      Refined label type
 * @param buffer    Data buffer
 * @return The length of the data that needs to be saved is that it does not save when 0
 */
int lf_tag_hidprox_data_savecb(tag_specific_type_t type, tag_data_buffer_t *buffer) {
    // Make sure to load this label before allowing saving
    if (m_tag_type != TAG_TYPE_UNDEFINED) {
        // Just save the original card package directly
        memcpy(buffer->buffer, m_hidprox_data, LF_HIDPROX_TAG_DATA_SIZE);
        return LF_HIDPROX_TAG_DATA_SIZE;
    } else {
        return 0;
    }
}

/** @brief Id card deposit card number before callback
 * @param slot     Card slot number
 * @param tag_type  Refined label type
 * @return Whether the format is successful, if the formatting is successful, it will return to True, otherwise False will be returned
 */
bool lf_tag_hidprox_data_factory(uint8_t slot, tag_specific_type_t tag_type) {
    // Default HID Corporate 1000 card - Company Code 123, Card ID 45678
    uint8_t tag_data[LF_HIDPROX_TAG_DATA_SIZE] = {0x07, 0xB0, 0xB2, 0x6E};
    
    // Write the data in Flash
    tag_sense_type_t sense_type = get_sense_type_from_tag_type(tag_type);
    fds_slot_record_map_t map_info; // Get the special card slot FDS record information
    get_fds_map_by_slot_sense_type_for_dump(slot, sense_type, &map_info);
    //Call the blocked FDS to write the function, and write the data of the specified field type of the card slot into the Flash
    bool ret = fds_write_sync(map_info.id, map_info.key, sizeof(tag_data), (uint8_t *)tag_data);
    if (ret) {
        NRF_LOG_INFO("Factory slot data success.");
    } else {
        NRF_LOG_ERROR("Factory slot data error.");
    }
    return ret;
}