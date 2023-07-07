#include "channel.h"

#include <stdint.h>

#include "IEEE802154E.h"
#include "tuning.h"

// TX tuning codes.
static tuning_code_t g_channel_tx_tuning_codes[NUM_CHANNELS];

// RX tuning codes.
static tuning_code_t g_channel_rx_tuning_codes[NUM_CHANNELS];

// Set the tuning code for the given channel.
static inline void channel_set_tuning_code_for_channel(
    const uint8_t channel, tuning_code_t* tuning_codes,
    const tuning_code_t* tuning_code) {
    tuning_codes[channel - MIN_CHANNEL] = *tuning_code;
}

// Get the tuning code for the given channel.
static inline void channel_get_tuning_code_for_channel(
    const uint8_t channel, const tuning_code_t* tuning_codes,
    tuning_code_t* tuning_code) {
    *tuning_code = tuning_codes[channel - MIN_CHANNEL];
}

void channel_set_tuning_code(const uint8_t channel, const channel_mode_e mode,
                             const tuning_code_t* tuning_code) {
    switch (mode) {
        case CHANNEL_MODE_TX: {
            channel_set_tuning_code_for_channel(
                channel, g_channel_tx_tuning_codes, tuning_code);
            break;
        }
        case CHANNEL_MODE_RX: {
            channel_set_tuning_code_for_channel(
                channel, g_channel_rx_tuning_codes, tuning_code);
            break;
        }
        default: {
            break;
        }
    }
}

void channel_get_tuning_code(const uint8_t channel, const channel_mode_e mode,
                             tuning_code_t* tuning_code) {
    switch (mode) {
        case CHANNEL_MODE_TX: {
            channel_get_tuning_code_for_channel(
                channel, g_channel_tx_tuning_codes, tuning_code);
            break;
        }
        case CHANNEL_MODE_RX: {
            channel_get_tuning_code_for_channel(
                channel, g_channel_rx_tuning_codes, tuning_code);
            break;
        }
        default: {
            break;
        }
    }
}
