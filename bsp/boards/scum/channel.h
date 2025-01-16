// The channel module handles managing the TX and RX frequency tuning codes for
// each 802.15.4 channel.

#ifndef __CHANNEL_H
#define __CHANNEL_H

#include <stdint.h>

#include "tuning.h"

typedef enum {
    CHANNEL_MODE_INVALID = -1,
    CHANNEL_MODE_TX,
    CHANNEL_MODE_RX,
} channel_mode_e;

// Convert the channel to an index.
uint8_t channel_convert_channel_to_index(uint8_t channel);

// Convert the index to a channel.
uint8_t channel_convert_index_to_channel(uint8_t index);

// Set the tuning code for a channel.
void channel_set_tuning_code(uint8_t channel, channel_mode_e mode,
                             const tuning_code_t* tuning_code);

// Get the tuning code for a channel.
void channel_get_tuning_code(uint8_t channel, channel_mode_e mode,
                             tuning_code_t* tuning_code);

#endif  // __CHANNEL_H
