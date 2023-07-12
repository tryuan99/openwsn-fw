// The tuning feedback module determines how the tuning codes should be adjusted
// to maintain the correct channel frequency. Tuning feedback should only happen
// after channel calibration has completed.

#ifndef __TUNING_FEEDBACK_H
#define __TUNING_FEEDBACK_H

#include <stdint.h>

// Adjust the RX tuning codes for a channel.
void tuning_feedback_adjust_rx(uint8_t channel, uint32_t if_estimate);

#endif  // __TUNING_FEEDBACK_H
