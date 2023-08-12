// The channel calibration module handles channel calibration for the RX and TX
// tuning codes. Currently, channel calibration only a single 802.15.4 channel
// is supported.

#ifndef __CHANNEL_CAL_H
#define __CHANNEL_CAL_H

#include "opendefs.h"
#include "tuning.h"

#define CHANNEL_CAL_ENABLED

// Initialize the channel calibration. The sweep range can be reduced if not all
// 802.15.4 channels need to be found.
bool channel_cal_init(void);

// Start the channel calibration for RX tuning codes.
// RX channel calibration sweeps the tuning codes while listening for enhanced
// beacons. This function should be called when the mote loses frequency
// calibration.
bool channel_cal_rx_start(void);

// End the channel calibration for RX tuning codes.
// This function should be called when the mote receives an enhanced beacon on
// the desired channel.
bool channel_cal_rx_end(void);

// Return whether RX channel calibration has finished.
bool channel_cal_rx_calibrated(void);

// Get the RX tuning code for the channel, which may or may not be calibrated.
void channel_cal_rx_get_tuning_code(tuning_code_t* tuning_code);

// Handle a failed TX attempt.
bool channel_cal_tx_handle_failure(void);

// End the channel calibration for TX tuning codes.
// This function should be called when transmitting a packet is successful on
// the desired channel.
bool channel_cal_tx_end(void);

// Return whether TX channel calibration has finished.
bool channel_cal_tx_calibrated(void);

// Get the TX tuning code for the channel, which may or may not be calibrated.
void channel_cal_tx_get_tuning_code(tuning_code_t* tuning_code);

#endif  // __CHANNEL_CAL_H
