// The channel calibration module handles channel calibration for the RX and TX
// tuning codes. Currently, channel calibration only a single 802.15.4 channel
// is supported.

#ifndef __CHANNEL_CAL_H
#define __CHANNEL_CAL_H

#include "config.h"
#include "opendefs.h"
#include "tuning.h"

#define CHANNEL_CAL_ENABLED

// If true, run channel calibration to find all channels.
#define CHANNEL_CAL_ALL_CHANNELS_ENABLED (IEEE802154E_SINGLE_CHANNEL == 0)

// Initial channel to calibrate.
#define CHANNEL_CAL_INITIAL_CHANNEL 17

// Initialize the channel calibration for the initial RX sweep. The sweep range
// can be reduced if not all 802.15.4 channels need to be found.
bool channel_cal_init_initial_rx_sweep(void);

// Initialize the channel calibration for the remaining sweeps using the result
// of the initial RX sweep. This function should be called after the initial RX
// sweep has finished. The sweep range can be reduced if not all 802.15.4
// channels needs to be found. The RX sweep configuration of the initial channel
// is re-initialized.
bool channel_cal_init_remaining_sweeps(void);

// Start the channel calibration's initial RX sweep. This step sweeps the tuning
// codes while listening for enhanced beacons. This function should be called
// when the mote loses frequency calibration.
void channel_cal_start_initial_rx_sweep(void);

// End the channel calibration's initial RX sweep.
// This function should be called when the mote receives an enhanced beacon on
// the initial channel. After this function has been called, the remaining
// sweeps may be initialized.
void channel_cal_end_initial_rx_sweep(void);

// Return whether the initial RX sweep has finished.
bool channel_cal_initial_rx_calibrated(void);

// Get the RX tuning code for the channel, which may or may not be calibrated.
void channel_cal_rx_get_tuning_code(uint8_t channel,
                                    tuning_code_t* tuning_code);

// Return whether the given RX channel has been calibrated.
bool channel_cal_rx_calibrated(uint8_t channel);

// Handle a failed RX on the given channel.
void channel_cal_rx_failure(uint8_t channel);

// Handle a successful RX on the given channel.
void channel_cal_rx_success(uint8_t channel);

// Get the TX tuning code for the channel, which may or may not be calibrated.
void channel_cal_tx_get_tuning_code(uint8_t channel,
                                    tuning_code_t* tuning_code);

// Return whether the given TX channel has been calibrated.
bool channel_cal_tx_calibrated(uint8_t channel);

// Handle a failed TX on the given channel.
void channel_cal_tx_failure(uint8_t channel);

// Handle a successful TX on the given channel.
void channel_cal_tx_success(uint8_t channel);

#endif  // __CHANNEL_CAL_H
