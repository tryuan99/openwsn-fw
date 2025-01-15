#ifndef __TUNING_H
#define __TUNING_H

#include <stdint.h>

#include "opendefs.h"

// Minimum tuning code.
#define TUNING_MIN_CODE 0

// Maximum tuning code.
#define TUNING_MAX_CODE 31

// Tuning code.
typedef struct __attribute__((packed)) {
    // Coarse code.
    uint8_t coarse;

    // Mid code.
    uint8_t mid;

    // Fine code.
    uint8_t fine;
} tuning_code_t;

// Sweep range.
typedef struct __attribute__((packed)) {
    // Start code of the sweep.
    uint8_t start;

    // End code of the sweep (inclusive).
    uint8_t end;
} tuning_sweep_range_t;

// Sweep configuration.
typedef struct __attribute__((packed)) {
    // Sweep range for the coarse code.
    tuning_sweep_range_t coarse;

    // Sweep range for the mid code.
    tuning_sweep_range_t mid;

    // Sweep range for the fine code.
    tuning_sweep_range_t fine;
} tuning_sweep_config_t;

// Increment the tuning code by some fine codes.
void tuning_increment_fine_codes(tuning_code_t* tuning_code,
                                 uint8_t num_fine_codes);

// Decrement the tuning code by some fine codes.
void tuning_decrement_fine_codes(tuning_code_t* tuning_code,
                                 uint8_t num_fine_codes);

// Increment the tuning code by some mid codes.
void tuning_increment_mid_codes(tuning_code_t* tuning_code,
                                uint8_t num_mid_codes);

// Decrement the tuning code by some mid codes.
void tuning_decrement_mid_codes(tuning_code_t* tuning_code,
                                uint8_t num_mid_codes);

// Rollover the mid code if it is too close to the maximum mid code.
void tuning_rollover_mid_code(tuning_code_t* tuning_code,
                              uint8_t mid_code_threshold);

// Estimate the tuning code for the previous channel.
void tuning_estimate_previous_channel(tuning_code_t* tuning_code);

// Estimate the tuning code for the next channel.
void tuning_estimate_next_channel(tuning_code_t* tuning_code);

// Estimate the TX tuning code from the RX tuning code.
void tuning_estimate_tx_from_rx(tuning_code_t* tuning_code);

// Estimate the RX tuning code from the TX tuning code.
void tuning_estimate_rx_from_tx(tuning_code_t* tuning_code);

// Initialize the tuning code to the minimum value given by the sweep
// configuration.
void tuning_init_for_sweep(tuning_code_t* tuning_code,
                           const tuning_sweep_config_t* sweep_config);

// Validate the sweep configuration.
bool tuning_validate_sweep_config(const tuning_sweep_config_t* sweep_config);

// Increment the tuning code by one fine code, rolling over at the range
// boundaries given by the sweep configuration.
void tuning_increment_fine_code_for_sweep(
    tuning_code_t* tuning_code, const tuning_sweep_config_t* sweep_config);

// Increment the tuning code by one mid code, rolling over at the range
// boundaries given by the sweep configuration.
void tuning_increment_mid_code_for_sweep(
    tuning_code_t* tuning_code, const tuning_sweep_config_t* sweep_config);

// Check whether the tuning code is at the end of the sweep.
bool tuning_end_of_sweep(const tuning_code_t* tuning_code,
                         const tuning_sweep_config_t* sweep_config);

// Tune the radio to the desired tuning code.
void tuning_tune_radio(const tuning_code_t* tuning_code);

#endif  // __TUNING_H
