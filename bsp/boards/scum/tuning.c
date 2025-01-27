#include "tuning.h"

#include <stdint.h>

#include "opendefs.h"
#include "scm3c_hw_interface.h"

// Number of mid codes between neighboring channels.
// This was empirically determined to be around 5-6 mid codes.
#define TUNING_NUM_MID_CODES_BETWEEN_CHANNELS 6

// Number of fine codes per mid code transition.
// This was empirically determined.
#define TUNING_NUM_FINE_CODES_PER_MID_CODE_TRANSITION 9

// Number of mid codes per coarse code transition.
// This was empirically determined.
#define TUNING_NUM_MID_CODES_PER_COARSE_CODE_TRANSITION 13

// Number of mid codes between TX and RX tuning codes (RX - TX).
// This was empirically determined.
#define TUNING_NUM_MID_CODES_BETWEEN_RX_AND_TX 1

void tuning_increment_fine_codes(tuning_code_t* tuning_code,
                                 const uint8_t num_fine_codes) {
    if (tuning_code->fine + num_fine_codes > TUNING_MAX_CODE) {
        tuning_code->fine += TUNING_NUM_FINE_CODES_PER_MID_CODE_TRANSITION +
                             num_fine_codes - TUNING_MAX_CODE - 1;
        tuning_increment_mid_codes(tuning_code, /*num_fine_codes=*/1);
    } else {
        tuning_code->fine += num_fine_codes;
    }
}

void tuning_decrement_fine_codes(tuning_code_t* tuning_code,
                                 const uint8_t num_fine_codes) {
    if (tuning_code->fine < TUNING_MIN_CODE + num_fine_codes) {
        tuning_code->fine += TUNING_MAX_CODE + 1 -
                             TUNING_NUM_FINE_CODES_PER_MID_CODE_TRANSITION -
                             num_fine_codes;
        tuning_decrement_mid_codes(tuning_code, /*num_mid_codes=*/1);
    } else {
        tuning_code->fine -= num_fine_codes;
    }
}

void tuning_increment_mid_codes(tuning_code_t* tuning_code,
                                const uint8_t num_mid_codes) {
    if (tuning_code->mid + num_mid_codes > TUNING_MAX_CODE) {
        tuning_code->mid += TUNING_NUM_MID_CODES_PER_COARSE_CODE_TRANSITION +
                            num_mid_codes - TUNING_MAX_CODE - 1;
        ++tuning_code->coarse;
    } else {
        tuning_code->mid += num_mid_codes;
    }
}

void tuning_decrement_mid_codes(tuning_code_t* tuning_code,
                                const uint8_t num_mid_codes) {
    if (tuning_code->mid < TUNING_MIN_CODE + num_mid_codes) {
        tuning_code->mid += TUNING_MAX_CODE + 1 -
                            TUNING_NUM_MID_CODES_PER_COARSE_CODE_TRANSITION -
                            num_mid_codes;
        --tuning_code->coarse;
    } else {
        tuning_code->mid -= num_mid_codes;
    }
}

void tuning_rollover_mid_code(tuning_code_t* tuning_code,
                              const uint8_t mid_code_threshold) {
    if (tuning_code->mid < TUNING_MIN_CODE + mid_code_threshold) {
        tuning_code->mid += TUNING_NUM_MID_CODES_PER_COARSE_CODE_TRANSITION;
        --tuning_code->coarse;
    }
    if (tuning_code->mid + mid_code_threshold > TUNING_MAX_CODE) {
        tuning_code->mid -= TUNING_NUM_MID_CODES_PER_COARSE_CODE_TRANSITION;
        ++tuning_code->coarse;
    }
}

void tuning_estimate_previous_channel(tuning_code_t* tuning_code) {
    tuning_decrement_mid_codes(tuning_code,
                               TUNING_NUM_MID_CODES_BETWEEN_CHANNELS);
}

void tuning_estimate_next_channel(tuning_code_t* tuning_code) {
    tuning_increment_mid_codes(tuning_code,
                               TUNING_NUM_MID_CODES_BETWEEN_CHANNELS);
}

void tuning_estimate_tx_from_rx(tuning_code_t* tuning_code) {
    tuning_decrement_mid_codes(tuning_code,
                               TUNING_NUM_MID_CODES_BETWEEN_RX_AND_TX);
}

void tuning_estimate_rx_from_tx(tuning_code_t* tuning_code) {
    tuning_increment_mid_codes(tuning_code,
                               TUNING_NUM_MID_CODES_BETWEEN_RX_AND_TX);
}

void tuning_init_for_sweep(tuning_code_t* tuning_code,
                           const tuning_sweep_config_t* sweep_config) {
    tuning_code->coarse = sweep_config->coarse.start;
    // Start from the middle of the mid code range if the coarse code is
    // constant.
    if (sweep_config->coarse.start == sweep_config->coarse.end) {
        tuning_code->mid =
            (sweep_config->mid.start + sweep_config->mid.end) / 2;
    } else {
        tuning_code->mid = sweep_config->mid.start;
    }
    tuning_code->fine = sweep_config->fine.start;
}

bool tuning_validate_sweep_config(const tuning_sweep_config_t* sweep_config) {
    if (sweep_config->coarse.start > TUNING_MAX_CODE ||
        sweep_config->coarse.end > TUNING_MAX_CODE) {
        return FALSE;
    }
    if (sweep_config->mid.start > TUNING_MAX_CODE ||
        sweep_config->mid.end > TUNING_MAX_CODE) {
        return FALSE;
    }
    if (sweep_config->fine.start > TUNING_MAX_CODE ||
        sweep_config->fine.end > TUNING_MAX_CODE) {
        return FALSE;
    }
    if (sweep_config->coarse.start > sweep_config->coarse.end ||
        sweep_config->mid.start > sweep_config->mid.end ||
        sweep_config->fine.start > sweep_config->fine.end) {
        return FALSE;
    }
    return TRUE;
}

void tuning_increment_fine_code_for_sweep(
    tuning_code_t* tuning_code, const tuning_sweep_config_t* sweep_config) {
    ++tuning_code->fine;
    if (tuning_code->fine > sweep_config->fine.end) {
        tuning_increment_mid_code_for_sweep(tuning_code, sweep_config);
    }
}

void tuning_increment_mid_code_for_sweep(
    tuning_code_t* tuning_code, const tuning_sweep_config_t* sweep_config) {
    tuning_code->fine = sweep_config->fine.start;
    if (sweep_config->coarse.start == sweep_config->coarse.end) {
        uint8_t initial_mid_code =
            (sweep_config->mid.start + sweep_config->mid.end) / 2;
        int16_t mid_code_difference =
            (int16_t)tuning_code->mid - (int16_t)initial_mid_code;
        tuning_code->mid = (mid_code_difference <= 0)
                               ? (initial_mid_code + 1 - mid_code_difference)
                               : (initial_mid_code - mid_code_difference);
        if (tuning_code->mid < sweep_config->mid.start ||
            tuning_code->mid > sweep_config->mid.end) {
            tuning_code->mid = initial_mid_code;
        }
        tuning_code->coarse = sweep_config->coarse.start;
    } else {
        ++tuning_code->mid;
        if (tuning_code->mid > sweep_config->mid.end) {
            tuning_code->mid = sweep_config->mid.start;
            ++tuning_code->coarse;
            if (tuning_code->coarse > sweep_config->coarse.end) {
                tuning_code->coarse = sweep_config->coarse.start;
            }
        }
    }
}

bool tuning_end_of_sweep(const tuning_code_t* tuning_code,
                         const tuning_sweep_config_t* sweep_config) {
    return tuning_code->coarse > sweep_config->coarse.end ||
           (tuning_code->coarse == sweep_config->coarse.end &&
            (tuning_code->mid > sweep_config->mid.end ||
             (tuning_code->mid == sweep_config->mid.end &&
              tuning_code->fine >= sweep_config->fine.end)));
}

void tuning_tune_radio(const tuning_code_t* tuning_code) {
    LC_FREQCHANGE(tuning_code->coarse, tuning_code->mid, tuning_code->fine);
}
