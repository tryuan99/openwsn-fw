#include "channel_cal.h"

#include <stdint.h>
#include <stdio.h>

#include "IEEE802154E.h"
#include "board_info.h"
#include "channel.h"
#include "memory_map.h"
#include "opendefs.h"
#include "opentimers.h"
#include "radio.h"
#include "schedule.h"
#include "scheduler.h"
#include "tuning.h"

// Number of slotframes to wait for a reception before proceeding to the next
// tuning code. This constant is used for the initial RX sweep.
#define CHANNEL_CAL_RX_NUM_SLOTFRAMES_PER_TUNING_CODE 2

// Number of tics to wait for a transmission before proceeding to the next
// tuning code. This constant is used for the initial RX sweep.
#define CHANNEL_CAL_RX_NUM_TICS_PER_TUNING_CODE                         \
    (CHANNEL_CAL_RX_NUM_SLOTFRAMES_PER_TUNING_CODE * SLOTFRAME_LENGTH * \
     (CHANNEL_CAL_ALL_CHANNELS_ENABLED ? NUM_CHANNELS : 1) * TsSlotDuration)

// Number of failed TX or RX before proceeding to the next tuning code.
#define CHANNEL_CAL_MAX_NUM_FAILURES 2

// Channel calibration state enumeration.
// The state only tracks the progress of calibrating the RX tuning codes. Each
// channel's TX tuning code is calibrated independently after the corresponding
// RX tuning code has been found.
typedef enum {
    CHANNEL_CAL_STATE_INVALID = -1,
    CHANNEL_CAL_INIT,
    CHANNEL_CAL_INITIAL_RX,
    CHANNEL_CAL_REMAINING_RX,
    CHANNEL_CAL_RX_DONE,
} channel_cal_state_e;

// Channel mode information.
typedef struct {
    // RX channel calibration state.
    bool calibrated;

    // tuning code.
    tuning_code_t tuning_code;

    // sweep configuration.
    tuning_sweep_config_t sweep_config;

    // Number of failures for the current tuning code.
    uint8_t num_failures;
} channel_cal_channel_mode_info_t;

// Channel information.
typedef struct {
    // RX information.
    channel_cal_channel_mode_info_t rx;

    // TX information.
    channel_cal_channel_mode_info_t tx;
} channel_cal_channel_info_t;

// Channel information for all channels.
static channel_cal_channel_info_t g_channel_cal_channel_infos[NUM_CHANNELS];

// If true, the initial RX sweep is finished.
static bool g_channel_cal_initial_rx_sweep_finished = FALSE;

// Number of consecutive TX failures.
static uint16_t g_channel_cal_num_tx_failures = 0;

// Number of channels that have finished RX calibration.
static uint8_t g_channel_cal_num_channels_rx_calibrated = 0;

// Number of channels that have finished TX calibration.
static uint8_t g_channel_cal_num_channels_tx_calibrated = 0;

// Channel calibration timer ID.
static opentimers_id_t g_channel_cal_timer_id;

// Print the tuning code over UART.
static inline void channel_cal_print_tuning_code(
    const uint8_t channel, const channel_mode_e channel_mode,
    const tuning_code_t* tuning_code) {
    switch (channel_mode) {
        case CHANNEL_MODE_TX: {
            UART_REG__TX_DATA = 'T';
            break;
        }
        case CHANNEL_MODE_RX: {
            UART_REG__TX_DATA = 'R';
            break;
        }
        default: {
            break;
        }
    }
    UART_REG__TX_DATA = 'X';
    UART_REG__TX_DATA = ' ';
    // Print the channel.
    UART_REG__TX_DATA = '0' + channel / 10;
    UART_REG__TX_DATA = '0' + channel % 10;
    UART_REG__TX_DATA = ' ';
    // Print the tuning code.
    UART_REG__TX_DATA = '0' + tuning_code->coarse / 10;
    UART_REG__TX_DATA = '0' + tuning_code->coarse % 10;
    UART_REG__TX_DATA = '.';
    UART_REG__TX_DATA = '0' + tuning_code->mid / 10;
    UART_REG__TX_DATA = '0' + tuning_code->mid % 10;
    UART_REG__TX_DATA = '.';
    UART_REG__TX_DATA = '0' + tuning_code->fine / 10;
    UART_REG__TX_DATA = '0' + tuning_code->fine % 10;
    UART_REG__TX_DATA = '\n';
}

// Print that channel calibration is finished for a channel.
static inline void channel_cal_print_channel_calibration_finished(
    const uint8_t channel, const channel_mode_e channel_mode) {
    switch (channel_mode) {
        case CHANNEL_MODE_TX: {
            UART_REG__TX_DATA = 'T';
            break;
        }
        case CHANNEL_MODE_RX: {
            UART_REG__TX_DATA = 'R';
            break;
        }
        default: {
            break;
        }
    }
    UART_REG__TX_DATA = 'X';
    UART_REG__TX_DATA = ' ';
    // Print the channel.
    UART_REG__TX_DATA = '0' + channel / 10;
    UART_REG__TX_DATA = '0' + channel % 10;
    UART_REG__TX_DATA = ' ';
    UART_REG__TX_DATA = '*';
    UART_REG__TX_DATA = '\n';
}

// Initialize the channel mode information.
static inline bool channel_cal_init_channel_mode_info(
    channel_cal_channel_mode_info_t* channel_mode_info,
    const tuning_code_t* tuning_code, const uint8_t num_additional_mid_codes) {
    memset(channel_mode_info, 0, sizeof(channel_cal_channel_mode_info_t));
    tuning_code_t tuning_code_rolled_over = *tuning_code;
    tuning_rollover_mid_code(
        &tuning_code_rolled_over,
        /*mid_code_threshold=*/1 + num_additional_mid_codes);
    channel_mode_info->sweep_config = (tuning_sweep_config_t){
        .coarse =
            {
                .start = tuning_code_rolled_over.coarse,
                .end = tuning_code_rolled_over.coarse,
            },
        .mid =
            {
                .start =
                    tuning_code_rolled_over.mid - 1 - num_additional_mid_codes,
                .end =
                    tuning_code_rolled_over.mid + 1 + num_additional_mid_codes,
            },
        .fine =
            {
                .start = TUNING_MIN_CODE,
                // The RX tuning code is incremented by 5 when receiving with a
                // guard time of less than 10 ms.
                .end = TUNING_MAX_CODE - 7,
            },
    };

    // Validate the sweep configuration.
    if (tuning_validate_sweep_config(&channel_mode_info->sweep_config) ==
        FALSE) {
        printf("Invalid sweep configuration.\n");
        return FALSE;
    }
    channel_mode_info->calibrated = FALSE;
    channel_mode_info->num_failures = 0;
    tuning_init_for_sweep(&channel_mode_info->tuning_code,
                          &channel_mode_info->sweep_config);
    return TRUE;
}

// Tune the radio for the initial RX channel calibration.
static inline void channel_cal_initial_rx_tune_radio(void) {
    const uint8_t initial_channel_index =
        channel_convert_channel_to_index(CHANNEL_CAL_INITIAL_CHANNEL);
    const tuning_code_t* rx_tuning_code =
        &g_channel_cal_channel_infos[initial_channel_index].rx.tuning_code;
    radio_rfOff();
    tuning_tune_radio(rx_tuning_code);
    radio_rxEnable();
    radio_rxNow();
}

// Timer callback function during the initial RX channel calibration. If this
// callback function is called, the mote did not receive an enhanced beacon on
// the current tuning code.
static void channel_cal_initial_rx_timer_cb(const opentimers_id_t timer_id) {
    const uint8_t initial_channel_index =
        channel_convert_channel_to_index(CHANNEL_CAL_INITIAL_CHANNEL);
    if (g_channel_cal_channel_infos[initial_channel_index].rx.calibrated ==
        TRUE) {
        // RX channel calibration has finished.
        return;
    }

    // Increment the RX tuning code.
    tuning_increment_fine_code_for_sweep(
        &g_channel_cal_channel_infos[initial_channel_index].rx.tuning_code,
        &g_channel_cal_channel_infos[initial_channel_index].rx.sweep_config);
    channel_cal_print_tuning_code(
        CHANNEL_CAL_INITIAL_CHANNEL, CHANNEL_MODE_RX,
        &g_channel_cal_channel_infos[initial_channel_index].rx.tuning_code);
    channel_cal_initial_rx_tune_radio();

    // Schedule the next timer callback in case no enhanced beacons are
    // received.
    opentimers_scheduleAbsolute(g_channel_cal_timer_id,
                                CHANNEL_CAL_RX_NUM_TICS_PER_TUNING_CODE,
                                opentimers_getCurrentCompareValue(), TIME_TICS,
                                channel_cal_initial_rx_timer_cb);
}

bool channel_cal_init_initial_rx_sweep(void) {
    const uint8_t start_coarse_code = TUNING_MIN_COARSE_CODE;
    const uint8_t end_coarse_code = TUNING_MAX_COARSE_CODE;

    if (start_coarse_code > end_coarse_code) {
        printf(
            "Start coarse code %u should be smaller than or equal to the end "
            "coarse code %u.\n",
            start_coarse_code, end_coarse_code);
        return FALSE;
    }
    if (start_coarse_code < TUNING_MIN_CODE) {
        printf("Start coarse code %u is out of range [%u, %u].\n",
               start_coarse_code, TUNING_MIN_CODE, TUNING_MAX_CODE);
        return FALSE;
    }
    if (end_coarse_code > TUNING_MAX_CODE) {
        printf("End coarse code %u is out of range [%u, %u].\n",
               end_coarse_code, TUNING_MIN_CODE, TUNING_MAX_CODE);
        return FALSE;
    }

    // Set the RX sweep configuration for the initial channel.
    const uint8_t initial_channel_index =
        channel_convert_channel_to_index(CHANNEL_CAL_INITIAL_CHANNEL);
    memset(&g_channel_cal_channel_infos[initial_channel_index].rx, 0,
           sizeof(channel_cal_channel_mode_info_t));
    g_channel_cal_channel_infos[initial_channel_index].rx.sweep_config =
        (tuning_sweep_config_t){
            .coarse =
                {
                    .start = start_coarse_code,
                    .end = end_coarse_code,
                },
            .mid =
                {
                    .start = TUNING_MID_CODE,
                    .end = TUNING_MID_CODE,
                },
            .fine =
                {
                    .start = TUNING_MIN_CODE,
                    // The RX tuning code is incremented by 5 when receiving
                    // with a guard time of less than 10 ms.
                    .end = TUNING_MAX_CODE - 7,
                },
        };

    // Validate the RX sweep configuration for the initial channel.
    if (tuning_validate_sweep_config(
            &g_channel_cal_channel_infos[initial_channel_index]
                 .rx.sweep_config) == FALSE) {
        printf("Invalid RX sweep configuration for channel %u.\n",
               CHANNEL_CAL_INITIAL_CHANNEL);
        return FALSE;
    }
    g_channel_cal_channel_infos[initial_channel_index].rx.calibrated = FALSE;
    g_channel_cal_channel_infos[initial_channel_index].rx.num_failures = 0;
    tuning_init_for_sweep(
        &g_channel_cal_channel_infos[initial_channel_index].rx.tuning_code,
        &g_channel_cal_channel_infos[initial_channel_index].rx.sweep_config);

    g_channel_cal_initial_rx_sweep_finished = FALSE;
    g_channel_cal_num_tx_failures = 0;
    g_channel_cal_num_channels_rx_calibrated = 0;
    g_channel_cal_num_channels_tx_calibrated = 0;
    g_channel_cal_timer_id =
        opentimers_create(TIMER_GENERAL_PURPOSE, TASKPRIO_NONE);
    return TRUE;
}

bool channel_cal_init_remaining_sweeps(void) {
    // Reset the RX sweep configuration for the initial channel to decrease the
    // sweep range.
    const uint8_t initial_channel_index =
        channel_convert_channel_to_index(CHANNEL_CAL_INITIAL_CHANNEL);
    tuning_code_t initial_channel_rx_tuning_code =
        g_channel_cal_channel_infos[initial_channel_index].rx.tuning_code;
    if (channel_cal_init_channel_mode_info(
            &g_channel_cal_channel_infos[initial_channel_index].rx,
            &initial_channel_rx_tuning_code,
            /*num_additional_mid_codes=*/0) == FALSE) {
        printf("Invalid RX sweep configuration for channel %u.\n",
               CHANNEL_CAL_INITIAL_CHANNEL);
        return FALSE;
    }
    g_channel_cal_channel_infos[initial_channel_index].rx.tuning_code =
        initial_channel_rx_tuning_code;
    g_channel_cal_channel_infos[initial_channel_index].rx.calibrated = TRUE;

    // Set the TX sweep configuration for the initial channel.
    tuning_code_t initial_channel_tx_tuning_code =
        g_channel_cal_channel_infos[initial_channel_index].rx.tuning_code;
    tuning_estimate_tx_from_rx(&initial_channel_tx_tuning_code);
    if (channel_cal_init_channel_mode_info(
            &g_channel_cal_channel_infos[initial_channel_index].tx,
            &initial_channel_tx_tuning_code,
            /*num_additional_mid_codes=*/0) == FALSE) {
        printf("Invalid TX sweep configuration for channel %u.\n",
               CHANNEL_CAL_INITIAL_CHANNEL);
        return FALSE;
    }

    // Set the sweep configurations for the remaining channels.
    for (uint8_t channel = CHANNEL_CAL_INITIAL_CHANNEL - 1;
         channel >= MIN_CHANNEL; --channel) {
        const uint8_t channel_index = channel_convert_channel_to_index(channel);
        const uint8_t last_channel = channel + 1;
        const uint8_t last_channel_index =
            channel_convert_channel_to_index(last_channel);

        // Set the RX sweep configuration.
        tuning_code_t channel_rx_tuning_code =
            g_channel_cal_channel_infos[last_channel_index].rx.tuning_code;
        tuning_estimate_previous_channel(&channel_rx_tuning_code);
        if (channel_cal_init_channel_mode_info(
                &g_channel_cal_channel_infos[channel_index].rx,
                &channel_rx_tuning_code,
                /*num_additional_mid_codes=*/
                (uint8_t)(initial_channel_rx_tuning_code.coarse -
                              channel_rx_tuning_code.coarse >=
                          2)) == FALSE) {
            printf("Invalid RX sweep configuration for channel %u.\n", channel);
            return FALSE;
        }

        // Set the TX sweep configuration.
        tuning_code_t channel_tx_tuning_code =
            g_channel_cal_channel_infos[last_channel_index].tx.tuning_code;
        tuning_estimate_previous_channel(&channel_tx_tuning_code);
        if (channel_cal_init_channel_mode_info(
                &g_channel_cal_channel_infos[channel_index].tx,
                &channel_tx_tuning_code,
                /*num_additional_mid_codes=*/
                (uint8_t)(initial_channel_tx_tuning_code.coarse -
                              channel_tx_tuning_code.coarse >=
                          2)) == FALSE) {
            printf("Invalid TX sweep configuration for channel %u.\n", channel);
            return FALSE;
        }
    }
    for (uint8_t channel = CHANNEL_CAL_INITIAL_CHANNEL + 1;
         channel <= MAX_CHANNEL; ++channel) {
        const uint8_t channel_index = channel_convert_channel_to_index(channel);
        const uint8_t last_channel = channel - 1;
        const uint8_t last_channel_index =
            channel_convert_channel_to_index(last_channel);

        // Set the RX sweep configuration.
        tuning_code_t channel_rx_tuning_code =
            g_channel_cal_channel_infos[last_channel_index].rx.tuning_code;
        tuning_estimate_next_channel(&channel_rx_tuning_code);
        if (channel_cal_init_channel_mode_info(
                &g_channel_cal_channel_infos[channel_index].rx,
                &channel_rx_tuning_code,
                /*num_additional_mid_codes=*/
                (uint8_t)(channel_rx_tuning_code.coarse -
                              initial_channel_rx_tuning_code.coarse >=
                          2)) == FALSE) {
            printf("Invalid RX sweep configuration for channel %u.\n", channel);
            return FALSE;
        }

        // Set the TX sweep configuration.
        tuning_code_t channel_tx_tuning_code =
            g_channel_cal_channel_infos[last_channel_index].tx.tuning_code;
        tuning_estimate_next_channel(&channel_tx_tuning_code);
        if (channel_cal_init_channel_mode_info(
                &g_channel_cal_channel_infos[channel_index].tx,
                &channel_tx_tuning_code,
                /*num_additional_codes=*/
                (uint8_t)(channel_tx_tuning_code.coarse -
                              initial_channel_tx_tuning_code.coarse >=
                          2)) == FALSE) {
            printf("Invalid TX sweep configuration for channel %u.\n", channel);
            return FALSE;
        }
    }
    return TRUE;
}

void channel_cal_start_initial_rx_sweep(void) {
    channel_cal_initial_rx_tune_radio();

    // Schedule the timer callback in case no enhanced beacons are received.
    opentimers_scheduleAbsolute(g_channel_cal_timer_id,
                                CHANNEL_CAL_RX_NUM_TICS_PER_TUNING_CODE,
                                opentimers_getCurrentCompareValue(), TIME_TICS,
                                channel_cal_initial_rx_timer_cb);
}

void channel_cal_end_initial_rx_sweep(void) {
    channel_cal_rx_success(CHANNEL_CAL_INITIAL_CHANNEL);
    g_channel_cal_initial_rx_sweep_finished = TRUE;
    opentimers_cancel(g_channel_cal_timer_id);
    channel_cal_print_channel_calibration_finished(CHANNEL_CAL_INITIAL_CHANNEL,
                                                   CHANNEL_MODE_RX);
}

bool channel_cal_initial_rx_calibrated(void) {
    return g_channel_cal_initial_rx_sweep_finished;
}

void channel_cal_rx_get_tuning_code(const uint8_t channel,
                                    tuning_code_t* tuning_code) {
    const uint8_t channel_index = channel_convert_channel_to_index(channel);
    *tuning_code = g_channel_cal_channel_infos[channel_index].rx.tuning_code;
}

bool channel_cal_rx_calibrated(const uint8_t channel) {
    const uint8_t channel_index = channel_convert_channel_to_index(channel);
    return g_channel_cal_channel_infos[channel_index].rx.calibrated;
}

void channel_cal_rx_failure(const uint8_t channel) {
    if (channel_cal_rx_calibrated(channel) == TRUE) {
        return;
    }

    const uint8_t channel_index = channel_convert_channel_to_index(channel);
    ++g_channel_cal_channel_infos[channel_index].rx.num_failures;
    if (g_channel_cal_channel_infos[channel_index].rx.num_failures ==
        CHANNEL_CAL_MAX_NUM_FAILURES) {
        // Proceed to the next tuning code.
        tuning_increment_fine_code_for_sweep(
            &g_channel_cal_channel_infos[channel_index].rx.tuning_code,
            &g_channel_cal_channel_infos[channel_index].rx.sweep_config);
        channel_cal_print_tuning_code(
            channel, CHANNEL_MODE_RX,
            &g_channel_cal_channel_infos[channel_index].rx.tuning_code);
        g_channel_cal_channel_infos[channel_index].rx.num_failures = 0;
    }
}

void channel_cal_rx_success(const uint8_t channel) {
    const uint8_t channel_index = channel_convert_channel_to_index(channel);
    g_channel_cal_channel_infos[channel_index].rx.num_failures = 0;
    channel_cal_print_channel_calibration_finished(channel, CHANNEL_MODE_RX);

    if (g_channel_cal_channel_infos[channel_index].rx.calibrated == FALSE) {
        ++g_channel_cal_num_channels_rx_calibrated;

        // Set the TX sweep configuration.
        if (channel_cal_tx_calibrated(channel) == FALSE) {
            tuning_code_t tuning_code =
                g_channel_cal_channel_infos[channel_index].rx.tuning_code;
            tuning_estimate_tx_from_rx(&tuning_code);
            if (channel_cal_init_channel_mode_info(
                    &g_channel_cal_channel_infos[channel_index].tx,
                    &tuning_code,
                    /*num_additional_mid_codes=*/0) == FALSE) {
                printf("Invalid TX sweep configuration for channel %u.\n",
                       CHANNEL_CAL_INITIAL_CHANNEL);
            }
        }
    }
    g_channel_cal_channel_infos[channel_index].rx.calibrated = TRUE;
}

bool channel_cal_all_rx_calibrated(void) {
    return g_channel_cal_num_channels_rx_calibrated >= NUM_CHANNELS;
}

void channel_cal_tx_get_tuning_code(const uint8_t channel,
                                    tuning_code_t* tuning_code) {
    const uint8_t channel_index = channel_convert_channel_to_index(channel);
    *tuning_code = g_channel_cal_channel_infos[channel_index].tx.tuning_code;
}

bool channel_cal_tx_calibrated(const uint8_t channel) {
    const uint8_t channel_index = channel_convert_channel_to_index(channel);
    return g_channel_cal_channel_infos[channel_index].tx.calibrated;
}

void channel_cal_tx_failure(const uint8_t channel) {
    ++g_channel_cal_num_tx_failures;

    if (channel_cal_tx_calibrated(channel) == TRUE) {
        return;
    }

    const uint8_t channel_index = channel_convert_channel_to_index(channel);
    ++g_channel_cal_channel_infos[channel_index].tx.num_failures;
    if (g_channel_cal_channel_infos[channel_index].tx.num_failures ==
        CHANNEL_CAL_MAX_NUM_FAILURES) {
        // Proceed to the next tuning code.
        tuning_increment_fine_code_for_sweep(
            &g_channel_cal_channel_infos[channel_index].tx.tuning_code,
            &g_channel_cal_channel_infos[channel_index].tx.sweep_config);
        channel_cal_print_tuning_code(
            channel, CHANNEL_MODE_TX,
            &g_channel_cal_channel_infos[channel_index].tx.tuning_code);
        g_channel_cal_channel_infos[channel_index].tx.num_failures = 0;
    }
}

void channel_cal_tx_success(const uint8_t channel) {
    g_channel_cal_num_tx_failures = 0;

    const uint8_t channel_index = channel_convert_channel_to_index(channel);
    g_channel_cal_channel_infos[channel_index].tx.calibrated = TRUE;
    g_channel_cal_channel_infos[channel_index].tx.num_failures = 0;
    ++g_channel_cal_num_channels_tx_calibrated;
    channel_cal_print_channel_calibration_finished(channel, CHANNEL_MODE_TX);
}

uint16_t channel_cal_num_tx_failures(void) {
    return g_channel_cal_num_tx_failures;
}

void channel_cal_reset_num_tx_failures(void) {
    g_channel_cal_num_tx_failures = 0;
}

bool channel_cal_all_tx_calibrated(void) {
    return g_channel_cal_num_channels_tx_calibrated >= NUM_CHANNELS;
}
