#include "tuning_feedback.h"

#include <stdint.h>

#include "IEEE802154E.h"
#include "channel.h"
#include "memory_map.h"
#include "opendefs.h"
#include "tuning.h"

// Nominally, an IF count of 500, i.e., 500 zero crossings within 100 us,
// corresponds to an IF of 2.5 MHz.
#define TUNING_FEEDBACK_NOMINAL_IF_COUNT 500

// An IF offset of 20 corresponds to 100 kHz, or around 40 ppm at 2.4 GHz.
// Each fine code also corresponds to around 100 kHz.
#define TUNING_FEEDBACK_MAX_IF_OFFSET 25

// Number of IF estimates to average over.
#define TUNING_FEEDBACK_NUM_IF_ESTIMATES_TO_AVERAGE 10

// Minimum number of IF estimates to average over.
#define TUNING_FEEDBACK_MIN_NUM_IF_ESTIMATES_TO_AVERAGE \
    (TUNING_FEEDBACK_NUM_IF_ESTIMATES_TO_AVERAGE / 3)

// Channel information.
typedef struct {
    // Array of the latest IF estimates.
    uint32_t if_estimates[TUNING_FEEDBACK_NUM_IF_ESTIMATES_TO_AVERAGE];

    // If true, the entire IF estimates array is valid.
    bool if_estimates_full_array;

    // Index for the next IF estimate.
    size_t if_estimate_index;
} tuning_feedback_channel_info_t;

// Channel information for all channels.
static tuning_feedback_channel_info_t
    g_tuning_feedback_channel_infos[NUM_CHANNELS];

// Increment the IF estimate index.
static inline void tuning_feedback_increment_if_estimate_index(
    const uint8_t channel) {
    const uint8_t channel_index = channel_convert_channel_to_index(channel);
    g_tuning_feedback_channel_infos[channel_index].if_estimate_index =
        (g_tuning_feedback_channel_infos[channel_index].if_estimate_index + 1) %
        TUNING_FEEDBACK_NUM_IF_ESTIMATES_TO_AVERAGE;
    if (g_tuning_feedback_channel_infos[channel_index].if_estimate_index == 0) {
        g_tuning_feedback_channel_infos[channel_index].if_estimates_full_array =
            TRUE;
    }
}

// Write the latest IF estimate to the array.
static inline void tuning_feedback_write_if_estimate(
    const uint8_t channel, const uint32_t if_estimate) {
    const uint8_t channel_index = channel_convert_channel_to_index(channel);
    const uint32_t index =
        g_tuning_feedback_channel_infos[channel_index].if_estimate_index;
    g_tuning_feedback_channel_infos[channel_index].if_estimates[index] =
        if_estimate;
    tuning_feedback_increment_if_estimate_index(channel);
}

// Get the number of IF estimates in the array.
static inline size_t tuning_feedback_num_if_estimates(const uint8_t channel) {
    const uint8_t channel_index = channel_convert_channel_to_index(channel);
    return (g_tuning_feedback_channel_infos[channel_index]
                .if_estimates_full_array == TRUE)
               ? TUNING_FEEDBACK_NUM_IF_ESTIMATES_TO_AVERAGE
               : g_tuning_feedback_channel_infos[channel_index]
                     .if_estimate_index;
}

// Average the IF estimates.
static inline uint32_t tuning_feedback_average_if_estimates(
    const uint8_t channel) {
    const uint8_t channel_index = channel_convert_channel_to_index(channel);
    const size_t num_if_estimates_to_average =
        tuning_feedback_num_if_estimates(channel);
    uint32_t if_estimate_sum = 0;
    uint32_t i = 0;
    for (i = 0; i < num_if_estimates_to_average; ++i) {
        if_estimate_sum +=
            g_tuning_feedback_channel_infos[channel_index].if_estimates[i];
    }
    return if_estimate_sum / num_if_estimates_to_average;
}

// Reset the IF estimates.
static inline void tuning_feedback_reset_if_estimates(const uint8_t channel) {
    const uint8_t channel_index = channel_convert_channel_to_index(channel);
    g_tuning_feedback_channel_infos[channel_index].if_estimates_full_array =
        FALSE;
    g_tuning_feedback_channel_infos[channel_index].if_estimate_index = 0;
}

// Print the tuning code feedback over UART.
static inline void tuning_feedback_print_tuning_code(
    const uint8_t channel, const tuning_code_t* tuning_code) {
    UART_REG__TX_DATA = '~';
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

void tuning_feedback_adjust_rx(const uint8_t channel,
                               const uint32_t if_estimate) {
    // An IF estimate of 0 is usually an invalid value.
    if (if_estimate == 0) {
        return;
    }

    // Write the IF estimate.
    tuning_feedback_write_if_estimate(channel, if_estimate);

    // Only tune the tuning codes if there are sufficient IF estimates.
    if (tuning_feedback_num_if_estimates(channel) <
        TUNING_FEEDBACK_MIN_NUM_IF_ESTIMATES_TO_AVERAGE) {
        return;
    }

    // Average the latest IF estimates.
    const uint32_t if_estimate_average =
        tuning_feedback_average_if_estimates(channel);

    // Adjust the RX tuning code.
    tuning_code_t tuning_code;
    channel_get_tuning_code(channel, CHANNEL_MODE_RX, &tuning_code);
    if (if_estimate_average >
        TUNING_FEEDBACK_NOMINAL_IF_COUNT + TUNING_FEEDBACK_MAX_IF_OFFSET) {
        // The IF estimate is too high.
        // TODO(titan): Handle fine code and mid code overflows.
        tuning_increment_fine_codes(&tuning_code, /*num_fine_codes=*/1);
        channel_set_tuning_code(channel, CHANNEL_MODE_RX, &tuning_code);
        tuning_feedback_print_tuning_code(channel, &tuning_code);
        tuning_feedback_reset_if_estimates(channel);
    } else if (if_estimate_average < TUNING_FEEDBACK_NOMINAL_IF_COUNT -
                                         TUNING_FEEDBACK_MAX_IF_OFFSET) {
        // The IF estimate is too low.
        // TODO(titan): Handle fine code and mid code underflows.
        tuning_decrement_fine_codes(&tuning_code, /*num_fine_codes=*/1);
        channel_set_tuning_code(channel, CHANNEL_MODE_RX, &tuning_code);
        tuning_feedback_print_tuning_code(channel, &tuning_code);
        tuning_feedback_reset_if_estimates(channel);
    }
}
