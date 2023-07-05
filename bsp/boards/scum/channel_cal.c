#include "channel_cal.h"

#include <stdint.h>
#include <stdio.h>

#include "IEEE802154E.h"
#include "board_info.h"
#include "opendefs.h"
#include "opentimers.h"
#include "schedule.h"
#include "scheduler.h"
#include "tuning.h"

// Number of slotframes to wait on before proceeding to the next tuning code.
#define CHANNEL_CAL_NUM_SLOTFRAMES_PER_TUNING_CODE 8

// RX channel calibration state.
static bool g_channel_cal_rx_calibrated = FALSE;

// RX tuning code.
static tuning_code_t g_channel_cal_rx_tuning_code;

// RX sweep configuration.
static tuning_sweep_config_t g_channel_cal_rx_sweep_config;

// Channel calibration timer ID.
static opentimers_id_t g_channel_cal_timer_id;

// Timer callback function during RX channel calibration. If this callback
// function is called, the mote did not receive an enhanced beacon on the
// current tuning code.
static void channel_cal_rx_timer_cb(const opentimers_id_t timer_id) {
    printf("Channel cal RX timer callback.\n");
    if (timer_id != g_channel_cal_timer_id) {
        return;
    }
    if (g_channel_cal_rx_calibrated == TRUE) {
        // RX channel calibration has finished.
        return;
    }

    // Increment the RX tuning code.
    tuning_increment_code_for_sweep(&g_channel_cal_rx_tuning_code,
                                    &g_channel_cal_rx_sweep_config);
    tuning_tune_radio(&g_channel_cal_rx_tuning_code);
    printf("Tuning to %u.%u%.%u for RX channel calibration.\n",
           g_channel_cal_rx_tuning_code.coarse,
           g_channel_cal_rx_tuning_code.mid, g_channel_cal_rx_tuning_code.fine);
    opentimers_scheduleAbsolute(g_channel_cal_timer_id,
                                CHANNEL_CAL_NUM_SLOTFRAMES_PER_TUNING_CODE *
                                    SLOTFRAME_LENGTH * TsSlotDuration,
                                opentimers_getCurrentCompareValue(), TIME_TICS,
                                channel_cal_rx_timer_cb);
}

bool channel_cal_init(void) {
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

    // Set the RX sweep configuration.
    g_channel_cal_rx_sweep_config = (tuning_sweep_config_t){
        .coarse =
            {
                .start = start_coarse_code,
                .end = end_coarse_code,
            },
        .mid =
            {
                .start = TUNING_MIN_CODE,
                .end = TUNING_MAX_CODE,
            },
        .fine =
            {
                .start = TUNING_MIN_CODE,
                // The RX tuning code is incremented by 5 when receiving with a
                // guard time of less than 10 ms.
                .end = TUNING_MAX_CODE - 5,
            },
    };

    // Validate the RX sweep configuration.
    if (tuning_validate_sweep_config(&g_channel_cal_rx_sweep_config) == FALSE) {
        printf("Invalid RX sweep configuration.\n");
        return FALSE;
    }

    g_channel_cal_rx_calibrated = FALSE;
    tuning_init_for_sweep(&g_channel_cal_rx_tuning_code,
                          &g_channel_cal_rx_sweep_config);
    g_channel_cal_timer_id =
        opentimers_create(TIMER_GENERAL_PURPOSE, TASKPRIO_NONE);
    return TRUE;
}

bool channel_cal_start_rx(void) {
    tuning_tune_radio(&g_channel_cal_rx_tuning_code);
    opentimers_scheduleAbsolute(g_channel_cal_timer_id,
                                CHANNEL_CAL_NUM_SLOTFRAMES_PER_TUNING_CODE *
                                    SLOTFRAME_LENGTH * TsSlotDuration,
                                opentimers_getCurrentCompareValue(), TIME_TICS,
                                channel_cal_rx_timer_cb);
    printf("Scheduled channel cal timer %d for %u tics.\n",
           g_channel_cal_timer_id,
           CHANNEL_CAL_NUM_SLOTFRAMES_PER_TUNING_CODE * SLOTFRAME_LENGTH *
               TsSlotDuration);
    return TRUE;
}

bool channel_cal_end_rx(void) {
    g_channel_cal_rx_calibrated = TRUE;
    printf("Channel cal RX ended.\n");
    opentimers_cancel(g_channel_cal_timer_id);
    return TRUE;
}

bool channel_cal_get_rx_tuning_code(tuning_code_t* tuning_code) {
    if (g_channel_cal_rx_calibrated == FALSE) {
        return FALSE;
    }
    *tuning_code = g_channel_cal_rx_tuning_code;
    return TRUE;
}
