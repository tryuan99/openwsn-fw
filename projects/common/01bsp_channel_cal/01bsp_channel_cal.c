// The channel calibration allows SCuM chips to find the correct TX and RX
// channel frequencies.
//
// Channel calibration happens in two steps:
//  1. In the first step, SCuM sweeps through its tuning codes while
//     transmitting one packet per tuning code with its tuning code information.
//     The OpenMote continuously receives packets from SCuM and records the
//     received tuning codes on each channel. When the RX timeout expires, the
//     OpenMote will proceed to the next channel.
//  2. In the second step, SCuM sweeps through its tuning codes while listening
//     for packets from the OpenMote. The OpenMote transmits packets on each
//     channel containing the SCuM tuning codes that were received in the
//     previous step. After each packet transmission, the OpenMote waits for an
//     acknowledgment from SCuM. The acknowledgment also contains a flag for the
//     OpenMote to proceed to the next channel.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "leds.h"
#include "radio.h"
#include "sctimer.h"
#include "uart_tx.h"

// Number of sensors.
#define SMART_STAKE_MAX_NUM_SENSORS 4

// The 802.15.4 channel to use for receiving ADC data.
#define SMART_STAKE_DEMO_CHANNEL 17

// The 802.15.4 channel range to calibrate for.
#define CHANNEL_CAL_CHANNEL_START 17
#define CHANNEL_CAL_CHANNEL_END 18

// Valid 802.15.4 channel range.
#define CHANNEL_CAL_MIN_CHANNEL 11
#define CHANNEL_CAL_MAX_CHANNEL 26
#define CHANNEL_CAL_NUM_CHANNELS                                               \
  (CHANNEL_CAL_MAX_CHANNEL - CHANNEL_CAL_MIN_CHANNEL + 1)

#if CHANNEL_CAL_CHANNEL_END < CHANNEL_CAL_CHANNEL_START
#error Channel calibration end channel must be greater than or equal to the start channel.
#endif // CHANNEL_CAL_CHANNEL_END < CHANNEL_CAL_CHANNEL_START

#if CHANNEL_CAL_CHANNEL_END > CHANNEL_CAL_MAX_CHANNEL
#error Channel calibration end channel out of range.
#endif // CHANNEL_CAL_CHANNEL_END > CHANNEL_CAL_MAX_CHANNEL

#if CHANNEL_CAL_CHANNEL_START < CHANNEL_CAL_MIN_CHANNEL
#error Channel calibration start channel out of range.
#endif // CHANNEL_CAL_CHANNEL_START < CHANNEL_CAL_MIN_CHANNEL

// The timer runs on a 32 kHz clock.
// The RX timeout period is timeout period for when the OpenMote is receiving
// packets from SCuM. When the RX timeout expires, the OpenMote will advance to
// the next 802.15.4 channel.
#define CHANNEL_CAL_RX_TIMEOUT (32768 >> 1) // 500 ms @ 32 kHz
// The long RX timeout period is used for receiving the first packet of a
// channel and after the coarse code rolls over.
#define CHANNEL_CAL_RX_LONG_TIMEOUT (32768 << 1) // 2 s @ 32 kHz
// The TX timeout period is the timeout period for when the OpenMote transmits a
// packet.
#define CHANNEL_CAL_TX_TIMEOUT (32768 >> 6) // 15.625 ms @ 32 kHz

// If the last received packet from SCuM was at the mid code threshold or
// higher, then the OpenMote should expect the coarse code to roll over.
#define CHANNEL_CAL_RX_MID_CODE_THRESHOLD 24

// Maximum number of TX tuning codes per channel.
#define CHANNEL_CAL_MAX_NUM_TX_TUNING_CODES_PER_CHANNEL 4

// Maximum number of recorded TX tuning codes for the current channel. After all
// tuning codes for a channel have been received, the received fine codes will
// be averaged.
#define CHANNEL_CAL_MAX_NUM_RECORDED_TX_TUNING_CODES 128

// Tuning code.
typedef struct __attribute__((packed)) {
  // Coarse code.
  uint8_t coarse;

  // Mid code.
  uint8_t mid;

  // Fine code.
  uint8_t fine;
} tuning_code_t;

// Channel calibratino state enum.
typedef enum {
  CHANNEL_CAL_STATE_INVALID = -1,
  CHANNEL_CAL_STATE_RX,
  CHANNEL_CAL_STATE_RX_IDLE,
  CHANNEL_CAL_STATE_RX_RECEIVED,
  CHANNEL_CAL_STATE_TX,
  CHANNEL_CAL_STATE_RX_ACK,
  CHANNEL_CAL_STATE_RX_ACK_IDLE,
  CHANNEL_CAL_STATE_RX_ACK_RECEIVED,
  CHANNEL_CAL_STATE_SMART_STAKE_RX,
  CHANNEL_CAL_STATE_SMART_STAKE_RX_IDLE,
  CHANNEL_CAL_STATE_SMART_STAKE_RX_RECEIVED,
} channel_cal_state_e;

// Channel calibration RX command from SCuM.
typedef enum {
  CHANNEL_CAL_COMMAND_NONE = 0x00,
  CHANNEL_CAL_COMMAND_CHANGE_CHANNEL = 0xFF,
} channel_cal_command_e;

// Channel calibration RX packet.
typedef struct __attribute__((packed)) {
  // Sequence number.
  uint8_t sequence_number;

  // Channel.
  uint8_t channel;

  // Reserved.
  uint8_t reserved1;

  // Reserved.
  uint8_t reserved2;

  // Command for the OpenMote.
  channel_cal_command_e command : 8;

  // Reserved.
  uint8_t reserved3;

  // Tuning code.
  tuning_code_t tuning_code;

  // Reserved.
  uint8_t reserved54;

  // CRC.
  uint16_t crc;
} channel_cal_rx_packet_t;

// Channel calibration TX packet.
typedef struct __attribute__((packed)) {
  // Sequence number.
  uint8_t sequence_number;

  // Channel.
  uint8_t channel;

  // TX tuning codes.
  tuning_code_t
      tx_tuning_codes[CHANNEL_CAL_MAX_NUM_TX_TUNING_CODES_PER_CHANNEL];

  // Reserved.
  uint8_t reserved1;

  // Reserved.
  uint8_t reserved2;

  // CRC.
  uint16_t crc;
} channel_cal_tx_packet_t;

// SmartStake RX packet containing the ADC data.
typedef struct __attribute__((packed)) {
  // Sequence number.
  uint8_t sequence_number;

  // Channel.
  uint8_t channel;

  // Reserved.
  uint8_t reserved1;

  // Reserved.
  uint8_t reserved2;

  // Measurement output.
  uint32_t output[SMART_STAKE_MAX_NUM_SENSORS];

  // Tuning code.
  tuning_code_t tuning_code;

  // Reserved.
  uint8_t reserved3;

  // CRC.
  uint16_t crc;
} smart_stake_rx_packet_t;

// Channel calibration state.
static channel_cal_state_e g_channel_cal_state = CHANNEL_CAL_STATE_INVALID;

// Current 802.15.4 channel.
static uint8_t g_channel_cal_channel = CHANNEL_CAL_CHANNEL_START;

// RX packet buffer.
static channel_cal_rx_packet_t g_channel_cal_rx_packet;

// Length of the RX packet.
static uint8_t g_channel_cal_rx_packet_length = 0;

// Received signal strength indicator of the RX packet.
static int8_t g_channel_cal_rx_packet_rssi = 0;

// Link quality indicator of the RX packet.
static uint8_t g_channel_cal_rx_packet_lqi = 0;

// If true, the CRC is valid on the RX packet.
static bool g_channel_cal_rx_packet_crc = false;

// TX packet buffer.
static channel_cal_tx_packet_t g_channel_cal_tx_packet;

// TX packet sequence number.
static uint8_t g_channel_cal_tx_packet_sequence_number = 0;

// Buffer for all of the received SCuM TX tuning codes.
static tuning_code_t g_channel_cal_scum_tx_tuning_codes
    [CHANNEL_CAL_NUM_CHANNELS][CHANNEL_CAL_MAX_NUM_TX_TUNING_CODES_PER_CHANNEL];

// Buffer for the received SCuM TX tuning codes on the current channel.
static tuning_code_t g_channel_cal_scum_tx_tuning_codes_for_channel
    [CHANNEL_CAL_MAX_NUM_RECORDED_TX_TUNING_CODES];

// Number of received SCuM TX tuning codes for the current channel.
static uint8_t g_channel_cal_scum_tx_num_received_tuning_codes_for_channel = 0;

// SmartStake RX packet buffer.
static smart_stake_rx_packet_t g_smart_stake_rx_packet;

// Length of the SmartStake RX packet.
static uint8_t g_smart_stake_rx_packet_length = 0;

// Received signal strength indicator of the SmartStake RX packet.
static int8_t g_smart_stake_rx_packet_rssi = 0;

// Link quality indicator of the SmartStake RX packet.
static uint8_t g_smart_stake_rx_packet_lqi = 0;

// If true, the CRC is valid on the SmartStake RX packet.
static bool g_smart_stake_rx_packet_crc = false;

// UART buffer.
static char g_channel_cal_uart_buffer[UART_TX_MAX_LENGTH];

// Start frame callback function.
static void channel_cal_start_frame_callback(const PORT_TIMER_WIDTH timestamp) {
  leds_sync_on();
}

// End frame callback function.
static void channel_cal_end_frame_callback(const PORT_TIMER_WIDTH timestamp) {
  leds_sync_off();

  if (g_channel_cal_state == CHANNEL_CAL_STATE_TX) {
    // The OpenMote just finished transmitting a packet to SCuM.
    g_channel_cal_state = CHANNEL_CAL_STATE_RX_ACK;
    return;
  }

  // The OpenMote just finished receiving a packet from SCuM.
  // Read the received packet.
  memset(&g_channel_cal_rx_packet, 0, sizeof(channel_cal_rx_packet_t));
  radio_getReceivedFrame(
      (uint8_t *)&g_channel_cal_rx_packet, &g_channel_cal_rx_packet_length,
      sizeof(channel_cal_rx_packet_t), &g_channel_cal_rx_packet_rssi,
      &g_channel_cal_rx_packet_lqi, &g_channel_cal_rx_packet_crc);
  if (g_channel_cal_rx_packet_length <= sizeof(channel_cal_rx_packet_t) &&
      g_channel_cal_rx_packet_crc) {
    if (g_channel_cal_state == CHANNEL_CAL_STATE_RX_IDLE) {
      g_channel_cal_state = CHANNEL_CAL_STATE_RX_RECEIVED;
    } else if (g_channel_cal_state == CHANNEL_CAL_STATE_RX_ACK_IDLE) {
      g_channel_cal_state = CHANNEL_CAL_STATE_RX_ACK_RECEIVED;
    }
  }
}

// End frame callback function for the ADC data.
static void smart_stake_end_frame_callback(const PORT_TIMER_WIDTH timestamp) {
  leds_sync_off();

  // The OpenMote just finished receiving a packet from SCuM containing the
  // ADC data. Read the received packet.
  memset(&g_smart_stake_rx_packet, 0, sizeof(smart_stake_rx_packet_t));
  radio_getReceivedFrame(
      (uint8_t *)&g_smart_stake_rx_packet, &g_smart_stake_rx_packet_length,
      sizeof(smart_stake_rx_packet_t), &g_smart_stake_rx_packet_rssi,
      &g_smart_stake_rx_packet_lqi, &g_smart_stake_rx_packet_crc);
  if (g_smart_stake_rx_packet_length <= sizeof(smart_stake_rx_packet_t) &&
      g_smart_stake_rx_packet_crc &&
      g_channel_cal_state == CHANNEL_CAL_STATE_SMART_STAKE_RX_IDLE) {
    g_channel_cal_state = CHANNEL_CAL_STATE_SMART_STAKE_RX_RECEIVED;
  }
}

// Average the received fine codes to find the optimal tuning code for each
// received coarse and mid code pair.
static void channel_cal_average_scum_tx_tuning_codes_for_channel(void) {
  uint8_t j = 0;
  for (uint8_t i = 0;
       i < g_channel_cal_scum_tx_num_received_tuning_codes_for_channel; ++i) {
    g_channel_cal_scum_tx_tuning_codes[g_channel_cal_channel -
                                       CHANNEL_CAL_MIN_CHANNEL][j] =
        g_channel_cal_scum_tx_tuning_codes_for_channel[i];
    while (i < g_channel_cal_scum_tx_num_received_tuning_codes_for_channel -
                   1 &&
           g_channel_cal_scum_tx_tuning_codes_for_channel[i + 1].coarse ==
               g_channel_cal_scum_tx_tuning_codes[g_channel_cal_channel -
                                                  CHANNEL_CAL_MIN_CHANNEL][j]
                   .coarse &&
           g_channel_cal_scum_tx_tuning_codes_for_channel[i + 1].mid ==
               g_channel_cal_scum_tx_tuning_codes[g_channel_cal_channel -
                                                  CHANNEL_CAL_MIN_CHANNEL][j]
                   .mid) {
      ++i;
    }
    g_channel_cal_scum_tx_tuning_codes[g_channel_cal_channel -
                                       CHANNEL_CAL_MIN_CHANNEL][j]
        .fine = (g_channel_cal_scum_tx_tuning_codes[g_channel_cal_channel -
                                                    CHANNEL_CAL_MIN_CHANNEL][j]
                     .fine +
                 g_channel_cal_scum_tx_tuning_codes_for_channel[i].fine) /
                2;

    ++j;
    if (j >= CHANNEL_CAL_MAX_NUM_TX_TUNING_CODES_PER_CHANNEL) {
      break;
    }
  }
}

// Timer callback function.
static void channel_cal_timer_callback(void) {
  switch (g_channel_cal_state) {
  case CHANNEL_CAL_STATE_RX_IDLE: {
    // The timeout expired while receiving packets from SCuM, so proceed
    // to the next channel.

    // TODO(titan): Handle the case where the coarse code has rolled
    // over here.

    // Average the received fine codes for the current channel.
    channel_cal_average_scum_tx_tuning_codes_for_channel();

    // Increment the channel.
    ++g_channel_cal_channel;

    // If the OpenMote finished receiving the last channel, proceed to
    // the second step of channel calibration.
    if (g_channel_cal_channel > CHANNEL_CAL_CHANNEL_END) {
      g_channel_cal_channel = CHANNEL_CAL_CHANNEL_START;
      g_channel_cal_state = CHANNEL_CAL_STATE_TX;
    } else {
      // Reset for receiving on the next channel.
      g_channel_cal_scum_tx_num_received_tuning_codes_for_channel = 0;
      g_channel_cal_state = CHANNEL_CAL_STATE_RX;
      sctimer_setCompare(sctimer_readCounter() + CHANNEL_CAL_RX_TIMEOUT);
    }
    break;
  }
  case CHANNEL_CAL_STATE_RX_ACK_IDLE: {
    // The timeout expired while receiving acknowledgment packets from
    // SCuM, so transmit the next packet.
    g_channel_cal_state = CHANNEL_CAL_STATE_TX;
    break;
  }
  default: {
    break;
  }
  }
}

// Print the current channel over UART.
static inline void channel_cal_print_channel(void) {
  snprintf(g_channel_cal_uart_buffer, UART_TX_MAX_LENGTH, "Channel %02d\n",
           g_channel_cal_channel);
  uart_tx_send_str(g_channel_cal_uart_buffer);
}

// Print the latest received SCuM TX tuning code over UART.
static inline void channel_cal_print_received_scum_tx_tuning_code(void) {
  snprintf(g_channel_cal_uart_buffer, UART_TX_MAX_LENGTH,
           "%c%02d %02d %02d %02d\n", g_channel_cal_rx_packet_crc ? '+' : '-',
           g_channel_cal_channel,
           g_channel_cal_scum_tx_tuning_codes_for_channel
               [g_channel_cal_scum_tx_num_received_tuning_codes_for_channel - 1]
                   .coarse,
           g_channel_cal_scum_tx_tuning_codes_for_channel
               [g_channel_cal_scum_tx_num_received_tuning_codes_for_channel - 1]
                   .mid,
           g_channel_cal_scum_tx_tuning_codes_for_channel
               [g_channel_cal_scum_tx_num_received_tuning_codes_for_channel - 1]
                   .fine);
  uart_tx_send_str(g_channel_cal_uart_buffer);
}

// Print the latest received SCuM ADC data over UART.
static inline void smart_stake_print_received_packet(void) {
  size_t buffer_offset = snprintf(
      g_channel_cal_uart_buffer, UART_TX_MAX_LENGTH, "%03d %02d %02d %02d %02d",
      g_smart_stake_rx_packet.sequence_number, g_smart_stake_rx_packet.channel,
      g_smart_stake_rx_packet.tuning_code.coarse,
      g_smart_stake_rx_packet.tuning_code.mid,
      g_smart_stake_rx_packet.tuning_code.fine);
  for (size_t i = 0; i < SMART_STAKE_MAX_NUM_SENSORS; ++i) {
    buffer_offset += snprintf(g_channel_cal_uart_buffer + buffer_offset,
                              UART_TX_MAX_LENGTH - buffer_offset, " %04ld",
                              g_smart_stake_rx_packet.output[i]);
  }
  snprintf(g_channel_cal_uart_buffer + buffer_offset,
           UART_TX_MAX_LENGTH - buffer_offset, " %d\n",
           g_smart_stake_rx_packet_rssi);
  uart_tx_send_str(g_channel_cal_uart_buffer);
}

int mote_main(void) {
  // Initialize the board.
  board_init();

  // Set the radio callback functions.
  radio_setStartFrameCb(channel_cal_start_frame_callback);
  radio_setEndFrameCb(channel_cal_end_frame_callback);

  // Set the timer callback function.
  sctimer_set_callback(channel_cal_timer_callback);

  // Initialize UART TX.
  uart_tx_init();

  // Turn on the radio.
  radio_rfOn();
  g_channel_cal_state = CHANNEL_CAL_STATE_SMART_STAKE_RX;
  g_channel_cal_channel = CHANNEL_CAL_CHANNEL_START;

  while (true) {
    switch (g_channel_cal_state) {
    case CHANNEL_CAL_STATE_RX: {
      // Listen for packets from SCuM.
      channel_cal_print_channel();
      radio_setFrequency(g_channel_cal_channel, FREQ_RX);
      radio_rxEnable();
      radio_rxNow();

      g_channel_cal_state = CHANNEL_CAL_STATE_RX_IDLE;
      break;
    }
    case CHANNEL_CAL_STATE_RX_RECEIVED: {
      // A packet was just received from SCuM.
      if (g_channel_cal_rx_packet.tuning_code.mid >
          CHANNEL_CAL_RX_MID_CODE_THRESHOLD) {
        sctimer_setCompare(sctimer_readCounter() + CHANNEL_CAL_RX_LONG_TIMEOUT);
      } else {
        sctimer_setCompare(sctimer_readCounter() + CHANNEL_CAL_RX_TIMEOUT);
      }
      sctimer_enable();
      leds_radio_on();

      // Record the received SCuM TX tuning code.
      if (g_channel_cal_rx_packet.tuning_code.coarse != 0 ||
          g_channel_cal_rx_packet.tuning_code.mid != 0 ||
          g_channel_cal_rx_packet.tuning_code.fine != 0) {
        g_channel_cal_scum_tx_tuning_codes_for_channel
            [g_channel_cal_scum_tx_num_received_tuning_codes_for_channel] =
                g_channel_cal_rx_packet.tuning_code;
        ++g_channel_cal_scum_tx_num_received_tuning_codes_for_channel;
      }
      channel_cal_print_received_scum_tx_tuning_code();
      leds_radio_off();

      g_channel_cal_state = CHANNEL_CAL_STATE_RX_IDLE;
      break;
    }
    case CHANNEL_CAL_STATE_TX: {
      // Transmit a packet to SCuM.
      leds_radio_toggle();

      memset(&g_channel_cal_tx_packet, 0, sizeof(channel_cal_rx_packet_t));
      g_channel_cal_tx_packet.sequence_number =
          g_channel_cal_tx_packet_sequence_number;
      g_channel_cal_tx_packet.channel = g_channel_cal_channel;
      ++g_channel_cal_tx_packet_sequence_number;

      // Copy over the received SCuM TX tuning codes.
      for (uint8_t i = 0; i < CHANNEL_CAL_MAX_NUM_TX_TUNING_CODES_PER_CHANNEL;
           ++i) {
        g_channel_cal_tx_packet.tx_tuning_codes[i] =
            g_channel_cal_scum_tx_tuning_codes[g_channel_cal_channel -
                                               CHANNEL_CAL_MIN_CHANNEL][i];
      }

      // Send the packet.
      radio_setFrequency(g_channel_cal_channel, FREQ_TX);
      radio_loadPacket((uint8_t *)&g_channel_cal_tx_packet,
                       sizeof(channel_cal_tx_packet_t));
      radio_txEnable();
      radio_txNow();

      // Start the timer for transmitting the next packet.
      sctimer_setCompare(sctimer_readCounter() + CHANNEL_CAL_TX_TIMEOUT);

      g_channel_cal_state = CHANNEL_CAL_STATE_RX_ACK;
      break;
    }
    case CHANNEL_CAL_STATE_RX_ACK: {
      // Listen for acknowledgment packets from SCuM.
      radio_setFrequency(g_channel_cal_channel, FREQ_RX);
      radio_rxEnable();
      radio_rxNow();

      g_channel_cal_state = CHANNEL_CAL_STATE_RX_ACK_IDLE;
      break;
    }
    case CHANNEL_CAL_STATE_RX_ACK_RECEIVED: {
      // An acknowledgment packet was just received from SCuM.
      if (g_channel_cal_rx_packet.channel == g_channel_cal_channel &&
          g_channel_cal_rx_packet.command ==
              CHANNEL_CAL_COMMAND_CHANGE_CHANNEL) {
        ++g_channel_cal_channel;
        channel_cal_print_channel();

        g_channel_cal_state = CHANNEL_CAL_STATE_RX_ACK;
        if (g_channel_cal_channel > CHANNEL_CAL_CHANNEL_END) {
          uart_tx_send_str("Channel calibration done.\n");
          g_channel_cal_state = CHANNEL_CAL_STATE_SMART_STAKE_RX;
        }
      } else {
        g_channel_cal_state = CHANNEL_CAL_STATE_RX_ACK_IDLE;
      }
      break;
    }
    case CHANNEL_CAL_STATE_SMART_STAKE_RX: {
      uart_tx_send_str("Starting SmartStake RX.\n");

      // Start receiving the ADC data.
      radio_setEndFrameCb(smart_stake_end_frame_callback);
      g_channel_cal_channel = SMART_STAKE_DEMO_CHANNEL;
      radio_setFrequency(g_channel_cal_channel, FREQ_RX);
      radio_rxEnable();
      radio_rxNow();

      g_channel_cal_state = CHANNEL_CAL_STATE_SMART_STAKE_RX_IDLE;
      break;
    }
    case CHANNEL_CAL_STATE_SMART_STAKE_RX_RECEIVED: {
      smart_stake_print_received_packet();

      g_channel_cal_state = CHANNEL_CAL_STATE_SMART_STAKE_RX_IDLE;
      break;
    }
    case CHANNEL_CAL_STATE_RX_IDLE:
    case CHANNEL_CAL_STATE_RX_ACK_IDLE:
    case CHANNEL_CAL_STATE_SMART_STAKE_RX_IDLE:
    default: {
      break;
    }
    }
  }

  return EXIT_SUCCESS;
}
