#include "config.h"

#if OPENWSN_USENSOR_NETWORK_C

#include "usensor_network.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "IEEE802154E.h"
#include "async.h"
#include "icmpv6rpl.h"
#include "idmanager.h"
#include "msf.h"
#include "opendefs.h"
#include "openrandom.h"
#include "openserial.h"
#include "opentimers.h"
#include "schedule.h"
#include "scheduler.h"
#include "sock.h"

#ifdef SCUM
#include "adc.h"
#include "memory_map.h"
#include "sensors.h"
#include "tuning.h"
#endif // SCUM

// Sensor network packet period in milliseconds.
#define USENSOR_NETWORK_PERIOD_MS 2000

// Sensor network traffic rate.
#define USENSOR_NETWORK_TRAFFIC_RATE 1

// Sensor network maximum buffer size in bytes.
#define USENSOR_NETWORK_MAX_BUFFER_SIZE 64

// Sensor network maximum number of sensors in the packet.
#define USENSOR_NETWORK_MAX_NUM_SENSORS 8

// Sensor payload.
typedef struct __attribute__((packed)) {
  // Sequence number.
  uint8_t sequence_number;

  // Channel. Unused for the sensor network application.
  uint8_t channel;

  // Reserved.
  uint8_t reserved1;

  // Reserved.
  uint8_t reserved2;

  // Measurement output.
  uint32_t output[USENSOR_NETWORK_MAX_NUM_SENSORS];

  // Tuning code. Unused for the sensor network application.
#ifdef SCUM
  tuning_code_t tuning_code;
#else  // !SCUM
uint8_t coarse;
uint8_t mid;
uint8_t fine;
#endif // SCUM

  // Reserved.
  uint8_t reserved3;

  // CRC. Unused for the sensor network application.
  uint16_t crc;
} sensor_network_payload_t;

// Data packet.
typedef struct __attribute__((packed)) {
  // 16-bit source address.
  uint16_t addr_16b;

  // Payload.
  sensor_network_payload_t payload;
} sensor_network_packet_t;

#ifdef SCUM
// ADC configuration.
static const adc_config_t g_sensor_network_adc_config = {
    .reset_source = ADC_RESET_SOURCE_FSM,
    .convert_source = ADC_CONVERT_SOURCE_FSM,
    .pga_amplify_source = ADC_PGA_AMPLIFY_SOURCE_FSM,
    .pga_gain = 0,
    .settling_time = 0,
    .bandgap_reference_tuning_code = 1,
    .const_gm_tuning_code = 0xFF,
    .vbat_div_4_enabled = FALSE,
    .ldo_enabled = TRUE,
    .input_mux_select = ADC_INPUT_MUX_SELECT_EXTERNAL_SIGNAL,
    .pga_bypass = TRUE,
};

// Sensors configuration.
static const sensors_config_t g_sensor_network_sensors_config = {
  .gpio_strobe = GPIO_0,
  .gpios_select =
      {
          GPIO_1,
          GPIO_2,
          GPIO_3,
      },
  .num_sensors = 5,
  .sensors =
      {
          SENSOR_TYPE_POTENTIOMETRIC,
          SENSOR_TYPE_POTENTIOMETRIC,
          SENSOR_TYPE_POTENTIOMETRIC,
          SENSOR_TYPE_POTENTIOMETRIC,
          SENSOR_TYPE_POTENTIOMETRIC,
      },
  .sensor_configs =
      {
          {0},
          {0},
          {0},
          {0},
          {0},
      },
};
#endif // SCUM

// Packet sequence number.
static uint8_t g_sensor_network_sequence_number = 0;

// Data packet.
static sensor_network_packet_t g_sensor_network_packet;

// Timer ID.
static opentimers_id_t g_sensor_network_timer_id;

// If true, the sensor network application is sending a packet.
static bool g_sensor_network_busy_sending = FALSE;

// UDP socket.
static sock_udp_t g_sensor_network_sock;

// Destination address.
static const uint8_t g_sensor_network_destination_address[] = {
    0xbb, 0xbb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x06, 0x06, 0x06, 0x05, 0x05, 0x05, 0x05, 0x09,
};

static void sensor_network_sock_handler(sock_udp_t *sock,
                                        sock_async_flags_t type, void *arg) {
  (void)arg;

  if (type & SOCK_ASYNC_MSG_RECV) {
    int16_t res = 0;
    char buffer[USENSOR_NETWORK_MAX_BUFFER_SIZE];
    sock_udp_ep_t remote;
    if ((res = sock_udp_recv(sock, buffer, USENSOR_NETWORK_MAX_BUFFER_SIZE, 0,
                             &remote)) >= 0) {
      openserial_printf("Received %d bytes from remote endpoint:\n", res);
      openserial_printf(" - port: %d", remote.port);
      openserial_printf(" - addr: ", remote.port);
      for (size_t i = 0; i < 16; ++i) {
        openserial_printf("%x ", remote.addr.ipv6[i]);
      }

      openserial_printf("\n\n");
      openserial_printf("Msg received: %s\n", buffer);

      // Print out sensor data.
      sensor_network_packet_t *received_packet =
          (sensor_network_packet_t *)buffer;
      openserial_printf("Sensor data:");
      for (size_t i = 0; i < USENSOR_NETWORK_MAX_NUM_SENSORS; ++i) {
        openserial_printf(" %u", received_packet->payload.output[i]);
      }
      openserial_printf("\n");
    }
  }

  if (type & SOCK_ASYNC_MSG_SENT) {
    owerror_t error = *(uint8_t *)arg;
    if (error == E_FAIL) {
      LOG_ERROR(COMPONENT_USENSOR_NETWORK, ERR_MAXRETRIES_REACHED,
                (errorparameter_t)g_sensor_network_sequence_number,
                (errorparameter_t)0);
    }
    printf("Sent sensor network packet\n");
    g_sensor_network_busy_sending = FALSE;
  }
}

static inline void sensor_network_task_cb(void) {
#ifdef SCUM
  UART_REG__TX_DATA = '%';
  UART_REG__TX_DATA = '\n';
#endif // SCUM

  // Check if synchronized.
  if (ieee154e_isSynch() == FALSE) {
    return;
  }

  // Check if DAG root.
  if (idmanager_getIsDAGroot()) {
    opentimers_destroy(g_sensor_network_timer_id);
    return;
  }

  // Check whether a parent exists.
  open_addr_t parent_neighbor;
  bool found_neighbor = icmpv6rpl_getPreferredParentEui64(&parent_neighbor);
  if (found_neighbor == FALSE) {
    return;
  }

  // Check for a TX cell.
  if (schedule_hasNegotiatedCellToNeighbor(&parent_neighbor, CELLTYPE_TX) ==
      FALSE) {
    return;
  }

  // Check whether a packet is already being sent.
  if (g_sensor_network_busy_sending == TRUE) {
    return;
  }

#ifdef SCUM
  // Measure the sensors.
  sensors_measurements_t sensor_measurements;
  sensors_measure(&sensor_measurements);
#endif // SCUM

  sock_udp_ep_t remote = (sock_udp_ep_t){
      .family = AF_INET6,
      .port = WKP_UDP_SENSOR_NETWORK,
  };
  memcpy(remote.addr.ipv6, g_sensor_network_destination_address,
         sizeof(g_sensor_network_destination_address));
  memset(&g_sensor_network_packet, 0, sizeof(sensor_network_packet_t));
  memcpy(&g_sensor_network_packet.addr_16b,
         idmanager_getMyID(ADDR_16B)->addr_16b, sizeof(uint16_t));

#ifdef SCUM
  // Assemble the payload of the sensor network packet.
  g_sensor_network_packet.payload.sequence_number =
      g_sensor_network_sequence_number;
  for (size_t i = 0; i < g_sensor_network_sensors_config.num_sensors; ++i) {
    switch (g_sensor_network_sensors_config.sensors[i]) {
    case SENSOR_TYPE_POTENTIOMETRIC: {
      g_sensor_network_packet.payload.output[i] =
          sensor_measurements.measurements[i].adc_output;
      break;
    }
    case SENSOR_TYPE_RESISTIVE:
    case SENSOR_TYPE_PH:
    case SENSOR_TYPE_INVALID:
    default: {
      break;
    }
    }
  }
#endif // SCUM

  if (sock_udp_send(&g_sensor_network_sock, &g_sensor_network_packet,
                    sizeof(sensor_network_packet_t), &remote) > 0) {
    g_sensor_network_busy_sending = TRUE;
  }
  ++g_sensor_network_sequence_number;
}

static void sensor_network_timer_cb(opentimers_id_t id) {
  if (openrandom_get16b() < (0xffff / USENSOR_NETWORK_TRAFFIC_RATE)) {
    sensor_network_task_cb();
  }
}

void usensor_network_init(void) {
  memset(&g_sensor_network_sock, 0, sizeof(sock_udp_t));
  memset(&g_sensor_network_packet, 0, sizeof(sensor_network_packet_t));

  sock_udp_ep_t local = (sock_udp_ep_t){
      .family = AF_INET6,
      .port = WKP_UDP_SENSOR_NETWORK,
  };
  if (sock_udp_create(&g_sensor_network_sock, &local, NULL, 0) < 0) {
    openserial_printf(
        "Failed to create a UDP socket for the sensor network application.\n");
    return;
  }
  openserial_printf(
      "Created a UDP socket for the sensor network application.\n");

  sock_udp_set_cb(&g_sensor_network_sock, sensor_network_sock_handler, NULL);

#ifdef SCUM
  // Configure the ADC.
  adc_config(&g_sensor_network_adc_config);
  adc_enable_interrupt();

  // Configure the sensors.
  sensors_init(&g_sensor_network_sensors_config);
#endif // SCUM

  // Start a periodic timer.
  g_sensor_network_timer_id =
      opentimers_create(TIMER_GENERAL_PURPOSE, TASKPRIO_UDP);
  opentimers_scheduleIn(g_sensor_network_timer_id, USENSOR_NETWORK_PERIOD_MS,
                        TIME_MS, TIMER_PERIODIC, sensor_network_timer_cb);
}

#endif // OPENWSN_USENSOR_NETWORK_C
