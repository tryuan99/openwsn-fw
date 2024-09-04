#include "config.h"

#if OPENWSN_USENSOR_NETWORK_C

#include "usensor_network.h"

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

// Sensor network packet period in milliseconds.
#define USENSOR_NETWORK_PERIOD_MS 3000

// Sensor network traffic rate.
#define USENSOR_NETWORK_TRAFFIC_RATE 1

// Sensor network maximum buffer size in bytes.
#define USENSOR_NETWORK_MAX_BUFFER_SIZE 50

// Sensor payload.
typedef struct __attribute__((packed)) {
  uint16_t data;
} sensor_network_payload_t;

// Data packet.
typedef struct __attribute__((packed)) {
  // 16-bit source address.
  uint16_t addr_16b;

  // Payload.
  sensor_network_payload_t payload;
} sensor_network_packet_t;

// Data packet.
static sensor_network_packet_t g_sensor_network_packet;

// Counter.
static uint16_t g_sensor_network_counter = 0;

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
    int i = 0;
    if ((res = sock_udp_recv(sock, buffer, USENSOR_NETWORK_MAX_BUFFER_SIZE, 0,
                             &remote)) >= 0) {
      openserial_printf("Received %d bytes from remote endpoint:\n", res);
      openserial_printf(" - port: %d", remote.port);
      openserial_printf(" - addr: ", remote.port);
      for (i = 0; i < 16; ++i)
        openserial_printf("%x ", remote.addr.ipv6[i]);

      openserial_printf("\n\n");
      openserial_printf("Msg received: %s\n\n", buffer);
    }
  }

  if (type & SOCK_ASYNC_MSG_SENT) {
    owerror_t error = *(uint8_t *)arg;
    if (error == E_FAIL) {
      LOG_ERROR(COMPONENT_USENSOR_NETWORK, ERR_MAXRETRIES_REACHED,
                (errorparameter_t)g_sensor_network_counter,
                (errorparameter_t)0);
    }
    g_sensor_network_busy_sending = FALSE;
  }
}

static inline void sensor_network_task_cb(void) {
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

  sock_udp_ep_t remote = (sock_udp_ep_t){
      .family = AF_INET6,
      .port = WKP_UDP_SENSOR_NETWORK,
  };
  memcpy(remote.addr.ipv6, g_sensor_network_destination_address,
         sizeof(g_sensor_network_destination_address));
  memset(&g_sensor_network_packet, 0, sizeof(sensor_network_packet_t));
  memcpy(&g_sensor_network_packet.addr_16b,
         idmanager_getMyID(ADDR_16B)->addr_16b, sizeof(uint16_t));
  g_sensor_network_packet.payload.data = g_sensor_network_counter;

  if (sock_udp_send(&g_sensor_network_sock, &g_sensor_network_packet,
                    sizeof(sensor_network_packet_t), &remote) > 0) {
    g_sensor_network_busy_sending = TRUE;
  }
  ++g_sensor_network_counter;
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

  // Start a periodic timer.
  g_sensor_network_timer_id =
      opentimers_create(TIMER_GENERAL_PURPOSE, TASKPRIO_UDP);
  opentimers_scheduleIn(g_sensor_network_timer_id, USENSOR_NETWORK_PERIOD_MS,
                        TIME_MS, TIMER_PERIODIC, sensor_network_timer_cb);
}

#endif // OPENWSN_USENSOR_NETWORK_C
