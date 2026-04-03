#include "idrivenew/transport/transport_backend.h"

#ifdef IDRIVE_ENABLE_BTSTACK

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "btstack.h"
#include "ble/att_server.h"
#include "ble/gatt-service/nordic_spp_service_server.h"
#include "idrivenew/protocol/transport_frame.h"
#include "idrivenew/transport/transport.h"
#include "idrivenew_ble_uart.h"

namespace idrivenew {
namespace {

static constexpr const char* kDefaultBtName = "Idrive";
static constexpr uint8_t kRfcommServerChannel = 1;
static constexpr uint32_t kClassicClassOfDevice = 0x001F00;

enum class BackendState : uint8_t {
  kOff = 0,
  kIdle,
  kAdvertisingOrListening,
  kConnected,
};

struct TxItem {
  uint16_t len = 0;
  uint16_t offset = 0;
  uint8_t data[kTransportMaxPayload] = {0};
};

struct TxQueueState {
  static constexpr uint8_t kDepth = 32;
  TxItem items[kDepth] = {};
  uint8_t head = 0;
  uint8_t tail = 0;
  uint32_t dropped = 0;
};

struct BleRuntime {
  BackendState state = BackendState::kOff;
  hci_con_handle_t con_handle = HCI_CON_HANDLE_INVALID;
  btstack_context_callback_registration_t can_send_request = {};
  bool can_send_request_pending = false;
  TxQueueState tx = {};
};

#ifdef IDRIVE_BT_CLASSIC
struct ClassicRuntime {
  BackendState state = BackendState::kOff;
  uint16_t rfcomm_cid = 0;
  uint16_t mtu = 0;
  bool can_send_request_pending = false;
  TxQueueState tx = {};
  uint8_t spp_service_buffer[150] = {};
};
#endif

struct BackendRuntime {
  bool hci_working = false;
  btstack_packet_callback_registration_t hci_event_callback_registration = {};
  BleRuntime ble = {};
#ifdef IDRIVE_BT_CLASSIC
  ClassicRuntime classic = {};
#endif
};

static BackendRuntime g_bt = {};

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);
static void on_transport_rx_data(uint8_t* packet, uint16_t size, TransportLink link);

static uint8_t rb_next(uint8_t v, uint8_t size) { return static_cast<uint8_t>((v + 1u) % size); }

static bool tx_queue_empty(const TxQueueState& queue) { return queue.head == queue.tail; }

static bool tx_queue_full(const TxQueueState& queue) {
  return rb_next(queue.head, TxQueueState::kDepth) == queue.tail;
}

static void tx_queue_reset(TxQueueState* queue) {
  if (!queue) return;
  queue->head = 0;
  queue->tail = 0;
  queue->dropped = 0;
}

static bool tx_queue_push_raw(TxQueueState* queue, const uint8_t* data, uint16_t len) {
  if (!queue || !data || len == 0) return false;
  if (tx_queue_full(*queue)) {
    ++queue->dropped;
    return false;
  }
  TxItem& item = queue->items[queue->head];
  item.len = len;
  item.offset = 0;
  std::memcpy(item.data, data, len);
  queue->head = rb_next(queue->head, TxQueueState::kDepth);
  return true;
}

static bool tx_queue_push_text_line(TxQueueState* queue, const char* text) {
  if (!queue || !text || !*text) return false;
  uint8_t line[kTransportMaxPayload] = {0};
  size_t len = 0;
  while (text[len] && len + 2u < sizeof(line)) {
    line[len] = static_cast<uint8_t>(text[len]);
    ++len;
  }
  line[len++] = '\r';
  line[len++] = '\n';
  return tx_queue_push_raw(queue, line, static_cast<uint16_t>(len));
}

static bool tx_queue_peek(TxQueueState* queue, TxItem** out) {
  if (!queue || !out || tx_queue_empty(*queue)) return false;
  *out = &queue->items[queue->tail];
  return true;
}

static bool tx_queue_discard_front(TxQueueState* queue) {
  if (!queue || tx_queue_empty(*queue)) return false;
  queue->tail = rb_next(queue->tail, TxQueueState::kDepth);
  return true;
}

static bool queue_status_payload(TxQueueState* queue, const uint8_t* data, size_t len) {
  if (!queue || !data || len == 0 || len > kTransportMaxPayload) return false;
  if (queue->dropped > 0) {
    char warn[32] = {0};
    std::snprintf(warn, sizeof(warn), "warn: dropped=%lu", static_cast<unsigned long>(queue->dropped));
    queue->dropped = 0;
    (void)tx_queue_push_text_line(queue, warn);
  }
  return tx_queue_push_raw(queue, data, static_cast<uint16_t>(len));
}

static void request_ble_can_send_now();
#ifdef IDRIVE_BT_CLASSIC
static void request_classic_can_send_now();
#endif

static void feed_text_command(const uint8_t* data, uint16_t len, TransportLink link) {
  if (!data || len == 0) return;
  const uint8_t payload_len = static_cast<uint8_t>(len > kTransportMaxPayload ? kTransportMaxPayload : len);
  uint8_t frame[2 + kTransportMaxPayload] = {0};
  frame[0] = static_cast<uint8_t>(MsgType::kCmd);
  frame[1] = payload_len;
  std::memcpy(frame + 2, data, payload_len);
  transport_on_rx_bytes(frame, static_cast<size_t>(payload_len + 2), link);
}

static void on_ble_can_send_now(void* context) {
  (void)context;
  g_bt.ble.can_send_request_pending = false;

  if (g_bt.ble.state != BackendState::kConnected || g_bt.ble.con_handle == HCI_CON_HANDLE_INVALID) return;

  TxItem* item = nullptr;
  if (!tx_queue_peek(&g_bt.ble.tx, &item)) return;

  uint16_t mtu = att_server_get_mtu(g_bt.ble.con_handle);
  if (mtu < 3u) mtu = 23u;
  const uint16_t chunk_cap = static_cast<uint16_t>(mtu - 3u);
  const uint16_t remaining = static_cast<uint16_t>(item->len - item->offset);
  const uint16_t chunk_len = (remaining < chunk_cap) ? remaining : chunk_cap;

  const int rc = nordic_spp_service_server_send(g_bt.ble.con_handle, item->data + item->offset, chunk_len);
  if (rc != 0) {
    std::printf("bt: ble send rc=%d len=%u off=%u mtu=%u\r\n", rc, static_cast<unsigned>(chunk_len),
                static_cast<unsigned>(item->offset), static_cast<unsigned>(mtu));
    request_ble_can_send_now();
    return;
  }

  item->offset = static_cast<uint16_t>(item->offset + chunk_len);
  if (item->offset >= item->len) {
    (void)tx_queue_discard_front(&g_bt.ble.tx);
  }

  request_ble_can_send_now();
}

static void request_ble_can_send_now() {
  if (g_bt.ble.state != BackendState::kConnected || g_bt.ble.con_handle == HCI_CON_HANDLE_INVALID) return;
  if (tx_queue_empty(g_bt.ble.tx) || g_bt.ble.can_send_request_pending) return;
  g_bt.ble.can_send_request.callback = &on_ble_can_send_now;
  g_bt.ble.can_send_request_pending = true;
  nordic_spp_service_server_request_can_send_now(&g_bt.ble.can_send_request, g_bt.ble.con_handle);
}

static void ble_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
  (void)channel;
  if (packet_type == HCI_EVENT_PACKET) {
    packet_handler(packet_type, channel, packet, size);
    return;
  }
  if (packet_type == RFCOMM_DATA_PACKET) {
    on_transport_rx_data(packet, size, TransportLink::kBle);
  }
}

#ifdef IDRIVE_BT_CLASSIC
static void on_classic_can_send_now() {
  g_bt.classic.can_send_request_pending = false;
  if (g_bt.classic.state != BackendState::kConnected || g_bt.classic.rfcomm_cid == 0) return;

  TxItem* item = nullptr;
  if (!tx_queue_peek(&g_bt.classic.tx, &item)) return;

  if (!rfcomm_can_send_packet_now(g_bt.classic.rfcomm_cid)) {
    request_classic_can_send_now();
    return;
  }

  const uint16_t remaining = static_cast<uint16_t>(item->len - item->offset);
  uint8_t* send_ptr = item->data + item->offset;
  uint16_t send_len = remaining;
  if (g_bt.classic.mtu != 0 && send_len > g_bt.classic.mtu) {
    send_len = g_bt.classic.mtu;
  }

  const uint8_t rc = rfcomm_send(g_bt.classic.rfcomm_cid, send_ptr, send_len);
  if (rc != 0) {
    std::printf("bt: classic send rc=%u len=%u off=%u mtu=%u\r\n", static_cast<unsigned>(rc),
                static_cast<unsigned>(send_len), static_cast<unsigned>(item->offset),
                static_cast<unsigned>(g_bt.classic.mtu));
    request_classic_can_send_now();
    return;
  }

  item->offset = static_cast<uint16_t>(item->offset + send_len);
  if (item->offset >= item->len) {
    (void)tx_queue_discard_front(&g_bt.classic.tx);
  }
  request_classic_can_send_now();
}

static void request_classic_can_send_now() {
  if (g_bt.classic.state != BackendState::kConnected || g_bt.classic.rfcomm_cid == 0) return;
  if (tx_queue_empty(g_bt.classic.tx) || g_bt.classic.can_send_request_pending) return;
  g_bt.classic.can_send_request_pending = true;
  (void)rfcomm_request_can_send_now_event(g_bt.classic.rfcomm_cid);
}
#endif

static void build_adv_data(const char* name, uint8_t* out, uint8_t* out_len) {
  if (!out || !out_len) return;
  out[0] = 2;
  out[1] = BLUETOOTH_DATA_TYPE_FLAGS;
  out[2] = 0x06;

  size_t name_len = 0;
  if (name) {
    while (name[name_len] && name_len < 26) ++name_len;
  }

  out[3] = static_cast<uint8_t>(name_len + 1);
  out[4] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
  for (size_t i = 0; i < name_len; ++i) {
    out[5 + i] = static_cast<uint8_t>(name[i]);
  }
  *out_len = static_cast<uint8_t>(5 + name_len);
}

static void on_transport_rx_data(uint8_t* packet, uint16_t size, TransportLink link) {
  if (!packet || size == 0) return;
  if (size >= 2 && packet[1] == static_cast<uint8_t>(size - 2) &&
      (packet[0] == static_cast<uint8_t>(MsgType::kCmd) || packet[0] == static_cast<uint8_t>(MsgType::kCanTx))) {
    transport_on_rx_bytes(packet, size, link);
  } else {
    feed_text_command(packet, size, link);
  }
}

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
  (void)channel;

  if (packet_type == HCI_EVENT_PACKET) {
    const uint8_t event_type = hci_event_packet_get_type(packet);
    switch (event_type) {
      case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
          g_bt.hci_working = true;
          g_bt.ble.state = BackendState::kAdvertisingOrListening;
#ifdef IDRIVE_BT_CLASSIC
          g_bt.classic.state = BackendState::kAdvertisingOrListening;
#endif
          std::printf("bt: hci working, dual transport active\r\n");
        }
        break;

      case HCI_EVENT_GATTSERVICE_META:
        switch (hci_event_gattservice_meta_get_subevent_code(packet)) {
          case GATTSERVICE_SUBEVENT_SPP_SERVICE_CONNECTED:
            g_bt.ble.con_handle = gattservice_subevent_spp_service_connected_get_con_handle(packet);
            g_bt.ble.state = BackendState::kConnected;
            g_bt.ble.can_send_request_pending = false;
            std::printf("bt: ble client connected handle=0x%04x\r\n", g_bt.ble.con_handle);
            request_ble_can_send_now();
            break;
          case GATTSERVICE_SUBEVENT_SPP_SERVICE_DISCONNECTED:
            std::printf("bt: ble client disconnected\r\n");
            g_bt.ble.con_handle = HCI_CON_HANDLE_INVALID;
            g_bt.ble.state = BackendState::kAdvertisingOrListening;
            g_bt.ble.can_send_request_pending = false;
            tx_queue_reset(&g_bt.ble.tx);
            break;
          default:
            break;
        }
        break;

#ifdef IDRIVE_BT_CLASSIC
      case HCI_EVENT_PIN_CODE_REQUEST: {
        bd_addr_t event_addr;
        hci_event_pin_code_request_get_bd_addr(packet, event_addr);
        gap_pin_code_response(event_addr, "0000");
        break;
      }
      case HCI_EVENT_USER_CONFIRMATION_REQUEST: {
        bd_addr_t event_addr;
        hci_event_user_confirmation_request_get_bd_addr(packet, event_addr);
        gap_ssp_confirmation_response(event_addr);
        break;
      }
      case RFCOMM_EVENT_INCOMING_CONNECTION:
        g_bt.classic.rfcomm_cid = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
        std::printf("bt: classic incoming cid=%u\r\n", static_cast<unsigned>(g_bt.classic.rfcomm_cid));
        rfcomm_accept_connection(g_bt.classic.rfcomm_cid);
        break;
      case RFCOMM_EVENT_CHANNEL_OPENED: {
        const uint8_t status = rfcomm_event_channel_opened_get_status(packet);
        if (status != 0) {
          std::printf("bt: classic open failed status=0x%02x\r\n", static_cast<unsigned>(status));
          g_bt.classic.rfcomm_cid = 0;
          g_bt.classic.mtu = 0;
          g_bt.classic.state = BackendState::kAdvertisingOrListening;
        } else {
          g_bt.classic.rfcomm_cid = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
          g_bt.classic.mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
          g_bt.classic.state = BackendState::kConnected;
          g_bt.classic.can_send_request_pending = false;
          std::printf("bt: classic connected cid=%u mtu=%u\r\n", static_cast<unsigned>(g_bt.classic.rfcomm_cid),
                      static_cast<unsigned>(g_bt.classic.mtu));
          request_classic_can_send_now();
        }
        break;
      }
      case RFCOMM_EVENT_CAN_SEND_NOW:
        on_classic_can_send_now();
        break;
      case RFCOMM_EVENT_CHANNEL_CLOSED:
        std::printf("bt: classic disconnected\r\n");
        g_bt.classic.rfcomm_cid = 0;
        g_bt.classic.mtu = 0;
        g_bt.classic.state = BackendState::kAdvertisingOrListening;
        g_bt.classic.can_send_request_pending = false;
        tx_queue_reset(&g_bt.classic.tx);
        break;
#endif
      default:
        break;
    }
    return;
  }

  if (packet_type == RFCOMM_DATA_PACKET) {
    on_transport_rx_data(packet, size, TransportLink::kClassic);
  }
}

static bool init_ble_uart(const BtConfig& cfg) {
  const char* local_name = (cfg.name[0] != '\0') ? cfg.name : kDefaultBtName;
  std::printf("bt: init ble name=%s\r\n", local_name);

  g_bt.ble.con_handle = HCI_CON_HANDLE_INVALID;
  g_bt.ble.state = BackendState::kIdle;
  g_bt.ble.can_send_request_pending = false;
  tx_queue_reset(&g_bt.ble.tx);

  att_server_init(profile_data, nullptr, nullptr);
  nordic_spp_service_server_init(&ble_packet_handler);

  uint8_t adv_data[31] = {0};
  uint8_t adv_len = 0;
  build_adv_data(local_name, adv_data, &adv_len);
  bd_addr_t null_addr = {0, 0, 0, 0, 0, 0};
  gap_advertisements_set_params(0x0030, 0x0030, 0, 0, null_addr, 0x07, 0x00);
  gap_advertisements_set_data(adv_len, adv_data);
  gap_advertisements_enable(1);
  std::printf("bt: ble advertising configured len=%u\r\n", static_cast<unsigned>(adv_len));
  return true;
}

#ifdef IDRIVE_BT_CLASSIC
static bool init_classic_spp(const BtConfig& cfg) {
  const char* local_name = (cfg.name[0] != '\0') ? cfg.name : kDefaultBtName;
  std::printf("bt: init classic spp name=%s\r\n", local_name);

  g_bt.classic.rfcomm_cid = 0;
  g_bt.classic.mtu = 0;
  g_bt.classic.state = BackendState::kIdle;
  g_bt.classic.can_send_request_pending = false;
  tx_queue_reset(&g_bt.classic.tx);

  rfcomm_init();
  sdp_init();
  rfcomm_register_service(packet_handler, kRfcommServerChannel, 0xffff);

  std::memset(g_bt.classic.spp_service_buffer, 0, sizeof(g_bt.classic.spp_service_buffer));
  spp_create_sdp_record(g_bt.classic.spp_service_buffer, sdp_create_service_record_handle(), kRfcommServerChannel,
                        local_name);
  btstack_assert(de_get_len(g_bt.classic.spp_service_buffer) <= sizeof(g_bt.classic.spp_service_buffer));
  sdp_register_service(g_bt.classic.spp_service_buffer);

  gap_set_local_name("IDRIVE_NEW 00:00:00:00:00:00");
  gap_set_class_of_device(kClassicClassOfDevice);
  gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
  gap_discoverable_control(1);
  gap_connectable_control(1);
  std::printf("bt: classic spp ready channel=%u\r\n", static_cast<unsigned>(kRfcommServerChannel));
  return true;
}
#endif

static bool send_status_frame(TxQueueState* queue, const uint8_t* data, size_t len) {
  if (!data || len < 2) return false;
  const uint8_t frame_type = data[0];
  const uint8_t payload_len = data[1];
  if (len < static_cast<size_t>(payload_len + 2u)) return false;
  if (static_cast<MsgType>(frame_type) != MsgType::kStatus) return false;
  return queue_status_payload(queue, data + 2, payload_len);
}

}  // namespace

bool transport_backend_init(const BtConfig& cfg) {
  g_bt = {};
  g_bt.hci_event_callback_registration.callback = &packet_handler;
  hci_add_event_handler(&g_bt.hci_event_callback_registration);

  l2cap_init();
  sm_init();

  if (!init_ble_uart(cfg)) return false;
#ifdef IDRIVE_BT_CLASSIC
  if (!init_classic_spp(cfg)) return false;
#endif

  const int power_rc = hci_power_control(HCI_POWER_ON);
  std::printf("bt: hci_power_control rc=%d\r\n", power_rc);
  return power_rc == 0;
}

bool transport_backend_send_bytes(const uint8_t* data, size_t len) {
  bool accepted = false;
  if (send_status_frame(&g_bt.ble.tx, data, len)) {
    accepted = true;
    request_ble_can_send_now();
  }
#ifdef IDRIVE_BT_CLASSIC
  if (send_status_frame(&g_bt.classic.tx, data, len)) {
    accepted = true;
    request_classic_can_send_now();
  }
#endif
  return accepted;
}

bool transport_backend_send_bytes_ble(const uint8_t* data, size_t len) {
  if (!send_status_frame(&g_bt.ble.tx, data, len)) return false;
  request_ble_can_send_now();
  return true;
}

bool transport_backend_send_bytes_classic(const uint8_t* data, size_t len) {
#ifdef IDRIVE_BT_CLASSIC
  if (!send_status_frame(&g_bt.classic.tx, data, len)) return false;
  request_classic_can_send_now();
  return true;
#else
  (void)data;
  (void)len;
  return false;
#endif
}

void transport_backend_poll() {}

bool transport_backend_connected() {
  return transport_backend_ble_connected() || transport_backend_classic_connected();
}

bool transport_backend_ble_connected() { return g_bt.ble.state == BackendState::kConnected; }

bool transport_backend_classic_connected() {
#ifdef IDRIVE_BT_CLASSIC
  return g_bt.classic.state == BackendState::kConnected;
#else
  return false;
#endif
}

}  // namespace idrivenew

#endif  // IDRIVE_ENABLE_BTSTACK
