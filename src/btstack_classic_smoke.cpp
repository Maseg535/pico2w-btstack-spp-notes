#include <cstdio>
#include <cstring>

#include "btstack.h"
#include "hci_dump_embedded_stdout.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#define CLASSIC_SMOKE_LOG(msg) std::printf("classic_smoke:%d %s\r\n", __LINE__, msg)
#define CLASSIC_SMOKE_LOGF(fmt, ...) std::printf("classic_smoke:%d " fmt "\r\n", __LINE__, __VA_ARGS__)

namespace {

constexpr uint8_t kRfcommServerChannel = 1;
constexpr size_t kEchoBufferSize = 256;
constexpr uint32_t kSppTestClassOfDevice = 0x001F00;

btstack_packet_callback_registration_t g_hci_callback = {};
uint16_t g_rfcomm_cid = 0;
bool g_hci_working = false;
bool g_can_send_pending = false;
char g_echo_buffer[kEchoBufferSize] = {};
uint16_t g_echo_len = 0;
uint8_t g_spp_service_buffer[150] = {};
uint8_t g_spp_attr_all_buffer[] = {0x35, 0x05, 0x0A, 0x00, 0x00, 0xFF, 0xFF};

void log_spp_match_state(const char* label) {
  uint8_t* spp_search_pattern = sdp_service_search_pattern_for_uuid16(BLUETOOTH_SERVICE_CLASS_SERIAL_PORT);
  const bool spp_match = sdp_record_matches_service_search_pattern(g_spp_service_buffer, spp_search_pattern);
  const uint16_t filtered_size = sdp_get_filtered_size(g_spp_service_buffer, g_spp_attr_all_buffer);
  const uint32_t handle = sdp_get_service_record_handle(g_spp_service_buffer);
  CLASSIC_SMOKE_LOGF("%s spp_match=%u filtered_size=%u handle=0x%08lx", label, spp_match ? 1u : 0u,
                     static_cast<unsigned>(filtered_size), static_cast<unsigned long>(handle));
}

void log_sdp_record(const uint8_t* data, uint16_t len) {
  std::printf("classic_smoke:%d sdp record bytes=", __LINE__);
  for (uint16_t i = 0; i < len; ++i) {
    std::printf("%02X", static_cast<unsigned>(data[i]));
    if (i + 1 != len) std::printf(" ");
  }
  std::printf("\r\n");
}

void request_can_send_now() {
  if (g_rfcomm_cid == 0 || g_can_send_pending || g_echo_len == 0) return;
  g_can_send_pending = true;
  rfcomm_request_can_send_now_event(g_rfcomm_cid);
}

void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
  (void)channel;

  if (packet_type == HCI_EVENT_PACKET) {
    switch (hci_event_packet_get_type(packet)) {
      case BTSTACK_EVENT_STATE: {
        const uint8_t state = btstack_event_state_get_state(packet);
        CLASSIC_SMOKE_LOGF("btstack state=%u", static_cast<unsigned>(state));
        if (state == HCI_STATE_WORKING) {
          g_hci_working = true;
        }
        break;
      }
      case HCI_EVENT_PIN_CODE_REQUEST: {
        bd_addr_t event_addr;
        hci_event_pin_code_request_get_bd_addr(packet, event_addr);
        CLASSIC_SMOKE_LOGF("pin code request addr=%s, using 0000", bd_addr_to_str(event_addr));
        gap_pin_code_response(event_addr, "0000");
        break;
      }
      case HCI_EVENT_USER_CONFIRMATION_REQUEST: {
        bd_addr_t event_addr;
        hci_event_user_confirmation_request_get_bd_addr(packet, event_addr);
        const uint32_t numeric_value = hci_event_user_confirmation_request_get_numeric_value(packet);
        CLASSIC_SMOKE_LOGF("ssp confirmation addr=%s value=%06lu, accepting", bd_addr_to_str(event_addr),
                           static_cast<unsigned long>(numeric_value));
        gap_ssp_confirmation_response(event_addr);
        break;
      }
      case HCI_EVENT_SIMPLE_PAIRING_COMPLETE:
        CLASSIC_SMOKE_LOGF("simple pairing complete status=0x%02x",
                           static_cast<unsigned>(hci_event_simple_pairing_complete_get_status(packet)));
        break;
      case HCI_EVENT_CONNECTION_COMPLETE: {
        bd_addr_t event_addr;
        hci_event_connection_complete_get_bd_addr(packet, event_addr);
        CLASSIC_SMOKE_LOGF("baseband connected status=0x%02x handle=0x%04x addr=%s",
                           static_cast<unsigned>(hci_event_connection_complete_get_status(packet)),
                           static_cast<unsigned>(hci_event_connection_complete_get_connection_handle(packet)),
                           bd_addr_to_str(event_addr));
        log_spp_match_state("on_acl_open");
        break;
      }
      case HCI_EVENT_AUTHENTICATION_COMPLETE_EVENT:
        CLASSIC_SMOKE_LOGF("authentication complete status=0x%02x handle=0x%04x",
                           static_cast<unsigned>(hci_event_authentication_complete_get_status(packet)),
                           static_cast<unsigned>(hci_event_authentication_complete_get_connection_handle(packet)));
        break;
      case HCI_EVENT_DISCONNECTION_COMPLETE:
        CLASSIC_SMOKE_LOGF("baseband disconnected handle=0x%04x reason=0x%02x",
                           static_cast<unsigned>(hci_event_disconnection_complete_get_connection_handle(packet)),
                           static_cast<unsigned>(hci_event_disconnection_complete_get_reason(packet)));
        break;
      case RFCOMM_EVENT_INCOMING_CONNECTION: {
        bd_addr_t event_addr;
        rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
        g_rfcomm_cid = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
        const uint8_t server_channel = rfcomm_event_incoming_connection_get_server_channel(packet);
        CLASSIC_SMOKE_LOGF("incoming rfcomm channel=%u addr=%s", static_cast<unsigned>(server_channel),
                           bd_addr_to_str(event_addr));
        rfcomm_accept_connection(g_rfcomm_cid);
        break;
      }
      case RFCOMM_EVENT_CHANNEL_OPENED: {
        const uint8_t status = rfcomm_event_channel_opened_get_status(packet);
        if (status != 0) {
          CLASSIC_SMOKE_LOGF("rfcomm open failed status=0x%02x", static_cast<unsigned>(status));
          g_rfcomm_cid = 0;
          g_can_send_pending = false;
          g_echo_len = 0;
        } else {
          g_rfcomm_cid = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
          const uint16_t mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
          CLASSIC_SMOKE_LOGF("rfcomm open ok cid=%u mtu=%u", static_cast<unsigned>(g_rfcomm_cid),
                             static_cast<unsigned>(mtu));
        }
        break;
      }
      case RFCOMM_EVENT_CAN_SEND_NOW:
        g_can_send_pending = false;
        if (g_rfcomm_cid != 0 && g_echo_len != 0) {
          rfcomm_send(g_rfcomm_cid, reinterpret_cast<uint8_t*>(g_echo_buffer), g_echo_len);
          g_echo_len = 0;
        }
        break;
      case RFCOMM_EVENT_CHANNEL_CLOSED:
        CLASSIC_SMOKE_LOG("rfcomm channel closed");
        g_rfcomm_cid = 0;
        g_can_send_pending = false;
        g_echo_len = 0;
        break;
      case L2CAP_EVENT_INCOMING_CONNECTION: {
        bd_addr_t event_addr;
        l2cap_event_incoming_connection_get_address(packet, event_addr);
        CLASSIC_SMOKE_LOGF("l2cap incoming addr=%s handle=0x%04x psm=0x%04x local_cid=0x%04x remote_cid=0x%04x",
                           bd_addr_to_str(event_addr),
                           static_cast<unsigned>(l2cap_event_incoming_connection_get_handle(packet)),
                           static_cast<unsigned>(l2cap_event_incoming_connection_get_psm(packet)),
                           static_cast<unsigned>(l2cap_event_incoming_connection_get_local_cid(packet)),
                           static_cast<unsigned>(l2cap_event_incoming_connection_get_remote_cid(packet)));
        if (l2cap_event_incoming_connection_get_psm(packet) == BLUETOOTH_PSM_SDP) {
          log_spp_match_state("on_sdp_incoming");
        }
        break;
      }
      case L2CAP_EVENT_CHANNEL_OPENED: {
        bd_addr_t event_addr;
        l2cap_event_channel_opened_get_address(packet, event_addr);
        CLASSIC_SMOKE_LOGF("l2cap opened status=0x%02x addr=%s handle=0x%04x psm=0x%04x local_cid=0x%04x remote_cid=0x%04x",
                           static_cast<unsigned>(l2cap_event_channel_opened_get_status(packet)),
                           bd_addr_to_str(event_addr),
                           static_cast<unsigned>(l2cap_event_channel_opened_get_handle(packet)),
                           static_cast<unsigned>(l2cap_event_channel_opened_get_psm(packet)),
                           static_cast<unsigned>(l2cap_event_channel_opened_get_local_cid(packet)),
                           static_cast<unsigned>(l2cap_event_channel_opened_get_remote_cid(packet)));
        break;
      }
      case L2CAP_EVENT_CHANNEL_CLOSED:
        CLASSIC_SMOKE_LOGF("l2cap closed local_cid=0x%04x",
                           static_cast<unsigned>(l2cap_event_channel_closed_get_local_cid(packet)));
        break;
      default:
        break;
    }
    return;
  }

  if (packet_type == RFCOMM_DATA_PACKET) {
    const uint16_t to_copy = size < (kEchoBufferSize - 2) ? size : static_cast<uint16_t>(kEchoBufferSize - 2);
    std::memcpy(g_echo_buffer, packet, to_copy);
    g_echo_buffer[to_copy] = '\r';
    g_echo_buffer[to_copy + 1] = '\n';
    g_echo_len = static_cast<uint16_t>(to_copy + 2);
    CLASSIC_SMOKE_LOGF("rfcomm rx size=%u", static_cast<unsigned>(size));
    request_can_send_now();
  }
}

void setup_classic_spp() {
  g_hci_callback.callback = &packet_handler;
  hci_add_event_handler(&g_hci_callback);

  l2cap_init();
  rfcomm_init();
  sdp_init();

  rfcomm_register_service(packet_handler, kRfcommServerChannel, 0xffff);
  std::memset(g_spp_service_buffer, 0, sizeof(g_spp_service_buffer));
  spp_create_sdp_record(g_spp_service_buffer, sdp_create_service_record_handle(), kRfcommServerChannel,
                        "IDRIVE Classic Smoke");
  const uint16_t sdp_len = de_get_len(g_spp_service_buffer);
  CLASSIC_SMOKE_LOGF("classic sdp record len=%u", static_cast<unsigned>(sdp_len));
  btstack_assert(sdp_len <= sizeof(g_spp_service_buffer));
  log_sdp_record(g_spp_service_buffer, sdp_len);
  log_spp_match_state("before_register");
  const uint8_t register_status = sdp_register_service(g_spp_service_buffer);
  CLASSIC_SMOKE_LOGF("sdp_register_service status=0x%02x", static_cast<unsigned>(register_status));
  btstack_assert(register_status == 0);
  log_spp_match_state("after_register");
  CLASSIC_SMOKE_LOGF("classic spp registered channel=%u", static_cast<unsigned>(kRfcommServerChannel));

  gap_set_local_name("IDRIVE Classic Smoke 00:00:00:00:00:00");
  gap_set_class_of_device(kSppTestClassOfDevice);
  gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
  gap_discoverable_control(1);
  gap_connectable_control(1);
  CLASSIC_SMOKE_LOGF("classic discoverable+connectable enabled cod=0x%06lx",
                     static_cast<unsigned long>(kSppTestClassOfDevice));
}

}  // namespace

int main() {
  stdio_init_all();
  sleep_ms(1200);
  CLASSIC_SMOKE_LOG("stdio ready");

  const int cyw43_rc = cyw43_arch_init();
  CLASSIC_SMOKE_LOGF("cyw43_arch_init rc=%d", cyw43_rc);
  if (cyw43_rc != 0) {
    while (true) {
      CLASSIC_SMOKE_LOG("cyw43 init failed");
      sleep_ms(1000);
    }
  }

  hci_dump_init(hci_dump_embedded_stdout_get_instance());
  hci_dump_enable_packet_log(true);
  hci_dump_enable_log_level(HCI_DUMP_LOG_LEVEL_INFO, 1);
  hci_dump_enable_log_level(HCI_DUMP_LOG_LEVEL_ERROR, 1);
  CLASSIC_SMOKE_LOG("hci dump enabled");

  setup_classic_spp();
  const int power_rc = hci_power_control(HCI_POWER_ON);
  CLASSIC_SMOKE_LOGF("hci_power_control rc=%d", power_rc);

  bool led_on = false;
  absolute_time_t next_toggle = delayed_by_ms(get_absolute_time(), 250);
  absolute_time_t next_status = delayed_by_ms(get_absolute_time(), 1000);

  while (true) {
    cyw43_arch_poll();
    const absolute_time_t now = get_absolute_time();

    if (absolute_time_diff_us(now, next_toggle) <= 0) {
      led_on = !led_on;
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on ? 1 : 0);
      next_toggle = delayed_by_ms(now, g_rfcomm_cid != 0 ? 1000 : 250);
    }

    if (absolute_time_diff_us(now, next_status) <= 0) {
      CLASSIC_SMOKE_LOGF("alive hci=%d rfcomm=%u", g_hci_working ? 1 : 0, static_cast<unsigned>(g_rfcomm_cid));
      next_status = delayed_by_ms(now, 1000);
    }

    tight_loop_contents();
  }
}
