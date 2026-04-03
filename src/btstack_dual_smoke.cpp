#include <cstdio>
#include <cstring>

#include "btstack.h"
#include "ble/att_server.h"
#include "ble/gatt-service/nordic_spp_service_server.h"
#include "idrivenew_ble_uart.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#define DUAL_SMOKE_LOG(msg) std::printf("dual_smoke:%d %s\r\n", __LINE__, msg)
#define DUAL_SMOKE_LOGF(fmt, ...) std::printf("dual_smoke:%d " fmt "\r\n", __LINE__, __VA_ARGS__)

namespace {

constexpr uint8_t kRfcommServerChannel = 1;
constexpr size_t kAdvNameMax = 22;
constexpr uint32_t kSppTestClassOfDevice = 0x001F00;

btstack_packet_callback_registration_t g_hci_callback = {};
uint16_t g_rfcomm_cid = 0;
hci_con_handle_t g_ble_con_handle = HCI_CON_HANDLE_INVALID;
bool g_hci_working = false;
uint8_t g_spp_service_buffer[150] = {};

void build_adv_data(const char* name, uint8_t* out, uint8_t* out_len) {
  out[0] = 2;
  out[1] = BLUETOOTH_DATA_TYPE_FLAGS;
  out[2] = 0x02;

  size_t name_len = 0;
  while (name[name_len] && name_len < kAdvNameMax) ++name_len;
  out[3] = static_cast<uint8_t>(name_len + 1);
  out[4] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
  for (size_t i = 0; i < name_len; ++i) {
    out[5 + i] = static_cast<uint8_t>(name[i]);
  }
  *out_len = static_cast<uint8_t>(5 + name_len);
}

void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
  (void)channel;
  (void)size;

  if (packet_type == HCI_EVENT_PACKET) {
    const uint8_t event_type = hci_event_packet_get_type(packet);
    switch (event_type) {
      case BTSTACK_EVENT_STATE: {
        const uint8_t state = btstack_event_state_get_state(packet);
        DUAL_SMOKE_LOGF("btstack state=%u", static_cast<unsigned>(state));
        if (state == HCI_STATE_WORKING) {
          g_hci_working = true;
        }
        break;
      }
      case HCI_EVENT_PIN_CODE_REQUEST: {
        bd_addr_t event_addr;
        hci_event_pin_code_request_get_bd_addr(packet, event_addr);
        DUAL_SMOKE_LOGF("pin code request addr=%s, using 0000", bd_addr_to_str(event_addr));
        gap_pin_code_response(event_addr, "0000");
        break;
      }
      case HCI_EVENT_USER_CONFIRMATION_REQUEST: {
        bd_addr_t event_addr;
        hci_event_user_confirmation_request_get_bd_addr(packet, event_addr);
        const uint32_t numeric_value = hci_event_user_confirmation_request_get_numeric_value(packet);
        DUAL_SMOKE_LOGF("ssp confirmation addr=%s value=%06lu, accepting", bd_addr_to_str(event_addr),
                        static_cast<unsigned long>(numeric_value));
        gap_ssp_confirmation_response(event_addr);
        break;
      }
      case HCI_EVENT_SIMPLE_PAIRING_COMPLETE:
        DUAL_SMOKE_LOGF("simple pairing complete status=0x%02x",
                        static_cast<unsigned>(hci_event_simple_pairing_complete_get_status(packet)));
        break;
      case HCI_EVENT_AUTHENTICATION_COMPLETE_EVENT:
        DUAL_SMOKE_LOGF("authentication complete status=0x%02x handle=0x%04x",
                        static_cast<unsigned>(hci_event_authentication_complete_get_status(packet)),
                        static_cast<unsigned>(hci_event_authentication_complete_get_connection_handle(packet)));
        break;
      case HCI_EVENT_DISCONNECTION_COMPLETE:
        DUAL_SMOKE_LOGF("baseband disconnected handle=0x%04x reason=0x%02x",
                        static_cast<unsigned>(hci_event_disconnection_complete_get_connection_handle(packet)),
                        static_cast<unsigned>(hci_event_disconnection_complete_get_reason(packet)));
        break;
      case RFCOMM_EVENT_INCOMING_CONNECTION:
        g_rfcomm_cid = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
        DUAL_SMOKE_LOGF("incoming classic cid=%u", static_cast<unsigned>(g_rfcomm_cid));
        rfcomm_accept_connection(g_rfcomm_cid);
        break;
      case RFCOMM_EVENT_CHANNEL_OPENED: {
        const uint8_t status = rfcomm_event_channel_opened_get_status(packet);
        if (status != 0) {
          DUAL_SMOKE_LOGF("classic open failed status=0x%02x", static_cast<unsigned>(status));
          g_rfcomm_cid = 0;
        } else {
          g_rfcomm_cid = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
          DUAL_SMOKE_LOGF("classic connected cid=%u mtu=%u", static_cast<unsigned>(g_rfcomm_cid),
                          static_cast<unsigned>(rfcomm_event_channel_opened_get_max_frame_size(packet)));
        }
        break;
      }
      case RFCOMM_EVENT_CHANNEL_CLOSED:
        DUAL_SMOKE_LOG("classic disconnected");
        g_rfcomm_cid = 0;
        break;
      case HCI_EVENT_GATTSERVICE_META:
        switch (hci_event_gattservice_meta_get_subevent_code(packet)) {
          case GATTSERVICE_SUBEVENT_SPP_SERVICE_CONNECTED:
            g_ble_con_handle = gattservice_subevent_spp_service_connected_get_con_handle(packet);
            DUAL_SMOKE_LOGF("ble connected handle=0x%04x", static_cast<unsigned>(g_ble_con_handle));
            break;
          case GATTSERVICE_SUBEVENT_SPP_SERVICE_DISCONNECTED:
            DUAL_SMOKE_LOG("ble disconnected");
            g_ble_con_handle = HCI_CON_HANDLE_INVALID;
            break;
          default:
            break;
        }
        break;
      default:
        break;
    }
    return;
  }

  if (packet_type == RFCOMM_DATA_PACKET) {
    DUAL_SMOKE_LOGF("rx size=%u", static_cast<unsigned>(size));
    if (size != 0) {
      std::printf("dual_smoke:%d rx data=", __LINE__);
      for (uint16_t i = 0; i < size; ++i) {
        std::printf("%02X", static_cast<unsigned>(packet[i]));
        if (i + 1 != size) std::printf(" ");
      }
      std::printf("\r\n");
    }
  }
}

void setup_ble() {
  l2cap_init();
  sm_init();
  DUAL_SMOKE_LOG("security manager initialized");
  att_server_init(profile_data, nullptr, nullptr);
  nordic_spp_service_server_init(packet_handler);
  DUAL_SMOKE_LOG("ble nordic spp service ready");

  uint8_t adv_data[31] = {0};
  uint8_t adv_len = 0;
  build_adv_data("IDRIVE Dual Smoke", adv_data, &adv_len);
  bd_addr_t null_addr = {0, 0, 0, 0, 0, 0};
  gap_advertisements_set_params(0x0030, 0x0030, 0, 0, null_addr, 0x07, 0x00);
  gap_advertisements_set_data(adv_len, adv_data);
  gap_advertisements_enable(1);
  DUAL_SMOKE_LOG("ble advertising enabled");
}

void setup_classic() {
  rfcomm_init();
  sdp_init();
  rfcomm_register_service(packet_handler, kRfcommServerChannel, 0xffff);
  std::memset(g_spp_service_buffer, 0, sizeof(g_spp_service_buffer));
  spp_create_sdp_record(g_spp_service_buffer, sdp_create_service_record_handle(), kRfcommServerChannel,
                        "IDRIVE Dual Smoke");
  const uint16_t sdp_len = de_get_len(g_spp_service_buffer);
  DUAL_SMOKE_LOGF("classic sdp record len=%u", static_cast<unsigned>(sdp_len));
  btstack_assert(sdp_len <= sizeof(g_spp_service_buffer));
  sdp_register_service(g_spp_service_buffer);
  DUAL_SMOKE_LOGF("classic spp registered channel=%u", static_cast<unsigned>(kRfcommServerChannel));
  gap_set_local_name("IDRIVE Dual Smoke 00:00:00:00:00:00");
  gap_set_class_of_device(kSppTestClassOfDevice);
  gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
  gap_discoverable_control(1);
  gap_connectable_control(1);
  DUAL_SMOKE_LOGF("classic discoverable+connectable enabled cod=0x%06lx",
                  static_cast<unsigned long>(kSppTestClassOfDevice));
}

}  // namespace

int main() {
  stdio_init_all();
  sleep_ms(1200);
  DUAL_SMOKE_LOG("stdio ready");

  const int cyw43_rc = cyw43_arch_init();
  DUAL_SMOKE_LOGF("cyw43_arch_init rc=%d", cyw43_rc);
  if (cyw43_rc != 0) {
    while (true) {
      DUAL_SMOKE_LOG("cyw43 init failed");
      sleep_ms(1000);
    }
  }

  g_hci_callback.callback = &packet_handler;
  hci_add_event_handler(&g_hci_callback);
  setup_ble();
  setup_classic();

  const int power_rc = hci_power_control(HCI_POWER_ON);
  DUAL_SMOKE_LOGF("hci_power_control rc=%d", power_rc);

  bool led_on = false;
  absolute_time_t next_toggle = delayed_by_ms(get_absolute_time(), 250);
  absolute_time_t next_status = delayed_by_ms(get_absolute_time(), 1000);

  while (true) {
    cyw43_arch_poll();
    const absolute_time_t now = get_absolute_time();
    const bool any_connected = (g_ble_con_handle != HCI_CON_HANDLE_INVALID) || (g_rfcomm_cid != 0);

    if (absolute_time_diff_us(now, next_toggle) <= 0) {
      led_on = !led_on;
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on ? 1 : 0);
      next_toggle = delayed_by_ms(now, any_connected ? 125 : 500);
    }

    if (absolute_time_diff_us(now, next_status) <= 0) {
      DUAL_SMOKE_LOGF("alive hci=%d ble=0x%04x classic=%u", g_hci_working ? 1 : 0,
                      static_cast<unsigned>(g_ble_con_handle), static_cast<unsigned>(g_rfcomm_cid));
      next_status = delayed_by_ms(now, 1000);
    }

    tight_loop_contents();
  }
}
