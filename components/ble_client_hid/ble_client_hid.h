#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#ifdef USE_ESP32
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#endif

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hid_report_map.h"  // provided by the component repo
#include "usages.h"          // provided by the component repo

namespace esphome {
namespace ble_client_hid {

enum class HIDState : uint8_t {
  IDLE = 0,
  NO_HID_SERVICE,
  HID_SERVICE_FOUND,
  BLE_CONNECTED,
  READING_CHARS,
  READ_CHARS,
  NOTIFICATIONS_REGISTERING,
  NOTIFICATIONS_REGISTERED,
  CONN_PARAMS_UPDATING,
  CONFIGURED,
};

struct PendingNotify {
  uint16_t handle{0};
  std::vector<uint8_t> value;
};

class BLEClientHID : public ble_client::BLEClientNode {
 public:
  void loop() override;
  void dump_config() override;

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;
  void gap_event_handler(esp_gap_ble_cb_event_t event,
                         esp_ble_gap_cb_param_t *param) override;

  void register_last_event_value_sensor(sensor::Sensor *last_event_value_sensor);
  void register_battery_sensor(sensor::Sensor *battery_sensor);
  void register_last_event_usage_text_sensor(text_sensor::TextSensor *last_event_usage_text_sensor);

 protected:
  // Core flow
  void read_client_characteristics_();
  void on_gatt_read_finished_(ble_client::GATTReadData *data);
  void configure_hid_client_();

  // Reads/helpers
  void schedule_read_char_(ble_client::BLECharacteristic *characteristic);
  uint8_t *parse_characteristic_data_(ble_client::BLEService *service, uint16_t uuid);

  // Notifications / HID processing
  void reset_connection_state_();
  void request_notifications_();  // subscribe ASAP (fix for missing first key)
  void handle_hid_notify_(uint16_t handle, const uint8_t *value, uint16_t len);
  void process_hid_report_(uint16_t handle, const uint8_t *value, uint16_t len);
  void flush_pending_reports_();

  // State
  HIDState hid_state_{HIDState::IDLE};

  // Sensors
  sensor::Sensor *last_event_value_sensor_{nullptr};
  text_sensor::TextSensor *last_event_usage_text_sensor_{nullptr};
  sensor::Sensor *battery_sensor_{nullptr};

  // Battery notify handle
  uint16_t battery_handle_{0};

  // Read bookkeeping
  std::map<uint16_t, ble_client::GATTReadData *> handles_to_read_;
  uint16_t handles_waiting_for_notify_registration_{0};

  // HID decode
  std::map<uint16_t, uint8_t> handle_report_id_;
  std::unique_ptr<HIDReportMap> hid_report_map_{nullptr};

  // “First key after wake” fix: subscribe early + buffer until decode ready
  bool notifications_requested_{false};
  bool notifications_registered_{false};
  bool hid_decode_ready_{false};
  std::deque<PendingNotify> pending_reports_;
  static constexpr size_t kMaxPendingReports = 16;

  // Preferred conn params from GAP (0x2A04)
  esp_ble_conn_update_params_t preferred_conn_params{};
  bool preferred_conn_params_valid{false};
};

}  // namespace ble_client_hid
}  // namespace esphome
