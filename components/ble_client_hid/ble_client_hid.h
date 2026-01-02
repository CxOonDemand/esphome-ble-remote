#pragma once

#ifdef USE_ESP32

#include <map>
#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/log.h"

#include "esphome/components/api/custom_api_device.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include "hid_parser.h"
#include "usages.h"

#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"

namespace esphome {
namespace ble_client_hid {

enum class HIDState : uint8_t {
  DISCONNECTED = 0,
  HID_SERVICE_FOUND,
  BLE_CONNECTED,
  READING_CHARS,
  READ_CHARS,
  NOTIFICATIONS_REGISTERING,
  NOTIFICATIONS_REGISTERED,
  CONN_PARAMS_UPDATING,
  CONFIGURED,
  NO_HID_SERVICE,
};

struct GATTReadData {
  uint16_t handle_;
  uint8_t *value_;
  uint16_t value_len_;

  GATTReadData(uint16_t handle, const uint8_t *value, uint16_t value_len) : handle_(handle), value_len_(value_len) {
    this->value_ = new uint8_t[value_len_];
    memcpy(this->value_, value, value_len_);
  }

  ~GATTReadData() { delete[] this->value_; }
};

// Buffered notifications arriving before we are ready to parse.
struct PendingNotify {
  uint16_t handle;
  std::vector<uint8_t> value;
};

class BLEClientHID : public Component,
                     public ble_client::BLEClientNode,
                     public api::CustomAPIDevice,
                     public ble_client::GAPEventHandler {
 public:
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;

  void gap_event_handler(esp_gap_ble_cb_event_t event,
                         esp_ble_gap_cb_param_t *param) override;

  void register_last_event_value_sensor(sensor::Sensor *last_event_value_sensor);
  void register_battery_sensor(sensor::Sensor *battery_sensor);
  void register_last_event_usage_text_sensor(text_sensor::TextSensor *last_event_usage_text_sensor);

 protected:
  void read_client_characteristics_();
  void on_gatt_read_finished_(GATTReadData *data);
  void configure_hid_client_();
  void schedule_read_char_(ble_client::BLECharacteristic *characteristic);

  void process_input_report_(uint16_t handle, const uint8_t *value, uint16_t value_len);
  void buffer_input_report_(uint16_t handle, const uint8_t *value, uint16_t value_len);
  void flush_buffered_reports_();
  bool ready_for_reports_();

  void request_conn_params_(uint16_t min_int, uint16_t max_int, uint16_t latency, uint16_t timeout);

  // State
  HIDState hid_state_{HIDState::DISCONNECTED};
  bool conn_params_request_sent_{false};

  // Handles, maps, parsers
  std::map<uint16_t, GATTReadData *> handles_to_read_;
  uint16_t handles_waiting_for_notify_registration_{0};

  std::map<uint16_t, uint8_t> handle_report_id_;
  HIDReportMap *hid_report_map_{nullptr};

  uint16_t battery_handle_{0};

  // Device metadata (optional)
  std::string device_name_;
  std::string manufacturer_;
  std::string serial_number_;
  uint16_t vendor_id_{0};
  uint16_t product_id_{0};
  uint16_t version_{0};

  // Preferred conn params read from GAP (0x2A04). We’ll enforce a sane timeout.
  esp_ble_conn_update_params_t preferred_conn_params_{};

  // Sensors
  sensor::Sensor *battery_sensor_{nullptr};
  sensor::Sensor *last_event_value_sensor_{nullptr};
  text_sensor::TextSensor *last_event_usage_text_sensor_{nullptr};

  // Buffer for “first key” reports that arrive before we’re configured.
  std::vector<PendingNotify> pending_reports_;
  static constexpr size_t MAX_PENDING_REPORTS = 8;
};

}  // namespace ble_client_hid
}  // namespace esphome

#endif  // USE_ESP32
