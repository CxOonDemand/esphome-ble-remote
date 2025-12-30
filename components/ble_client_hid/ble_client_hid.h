#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#ifdef USE_ESP32
#include <deque>
#include <map>
#include <string>
#include <vector>

#include <cstring>

#include "hid_parser.h"

#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"

namespace esphome {
namespace ble_client_hid {

enum class HIDState {
  IDLE = 0,
  HID_SERVICE_FOUND,
  NO_HID_SERVICE,
  BLE_CONNECTED,
  READING_CHARS,
  READ_CHARS,
  NOTIFICATIONS_REGISTERING,
  NOTIFICATIONS_REGISTERED,
  CONN_PARAMS_UPDATING,
  CONFIGURED,
};

class GATTReadData {
 public:
  uint16_t handle_{0};
  uint8_t *value_{nullptr};
  uint16_t value_len_{0};

  GATTReadData(uint16_t handle, const uint8_t *value, uint16_t value_len)
      : handle_(handle), value_len_(value_len) {
    this->value_ = new uint8_t[value_len_];
    memcpy(this->value_, value, value_len_);
  }

  ~GATTReadData() {
    delete[] this->value_;
    this->value_ = nullptr;
  }
};

class BLEClientHID : public ble_client::BLEClientNode, public Component {
 public:
  void loop() override;
  void dump_config() override;

  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;

  void register_last_event_value_sensor(sensor::Sensor *last_event_value_sensor);
  void register_battery_sensor(sensor::Sensor *battery_sensor);
  void register_last_event_usage_text_sensor(text_sensor::TextSensor *last_event_usage_text_sensor);

 protected:
  void read_client_characteristics();
  void on_gatt_read_finished(GATTReadData *data);
  void configure_hid_client();

  void schedule_read_char(ble_client::BLECharacteristic *characteristic);
  uint8_t *parse_characteristic_data(ble_client::BLEService *service, uint16_t uuid);

  // Original entry point from ESP_GATTC_NOTIFY_EVT
  void send_input_report_event(esp_ble_gattc_cb_param_t *p_data);

  // New: robust handling for “first key after wake” scenarios
  void queue_pending_input_report_(uint16_t handle, const uint8_t *value, uint16_t value_len);
  void flush_pending_input_reports_();
  bool can_process_input_reports_() const;
  void process_input_report_(uint16_t handle, const uint8_t *value, uint16_t value_len);

  // Internal state
  HIDState hid_state{HIDState::IDLE};

  // Read staging
  std::map<uint16_t, GATTReadData *> handles_to_read;

  // Report parsing
  HIDReportMap *hid_report_map{nullptr};
  std::map<uint16_t, uint8_t> handle_report_id;

  // Notify registration tracking
  int handles_waiting_for_notify_registration{0};

  // Battery
  uint16_t battery_handle{0};
  sensor::Sensor *battery_sensor{nullptr};

  // Exposed sensors
  text_sensor::TextSensor *last_event_usage_text_sensor{nullptr};
  sensor::Sensor *last_event_value_sensor{nullptr};

  // Device info
  std::string device_name;
  std::string manufacturer;
  std::string serial_number;
  uint16_t vendor_id{0};
  uint16_t product_id{0};
  uint16_t version{0};

  // Connection params
  esp_gap_conn_params_t preferred_conn_params{};

  // NEW: buffer early HID reports until fully configured/established
  struct PendingInputReport {
    uint16_t handle{0};
    std::vector<uint8_t> value;
  };
  std::deque<PendingInputReport> pending_input_reports_;

  static constexpr size_t MAX_PENDING_INPUT_REPORTS = 8;
};

}  // namespace ble_client_hid
}  // namespace esphome
#endif  // USE_ESP32
