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

// ESP-IDF BLE headers (ESPHome uses these under ESP-IDF framework)
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"

#include "hid_parser.h"
#include "usages.h"

namespace esphome {
namespace ble_client_hid {

// Simple container for read results
struct GATTReadData {
  uint16_t handle_{0};
  uint16_t value_len_{0};
  uint8_t *value_{nullptr};

  GATTReadData(uint16_t handle, const uint8_t *value, uint16_t value_len) : handle_(handle), value_len_(value_len) {
    value_ = new uint8_t[value_len_];
    memcpy(value_, value, value_len_);
  }

  ~GATTReadData() { delete[] value_; }
};

enum class HIDState : uint8_t {
  IDLE = 0,
  HID_SERVICE_FOUND,
  BLE_CONNECTED,
  READING_CHARS,
  READ_CHARS,
  NOTIFICATIONS_REGISTERING,
  NOTIFICATIONS_REGISTERED,
  CONFIGURED,
  NO_HID_SERVICE,
};

class BLEClientHID : public Component,
                     public ble_client::BLEClientNode,
                     public api::CustomAPIDevice {
 public:
  void dump_config() override;
  void loop() override;

  // BLEClientNode callback
  void gattc_event_handler(esp_gattc_cb_event_t event,
                           esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;

  // Sensor/text_sensor registration from the platform code
  void register_battery_sensor(sensor::Sensor *battery_sensor);
  void register_last_event_value_sensor(sensor::Sensor *last_event_value_sensor);
  void register_last_event_usage_text_sensor(text_sensor::TextSensor *last_event_usage_text_sensor);

 protected:
  void read_client_characteristics_();
  void on_gatt_read_finished_(GATTReadData *data);
  void configure_hid_client_();

  void schedule_read_char_(ble_client::BLECharacteristic *characteristic);
  void schedule_read_descr_(ble_client::BLEDescriptor *descriptor);

  uint8_t *parse_characteristic_data_(ble_client::BLEService *service, uint16_t uuid16);

  void process_hid_report_(uint16_t handle, const uint8_t *value, uint16_t value_len);

  bool can_process_input_reports_();  // intentionally non-const (ESPHome BLEClientNode::parent() is non-const)

 protected:
  static const char *const TAG;

  HIDState hid_state_{HIDState::IDLE};

  // Read tracking
  std::map<uint16_t, GATTReadData *> handles_to_read_;

  // Report parsing
  HIDReportMap *hid_report_map_{nullptr};
  std::map<uint16_t, uint8_t> handle_report_id_;

  // Battery notify handle
  uint16_t battery_handle_{0};

  // Notify registration tracking
  int handles_waiting_for_notify_registration_{0};

  // Preferred connection params (from GAP 0x2A04)
  bool have_preferred_conn_params_{false};
  esp_ble_conn_update_params_t preferred_conn_params_{};

  // Metadata
  std::string device_name_{"Generic"};
  std::string manufacturer_{"Generic"};
  std::string serial_number_{"000000"};
  uint16_t vendor_id_{0};
  uint16_t product_id_{0};
  uint16_t version_{0};

  // Exposed sensors
  sensor::Sensor *battery_sensor_{nullptr};
  sensor::Sensor *last_event_value_sensor_{nullptr};
  text_sensor::TextSensor *last_event_usage_text_sensor_{nullptr};
};

}  // namespace ble_client_hid
}  // namespace esphome

#endif  // USE_ESP32
