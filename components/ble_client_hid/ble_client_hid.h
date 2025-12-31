#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#ifdef USE_API
#include "esphome/components/api/custom_api_device.h"
#endif

#include <map>
#include <string>
#include <vector>

#include "hid_parser.h"  // from this component repo

#ifdef USE_ESP32
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#endif

namespace esphome {
namespace ble_client_hid {

enum class HIDState : uint8_t {
  IDLE = 0,
  HID_SERVICE_FOUND,
  BLE_CONNECTED,
  NOTIFICATIONS_REGISTERING,
  NOTIFICATIONS_REGISTERED,
  READING_CHARS,
  READ_CHARS,
  CONN_PARAMS_UPDATING,
  CONFIGURED,
  NO_HID_SERVICE,
};

struct GATTReadData {
  uint16_t handle_{0};
  uint8_t *value_{nullptr};
  uint16_t value_len_{0};

  GATTReadData(uint16_t handle, const uint8_t *value, uint16_t len) : handle_(handle), value_len_(len) {
    this->value_ = new uint8_t[len];
    memcpy(this->value_, value, len);
  }

  ~GATTReadData() {
    delete[] this->value_;
    this->value_ = nullptr;
    this->value_len_ = 0;
  }
};

struct PendingNotify {
  uint16_t handle{0};
  std::vector<uint8_t> bytes;
};

class BLEClientHID : public ble_client::BLEClientNode
#ifdef USE_API
  , public api::CustomAPIDevice
#endif
{
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
  void read_client_characteristics_();
  void on_gatt_read_finished_(GATTReadData *data);

  void register_notifications_();
  void write_cccd_(uint16_t cccd_handle);

  uint8_t *parse_characteristic_data_(ble_client::BLEService *service, uint16_t uuid);
  void schedule_read_char_(ble_client::BLECharacteristic *characteristic);

  void configure_hid_client_();
  void process_hid_notify_(uint16_t handle, const uint8_t *data, uint16_t len);
  void flush_pending_notifies_();

  bool can_process_input_reports_() const;

 private:
  HIDState hid_state_{HIDState::IDLE};

  // Read bookkeeping
  std::map<uint16_t, GATTReadData *> handles_to_read_;

  // Notification bookkeeping
  int32_t handles_waiting_for_notify_registration_{0};
  uint16_t battery_handle_{0};

  // For enabling notifications quickly
  std::map<uint16_t, uint16_t> cccd_by_char_handle_;   // char_handle -> cccd_handle (0x2902)

  // HID parsing
  HIDReportMap *hid_report_map_{nullptr};              // Owned by parser; created once we parse report map
  std::map<uint16_t, uint8_t> handle_report_id_;       // char_handle -> report_id

  // Output/telemetry
  sensor::Sensor *last_event_value_sensor_{nullptr};
  sensor::Sensor *battery_sensor_{nullptr};
  text_sensor::TextSensor *last_event_usage_text_sensor_{nullptr};

  std::string device_name_;
  std::string manufacturer_;
  std::string serial_number_;
  uint16_t vendor_id_{0};
  uint16_t product_id_{0};
  uint16_t version_{0};

  // Buffer early notifications until report map + report IDs are known
  std::vector<PendingNotify> pending_notifies_;

#ifdef USE_ESP32
  // Correct type for esp_ble_gap_update_conn_params()
  esp_ble_conn_update_params_t preferred_conn_params_{};
  bool preferred_conn_params_valid_{false};
#endif
};

}  // namespace ble_client_hid
}  // namespace esphome
