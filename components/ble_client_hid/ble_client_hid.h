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

#ifdef USE_ESP32

namespace esphome {
namespace ble_client_hid {

enum class HIDState {
  IDLE = 0,
  BLE_CONNECTED,
  NOTIFICATIONS_REGISTERING,
  NOTIFICATIONS_REGISTERED,
  READ_CHARS,
  CONN_PARAMS_UPDATING,
  CONFIGURED,
};

struct PendingNotify {
  uint16_t handle;
  std::vector<uint8_t> data;
};

class BLEClientHID : public Component, public ble_client::BLEClientNode
#ifdef USE_API
  ,
                    public api::CustomAPIDevice
#endif
{
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;

  void register_battery_sensor(sensor::Sensor *battery_sensor);
  void register_last_event_value_sensor(sensor::Sensor *last_event_value_sensor);
  void register_last_event_usage_text_sensor(text_sensor::TextSensor *last_event_usage_text_sensor);

 protected:
  // Notification + parsing pipeline
  bool can_process_input_reports_();
  void register_notifications_();

  void queue_notify_(uint16_t handle, const uint8_t *value, uint16_t value_len);
  void flush_pending_notifies_();

  void process_hid_notify_(uint16_t handle, const uint8_t *value, uint16_t value_len);

  // Read/parse phase after notifications are already enabled
  void read_client_characteristics_();
  void configure_hid_client_();
  void process_input_report_(uint16_t handle, const uint8_t *value, uint16_t value_len);
  void process_hid_report_(uint16_t handle, const uint8_t *value, uint16_t value_len);

  // Sensors/text sensors
  sensor::Sensor *battery_sensor_{nullptr};
  sensor::Sensor *last_event_value_sensor_{nullptr};
  text_sensor::TextSensor *last_event_usage_text_sensor_{nullptr};

  // State
  HIDState hid_state_{HIDState::IDLE};

  // Handles and configuration
  uint16_t battery_handle_{0};
  uint16_t hid_info_handle_{0};
  uint16_t protocol_mode_handle_{0};
  uint16_t control_point_handle_{0};
  uint16_t report_map_handle_{0};

  uint16_t handles_waiting_for_notify_registration_{0};

  // Parsed HID map: report handle -> report id
  std::map<uint16_t, uint8_t> report_ids_by_handle_;
  std::vector<uint8_t> report_map_;

  // Preferred connection parameters
  esp_ble_conn_update_params_t preferred_conn_params_{};
  bool preferred_conn_params_valid_{false};

  // Buffered notifies received before we are "configured"
  std::vector<PendingNotify> pending_notifies_;

  // Bookkeeping for last event
  int last_event_value_{0};
  std::string last_event_usage_{};
};

}  // namespace ble_client_hid
}  // namespace esphome

#endif  // USE_ESP32
