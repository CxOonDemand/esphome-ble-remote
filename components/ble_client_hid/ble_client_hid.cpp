#include "ble_client_hid.h"

#ifdef USE_ESP32

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/components/ble_client_hid/hid_parser.h"
#include "esphome/components/ble_client_hid/usages.h"

#include <cstring>

namespace esphome {
namespace ble_client_hid {

static const char *const TAG = "ble_client_hid";

void BLEClientHID::setup() {
  // Defaults chosen to match typical BLE remotes that use slave latency for low-power idle.
  // These will be overridden if the remote exposes GAP preferred connection parameters.
  memset(&this->preferred_conn_params_, 0, sizeof(this->preferred_conn_params_));
  this->preferred_conn_params_.min_int = 8;     // 10 ms (1.25ms units)
  this->preferred_conn_params_.max_int = 8;     // 10 ms
  this->preferred_conn_params_.latency = 49;    // allow long peripheral sleep while staying connected
  this->preferred_conn_params_.timeout = 3200;  // 32 s (10ms units)
  this->preferred_conn_params_valid_ = true;
}

void BLEClientHID::loop() {
  // Nothing to do if not connected.
  if (!this->parent() || !this->parent()->connected())
    return;

  switch (this->hid_state_) {
    case HIDState::BLE_CONNECTED:
      // Enable notifications ASAP (before slow HID parsing/reads).
      this->register_notifications_();
      break;

    case HIDState::NOTIFICATIONS_REGISTERED:
      // We can start reading characteristics now (slower path).
      this->read_client_characteristics_();
      this->hid_state_ = HIDState::READ_CHARS;
      break;

    case HIDState::READ_CHARS:
      // Once reads are done (driven by gattc read events), configure HID parsing.
      this->configure_hid_client_();

      // Apply connection parameter update using the best-known params.
      if (this->preferred_conn_params_valid_) {
        memcpy(this->preferred_conn_params_.bda, this->parent()->get_remote_bda(), 6);
        // Keep a generous timeout; this helps avoid disconnects that cause “first key after idle” loss.
        this->preferred_conn_params_.timeout = 3200;  // 32 seconds
        esp_err_t st = esp_ble_gap_update_conn_params(&this->preferred_conn_params_);
        if (st != ESP_OK) {
          ESP_LOGW(TAG, "Conn param update request failed with status=%d", st);
          // Even if update fails, proceed to configured so we can emit events.
          this->hid_state_ = HIDState::CONFIGURED;
          this->flush_pending_notifies_();
        } else {
          this->hid_state_ = HIDState::CONN_PARAMS_UPDATING;
        }
      } else {
        // No preferred params available; proceed anyway.
        this->hid_state_ = HIDState::CONFIGURED;
        this->flush_pending_notifies_();
      }
      break;

    default:
      break;
  }
}

void BLEClientHID::dump_config() {
  ESP_LOGCONFIG(TAG, "BLE Client HID");
  LOG_SENSOR("  ", "Battery", this->battery_sensor_);
  LOG_SENSOR("  ", "Last Event Value", this->last_event_value_sensor_);
  LOG_TEXT_SENSOR("  ", "Last Event Usage", this->last_event_usage_text_sensor_);
}

bool BLEClientHID::can_process_input_reports_() {
  // We only emit HA events once we're configured.
  return this->parent() != nullptr && this->parent()->connected() && this->hid_state_ == HIDState::CONFIGURED;
}

void BLEClientHID::queue_notify_(uint16_t handle, const uint8_t *value, uint16_t value_len) {
  PendingNotify pn;
  pn.handle = handle;
  pn.data.assign(value, value + value_len);
  this->pending_notifies_.push_back(std::move(pn));
}

void BLEClientHID::flush_pending_notifies_() {
  if (!this->can_process_input_reports_())
    return;

  // Process in arrival order
  for (auto &pn : this->pending_notifies_) {
    this->process_hid_notify_(pn.handle, pn.data.data(), pn.data.size());
  }
  this->pending_notifies_.clear();
}

void BLEClientHID::process_hid_notify_(uint16_t handle, const uint8_t *value, uint16_t value_len) {
  if (handle == this->battery_handle_) {
    if (this->battery_sensor_ != nullptr && value_len >= 1) {
      this->battery_sensor_->publish_state(value[0]);
    }
    return;
  }

  // HID report notifications
  this->process_hid_report_(handle, value, value_len);
}

void BLEClientHID::register_notifications_() {
  using namespace ble_client;
  BLEService *battery_service = this->parent()->get_service(ESP_GATT_UUID_BATTERY_SERVICE_SVC);
  BLEService *hid_service = this->parent()->get_service(ESP_GATT_UUID_HID_SVC);

  this->handles_waiting_for_notify_registration_ = 0;

  // Battery level notify (optional)
  if (battery_service != nullptr) {
    BLECharacteristic *battery_level_char = battery_service->get_characteristic(ESP_GATT_UUID_BATTERY_LEVEL);
    if (battery_level_char != nullptr &&
        ((battery_level_char->properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) != 0)) {
      this->battery_handle_ = battery_level_char->handle;
      esp_err_t status = esp_ble_gattc_register_for_notify(this->parent()->get_gattc_if(),
                                                          this->parent()->get_remote_bda(),
                                                          battery_level_char->handle);
      if (status != ESP_OK) {
        ESP_LOGW(TAG, "Register for notify failed for battery handle %u with status=%d",
                 battery_level_char->handle, status);
      } else {
        this->handles_waiting_for_notify_registration_++;
      }
    }
  }

  // HID input reports notify
  if (hid_service != nullptr) {
    for (BLECharacteristic *chr : hid_service->characteristics) {
      if (chr == nullptr)
        continue;
      if (chr->uuid.get_uuid().uuid.uuid16 != ESP_GATT_UUID_HID_REPORT)
        continue;
      if ((chr->properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) == 0)
        continue;

      esp_err_t status = esp_ble_gattc_register_for_notify(this->parent()->get_gattc_if(),
                                                          this->parent()->get_remote_bda(),
                                                          chr->handle);
      if (status != ESP_OK) {
        ESP_LOGW(TAG, "Register for notify failed for HID report handle %u with status=%d", chr->handle, status);
      } else {
        this->handles_waiting_for_notify_registration_++;
      }
    }
  }

  if (this->handles_waiting_for_notify_registration_ == 0) {
    this->hid_state_ = HIDState::NOTIFICATIONS_REGISTERED;
  } else {
    this->hid_state_ = HIDState::NOTIFICATIONS_REGISTERING;
  }
}

void BLEClientHID::read_client_characteristics_() {
  // Read handles we need for parsing/mapping.
  using namespace ble_client;

  BLEService *hid_service = this->parent()->get_service(ESP_GATT_UUID_HID_SVC);
  if (hid_service == nullptr) {
    ESP_LOGW(TAG, "HID service not found");
    return;
  }

  BLECharacteristic *hid_info = hid_service->get_characteristic(ESP_GATT_UUID_HID_INFORMATION);
  BLECharacteristic *protocol_mode = hid_service->get_characteristic(ESP_GATT_UUID_HID_PROTOCOL_MODE);
  BLECharacteristic *control_point = hid_service->get_characteristic(ESP_GATT_UUID_HID_CONTROL_POINT);
  BLECharacteristic *report_map = hid_service->get_characteristic(ESP_GATT_UUID_HID_REPORT_MAP);

  if (hid_info != nullptr)
    this->hid_info_handle_ = hid_info->handle;
  if (protocol_mode != nullptr)
    this->protocol_mode_handle_ = protocol_mode->handle;
  if (control_point != nullptr)
    this->control_point_handle_ = control_point->handle;
  if (report_map != nullptr)
    this->report_map_handle_ = report_map->handle;

  // GAP preferred connection parameters (0x2A04) are in the GAP service.
  BLEService *gap_service = this->parent()->get_service(ESP_GATT_UUID_GAP_SERVICE_SVC);
  if (gap_service != nullptr) {
    BLECharacteristic *pref_conn = gap_service->get_characteristic(ESP_GATT_UUID_GAP_PREF_CONN_PARAM);
    if (pref_conn != nullptr) {
      // Read in gattc read event; we parse in configure_hid_client_().
      this->parent()->read_characteristic(pref_conn);
    }
  }

  // Read report map; we parse in configure_hid_client_().
  if (report_map != nullptr) {
    this->parent()->read_characteristic(report_map);
  }

  // Read report reference descriptors for report IDs.
  for (BLECharacteristic *chr : hid_service->characteristics) {
    if (chr == nullptr)
      continue;
    if (chr->uuid.get_uuid().uuid.uuid16 != ESP_GATT_UUID_HID_REPORT)
      continue;

    // Report Reference Descriptor UUID 0x2908
    auto *desc = chr->get_descriptor(ESP_GATT_UUID_RPT_REF_DESCR);
    if (desc != nullptr) {
      this->parent()->read_descriptor(desc);
    }
  }
}

void BLEClientHID::configure_hid_client_() {
  // Parse any characteristic/descriptor reads collected by ble_client and build mapping.
  using namespace ble_client;

  // Parse GAP preferred conn params if present.
  BLEService *gap_service = this->parent()->get_service(ESP_GATT_UUID_GAP_SERVICE_SVC);
  if (gap_service != nullptr) {
    BLECharacteristic *pref_conn = gap_service->get_characteristic(ESP_GATT_UUID_GAP_PREF_CONN_PARAM);
    if (pref_conn != nullptr && pref_conn->value.size() >= 8) {
      const uint8_t *t_conn_params = pref_conn->value.data();
      this->preferred_conn_params_.min_int = t_conn_params[0] | (t_conn_params[1] << 8);
      this->preferred_conn_params_.max_int = t_conn_params[2] | (t_conn_params[3] << 8);
      this->preferred_conn_params_.latency = t_conn_params[4] | (t_conn_params[5] << 8);
      this->preferred_conn_params_.timeout = t_conn_params[6] | (t_conn_params[7] << 8);
      this->preferred_conn_params_valid_ = true;

      ESP_LOGI(TAG,
               "Got preferred connection paramters: interval: %.2f - %.2f ms, latency: %u, timeout: %.1f ms",
               this->preferred_conn_params_.min_int * 1.25f, this->preferred_conn_params_.max_int * 1.25f,
               this->preferred_conn_params_.latency, this->preferred_conn_params_.timeout * 10.0f);
    }
  }

  // Cache report map for HID parsing.
  if (this->report_map_handle_ != 0) {
    BLEService *hid_service = this->parent()->get_service(ESP_GATT_UUID_HID_SVC);
    if (hid_service != nullptr) {
      BLECharacteristic *report_map = hid_service->get_characteristic(ESP_GATT_UUID_HID_REPORT_MAP);
      if (report_map != nullptr) {
        this->report_map_ = report_map->value;
      }
    }
  }

  // Build report handle -> report ID map based on Report Reference descriptors (0x2908).
  this->report_ids_by_handle_.clear();
  BLEService *hid_service = this->parent()->get_service(ESP_GATT_UUID_HID_SVC);
  if (hid_service != nullptr) {
    for (BLECharacteristic *chr : hid_service->characteristics) {
      if (chr == nullptr)
        continue;
      if (chr->uuid.get_uuid().uuid.uuid16 != ESP_GATT_UUID_HID_REPORT)
        continue;

      auto *desc = chr->get_descriptor(ESP_GATT_UUID_RPT_REF_DESCR);
      if (desc != nullptr && desc->value.size() >= 2) {
        uint8_t report_id = desc->value[0];      // Report ID
        // uint8_t report_type = desc->value[1]; // (input/output/feature)
        this->report_ids_by_handle_[chr->handle] = report_id;
      }
    }
  }
}

void BLEClientHID::process_input_report_(uint16_t handle, const uint8_t *value, uint16_t value_len) {
  // Parse HID input report into usage/value.
  uint8_t report_id = 0;
  auto it = this->report_ids_by_handle_.find(handle);
  if (it != this->report_ids_by_handle_.end()) {
    report_id = it->second;
  }

  HIDParser parser;
  parser.set_report_map(this->report_map_);
  HIDEvent evt{};
  if (!parser.parse_input_report(report_id, value, value_len, evt)) {
    return;
  }

  this->last_event_value_ = evt.value;
  this->last_event_usage_ = evt.usage;

  if (this->last_event_value_sensor_ != nullptr) {
    this->last_event_value_sensor_->publish_state(this->last_event_value_);
  }
  if (this->last_event_usage_text_sensor_ != nullptr) {
    this->last_event_usage_text_sensor_->publish_state(this->last_event_usage_);
  }

#ifdef USE_API
  // Fire HA event immediately for automations.
  this->fire_homeassistant_event("esphome.hid_events",
                                 {
                                     {"usage", this->last_event_usage_},
                                     {"value", this->last_event_value_},
                                 });
#endif

  ESP_LOGI(TAG, "Send HID event to HomeAssistant: usage: %s, value: %d", this->last_event_usage_.c_str(),
           this->last_event_value_);
}

void BLEClientHID::process_hid_report_(uint16_t handle, const uint8_t *value, uint16_t value_len) {
  // Buffer until configured, then parse.
  if (!this->can_process_input_reports_()) {
    this->queue_notify_(handle, value, value_len);
    return;
  }
  this->process_input_report_(handle, value, value_len);
}

void BLEClientHID::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                       esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_CONNECT_EVT: {
      esp_gap_conn_params_t conn_params = esp_ble_gattc_get_conn_params(this->parent()->get_remote_bda());
      ESP_LOGI(TAG, "conn params: interval=%u, latency=%u, timeout=%u", conn_params.interval, conn_params.latency,
               conn_params.timeout);

      // Start encryption if needed (ESPHome handles bonding/pairing elsewhere as well).
      esp_ble_set_encryption(this->parent()->get_remote_bda(), ESP_BLE_SEC_ENCRYPT);

      // Proactively request low-power/stable connection parameters on connect.
      // This helps keep the link alive between keypresses so the first key is not missed after idle.
      if (this->preferred_conn_params_valid_) {
        memcpy(this->preferred_conn_params_.bda, this->parent()->get_remote_bda(), 6);
        esp_err_t st = esp_ble_gap_update_conn_params(&this->preferred_conn_params_);
        if (st != ESP_OK) {
          ESP_LOGW(TAG, "Early conn param update request failed with status=%d", st);
        }
      }

      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "[%02X:%02X:%02X:%02X:%02X:%02X] Disconnected!", this->parent()->get_remote_bda()[0],
               this->parent()->get_remote_bda()[1], this->parent()->get_remote_bda()[2],
               this->parent()->get_remote_bda()[3], this->parent()->get_remote_bda()[4],
               this->parent()->get_remote_bda()[5]);

      // Reset state and clear buffered notifies.
      this->hid_state_ = HIDState::IDLE;
      this->pending_notifies_.clear();
      this->handles_waiting_for_notify_registration_ = 0;

      break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
      // Service discovery complete; enable notifications ASAP.
      this->hid_state_ = HIDState::BLE_CONNECTED;
      break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      // ESPHome/ble_client will write CCCD after this; we just track completion count.
      if (param->reg_for_notify.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "REG_FOR_NOTIFY failed status=%d handle=%u", param->reg_for_notify.status,
                 param->reg_for_notify.handle);
        break;
      }

      if (this->handles_waiting_for_notify_registration_ > 0) {
        this->handles_waiting_for_notify_registration_--;
      }

      if (this->handles_waiting_for_notify_registration_ == 0) {
        this->hid_state_ = HIDState::NOTIFICATIONS_REGISTERED;
      }
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      // Notifications (battery or HID report).
      const uint16_t handle = param->notify.handle;
      const uint8_t *value = param->notify.value;
      const uint16_t value_len = param->notify.value_len;

      this->process_hid_notify_(handle, value, value_len);
      break;
    }

    default:
      break;
  }
}

void BLEClientHID::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT: {
      ESP_LOGI(TAG, "Conn params updated: status=%d, interval=%u, latency=%u, timeout=%u",
               param->update_conn_params.status, param->update_conn_params.conn_int,
               param->update_conn_params.latency, param->update_conn_params.timeout);

      if (this->hid_state_ == HIDState::CONN_PARAMS_UPDATING) {
        this->hid_state_ = HIDState::CONFIGURED;
        this->flush_pending_notifies_();
      }
      break;
    }

    default:
      break;
  }
}

void BLEClientHID::register_battery_sensor(sensor::Sensor *battery_sensor) { this->battery_sensor_ = battery_sensor; }

void BLEClientHID::register_last_event_value_sensor(sensor::Sensor *last_event_value_sensor) {
  this->last_event_value_sensor_ = last_event_value_sensor;
}

void BLEClientHID::register_last_event_usage_text_sensor(text_sensor::TextSensor *last_event_usage_text_sensor) {
  this->last_event_usage_text_sensor_ = last_event_usage_text_sensor;
}

}  // namespace ble_client_hid
}  // namespace esphome

#endif  // USE_ESP32
