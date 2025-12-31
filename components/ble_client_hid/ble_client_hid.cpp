#include "ble_client_hid.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"

#ifdef USE_API
#include "esphome/components/api/custom_api_device.h"
#endif

namespace esphome {
namespace ble_client_hid {

static const char *const TAG = "ble_client_hid";

#ifdef USE_API
// fire_homeassistant_event() lives on CustomAPIDevice (per ESPHome examples). 
static api::CustomAPIDevice s_api;
#endif

void BLEClientHID::reset_connection_state_() {
  this->battery_handle_ = 0;

  // Per-connection flags
  this->notifications_requested_ = false;
  this->notifications_registered_ = false;
  this->handles_waiting_for_notify_registration_ = 0;
  this->preferred_params_valid_ = false;

  // Keep cached decode artifacts across reconnects if already built:
  // - hid_report_map_ (stable)
  // - handle_report_id_ (handles are stable for a given device’s GATT table)
  this->hid_decode_ready_ = (this->hid_report_map_ != nullptr) && (!this->handle_report_id_.empty());

  // Buffer cleared per connection; we only need to buffer until decode is ready.
  this->pending_reports_.clear();
}

void BLEClientHID::loop() {
  switch (this->hid_state_) {
    case HIDState::BLE_CONNECTED: {
      // If we already have decode artifacts cached, skip expensive reads and just wait for notify registration.
      if (!this->hid_decode_ready_) {
        this->read_client_characteristics_();
        this->hid_state_ = HIDState::READING_CHARS;
      } else {
        // No-op; notifications are requested on SEARCH_CMPL; we just wait.
        this->hid_state_ = HIDState::NOTIFICATIONS_REGISTERING;
      }
      break;
    }

    case HIDState::READ_CHARS: {
      this->configure_hid_client_();
      this->hid_state_ = HIDState::NOTIFICATIONS_REGISTERING;
      break;
    }

    case HIDState::NOTIFICATIONS_REGISTERED: {
      // Apply preferred connection parameters (if the device provided them).
      if (this->preferred_params_valid_) {
        esp_ble_gap_update_conn_params(&this->preferred_conn_params_);
      }
      this->hid_state_ = HIDState::CONN_PARAMS_UPDATING;

      // If any buffered reports arrived before decode was ready, flush now.
      this->flush_pending_reports_();
      break;
    }

    default:
      break;
  }
}

void BLEClientHID::dump_config() {
  ESP_LOGCONFIG(TAG, "BLE Client HID:");
  ESP_LOGCONFIG(TAG, "  MAC address        : %s", this->parent()->address_str());
}

void BLEClientHID::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT: {
      ESP_LOGI(TAG,
               "Updated conn params to interval=%.2f ms, latency=%u, timeout=%.1f ms",
               param->update_conn_params.conn_int * 1.25f,
               param->update_conn_params.latency,
               param->update_conn_params.timeout * 10.f);

      this->hid_state_ = HIDState::CONFIGURED;
      // espbt alias is in ble_client namespace. 
      this->node_state = ble_client::espbt::ClientState::ESTABLISHED;

      // If we got here before we flushed, flush now.
      this->flush_pending_reports_();
      break;
    }
    default:
      break;
  }
}

void BLEClientHID::request_notifications_() {
  if (this->notifications_requested_) return;

  using namespace ble_client;

  BLEService *hid_service = this->parent()->get_service(ESP_GATT_UUID_HID_SVC);
  BLEService *battery_service = this->parent()->get_service(ESP_GATT_UUID_BATTERY_SERVICE_SVC);

  // Subscribe to battery (optional)
  if (battery_service != nullptr) {
    BLECharacteristic *battery_level_char = battery_service->get_characteristic(ESP_GATT_UUID_BATTERY_LEVEL);
    if (battery_level_char != nullptr &&
        ((battery_level_char->properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) != 0)) {
      this->battery_handle_ = battery_level_char->handle;

      auto status = esp_ble_gattc_register_for_notify(
          this->parent()->get_gattc_if(), this->parent()->get_remote_bda(), battery_level_char->handle);

      if (status != ESP_OK) {
        ESP_LOGW(TAG, "Register for notify failed for battery handle %d (status=%d)",
                 battery_level_char->handle, status);
      } else {
        this->handles_waiting_for_notify_registration_++;
      }
    }
  }

  // Subscribe to HID reports ASAP (key fix: do this before the expensive reads/parse)
  if (hid_service != nullptr) {
    for (auto *chr : hid_service->characteristics) {
      if (chr->uuid.get_uuid().uuid.uuid16 != ESP_GATT_UUID_HID_REPORT) continue;
      if ((chr->properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) == 0) continue;

      auto status = esp_ble_gattc_register_for_notify(
          this->parent()->get_gattc_if(), this->parent()->get_remote_bda(), chr->handle);

      if (status != ESP_OK) {
        ESP_LOGW(TAG, "Register for notify failed for HID handle %d (status=%d)", chr->handle, status);
      } else {
        this->handles_waiting_for_notify_registration_++;
      }
    }
  }

  this->notifications_requested_ = true;

  // If nothing to register, consider us “registered” to avoid deadlock.
  if (this->handles_waiting_for_notify_registration_ == 0) {
    this->notifications_registered_ = true;
    this->hid_state_ = HIDState::NOTIFICATIONS_REGISTERED;
  } else {
    this->hid_state_ = HIDState::NOTIFICATIONS_REGISTERING;
  }
}

void BLEClientHID::read_client_characteristics_() {
  ESP_LOGD(TAG, "Reading client characteristics");
  using namespace ble_client;

  BLEService *battery_service = this->parent()->get_service(ESP_GATT_UUID_BATTERY_SERVICE_SVC);
  BLEService *device_info_service = this->parent()->get_service(ESP_GATT_UUID_DEVICE_INFO_SVC);
  BLEService *hid_service = this->parent()->get_service(ESP_GATT_UUID_HID_SVC);
  BLEService *generic_access_service = this->parent()->get_service(0x1800);

  if (generic_access_service != nullptr) {
    BLECharacteristic *device_name_char = generic_access_service->get_characteristic(ESP_GATT_UUID_GAP_DEVICE_NAME);
    BLECharacteristic *pref_conn_params_char = generic_access_service->get_characteristic(ESP_GATT_UUID_GAP_PREF_CONN_PARAM);
    this->schedule_read_char_(pref_conn_params_char);
    this->schedule_read_char_(device_name_char);
  }

  if (device_info_service != nullptr) {
    BLECharacteristic *pnp_id_char = device_info_service->get_characteristic(ESP_GATT_UUID_PNP_ID);
    this->schedule_read_char_(pnp_id_char);

    BLECharacteristic *manufacturer_char = device_info_service->get_characteristic(ESP_GATT_UUID_MANU_NAME);
    this->schedule_read_char_(manufacturer_char);

    BLECharacteristic *serial_number_char = device_info_service->get_characteristic(ESP_GATT_UUID_SERIAL_NUMBER_STR);
    this->schedule_read_char_(serial_number_char);
  }

  if (hid_service != nullptr) {
    BLECharacteristic *hid_report_map_char = hid_service->get_characteristic(ESP_GATT_UUID_HID_REPORT_MAP);
    this->schedule_read_char_(hid_report_map_char);

    ESP_LOGD(TAG, "Found %d characteristics", (int) hid_service->characteristics.size());
    for (auto *chr : hid_service->characteristics) {
      if (chr->uuid.get_uuid().uuid.uuid16 != ESP_GATT_UUID_HID_REPORT) continue;

      BLEDescriptor *rpt_ref_desc = chr->get_descriptor(ESP_GATT_UUID_RPT_REF_DESCR);
      if (rpt_ref_desc != nullptr) {
        if (esp_ble_gattc_read_char_descr(this->parent()->get_gattc_if(),
                                          this->parent()->get_conn_id(),
                                          rpt_ref_desc->handle,
                                          ESP_GATT_AUTH_REQ_NO_MITM) != ESP_OK) {
          ESP_LOGW(TAG, "scheduling reading of RPT_REF_DESCR failed.");
        }
        this->handles_to_read_.insert(std::make_pair(rpt_ref_desc->handle, nullptr));
      }
    }
  }

  (void) battery_service;  // battery notify handle is handled by request_notifications_()
}

void BLEClientHID::on_gatt_read_finished_(ble_client::GATTReadData *data) {
  auto itr = this->handles_to_read_.find(data->handle_);
  if (itr != this->handles_to_read_.end()) {
    itr->second = data;
  }

  // Check if all handles have been read:
  for (auto const &element : this->handles_to_read_) {
    if (element.second == nullptr) return;
  }

  this->hid_state_ = HIDState::READ_CHARS;
}

void BLEClientHID::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                       esp_ble_gattc_cb_param_t *param) {
  (void) gattc_if;
  auto *p_data = param;

  switch (event) {
    case ESP_GATTC_CONNECT_EVT: {
      this->reset_connection_state_();

      auto ret = esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);
      if (ret) {
        ESP_LOGE(TAG, "[%d] [%s] esp_ble_set_encryption error, status=%d",
                 this->parent()->get_connection_index(), this->parent()->address_str(), ret);
      }

      esp_gap_conn_params_t params;
      ret = esp_ble_get_current_conn_params(this->parent()->get_remote_bda(), &params);
      if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get conn params");
      }
      ESP_LOGI(TAG, "conn params: interval=%u, latency=%u, timeout=%u", params.interval, params.latency, params.timeout);
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "[%s] Disconnected!", this->parent()->address_str());
      this->status_set_warning("Disconnected");
      // Keep hid_report_map_ cached, but clear per-connection state.
      this->reset_connection_state_();
      break;
    }

    case ESP_GATTC_SEARCH_RES_EVT: {
      if (p_data->search_res.srvc_id.uuid.uuid.uuid16 == ESP_GATT_UUID_HID_SVC) {
        this->hid_state_ = HIDState::HID_SERVICE_FOUND;
        ESP_LOGD(TAG, "GATT HID service found on device %s", this->parent()->address_str());
      }
      break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
      if (this->hid_state_ != HIDState::HID_SERVICE_FOUND) {
        ESP_LOGW(TAG, "No GATT HID service found on device %s", this->parent()->address_str());
        this->hid_state_ = HIDState::NO_HID_SERVICE;
        this->status_set_warning("Invalid device config");
        break;
      }

      ESP_LOGD(TAG, "GATTC search finished with status code %d", p_data->search_cmpl.status);
      this->hid_state_ = HIDState::BLE_CONNECTED;

      // Critical: subscribe ASAP so the wake key isn’t missed.
      this->request_notifications_();

      esp_gap_conn_params_t params;
      esp_err_t ret = esp_ble_get_current_conn_params(this->parent()->get_remote_bda(), &params);
      if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get conn params");
      }
      ESP_LOGI(TAG, "conn params: interval=%u, latency=%u, timeout=%u", params.interval, params.latency, params.timeout);
      break;
    }

    case ESP_GATTC_READ_CHAR_EVT:
    case ESP_GATTC_READ_DESCR_EVT: {
      if (param->read.conn_id != this->parent()->get_conn_id()) break;
      if (param->read.status != ESP_OK) {
        ESP_LOGW(TAG, "GATTC read failed with status code %d", param->read.status);
        break;
      }

      auto *data = new ble_client::GATTReadData(param->read.handle, param->read.value, param->read.value_len);
      this->on_gatt_read_finished_(data);
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.conn_id != this->parent()->get_conn_id()) break;

      if (p_data->notify.handle == this->battery_handle_) {
        uint8_t battery_level = p_data->notify.value[0];
        if (this->battery_sensor_ != nullptr) {
          this->battery_sensor_->publish_state(battery_level);
        }
      } else {
        // HID report notify; buffer/process depending on readiness.
        this->handle_hid_notify_(p_data->notify.handle, p_data->notify.value, p_data->notify.value_len);
      }
      break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      if (param->notify.conn_id != this->parent()->get_conn_id()) break;

      if (this->handles_waiting_for_notify_registration_ > 0) {
        this->handles_waiting_for_notify_registration_--;
      }
      if (this->handles_waiting_for_notify_registration_ == 0) {
        this->notifications_registered_ = true;
        this->hid_state_ = HIDState::NOTIFICATIONS_REGISTERED;
        this->flush_pending_reports_();
      }
      break;
    }

    default:
      break;
  }
}

void BLEClientHID::handle_hid_notify_(uint16_t handle, const uint8_t *value, uint16_t len) {
  // If we are not ready to decode yet (common right after wake), buffer the report.
  if (!this->notifications_registered_ || !this->hid_decode_ready_) {
    PendingNotify pn;
    pn.handle = handle;
    pn.value.assign(value, value + len);

    if (this->pending_reports_.size() >= kMaxPendingReports) {
      this->pending_reports_.pop_front();
    }
    this->pending_reports_.push_back(std::move(pn));
    return;
  }

  this->process_hid_report_(handle, value, len);
}

void BLEClientHID::flush_pending_reports_() {
  if (!this->notifications_registered_ || !this->hid_decode_ready_) return;

  while (!this->pending_reports_.empty()) {
    auto pn = std::move(this->pending_reports_.front());
    this->pending_reports_.pop_front();
    this->process_hid_report_(pn.handle, pn.value.data(), pn.value.size());
  }
}

void BLEClientHID::process_hid_report_(uint16_t handle, const uint8_t *value, uint16_t len) {
  if (this->hid_report_map_ == nullptr) return;

  // Compose: [report_id][payload...]
  uint8_t report_id = 0;
  auto it = this->handle_report_id_.find(handle);
  if (it != this->handle_report_id_.end()) report_id = it->second;

  std::vector<uint8_t> data(len + 1);
  data[0] = report_id;
  memcpy(&data[1], value, len);

  std::vector<HIDReportItemValue> hid_report_values = this->hid_report_map_->parse(data.data());
  if (hid_report_values.empty()) return;

  for (const HIDReportItemValue &v : hid_report_values) {
    std::string usage;

    if (USAGE_PAGES.count(v.usage.page) > 0 &&
        USAGE_PAGES.at(v.usage.page).usages_.count(v.usage.usage) > 0) {
      usage = USAGE_PAGES.at(v.usage.page).usages_.at(v.usage.usage);
    } else {
      usage = std::to_string(v.usage.page) + "_" + std::to_string(v.usage.usage);
    }

#ifdef USE_API
    s_api.fire_homeassistant_event("esphome.hid_events",
                                  {{"usage", usage}, {"value", std::to_string(v.value)}});
    ESP_LOGD(TAG, "Send HID event to HomeAssistant: usage: %s, value: %d", usage.c_str(), v.value);
#endif

    if (this->last_event_usage_text_sensor_ != nullptr) {
      this->last_event_usage_text_sensor_->publish_state(usage);
    }
    if (this->last_event_value_sensor_ != nullptr) {
      this->last_event_value_sensor_->publish_state(v.value);
    }

    ESP_LOGI(TAG, "Send HID event to HomeAssistant: usage: %s, value: %d", usage.c_str(), v.value);
  }
}

void BLEClientHID::register_last_event_value_sensor(sensor::Sensor *s) { this->last_event_value_sensor_ = s; }
void BLEClientHID::register_battery_sensor(sensor::Sensor *s) { this->battery_sensor_ = s; }
void BLEClientHID::register_last_event_usage_text_sensor(text_sensor::TextSensor *s) { this->last_event_usage_text_sensor_ = s; }

void BLEClientHID::schedule_read_char_(ble_client::BLECharacteristic *characteristic) {
  if (characteristic != nullptr &&
      ((characteristic->properties & ESP_GATT_CHAR_PROP_BIT_READ) != 0)) {
    if (esp_ble_gattc_read_char(this->parent()->get_gattc_if(),
                                this->parent()->get_conn_id(),
                                characteristic->handle,
                                ESP_GATT_AUTH_REQ_NO_MITM) != ESP_OK) {
      ESP_LOGW(TAG, "read_char failed");
    }
    this->handles_to_read_.insert(std::make_pair(characteristic->handle, nullptr));
    return;
  }
  ESP_LOGW(TAG, "characteristic not found");
}

uint8_t *BLEClientHID::parse_characteristic_data_(ble_client::BLEService *service, uint16_t uuid) {
  using namespace ble_client;

  BLECharacteristic *characteristic = service->get_characteristic(uuid);
  if (characteristic == nullptr) {
    ESP_LOGD(TAG, "No characteristic with uuid %#X found on device", uuid);
    return nullptr;
  }
  if (handles_to_read_.count(characteristic->handle) >= 1) {
    ESP_LOGD(TAG,
             "Characteristic parsed for uuid %#X and handle %#X starts with %#X",
             uuid, characteristic->handle,
             *(handles_to_read_[characteristic->handle]->value_));
    return handles_to_read_[characteristic->handle]->value_;
  }
  ESP_LOGD(TAG, "Characteristic with uuid %#X and handle %#X not stored in handles_to_read",
           uuid, characteristic->handle);
  return nullptr;
}

void BLEClientHID::configure_hid_client_() {
  using namespace ble_client;

  BLEService *device_info_service = this->parent()->get_service(ESP_GATT_UUID_DEVICE_INFO_SVC);
  BLEService *hid_service = this->parent()->get_service(ESP_GATT_UUID_HID_SVC);
  BLEService *generic_access_service = this->parent()->get_service(0x1800);

  if (hid_service != nullptr) {
    BLECharacteristic *hid_report_map_char = hid_service->get_characteristic(ESP_GATT_UUID_HID_REPORT_MAP);
    if (hid_report_map_char != nullptr && this->handles_to_read_.count(hid_report_map_char->handle)) {
      ESP_LOGD(TAG, "Parse HID Report Map");

      HIDReportMap::esp_logd_report_map(this->handles_to_read_[hid_report_map_char->handle]->value_,
                                        this->handles_to_read_[hid_report_map_char->handle]->value_len_);

      this->hid_report_map_ = HIDReportMap::parse_report_map_data(
          this->handles_to_read_[hid_report_map_char->handle]->value_,
          this->handles_to_read_[hid_report_map_char->handle]->value_len_);

      ESP_LOGD(TAG, "Parse HID Report Map Done");
    }

    // Build handle->report_id mapping from RPT_REF_DESCR reads
    for (BLECharacteristic *hid_char : hid_service->characteristics) {
      if (hid_char->uuid.get_uuid().uuid.uuid16 != ESP_GATT_UUID_HID_REPORT) continue;

      BLEDescriptor *rpt_ref_desc = hid_char->get_descriptor(ESP_GATT_UUID_RPT_REF_DESCR);
      if (rpt_ref_desc != nullptr && this->handles_to_read_.count(rpt_ref_desc->handle)) {
        this->handle_report_id_.insert(std::make_pair(hid_char->handle,
                                                     this->handles_to_read_[rpt_ref_desc->handle]->value_[0]));
        ESP_LOGD(TAG, "Report ID for handle %d is %d", hid_char->handle,
                 this->handles_to_read_[rpt_ref_desc->handle]->value_[0]);
      }
    }
  }

  // Preferred connection params (0x2A04) => esp_ble_conn_update_params_t
  if (generic_access_service != nullptr) {
    uint8_t *t_conn_params = this->parse_characteristic_data_(generic_access_service, ESP_GATT_UUID_GAP_PREF_CONN_PARAM);
    if (t_conn_params != nullptr) {
      memset(&this->preferred_conn_params_, 0, sizeof(this->preferred_conn_params_));
      this->preferred_conn_params_.min_int = t_conn_params[0] | (t_conn_params[1] << 8);
      this->preferred_conn_params_.max_int = t_conn_params[2] | (t_conn_params[3] << 8);
      this->preferred_conn_params_.latency = t_conn_params[4] | (t_conn_params[5] << 8);
      this->preferred_conn_params_.timeout = t_conn_params[6] | (t_conn_params[7] << 8);
      memcpy(this->preferred_conn_params_.bda, this->parent()->get_remote_bda(), 6);

      this->preferred_params_valid_ = true;

      ESP_LOGI(TAG,
               "Got preferred connection parameters: interval: %.2f - %.2f ms, latency: %u, timeout: %.1f ms",
               this->preferred_conn_params_.min_int * 1.25f,
               this->preferred_conn_params_.max_int * 1.25f,
               this->preferred_conn_params_.latency,
               this->preferred_conn_params_.timeout * 10.f);
    }
  }

  // Delete read data (as in your original)
  for (auto &kv : this->handles_to_read_) {
    delete kv.second;
  }
  this->handles_to_read_.clear();

  // Decode is now ready; flush any buffered wake-key reports
  this->hid_decode_ready_ = (this->hid_report_map_ != nullptr) && (!this->handle_report_id_.empty());
  this->flush_pending_reports_();
}

}  // namespace ble_client_hid
}  // namespace esphome

#endif  // USE_ESP32
