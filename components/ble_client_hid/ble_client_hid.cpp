#include "ble_client_hid.h"

#ifdef USE_ESP32

namespace esphome {
namespace ble_client_hid {

const char *const BLEClientHID::TAG = "ble_client_hid";

// Change this string when you want to *prove* which code is running.
static const char *const BUILD_FINGERPRINT = "ble_client_hid-2026-01-02-A";

void BLEClientHID::dump_config() {
  ESP_LOGCONFIG(TAG, "BLE Client HID:");
  ESP_LOGCONFIG(TAG, "  build fingerprint : %s", BUILD_FINGERPRINT);
  ESP_LOGCONFIG(TAG, "  MAC address        : %s", this->parent()->address_str());
}

void BLEClientHID::loop() {
  switch (this->hid_state_) {
    case HIDState::BLE_CONNECTED:
      this->read_client_characteristics_();
      this->hid_state_ = HIDState::READING_CHARS;
      break;

    case HIDState::READ_CHARS:
      this->configure_hid_client_();
      this->hid_state_ = HIDState::NOTIFICATIONS_REGISTERING;
      break;

    case HIDState::NOTIFICATIONS_REGISTERED: {
      // Apply preferred connection params (non-blocking). This is the one change aimed at
      // reducing idle disconnects (and therefore reducing “first key after wake” loss).
      if (this->have_preferred_conn_params_) {
        // IMPORTANT: ESP-IDF uses timeout units of 10ms. Many remotes advertise very short
        // timeouts; that can lead to aggressive link drops. Raise to >= 15s.
        this->preferred_conn_params_.timeout =
            std::max<uint16_t>(this->preferred_conn_params_.timeout, 1500);  // 1500 * 10ms = 15s

        esp_err_t err = esp_ble_gap_update_conn_params(&this->preferred_conn_params_);
        if (err != ESP_OK) {
          ESP_LOGW(TAG, "esp_ble_gap_update_conn_params failed (%d)", err);
        } else {
          ESP_LOGI(TAG,
                   "Requested conn params: interval %.2f-%.2f ms, latency=%u, timeout=%.1f ms",
                   this->preferred_conn_params_.min_int * 1.25f,
                   this->preferred_conn_params_.max_int * 1.25f,
                   this->preferred_conn_params_.latency,
                   this->preferred_conn_params_.timeout * 10.0f);
        }
      }
      this->hid_state_ = HIDState::CONFIGURED;
      break;
    }

    default:
      break;
  }
}

bool BLEClientHID::can_process_input_reports_() {
  // Parent pointer is non-const in ESPHome, so keep this helper non-const.
  return this->parent() != nullptr && this->parent()->connected() &&
         (this->hid_state_ == HIDState::CONFIGURED ||
          this->hid_state_ == HIDState::NOTIFICATIONS_REGISTERED ||
          this->hid_state_ == HIDState::NOTIFICATIONS_REGISTERING);
}

void BLEClientHID::gattc_event_handler(esp_gattc_cb_event_t event,
                                       esp_gatt_if_t gattc_if,
                                       esp_ble_gattc_cb_param_t *param) {
  auto *p_data = param;

  switch (event) {
    case ESP_GATTC_CONNECT_EVT: {
      // Request encryption (helps some HID devices behave “like a real host”)
      esp_err_t ret = esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);
      if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[%s] esp_ble_set_encryption returned %d", this->parent()->address_str(), ret);
      }
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "[%s] Disconnected!", this->parent()->address_str());
      this->status_set_warning("Disconnected");

      // Reset state
      this->hid_state_ = HIDState::IDLE;
      this->battery_handle_ = 0;
      this->handles_waiting_for_notify_registration_ = 0;
      this->have_preferred_conn_params_ = false;
      this->handle_report_id_.clear();

      // Clear published sensors (optional)
      if (this->last_event_usage_text_sensor_ != nullptr) this->last_event_usage_text_sensor_->publish_state("");
      if (this->last_event_value_sensor_ != nullptr) this->last_event_value_sensor_->publish_state(NAN);

      break;
    }

    case ESP_GATTC_SEARCH_RES_EVT: {
      // Detect HID service
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
        this->status_set_warning("No HID service");
        break;
      }

      ESP_LOGI(TAG, "[%s] Service discovery complete", this->parent()->address_str());
      this->status_clear_warning();
      this->hid_state_ = HIDState::BLE_CONNECTED;
      break;
    }

    case ESP_GATTC_READ_CHAR_EVT:
    case ESP_GATTC_READ_DESCR_EVT: {
      if (param->read.conn_id != this->parent()->get_conn_id()) break;
      if (param->read.status != ESP_OK) {
        ESP_LOGW(TAG, "GATT read failed status=%d handle=%d", param->read.status, param->read.handle);
        break;
      }
      auto *data = new GATTReadData(param->read.handle, param->read.value, param->read.value_len);
      this->on_gatt_read_finished_(data);
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (!this->can_process_input_reports_()) break;

      // Battery notifications
      if (p_data->notify.handle == this->battery_handle_) {
        if (this->battery_sensor_ != nullptr && p_data->notify.value_len >= 1) {
          this->battery_sensor_->publish_state(p_data->notify.value[0]);
        }
      } else {
        // HID report notifications
        this->process_hid_report_(p_data->notify.handle, p_data->notify.value, p_data->notify.value_len);
      }
      break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      // ESP-IDF’s reg_for_notify param does not reliably expose conn_id across versions.
      // We track completion by counting down.
      this->handles_waiting_for_notify_registration_--;
      if (this->handles_waiting_for_notify_registration_ <= 0) {
        this->handles_waiting_for_notify_registration_ = 0;
        this->hid_state_ = HIDState::NOTIFICATIONS_REGISTERED;
      }
      break;
    }

    default:
      break;
  }
}

void BLEClientHID::read_client_characteristics_() {
  using namespace ble_client;

  BLEService *battery_service = this->parent()->get_service(ESP_GATT_UUID_BATTERY_SERVICE_SVC);
  BLEService *device_info_service = this->parent()->get_service(ESP_GATT_UUID_DEVICE_INFO_SVC);
  BLEService *hid_service = this->parent()->get_service(ESP_GATT_UUID_HID_SVC);
  BLEService *generic_access_service = this->parent()->get_service(0x1800);  // GAP

  if (generic_access_service != nullptr) {
    auto *device_name_char = generic_access_service->get_characteristic(ESP_GATT_UUID_GAP_DEVICE_NAME);
    auto *pref_conn_params_char = generic_access_service->get_characteristic(ESP_GATT_UUID_GAP_PREF_CONN_PARAM);
    this->schedule_read_char_(pref_conn_params_char);
    this->schedule_read_char_(device_name_char);
  }

  if (device_info_service != nullptr) {
    auto *pnp_id_char = device_info_service->get_characteristic(ESP_GATT_UUID_PNP_ID);
    auto *manufacturer_char = device_info_service->get_characteristic(ESP_GATT_UUID_MANU_NAME);
    auto *serial_number_char = device_info_service->get_characteristic(ESP_GATT_UUID_SERIAL_NUMBER_STR);
    this->schedule_read_char_(pnp_id_char);
    this->schedule_read_char_(manufacturer_char);
    this->schedule_read_char_(serial_number_char);
  }

  if (hid_service != nullptr) {
    // Report map
    auto *hid_report_map_char = hid_service->get_characteristic(ESP_GATT_UUID_HID_REPORT_MAP);
    this->schedule_read_char_(hid_report_map_char);

    // Report reference descriptors (needed to map handle->report_id)
    for (auto *chr : hid_service->characteristics) {
      if (chr->uuid.get_uuid().uuid.uuid16 != ESP_GATT_UUID_HID_REPORT) continue;
      BLEDescriptor *rpt_ref_desc = chr->get_descriptor(ESP_GATT_UUID_RPT_REF_DESCR);
      if (rpt_ref_desc != nullptr) {
        this->schedule_read_descr_(rpt_ref_desc);
      }
    }
  }

  // We don’t *need* to read battery level for notify to work, but we set up notify later.
  (void) battery_service;

  // When all reads complete, on_gatt_read_finished_ will advance state to READ_CHARS
}

void BLEClientHID::schedule_read_char_(ble_client::BLECharacteristic *characteristic) {
  if (characteristic == nullptr) return;
  if ((characteristic->properties & ESP_GATT_CHAR_PROP_BIT_READ) == 0) return;

  if (esp_ble_gattc_read_char(this->parent()->get_gattc_if(),
                              this->parent()->get_conn_id(),
                              characteristic->handle,
                              ESP_GATT_AUTH_REQ_NO_MITM) != ESP_OK) {
    ESP_LOGW(TAG, "esp_ble_gattc_read_char failed handle=%u", characteristic->handle);
    return;
  }
  this->handles_to_read_.insert(std::make_pair(characteristic->handle, nullptr));
}

void BLEClientHID::schedule_read_descr_(ble_client::BLEDescriptor *descriptor) {
  if (descriptor == nullptr) return;

  if (esp_ble_gattc_read_char_descr(this->parent()->get_gattc_if(),
                                    this->parent()->get_conn_id(),
                                    descriptor->handle,
                                    ESP_GATT_AUTH_REQ_NO_MITM) != ESP_OK) {
    ESP_LOGW(TAG, "esp_ble_gattc_read_char_descr failed handle=%u", descriptor->handle);
    return;
  }
  this->handles_to_read_.insert(std::make_pair(descriptor->handle, nullptr));
}

void BLEClientHID::on_gatt_read_finished_(GATTReadData *data) {
  auto itr = this->handles_to_read_.find(data->handle_);
  if (itr != this->handles_to_read_.end()) {
    itr->second = data;
  } else {
    delete data;
    return;
  }

  // Check if all scheduled reads are complete
  for (auto const &kv : this->handles_to_read_) {
    if (kv.second == nullptr) return;
  }

  this->hid_state_ = HIDState::READ_CHARS;
}

uint8_t *BLEClientHID::parse_characteristic_data_(ble_client::BLEService *service, uint16_t uuid16) {
  using namespace ble_client;

  if (service == nullptr) return nullptr;

  BLECharacteristic *characteristic = service->get_characteristic(uuid16);
  if (characteristic == nullptr) return nullptr;

  auto it = this->handles_to_read_.find(characteristic->handle);
  if (it == this->handles_to_read_.end() || it->second == nullptr) return nullptr;

  return it->second->value_;
}

void BLEClientHID::configure_hid_client_() {
  using namespace ble_client;

  BLEService *battery_service = this->parent()->get_service(ESP_GATT_UUID_BATTERY_SERVICE_SVC);
  BLEService *device_info_service = this->parent()->get_service(ESP_GATT_UUID_DEVICE_INFO_SVC);
  BLEService *hid_service = this->parent()->get_service(ESP_GATT_UUID_HID_SVC);
  BLEService *generic_access_service = this->parent()->get_service(0x1800);  // GAP

  // GAP: device name + preferred conn params
  if (generic_access_service != nullptr) {
    uint8_t *t_device_name = this->parse_characteristic_data_(generic_access_service, ESP_GATT_UUID_GAP_DEVICE_NAME);
    if (t_device_name != nullptr) this->device_name_ = reinterpret_cast<const char *>(t_device_name);

    uint8_t *t_conn_params = this->parse_characteristic_data_(generic_access_service, ESP_GATT_UUID_GAP_PREF_CONN_PARAM);
    if (t_conn_params != nullptr) {
      // 8 bytes: min_int, max_int, latency, timeout (all uint16 little-endian)
      this->preferred_conn_params_.min_int = t_conn_params[0] | (t_conn_params[1] << 8);
      this->preferred_conn_params_.max_int = t_conn_params[2] | (t_conn_params[3] << 8);
      this->preferred_conn_params_.latency = t_conn_params[4] | (t_conn_params[5] << 8);
      this->preferred_conn_params_.timeout = t_conn_params[6] | (t_conn_params[7] << 8);
      memcpy(this->preferred_conn_params_.bda, this->parent()->get_remote_bda(), 6);
      this->have_preferred_conn_params_ = true;

      ESP_LOGI(TAG,
               "Got preferred conn params: interval %.2f-%.2f ms, latency=%u, timeout=%.1f ms",
               this->preferred_conn_params_.min_int * 1.25f,
               this->preferred_conn_params_.max_int * 1.25f,
               this->preferred_conn_params_.latency,
               this->preferred_conn_params_.timeout * 10.0f);
    }
  }

  // Device Info: PnP, manufacturer, serial
  if (device_info_service != nullptr) {
    BLECharacteristic *pnp_id_char = device_info_service->get_characteristic(ESP_GATT_UUID_PNP_ID);
    if (pnp_id_char != nullptr) {
      auto it = this->handles_to_read_.find(pnp_id_char->handle);
      if (it != this->handles_to_read_.end() && it->second != nullptr && it->second->value_len_ >= 7) {
        uint8_t *rdata = it->second->value_;
        this->vendor_id_ = *reinterpret_cast<uint16_t *>(&rdata[1]);
        this->product_id_ = *reinterpret_cast<uint16_t *>(&rdata[3]);
        this->version_ = *reinterpret_cast<uint16_t *>(&rdata[5]);
      }
    }

    uint8_t *t_manufacturer = this->parse_characteristic_data_(device_info_service, ESP_GATT_UUID_MANU_NAME);
    if (t_manufacturer != nullptr) this->manufacturer_ = reinterpret_cast<const char *>(t_manufacturer);

    uint8_t *t_serial = this->parse_characteristic_data_(device_info_service, ESP_GATT_UUID_SERIAL_NUMBER_STR);
    if (t_serial != nullptr) this->serial_number_ = reinterpret_cast<const char *>(t_serial);
  }

  // HID: parse report map + map report IDs + register notify
  if (hid_service != nullptr) {
    BLECharacteristic *hid_report_map_char = hid_service->get_characteristic(ESP_GATT_UUID_HID_REPORT_MAP);
    if (hid_report_map_char != nullptr) {
      auto it = this->handles_to_read_.find(hid_report_map_char->handle);
      if (it != this->handles_to_read_.end() && it->second != nullptr) {
        ESP_LOGD(TAG, "Parse HID Report Map");
        HIDReportMap::esp_logd_report_map(it->second->value_, it->second->value_len_);
        this->hid_report_map_ = HIDReportMap::parse_report_map_data(it->second->value_, it->second->value_len_);
        ESP_LOGD(TAG, "Parse HID Report Map Done");
      }
    }

    // Register notify for each HID input report characteristic and capture Report IDs
    for (BLECharacteristic *hid_char : hid_service->characteristics) {
      if (hid_char->uuid.get_uuid().uuid.uuid16 != ESP_GATT_UUID_HID_REPORT) continue;

      // Map handle -> report_id using Report Reference descriptor (0x2908)
      BLEDescriptor *rpt_ref_desc = hid_char->get_descriptor(ESP_GATT_UUID_RPT_REF_DESCR);
      if (rpt_ref_desc != nullptr) {
        auto it = this->handles_to_read_.find(rpt_ref_desc->handle);
        if (it != this->handles_to_read_.end() && it->second != nullptr && it->second->value_len_ >= 1) {
          uint8_t report_id = it->second->value_[0];
          this->handle_report_id_.insert(std::make_pair(hid_char->handle, report_id));
          ESP_LOGD(TAG, "Report ID for handle %u is %u", hid_char->handle, report_id);
        }
      }

      // Register for notify if supported
      if ((hid_char->properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) != 0) {
        esp_err_t status = esp_ble_gattc_register_for_notify(this->parent()->get_gattc_if(),
                                                            this->parent()->get_remote_bda(),
                                                            hid_char->handle);
        if (status != ESP_OK) {
          ESP_LOGW(TAG, "Register for notify failed for handle %u (status=%d)", hid_char->handle, status);
        } else {
          this->handles_waiting_for_notify_registration_++;
        }
      }
    }
  }

  // Battery notify
  if (battery_service != nullptr) {
    BLECharacteristic *battery_level_char = battery_service->get_characteristic(ESP_GATT_UUID_BATTERY_LEVEL);
    if (battery_level_char != nullptr && ((battery_level_char->properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) != 0)) {
      this->battery_handle_ = battery_level_char->handle;
      esp_err_t status = esp_ble_gattc_register_for_notify(this->parent()->get_gattc_if(),
                                                          this->parent()->get_remote_bda(),
                                                          battery_level_char->handle);
      if (status != ESP_OK) {
        ESP_LOGW(TAG, "Register for notify failed for battery handle %u (status=%d)", battery_level_char->handle, status);
      } else {
        this->handles_waiting_for_notify_registration_++;
      }
    }
  }

  // Cleanup read data
  for (auto &kv : this->handles_to_read_) {
    delete kv.second;
  }
  this->handles_to_read_.clear();
}

void BLEClientHID::process_hid_report_(uint16_t handle, const uint8_t *value, uint16_t value_len) {
  if (this->hid_report_map_ == nullptr) return;
  if (this->handle_report_id_.count(handle) == 0) return;

  uint8_t report_id = this->handle_report_id_[handle];

  // Prepend report_id (parser expects [report_id | payload...])
  uint8_t *data = new uint8_t[value_len + 1];
  data[0] = report_id;
  memcpy(data + 1, value, value_len);

  std::vector<HIDReportItemValue> hid_report_values = this->hid_report_map_->parse(data);
  delete[] data;

  if (hid_report_values.empty()) return;

  for (const HIDReportItemValue &v : hid_report_values) {
    std::string usage;

    if (USAGE_PAGES.count(v.usage.page) > 0 &&
        USAGE_PAGES.at(v.usage.page).usages_.count(v.usage.usage) > 0) {
      usage = USAGE_PAGES.at(v.usage.page).usages_.at(v.usage.usage);
    } else {
      usage = std::to_string(v.usage.page) + "_" + std::to_string(v.usage.usage);
    }

    // Fire HA event (must be map<string,string>)
    std::map<std::string, std::string> evt;
    evt["usage"] = usage;
    evt["value"] = std::to_string(v.value);
    this->fire_homeassistant_event("esphome.hid_events", evt);

    ESP_LOGI(TAG, "Send HID event to HomeAssistant: usage: %s, value: %d", usage.c_str(), v.value);

    if (this->last_event_usage_text_sensor_ != nullptr) {
      this->last_event_usage_text_sensor_->publish_state(usage);
    }
    if (this->last_event_value_sensor_ != nullptr) {
      this->last_event_value_sensor_->publish_state(v.value);
    }
  }
}

void BLEClientHID::register_battery_sensor(sensor::Sensor *battery_sensor) {
  this->battery_sensor_ = battery_sensor;
}

void BLEClientHID::register_last_event_value_sensor(sensor::Sensor *last_event_value_sensor) {
  this->last_event_value_sensor_ = last_event_value_sensor;
}

void BLEClientHID::register_last_event_usage_text_sensor(text_sensor::TextSensor *last_event_usage_text_sensor) {
  this->last_event_usage_text_sensor_ = last_event_usage_text_sensor;
}

}  // namespace ble_client_hid
}  // namespace esphome

#endif  // USE_ESP32
