#include "ble_client_hid.h"
#include "usages.h"

#ifdef USE_ESP32

namespace esphome {
namespace ble_client_hid {

static const char *const TAG = "ble_client_hid";

void BLEClientHID::loop() {
  switch (this->hid_state_) {
    case HIDState::BLE_CONNECTED:
      // Key change: enable notifications ASAP (before slow characteristic reads)
      this->register_notifications_();
      this->hid_state_ = HIDState::NOTIFICATIONS_REGISTERING;
      break;

    case HIDState::NOTIFICATIONS_REGISTERED:
      // Now do the slower reads (report map, report refs, etc.)
      this->read_client_characteristics_();
      this->hid_state_ = HIDState::READING_CHARS;
      break;

    case HIDState::READ_CHARS:
      // Parse report map + report refs; then apply connection params update
      this->configure_hid_client_();

      if (this->preferred_conn_params_valid_) {
        // Force a higher supervision timeout to reduce idle disconnects (first-key loss symptom).
        // Supervision timeout is in 10ms units; max is 3200 (32s).
        if (this->preferred_conn_params_.timeout < 3200) {
          this->preferred_conn_params_.timeout = 3200;
        }

        auto st = esp_ble_gap_update_conn_params(&this->preferred_conn_params_);
        ESP_LOGD(TAG, "esp_ble_gap_update_conn_params() => %d", st);
        this->hid_state_ = HIDState::CONN_PARAMS_UPDATING;
      } else {
        this->hid_state_ = HIDState::CONFIGURED;
      }

      // If we received reports “too early”, process them now
      this->flush_pending_notifies_();
      break;

    default:
      break;
  }
}

void BLEClientHID::dump_config() {
  ESP_LOGCONFIG(TAG, "BLE Client HID:");
  ESP_LOGCONFIG(TAG, "  MAC address        : %s", this->parent()->address_str());
}

bool BLEClientHID::can_process_input_reports_() const {
  // Avoid espbt::ClientState dependency; rely on our state machine + connection status.
  return (this->hid_state_ == HIDState::CONFIGURED || this->hid_state_ == HIDState::CONN_PARAMS_UPDATING) &&
         this->parent()->connected() &&
         (this->hid_report_map_ != nullptr);
}

void BLEClientHID::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
      ESP_LOGI(TAG,
               "Updated conn params to interval=%.2f ms, latency=%u, timeout=%.1f ms",
               param->update_conn_params.conn_int * 1.25f,
               param->update_conn_params.latency,
               param->update_conn_params.timeout * 10.0f);
      // Once conn params are updated, consider fully configured
      if (this->hid_state_ == HIDState::CONN_PARAMS_UPDATING) {
        this->hid_state_ = HIDState::CONFIGURED;
      }
      break;

    default:
      break;
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
    BLECharacteristic *pref_conn_params_char =
        generic_access_service->get_characteristic(ESP_GATT_UUID_GAP_PREF_CONN_PARAM);
    this->schedule_read_char_(pref_conn_params_char);
    this->schedule_read_char_(device_name_char);
  }

  if (device_info_service != nullptr) {
    BLECharacteristic *pnp_id_char = device_info_service->get_characteristic(ESP_GATT_UUID_PNP_ID);
    this->schedule_read_char_(pnp_id_char);

    BLECharacteristic *manufacturer_char = device_info_service->get_characteristic(ESP_GATT_UUID_MANU_NAME);
    this->schedule_read_char_(manufacturer_char);

    BLECharacteristic *serial_number_char =
        device_info_service->get_characteristic(ESP_GATT_UUID_SERIAL_NUMBER_STR);
    this->schedule_read_char_(serial_number_char);
  }

  if (hid_service != nullptr) {
    BLECharacteristic *hid_report_map_char = hid_service->get_characteristic(ESP_GATT_UUID_HID_REPORT_MAP);
    this->schedule_read_char_(hid_report_map_char);

    ESP_LOGD(TAG, "Found %d characteristics", hid_service->characteristics.size());

    for (auto *chr : hid_service->characteristics) {
      if (chr->uuid.get_uuid().uuid.uuid16 != ESP_GATT_UUID_HID_REPORT) continue;

      BLEDescriptor *rpt_ref_desc = chr->get_descriptor(ESP_GATT_UUID_RPT_REF_DESCR);
      if (rpt_ref_desc != nullptr) {
        if (esp_ble_gattc_read_char_descr(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                          rpt_ref_desc->handle, ESP_GATT_AUTH_REQ_NO_MITM) != ESP_OK) {
          ESP_LOGW(TAG, "scheduling reading of RPT_REF_DESCR failed.");
        }
        this->handles_to_read_.insert(std::make_pair(rpt_ref_desc->handle, nullptr));
      }
    }
  }

  // If nothing was scheduled, move forward so we don't stall.
  if (this->handles_to_read_.empty()) {
    this->hid_state_ = HIDState::READ_CHARS;
  }
}

void BLEClientHID::on_gatt_read_finished_(GATTReadData *data) {
  auto itr = this->handles_to_read_.find(data->handle_);
  if (itr != this->handles_to_read_.end()) {
    itr->second = data;
  } else {
    delete data;
    return;
  }

  // Check if all handles have been read
  for (auto const &element : this->handles_to_read_) {
    if (element.second == nullptr) return;
  }

  this->hid_state_ = HIDState::READ_CHARS;
}

void BLEClientHID::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                       esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_CONNECT_EVT: {
      auto ret = esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);
      if (ret) {
        ESP_LOGE(TAG, "[%d] [%s] esp_ble_set_encryption error, status=%d",
                 this->parent()->get_connection_index(), this->parent()->address_str(), ret);
      }

      esp_gap_conn_params_t params{};
      ret = esp_ble_get_current_conn_params(this->parent()->get_remote_bda(), &params);
      if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get conn params");
      } else {
        ESP_LOGI(TAG, "conn params: interval=%u, latency=%u, timeout=%u", params.interval, params.latency, params.timeout);
      }
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "[%s] Disconnected!", this->parent()->address_str());
      this->status_set_warning("Disconnected");
      this->hid_state_ = HIDState::IDLE;

      // Clear in-flight read state
      for (auto &kv : this->handles_to_read_) {
        delete kv.second;
      }
      this->handles_to_read_.clear();
      this->handles_waiting_for_notify_registration_ = 0;
      this->battery_handle_ = 0;
      this->cccd_by_char_handle_.clear();
      this->pending_notifies_.clear();
      break;
    }

    case ESP_GATTC_SEARCH_RES_EVT: {
      if (param->search_res.srvc_id.uuid.uuid.uuid16 == ESP_GATT_UUID_HID_SVC) {
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

      ESP_LOGD(TAG, "GATTC search finished with status code %d", param->search_cmpl.status);
      this->hid_state_ = HIDState::BLE_CONNECTED;

      esp_gap_conn_params_t params{};
      esp_err_t ret = esp_ble_get_current_conn_params(this->parent()->get_remote_bda(), &params);
      if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get conn params");
      } else {
        ESP_LOGI(TAG, "conn params: interval=%u, latency=%u, timeout=%u", params.interval, params.latency, params.timeout);
      }
      break;
    }

    case ESP_GATTC_READ_CHAR_EVT:
    case ESP_GATTC_READ_DESCR_EVT: {
      if (param->read.conn_id != this->parent()->get_conn_id()) break;
      if (param->read.status != ESP_OK) {
        ESP_LOGW(TAG, "GATTC read failed with status code %d", param->read.status);
        break;
      }
      auto *data = new GATTReadData(param->read.handle, param->read.value, param->read.value_len);
      this->on_gatt_read_finished_(data);
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.conn_id != this->parent()->get_conn_id()) break;

      if (param->notify.handle == this->battery_handle_) {
        if (this->battery_sensor_ != nullptr && param->notify.value_len >= 1) {
          this->battery_sensor_->publish_state(param->notify.value[0]);
        }
      } else {
        this->process_hid_notify_(param->notify.handle, param->notify.value, param->notify.value_len);
      }
      break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      if (param->reg_for_notify.conn_id != this->parent()->get_conn_id()) break;

      // We proactively wrote CCCDs in register_notifications_(), but keep state progression here.
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

void BLEClientHID::register_notifications_() {
  using namespace ble_client;

  BLEService *battery_service = this->parent()->get_service(ESP_GATT_UUID_BATTERY_SERVICE_SVC);
  BLEService *hid_service = this->parent()->get_service(ESP_GATT_UUID_HID_SVC);

  // Battery notifications (optional)
  if (battery_service != nullptr) {
    BLECharacteristic *battery_level_char = battery_service->get_characteristic(ESP_GATT_UUID_BATTERY_LEVEL);
    if (battery_level_char != nullptr && (battery_level_char->properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)) {
      this->battery_handle_ = battery_level_char->handle;

      if (auto *cccd = battery_level_char->get_descriptor(ESP_GATT_UUID_CHAR_CLIENT_CONFIG); cccd != nullptr) {
        this->cccd_by_char_handle_[battery_level_char->handle] = cccd->handle;
        this->write_cccd_(cccd->handle);
      }

      auto st = esp_ble_gattc_register_for_notify(this->parent()->get_gattc_if(), this->parent()->get_remote_bda(),
                                                 battery_level_char->handle);
      if (st != ESP_OK) {
        ESP_LOGW(TAG, "Register for notify failed for battery handle %d with status=%d",
                 battery_level_char->handle, st);
      } else {
        this->handles_waiting_for_notify_registration_++;
      }
    }
  }

  // HID report notifications (critical)
  if (hid_service != nullptr) {
    for (BLECharacteristic *hid_char : hid_service->characteristics) {
      if (hid_char->uuid.get_uuid().uuid.uuid16 != ESP_GATT_UUID_HID_REPORT) continue;
      if (!(hid_char->properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)) continue;

      if (auto *cccd = hid_char->get_descriptor(ESP_GATT_UUID_CHAR_CLIENT_CONFIG); cccd != nullptr) {
        this->cccd_by_char_handle_[hid_char->handle] = cccd->handle;
        this->write_cccd_(cccd->handle);
      }

      auto st = esp_ble_gattc_register_for_notify(this->parent()->get_gattc_if(), this->parent()->get_remote_bda(),
                                                 hid_char->handle);
      if (st != ESP_OK) {
        ESP_LOGW(TAG, "Register for notify failed for HID handle %d with status=%d", hid_char->handle, st);
      } else {
        this->handles_waiting_for_notify_registration_++;
      }
    }
  }

  // If nothing registered, advance immediately
  if (this->handles_waiting_for_notify_registration_ == 0) {
    this->hid_state_ = HIDState::NOTIFICATIONS_REGISTERED;
  }
}

void BLEClientHID::write_cccd_(uint16_t cccd_handle) {
  // Enable notifications (0x0001)
  uint8_t notify_en[2] = {0x01, 0x00};
  auto st = esp_ble_gattc_write_char_descr(this->parent()->get_gattc_if(), this->parent()->get_conn_id(), cccd_handle,
                                          sizeof(notify_en), notify_en, ESP_GATT_WRITE_TYPE_RSP,
                                          ESP_GATT_AUTH_REQ_NO_MITM);
  ESP_LOGD(TAG, "Write CCCD handle=%u => %d", cccd_handle, st);
}

void BLEClientHID::process_hid_notify_(uint16_t handle, const uint8_t *data, uint16_t len) {
  // If we are not ready to decode yet, buffer the raw bytes.
  if (!this->can_process_input_reports_()) {
    PendingNotify p;
    p.handle = handle;
    p.bytes.assign(data, data + len);

    // Keep buffer bounded (avoid RAM creep on noisy links)
    if (this->pending_notifies_.size() >= 16) {
      this->pending_notifies_.erase(this->pending_notifies_.begin());
    }
    this->pending_notifies_.push_back(std::move(p));
    return;
  }

  // Decode now
  this->flush_pending_notifies_();

  // Normal decode path
  uint8_t report_id = 0;
  if (this->handle_report_id_.count(handle) > 0) {
    report_id = this->handle_report_id_.at(handle);
  }

  std::vector<uint8_t> buf;
  buf.resize(len + 1);
  buf[0] = report_id;
  memcpy(&buf[1], data, len);

  std::vector<HIDReportItemValue> hid_report_values = this->hid_report_map_->parse(buf.data());
  if (hid_report_values.empty()) return;

  for (HIDReportItemValue value : hid_report_values) {
    std::string usage;
    if (USAGE_PAGES.count(value.usage.page) > 0 &&
        USAGE_PAGES.at(value.usage.page).usages_.count(value.usage.usage) > 0) {
      usage = USAGE_PAGES.at(value.usage.page).usages_.at(value.usage.usage);
    } else {
      usage = std::to_string(value.usage.page) + "_" + std::to_string(value.usage.usage);
    }

#ifdef USE_API
    this->fire_homeassistant_event("esphome.hid_events",
                                   {{"usage", usage}, {"value", std::to_string(value.value)}});
#endif

    if (this->last_event_usage_text_sensor_ != nullptr) {
      this->last_event_usage_text_sensor_->publish_state(usage);
    }
    if (this->last_event_value_sensor_ != nullptr) {
      this->last_event_value_sensor_->publish_state(value.value);
    }

    ESP_LOGI(TAG, "Send HID event to HomeAssistant: usage: %s, value: %d", usage.c_str(), value.value);
  }
}

void BLEClientHID::flush_pending_notifies_() {
  if (!this->can_process_input_reports_()) return;
  if (this->pending_notifies_.empty()) return;

  auto pending = std::move(this->pending_notifies_);
  this->pending_notifies_.clear();

  for (auto &p : pending) {
    this->process_hid_notify_(p.handle, p.bytes.data(), (uint16_t) p.bytes.size());
  }
}

uint8_t *BLEClientHID::parse_characteristic_data_(ble_client::BLEService *service, uint16_t uuid) {
  using namespace ble_client;

  BLECharacteristic *characteristic = service->get_characteristic(uuid);
  if (characteristic == nullptr) {
    ESP_LOGD(TAG, "No characteristic with uuid %#X found on device", uuid);
    return nullptr;
  }
  if (this->handles_to_read_.count(characteristic->handle) >= 1 &&
      this->handles_to_read_[characteristic->handle] != nullptr) {
    return this->handles_to_read_[characteristic->handle]->value_;
  }
  return nullptr;
}

void BLEClientHID::schedule_read_char_(ble_client::BLECharacteristic *characteristic) {
  if (characteristic != nullptr && ((characteristic->properties & ESP_GATT_CHAR_PROP_BIT_READ) != 0)) {
    if (esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                               characteristic->handle, ESP_GATT_AUTH_REQ_NO_MITM) != ESP_OK) {
      ESP_LOGW(TAG, "read_char failed");
    }
    this->handles_to_read_.insert(std::make_pair(characteristic->handle, nullptr));
    return;
  }
  ESP_LOGW(TAG, "characteristic not found / not readable");
}

void BLEClientHID::configure_hid_client_() {
  using namespace ble_client;

  BLEService *battery_service = this->parent()->get_service(ESP_GATT_UUID_BATTERY_SERVICE_SVC);
  BLEService *device_info_service = this->parent()->get_service(ESP_GATT_UUID_DEVICE_INFO_SVC);
  BLEService *hid_service = this->parent()->get_service(ESP_GATT_UUID_HID_SVC);
  BLEService *generic_access_service = this->parent()->get_service(0x1800);

  if (generic_access_service != nullptr) {
    uint8_t *t_device_name =
        this->parse_characteristic_data_(generic_access_service, ESP_GATT_UUID_GAP_DEVICE_NAME);
    this->device_name_ = (t_device_name != nullptr) ? (const char *) t_device_name : "Generic";

    uint8_t *t_conn_params =
        this->parse_characteristic_data_(generic_access_service, ESP_GATT_UUID_GAP_PREF_CONN_PARAM);
    if (t_conn_params != nullptr) {
      // GAP preferred connection parameters are little-endian: min_int, max_int, latency, timeout (each uint16)
      uint16_t min_int = t_conn_params[0] | (t_conn_params[1] << 8);
      uint16_t max_int = t_conn_params[2] | (t_conn_params[3] << 8);
      uint16_t latency = t_conn_params[4] | (t_conn_params[5] << 8);
      uint16_t timeout = t_conn_params[6] | (t_conn_params[7] << 8);

      // Build the correct struct for esp_ble_gap_update_conn_params()
      memset(&this->preferred_conn_params_, 0, sizeof(this->preferred_conn_params_));
      this->preferred_conn_params_.min_int = min_int;
      this->preferred_conn_params_.max_int = max_int;
      this->preferred_conn_params_.latency = latency;
      this->preferred_conn_params_.timeout = timeout;
      memcpy(this->preferred_conn_params_.bda, this->parent()->get_remote_bda(), 6);
      this->preferred_conn_params_valid_ = true;

      ESP_LOGI(TAG,
               "Got preferred connection parameters: interval %.2f - %.2f ms, latency %u, timeout %.1f ms",
               min_int * 1.25f, max_int * 1.25f, latency, timeout * 10.0f);
    }
  }

  if (device_info_service != nullptr) {
    BLECharacteristic *pnp_id_char = device_info_service->get_characteristic(ESP_GATT_UUID_PNP_ID);
    if (pnp_id_char != nullptr && this->handles_to_read_.count(pnp_id_char->handle) &&
        this->handles_to_read_[pnp_id_char->handle] != nullptr) {
      uint8_t *rdata = this->handles_to_read_[pnp_id_char->handle]->value_;
      this->vendor_id_ = *((uint16_t *) &rdata[1]);
      this->product_id_ = *((uint16_t *) &rdata[3]);
      this->version_ = *((uint16_t *) &rdata[5]);
    }

    uint8_t *t_manufacturer = this->parse_characteristic_data_(device_info_service, ESP_GATT_UUID_MANU_NAME);
    this->manufacturer_ = (t_manufacturer != nullptr) ? (const char *) t_manufacturer : "Generic";

    uint8_t *t_serial = this->parse_characteristic_data_(device_info_service, ESP_GATT_UUID_SERIAL_NUMBER_STR);
    this->serial_number_ = (t_serial != nullptr) ? (const char *) t_serial : "000000";
  }

  if (hid_service != nullptr) {
    BLECharacteristic *hid_report_map_char = hid_service->get_characteristic(ESP_GATT_UUID_HID_REPORT_MAP);
    if (hid_report_map_char != nullptr &&
        this->handles_to_read_.count(hid_report_map_char->handle) &&
        this->handles_to_read_[hid_report_map_char->handle] != nullptr) {

      ESP_LOGD(TAG, "Parse HID Report Map");
      HIDReportMap::esp_logd_report_map(this->handles_to_read_[hid_report_map_char->handle]->value_,
                                       this->handles_to_read_[hid_report_map_char->handle]->value_len_);
      this->hid_report_map_ = HIDReportMap::parse_report_map_data(
          this->handles_to_read_[hid_report_map_char->handle]->value_,
          this->handles_to_read_[hid_report_map_char->handle]->value_len_);
      ESP_LOGD(TAG, "Parse HID Report Map Done");
    }

    // Fill report IDs per handle (from RPT_REF_DESCR reads)
    for (BLECharacteristic *hid_char : hid_service->characteristics) {
      if (hid_char->uuid.get_uuid().uuid.uuid16 != ESP_GATT_UUID_HID_REPORT) continue;

      BLEDescriptor *rpt_ref_desc = hid_char->get_descriptor(ESP_GATT_UUID_RPT_REF_DESCR);
      if (rpt_ref_desc != nullptr &&
          this->handles_to_read_.count(rpt_ref_desc->handle) &&
          this->handles_to_read_[rpt_ref_desc->handle] != nullptr) {
        uint8_t report_id = this->handles_to_read_[rpt_ref_desc->handle]->value_[0];
        this->handle_report_id_[hid_char->handle] = report_id;
        ESP_LOGD(TAG, "Report ID for handle %d is %d", hid_char->handle, report_id);
      }
    }
  }

  // Cleanup read data
  for (auto &kv : this->handles_to_read_) {
    delete kv.second;
  }
  this->handles_to_read_.clear();
}

void BLEClientHID::register_last_event_value_sensor(sensor::Sensor *last_event_value_sensor) {
  this->last_event_value_sensor_ = last_event_value_sensor;
}

void BLEClientHID::register_battery_sensor(sensor::Sensor *battery_sensor) {
  this->battery_sensor_ = battery_sensor;
}

void BLEClientHID::register_last_event_usage_text_sensor(text_sensor::TextSensor *last_event_usage_text_sensor) {
  this->last_event_usage_text_sensor_ = last_event_usage_text_sensor;
}

}  // namespace ble_client_hid
}  // namespace esphome

#endif  // USE_ESP32
