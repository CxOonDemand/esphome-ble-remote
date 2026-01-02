#include "ble_client_hid.h"

#ifdef USE_ESP32

namespace esphome {
namespace ble_client_hid {

static const char *const TAG = "ble_client_hid";

static constexpr uint16_t UUID_GAP_SERVICE = 0x1800;
static constexpr uint16_t UUID_GAP_PREF_CONN_PARAM = 0x2A04;

void BLEClientHID::dump_config() {
  ESP_LOGCONFIG(TAG, "BLE Client HID:");
  ESP_LOGCONFIG(TAG, "  MAC address        : %s", this->parent()->address_str());
}

bool BLEClientHID::ready_for_reports_() {
  // We consider ourselves ready once:
  // - report map parsed
  // - report-id mapping is known (at least one handle)
  // - notifications have been registered (counter hit zero)
  return (this->hid_report_map_ != nullptr) &&
         (!this->handle_report_id_.empty()) &&
         (this->hid_state_ >= HIDState::NOTIFICATIONS_REGISTERED);
}

void BLEClientHID::buffer_input_report_(uint16_t handle, const uint8_t *value, uint16_t value_len) {
  // Prevent unbounded growth
  if (this->pending_reports_.size() >= MAX_PENDING_REPORTS) {
    this->pending_reports_.erase(this->pending_reports_.begin());
  }
  PendingNotify pn;
  pn.handle = handle;
  pn.value.assign(value, value + value_len);
  this->pending_reports_.push_back(std::move(pn));
}

void BLEClientHID::flush_buffered_reports_() {
  if (!this->ready_for_reports_() || this->pending_reports_.empty())
    return;

  ESP_LOGI(TAG, "Flushing %u buffered HID report(s)", (unsigned) this->pending_reports_.size());

  for (auto &pn : this->pending_reports_) {
    this->process_input_report_(pn.handle, pn.value.data(), (uint16_t) pn.value.size());
  }
  this->pending_reports_.clear();
}

void BLEClientHID::request_conn_params_(uint16_t min_int, uint16_t max_int, uint16_t latency, uint16_t timeout) {
  // esp_ble_conn_update_params_t uses:
  // - interval units: 1.25ms
  // - timeout units: 10ms
  esp_ble_conn_update_params_t p{};
  memcpy(p.bda, this->parent()->get_remote_bda(), 6);
  p.min_int = min_int;
  p.max_int = max_int;
  p.latency = latency;

  // Enforce a “don’t disconnect just because the remote naps” timeout floor.
  // 2000 => 20 seconds (in 10ms units).
  if (timeout < 2000) timeout = 2000;
  if (timeout > 3200) timeout = 3200;  // typical max

  p.timeout = timeout;

  esp_err_t err = esp_ble_gap_update_conn_params(&p);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "esp_ble_gap_update_conn_params failed: %d", err);
  } else {
    this->conn_params_request_sent_ = true;
    ESP_LOGI(TAG, "Requested conn params: interval %.2f-%.2f ms, latency %u, timeout %.1f ms",
             p.min_int * 1.25f, p.max_int * 1.25f, p.latency, p.timeout * 10.f);
  }
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

    case HIDState::NOTIFICATIONS_REGISTERED:
      // Once notify is registered, request better conn params (once).
      if (!this->conn_params_request_sent_) {
        // If GAP preferred params were read, use them (but with enforced timeout floor).
        // If they were not read, request a conservative default:
        //  - 30ms interval (24 units), latency 49, timeout 20s.
        if (this->preferred_conn_params_.min_int != 0 && this->preferred_conn_params_.max_int != 0) {
          this->request_conn_params_(this->preferred_conn_params_.min_int,
                                     this->preferred_conn_params_.max_int,
                                     this->preferred_conn_params_.latency,
                                     this->preferred_conn_params_.timeout);
        } else {
          this->request_conn_params_(24, 24, 49, 2000);
        }
      }
      this->hid_state_ = HIDState::CONN_PARAMS_UPDATING;

      // If the “first key” arrived before we were ready, flush it now.
      this->flush_buffered_reports_();
      break;

    default:
      break;
  }
}

void BLEClientHID::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
      ESP_LOGI(TAG,
               "Updated conn params: interval=%.2f ms, latency=%u, timeout=%.1f ms",
               param->update_conn_params.conn_int * 1.25f,
               param->update_conn_params.latency,
               param->update_conn_params.timeout * 10.f);

      // Mark as configured once update is done; flush any buffered reports again.
      this->hid_state_ = HIDState::CONFIGURED;
      this->flush_buffered_reports_();
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
  BLEService *generic_access_service = this->parent()->get_service(UUID_GAP_SERVICE);

  if (generic_access_service != nullptr) {
    BLECharacteristic *device_name_char =
        generic_access_service->get_characteristic(ESP_GATT_UUID_GAP_DEVICE_NAME);
    BLECharacteristic *pref_conn_params_char =
        generic_access_service->get_characteristic(UUID_GAP_PREF_CONN_PARAM);

    this->schedule_read_char_(pref_conn_params_char);
    this->schedule_read_char_(device_name_char);
  }

  if (device_info_service != nullptr) {
    BLECharacteristic *pnp_id_char =
        device_info_service->get_characteristic(ESP_GATT_UUID_PNP_ID);
    this->schedule_read_char_(pnp_id_char);

    BLECharacteristic *manufacturer_char =
        device_info_service->get_characteristic(ESP_GATT_UUID_MANU_NAME);
    this->schedule_read_char_(manufacturer_char);

    BLECharacteristic *serial_number_char =
        device_info_service->get_characteristic(ESP_GATT_UUID_SERIAL_NUMBER_STR);
    this->schedule_read_char_(serial_number_char);
  }

  if (hid_service != nullptr) {
    BLECharacteristic *hid_report_map_char =
        hid_service->get_characteristic(ESP_GATT_UUID_HID_REPORT_MAP);
    this->schedule_read_char_(hid_report_map_char);

    ESP_LOGD(TAG, "Found %d characteristics", hid_service->characteristics.size());

    for (auto *chr : hid_service->characteristics) {
      // HID Report characteristic UUID is 0x2A4D (ESP_GATT_UUID_HID_REPORT)
      if (chr->uuid.get_uuid().uuid.uuid16 != ESP_GATT_UUID_HID_REPORT)
        continue;

      BLEDescriptor *rpt_ref_desc = chr->get_descriptor(ESP_GATT_UUID_RPT_REF_DESCR);
      if (rpt_ref_desc != nullptr) {
        if (esp_ble_gattc_read_char_descr(this->parent()->get_gattc_if(),
                                          this->parent()->get_conn_id(),
                                          rpt_ref_desc->handle,
                                          ESP_GATT_AUTH_REQ_NO_MITM) != ESP_OK) {
          ESP_LOGW(TAG, "Scheduling reading of RPT_REF_DESCR failed.");
        }
        this->handles_to_read_.insert(std::make_pair(rpt_ref_desc->handle, nullptr));
      }
    }
  }

  // Note: battery level will be configured for notify later, no need to read now.
}

void BLEClientHID::on_gatt_read_finished_(GATTReadData *data) {
  auto itr = this->handles_to_read_.find(data->handle_);
  if (itr != this->handles_to_read_.end()) {
    itr->second = data;
  }

  for (auto const &element : this->handles_to_read_) {
    if (element.second == nullptr) {
      return;  // wait until all reads arrived
    }
  }

  this->hid_state_ = HIDState::READ_CHARS;
}

void BLEClientHID::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                       esp_ble_gattc_cb_param_t *param) {
  auto *p_data = param;

  switch (event) {
    case ESP_GATTC_CONNECT_EVT: {
      // Reset state for new connection
      this->pending_reports_.clear();
      this->handle_report_id_.clear();
      this->handles_to_read_.clear();
      this->handles_waiting_for_notify_registration_ = 0;
      this->conn_params_request_sent_ = false;
      this->preferred_conn_params_ = {};

      // Request encryption (as in your working code)
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
        ESP_LOGI(TAG, "conn params: interval=%u, latency=%u, timeout=%u",
                 params.interval, params.latency, params.timeout);
      }
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "[%s] Disconnected!", this->parent()->address_str());
      this->status_set_warning("Disconnected");
      this->hid_state_ = HIDState::DISCONNECTED;

      // Drop any buffered reports; next connection will reconfigure.
      this->pending_reports_.clear();
      this->handle_report_id_.clear();

      // If we had a parsed report map, keep it; handles may remain stable.
      // (If you prefer, you can delete + null it here to force re-parse.)
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

      esp_gap_conn_params_t params{};
      esp_err_t ret = esp_ble_get_current_conn_params(this->parent()->get_remote_bda(), &params);
      if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get conn params");
      } else {
        ESP_LOGI(TAG, "conn params: interval=%u, latency=%u, timeout=%u",
                 params.interval, params.latency, params.timeout);
      }
      break;
    }

    case ESP_GATTC_READ_CHAR_EVT:
    case ESP_GATTC_READ_DESCR_EVT: {
      if (param->read.conn_id != this->parent()->get_conn_id())
        break;
      if (param->read.status != ESP_OK) {
        ESP_LOGW(TAG, "GATTC read failed with status code %d", param->read.status);
        break;
      }
      auto *data = new GATTReadData(param->read.handle, param->read.value, param->read.value_len);
      this->on_gatt_read_finished_(data);
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.conn_id != this->parent()->get_conn_id())
        break;

      if (p_data->notify.handle == this->battery_handle_) {
        uint8_t battery_level = p_data->notify.value[0];
        if (this->battery_sensor_ != nullptr) {
          this->battery_sensor_->publish_state(battery_level);
        }
        break;
      }

      // IMPORTANT CHANGE #1:
      // If the first key arrives before we’ve built report-id mapping or parsed the report map,
      // buffer it and replay once ready. This is the most common “missing first key” mechanism.
      if (!this->ready_for_reports_()) {
        this->buffer_input_report_(p_data->notify.handle, p_data->notify.value, p_data->notify.value_len);
        break;
      }

      this->process_input_report_(p_data->notify.handle, p_data->notify.value, p_data->notify.value_len);
      break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      // IDF5 struct fields differ; do not rely on conn_id here.
      // We just count down; ESPHome core handles CCCD writes after this.
      this->handles_waiting_for_notify_registration_ =
          (this->handles_waiting_for_notify_registration_ > 0)
              ? (this->handles_waiting_for_notify_registration_ - 1)
              : 0;

      if (this->handles_waiting_for_notify_registration_ == 0) {
        this->hid_state_ = HIDState::NOTIFICATIONS_REGISTERED;
        // Flush if we can already parse (report map + report ids ready)
        this->flush_buffered_reports_();
      }
      break;
    }

    default:
      break;
  }
}

void BLEClientHID::schedule_read_char_(ble_client::BLECharacteristic *characteristic) {
  using namespace ble_client;

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
  ESP_LOGD(TAG, "characteristic not found or not readable");
}

static uint8_t *get_read_value_or_null(std::map<uint16_t, GATTReadData *> &handles_to_read, ble_client::BLECharacteristic *ch) {
  if (ch == nullptr)
    return nullptr;
  auto it = handles_to_read.find(ch->handle);
  if (it == handles_to_read.end() || it->second == nullptr)
    return nullptr;
  return it->second->value_;
}

void BLEClientHID::configure_hid_client_() {
  using namespace ble_client;

  BLEService *battery_service = this->parent()->get_service(ESP_GATT_UUID_BATTERY_SERVICE_SVC);
  BLEService *device_info_service = this->parent()->get_service(ESP_GATT_UUID_DEVICE_INFO_SVC);
  BLEService *hid_service = this->parent()->get_service(ESP_GATT_UUID_HID_SVC);
  BLEService *gap_service = this->parent()->get_service(UUID_GAP_SERVICE);

  // GAP: device name + preferred conn params
  if (gap_service != nullptr) {
    BLECharacteristic *device_name_ch = gap_service->get_characteristic(ESP_GATT_UUID_GAP_DEVICE_NAME);
    uint8_t *t_device_name = get_read_value_or_null(this->handles_to_read_, device_name_ch);
    this->device_name_ = (t_device_name != nullptr) ? (const char *) t_device_name : "Generic";

    BLECharacteristic *pref_conn_ch = gap_service->get_characteristic(UUID_GAP_PREF_CONN_PARAM);
    uint8_t *t_conn_params = get_read_value_or_null(this->handles_to_read_, pref_conn_ch);
    if (t_conn_params != nullptr) {
      // Units: min/max interval = 1.25ms, latency = events, timeout = 10ms
      uint16_t min_int = t_conn_params[0] | (t_conn_params[1] << 8);
      uint16_t max_int = t_conn_params[2] | (t_conn_params[3] << 8);
      uint16_t latency  = t_conn_params[4] | (t_conn_params[5] << 8);
      uint16_t timeout  = t_conn_params[6] | (t_conn_params[7] << 8);

      memcpy(this->preferred_conn_params_.bda, this->parent()->get_remote_bda(), 6);
      this->preferred_conn_params_.min_int = min_int;
      this->preferred_conn_params_.max_int = max_int;
      this->preferred_conn_params_.latency = latency;

      // IMPORTANT CHANGE #2:
      // Many remotes will “nap” longer than an aggressive timeout. Enforce a safer floor.
      if (timeout < 2000) timeout = 2000;  // 20s in 10ms units
      if (timeout > 3200) timeout = 3200;

      this->preferred_conn_params_.timeout = timeout;

      ESP_LOGI(TAG,
               "Preferred conn params (with timeout floor): interval %.2f-%.2f ms, latency %u, timeout %.1f ms",
               min_int * 1.25f, max_int * 1.25f, latency, timeout * 10.f);
    }
  }

  // Device info
  if (device_info_service != nullptr) {
    BLECharacteristic *pnp_id_ch = device_info_service->get_characteristic(ESP_GATT_UUID_PNP_ID);
    if (pnp_id_ch != nullptr) {
      auto it = this->handles_to_read_.find(pnp_id_ch->handle);
      if (it != this->handles_to_read_.end() && it->second != nullptr && it->second->value_len_ >= 7) {
        uint8_t *rdata = it->second->value_;
        this->vendor_id_  = *((uint16_t *) &rdata[1]);
        this->product_id_ = *((uint16_t *) &rdata[3]);
        this->version_    = *((uint16_t *) &rdata[5]);
      }
    }

    BLECharacteristic *manufacturer_ch = device_info_service->get_characteristic(ESP_GATT_UUID_MANU_NAME);
    uint8_t *t_manufacturer = get_read_value_or_null(this->handles_to_read_, manufacturer_ch);
    this->manufacturer_ = (t_manufacturer != nullptr) ? (const char *) t_manufacturer : "Generic";

    BLECharacteristic *serial_ch = device_info_service->get_characteristic(ESP_GATT_UUID_SERIAL_NUMBER_STR);
    uint8_t *t_serial = get_read_value_or_null(this->handles_to_read_, serial_ch);
    this->serial_number_ = (t_serial != nullptr) ? (const char *) t_serial : "000000";
  }

  // Battery notify
  if (battery_service != nullptr) {
    BLECharacteristic *battery_level_ch = battery_service->get_characteristic(ESP_GATT_UUID_BATTERY_LEVEL);
    if (battery_level_ch != nullptr &&
        ((battery_level_ch->properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) != 0)) {
      this->battery_handle_ = battery_level_ch->handle;
      auto status = esp_ble_gattc_register_for_notify(this->parent()->get_gattc_if(),
                                                      this->parent()->get_remote_bda(),
                                                      battery_level_ch->handle);
      if (status != ESP_OK) {
        ESP_LOGW(TAG, "Register for notify failed for battery handle %d status=%d",
                 battery_level_ch->handle, status);
      } else {
        this->handles_waiting_for_notify_registration_++;
      }
    }
  }

  // HID report map + input reports notify + report id mapping (from Report Reference descriptor)
  if (hid_service != nullptr) {
    BLECharacteristic *report_map_ch = hid_service->get_characteristic(ESP_GATT_UUID_HID_REPORT_MAP);
    if (report_map_ch != nullptr) {
      auto it = this->handles_to_read_.find(report_map_ch->handle);
      if (it != this->handles_to_read_.end() && it->second != nullptr) {
        ESP_LOGD(TAG, "Parse HID Report Map");
        HIDReportMap::esp_logd_report_map(it->second->value_, it->second->value_len_);

        // Rebuild report map each connect (safe)
        if (this->hid_report_map_ != nullptr) {
          delete this->hid_report_map_;
          this->hid_report_map_ = nullptr;
        }
        this->hid_report_map_ = HIDReportMap::parse_report_map_data(it->second->value_, it->second->value_len_);
        ESP_LOGD(TAG, "Parse HID Report Map Done");
      }
    }

    for (auto *hid_ch : hid_service->characteristics) {
      if (hid_ch->uuid.get_uuid().uuid.uuid16 != ESP_GATT_UUID_HID_REPORT)
        continue;

      if (hid_ch->properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {
        auto status = esp_ble_gattc_register_for_notify(this->parent()->get_gattc_if(),
                                                        this->parent()->get_remote_bda(),
                                                        hid_ch->handle);
        if (status != ESP_OK) {
          ESP_LOGW(TAG, "Register for notify failed for handle %d status=%d", hid_ch->handle, status);
        } else {
          this->handles_waiting_for_notify_registration_++;
        }
      }

      BLEDescriptor *rpt_ref_desc = hid_ch->get_descriptor(ESP_GATT_UUID_RPT_REF_DESCR);
      if (rpt_ref_desc != nullptr) {
        auto it = this->handles_to_read_.find(rpt_ref_desc->handle);
        if (it != this->handles_to_read_.end() && it->second != nullptr && it->second->value_len_ >= 1) {
          uint8_t report_id = it->second->value_[0];
          this->handle_report_id_.insert(std::make_pair(hid_ch->handle, report_id));
          ESP_LOGD(TAG, "Report ID for handle %d is %d", hid_ch->handle, report_id);
        }
      }
    }
  }

  // Cleanup read data
  for (auto &kv : this->handles_to_read_) {
    delete kv.second;
  }
  this->handles_to_read_.clear();

  // If notifications already registered (rare ordering), flush buffer.
  this->flush_buffered_reports_();
}

void BLEClientHID::process_input_report_(uint16_t handle, const uint8_t *value, uint16_t value_len) {
  // Map GATT handle -> report ID. Avoid operator[] (it inserts default 0).
  auto it = this->handle_report_id_.find(handle);
  if (it == this->handle_report_id_.end()) {
    ESP_LOGD(TAG, "No report id for handle %u (dropping)", handle);
    return;
  }
  uint8_t report_id = it->second;

  if (this->hid_report_map_ == nullptr) {
    ESP_LOGD(TAG, "No report map yet (dropping)");
    return;
  }

  std::vector<uint8_t> buf;
  buf.reserve((size_t) value_len + 1);
  buf.push_back(report_id);
  buf.insert(buf.end(), value, value + value_len);

  std::vector<HIDReportItemValue> hid_report_values = this->hid_report_map_->parse(buf.data());
  if (hid_report_values.empty())
    return;

  for (auto &v : hid_report_values) {
    std::string usage;
    if (USAGE_PAGES.count(v.usage.page) > 0 &&
        USAGE_PAGES.at(v.usage.page).usages_.count(v.usage.usage) > 0) {
      usage = USAGE_PAGES.at(v.usage.page).usages_.at(v.usage.usage);
    } else {
      usage = std::to_string(v.usage.page) + "_" + std::to_string(v.usage.usage);
    }

#ifdef USE_API
    std::map<std::string, std::string> data{
        {"usage", usage},
        {"value", std::to_string(v.value)},
    };
    this->fire_homeassistant_event("esphome.hid_events", data);
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

}  // namespace ble_client_hid
}  // namespace esphome

#endif  // USE_ESP32
