#include "jbd_bms_api.h"

namespace jbd {

static const uint8_t kPktStart = 0xDD;
static const uint8_t kPktEnd = 0x77;
static const uint8_t kCmdRead = 0xA5;
static const uint8_t kCmdWrite = 0x5A;
static const uint8_t kCmdHwInfo = 0x03;
static const uint8_t kCmdCellInfo = 0x04;
static const uint8_t kCmdHwVersion = 0x05;
static const uint8_t kCmdHistoryData = 0x08; // History data (BMS-dependent)
static const uint8_t kCmdRtc = 0x0B;         // RTC set (BMS-dependent)
static const uint8_t kCmdErrorCounts = 0xAA;
static const uint8_t kCmdMos = 0xE1;
static const uint8_t kCmdBalancer = 0xE2;
static const uint8_t kCmdForceSocReset = 0x0A;
static const uint8_t kCmdEnterFactory = 0x00;
static const uint8_t kCmdExitFactory = 0x01;
static const uint8_t kCmdCapRem = 0xE0;
static const uint8_t kMosCharge = 0x01;
static const uint8_t kMosDischarge = 0x02;
static const uint8_t kRegDesignCap = 0x10;
static const uint8_t kRegCycleCap = 0x11;
static const uint8_t kRegChgOverTemp = 0x18;
static const uint8_t kRegChgOverTempRel = 0x19;
static const uint8_t kRegChgUnderTemp = 0x1A;
static const uint8_t kRegChgUnderTempRel = 0x1B;
static const uint8_t kRegDsgOverTemp = 0x1C;
static const uint8_t kRegDsgOverTempRel = 0x1D;
static const uint8_t kRegDsgUnderTemp = 0x1E;
static const uint8_t kRegDsgUnderTempRel = 0x1F;
static const uint8_t kRegPovp = 0x20;
static const uint8_t kRegPovpRel = 0x21;
static const uint8_t kRegPuvp = 0x22;
static const uint8_t kRegPuvpRel = 0x23;
static const uint8_t kRegCovp = 0x24;
static const uint8_t kRegCovpRel = 0x25;
static const uint8_t kRegCuvp = 0x26;
static const uint8_t kRegCuvpRel = 0x27;
static const uint8_t kRegChgOc = 0x28;
static const uint8_t kRegDsgOc = 0x29;
static const uint8_t kRegBalStart = 0x2A;
static const uint8_t kRegBalWindow = 0x2B;
static const uint8_t kRegShuntRes = 0x2C;
static const uint8_t kRegFuncConfig = 0x2D;
static const uint8_t kRegNtcConfig = 0x2E;
static const uint8_t kRegCellCnt = 0x2F;
static const uint8_t kRegFetCtrl = 0x30;
static const uint8_t kRegLedTimer = 0x31;
static const uint8_t kRegCapFull = 0x12; // Full charge voltage (BMS-dependent)
static const uint8_t kRegCap0 = 0x13;    // End of discharge voltage
static const uint8_t kRegDsgRate = 0x14;
static const uint8_t kRegMfgDate = 0x15;
static const uint8_t kRegSerialNum = 0x16;
static const uint8_t kRegCycleCnt = 0x17;
static const uint8_t kRegCap80 = 0x32;
static const uint8_t kRegCap60 = 0x33;
static const uint8_t kRegCap40 = 0x34;
static const uint8_t kRegCap20 = 0x35;
static const uint8_t kRegCovpHigh = 0x36;
static const uint8_t kRegCuvpHigh = 0x37;
static const uint8_t kRegScDsgoc2 = 0x38;
static const uint8_t kRegCxvpHighDelayScRel = 0x39;
static const uint8_t kRegChgTempDelays = 0x3A;
static const uint8_t kRegDsgTempDelays = 0x3B;
static const uint8_t kRegPackVDelays = 0x3C;
static const uint8_t kRegCellVDelays = 0x3D;
static const uint8_t kRegChgOcDelays = 0x3E;
static const uint8_t kRegDsgOcDelays = 0x3F;
static const uint8_t kRegGpsOff = 0x40;     // GPS off voltage (BMS-dependent)
static const uint8_t kRegGpsOffTime = 0x41; // GPS off time (BMS-dependent)
static const uint8_t kRegCap90 = 0x42;
static const uint8_t kRegCap70 = 0x43;
static const uint8_t kRegCap50 = 0x44;
static const uint8_t kRegCap30 = 0x45;
static const uint8_t kRegCap10 = 0x46;
static const uint8_t kRegCap100Pct = 0x47; // 100% SOC voltage (BMS-dependent)
static const uint8_t kRegCellCalBase = 0xB0;
static const uint8_t kRegNtcCalBase = 0xD0;
static const uint8_t kRegIdleCurrentCal = 0xAD;
static const uint8_t kRegChargeCurrentCal = 0xAE;
static const uint8_t kRegDischargeCurrentCal = 0xAF;

static uint16_t to_mv_10(float v);
static uint16_t to_mv_1(float v);
static uint16_t to_kelvin_tenth(float c);
static float from_mv_10(uint16_t raw);
static float from_mv_1(uint16_t raw);
static float from_kelvin_tenth(uint16_t raw);
static const size_t kMaxFrameLen = 4 + 64 + 3;

BlackBoxLogger::BlackBoxLogger(size_t capacity) : capacity_(capacity) {
  entries_.reserve(capacity_);
}

void BlackBoxLogger::set_capacity(size_t capacity) {
  capacity_ = capacity;
  if (entries_.size() > capacity_) {
    entries_.erase(entries_.begin(), entries_.end() - capacity_);
  }
}

void BlackBoxLogger::set_permanent_threshold_ms(uint32_t ms) { permanent_threshold_ms_ = ms; }

void BlackBoxLogger::add_(uint32_t ms, const String &message) {
  if (capacity_ == 0) {
    return;
  }
  if (entries_.size() >= capacity_) {
    entries_.erase(entries_.begin());
  }
  entries_.push_back({ms, message});
}

void BlackBoxLogger::record_basic(const BasicInfo &info, uint32_t now_ms) {
  const uint16_t protection = info.protection_status;
  if (protection != last_protection_) {
    String msg = "Protection change: 0x" + String(last_protection_, HEX) + " -> 0x" +
                 String(protection, HEX);
    add_(now_ms, msg);
    last_protection_ = protection;
    protection_since_ms_ = protection ? now_ms : 0;
    permanent_logged_ = false;
  }
  if (protection != 0 && !permanent_logged_) {
    if (protection_since_ms_ != 0 && (now_ms - protection_since_ms_) >= permanent_threshold_ms_) {
      String msg = "Permanent failure: protection 0x" + String(protection, HEX);
      add_(now_ms, msg);
      permanent_logged_ = true;
    }
  }
}

void BlackBoxLogger::record_errors(const ErrorCounts &errors, uint32_t now_ms) {
  if (!has_last_errors_) {
    last_errors_ = errors;
    has_last_errors_ = true;
    return;
  }

  auto check = [&](const char *name, uint16_t prev, uint16_t now) {
    if (now > prev) {
      String msg = String("Error count ") + name + " -> " + String(now);
      add_(now_ms, msg);
    }
  };

  check("short_circuit", last_errors_.short_circuit, errors.short_circuit);
  check("charge_overcurrent", last_errors_.charge_overcurrent, errors.charge_overcurrent);
  check("discharge_overcurrent", last_errors_.discharge_overcurrent, errors.discharge_overcurrent);
  check("cell_overvoltage", last_errors_.cell_overvoltage, errors.cell_overvoltage);
  check("cell_undervoltage", last_errors_.cell_undervoltage, errors.cell_undervoltage);
  check("charge_overtemperature", last_errors_.charge_overtemperature, errors.charge_overtemperature);
  check("charge_undertemperature", last_errors_.charge_undertemperature, errors.charge_undertemperature);
  check("discharge_overtemperature", last_errors_.discharge_overtemperature, errors.discharge_overtemperature);
  check("discharge_undertemperature", last_errors_.discharge_undertemperature,
        errors.discharge_undertemperature);
  check("battery_overvoltage", last_errors_.battery_overvoltage, errors.battery_overvoltage);
  check("battery_undervoltage", last_errors_.battery_undervoltage, errors.battery_undervoltage);
  check("reset_count", last_errors_.reset_count, errors.reset_count);

  last_errors_ = errors;
}

void BlackBoxLogger::dump(Print &out) const {
  out.println("=== BLACK BOX LOG ===");
  for (const auto &entry : entries_) {
    out.print("[");
    out.print(entry.ms);
    out.print(" ms] ");
    out.println(entry.message);
  }
  out.println("=====================");
}

void BlackBoxLogger::clear() {
  entries_.clear();
  last_protection_ = 0;
  protection_since_ms_ = 0;
  permanent_logged_ = false;
  has_last_errors_ = false;
}

Api::Api(HardwareSerial &port, Print *log) : port_(port), log_(log) {}

void Api::begin(uint32_t baud) { port_.begin(baud); }

void Api::set_debug(bool enabled) { debug_ = enabled; }

uint16_t Api::checksum(const uint8_t *data, uint16_t len) {
  uint16_t checksum = 0x00;
  for (uint16_t i = 0; i < len; i++) {
    checksum = checksum - data[i];
  }
  return checksum;
}

uint16_t Api::checksum_china(const uint8_t *raw, uint8_t data_len) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < data_len; i++) {
    sum += raw[4 + i];
  }
  return uint16_t((sum + data_len - 1) ^ 0xFFFF);
}

void Api::log_hex_frame_(const char *prefix, const uint8_t *data, size_t len) {
  if (!debug_ || log_ == nullptr) {
    return;
  }
  log_->print(prefix);
  for (size_t i = 0; i < len; i++) {
    if (data[i] < 0x10) {
      log_->print('0');
    }
    log_->print(data[i], HEX);
    if (i + 1 < len) {
      log_->print(' ');
    }
  }
  log_->println();
}

void Api::send_action_(uint8_t action, uint8_t function, uint8_t data_len, const uint8_t *data) {
  std::vector<uint8_t> frame(7 + data_len, 0);
  frame[0] = kPktStart;
  frame[1] = action;
  frame[2] = function;
  frame[3] = data_len;
  for (uint8_t i = 0; i < data_len; i++) {
    frame[4 + i] = data[i];
  }
  const uint16_t crc = checksum(frame.data() + 2, data_len + 2);
  frame[4 + data_len] = crc >> 8;
  frame[5 + data_len] = crc >> 0;
  frame[6 + data_len] = kPktEnd;

  log_hex_frame_("TX -> ", frame.data(), frame.size());
  port_.write(frame.data(), frame.size());
  port_.flush();
}

void Api::send_read_(uint8_t function) { send_action_(kCmdRead, function, 0, nullptr); }

bool Api::parse_byte_(uint8_t byte, Frame &frame) {
  rx_buffer_.push_back(byte);
  if (rx_buffer_.size() > kMaxFrameLen) {
    rx_buffer_.clear();
    return false;
  }

  while (!rx_buffer_.empty() && rx_buffer_.front() != kPktStart) {
    rx_buffer_.erase(rx_buffer_.begin());
  }

  if (rx_buffer_.size() < 4) {
    return false;
  }

  const uint8_t *raw = rx_buffer_.data();
  const uint8_t data_len = raw[3];
  const uint16_t frame_len = 4 + data_len + 3;
  if (frame_len > kMaxFrameLen) {
    rx_buffer_.clear();
    return false;
  }
  if (rx_buffer_.size() < frame_len) {
    return false;
  }
  if (raw[frame_len - 1] != kPktEnd) {
    rx_buffer_.clear();
    return false;
  }

  const uint16_t computed = checksum(raw + 2, data_len + 2);
  const uint16_t computed_alt = checksum_china(raw, data_len);
  const uint16_t received = (uint16_t(raw[frame_len - 3]) << 8) | uint16_t(raw[frame_len - 2]);
  if (computed != received && computed_alt != received) {
    if (debug_ && log_ != nullptr) {
      log_->print("CRC mismatch. Expected 0x");
      log_->print(computed, HEX);
      log_->print(" or 0x");
      log_->print(computed_alt, HEX);
      log_->print(" got 0x");
      log_->println(received, HEX);
    }
    rx_buffer_.erase(rx_buffer_.begin());
    return false;
  }

  frame.function = raw[1];
  frame.status = raw[2];
  frame.data_len = data_len;
  frame.checksum = received;
  frame.data.assign(rx_buffer_.begin() + 4, rx_buffer_.begin() + frame_len - 3);

  log_hex_frame_("RX <- ", rx_buffer_.data(), frame_len);
  rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frame_len);
  return true;
}

bool Api::poll(Frame &frame) {
  while (port_.available() > 0) {
    const uint8_t b = static_cast<uint8_t>(port_.read());
    if (parse_byte_(b, frame)) {
      return true;
    }
  }
  return false;
}

bool Api::wait_for_frame(uint8_t function, Frame &frame, uint32_t timeout_ms) {
  const uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    if (poll(frame)) {
      if (frame.function == function) {
        return true;
      }
    }
  }
  return false;
}

uint16_t Api::get16_(const std::vector<uint8_t> &data, size_t index) {
  return (uint16_t(data[index]) << 8) | uint16_t(data[index + 1]);
}

uint8_t Api::mos_action_for_states_(bool charge_on, bool discharge_on) {
  if (charge_on && discharge_on) {
    return 0x00;
  }
  if (!charge_on && discharge_on) {
    return 0x01;
  }
  if (charge_on && !discharge_on) {
    return 0x02;
  }
  return 0x03;
}

bool Api::read_basic_info(BasicInfo &out, uint32_t timeout_ms) {
  send_read_(kCmdHwInfo);
  Frame frame;
  if (!wait_for_frame(kCmdHwInfo, frame, timeout_ms)) {
    return false;
  }
  if (frame.status != 0x00 || frame.data.size() < 23) {
    return false;
  }
  const auto &data = frame.data;
  out.total_voltage_raw = get16_(data, 0);
  out.total_voltage = out.total_voltage_raw * 0.01f;
  out.current_raw = int16_t(get16_(data, 2));
  out.current = float(out.current_raw) * 0.01f;
  out.power = out.total_voltage * out.current;
  out.charging_power = (out.power > 0.0f) ? out.power : 0.0f;
  out.discharging_power = (out.power < 0.0f) ? -out.power : 0.0f;
  out.capacity_remaining_raw = get16_(data, 4);
  out.capacity_remaining_ah = out.capacity_remaining_raw * 0.01f;
  out.nominal_capacity_raw = get16_(data, 6);
  out.nominal_capacity_ah = out.nominal_capacity_raw * 0.01f;
  out.cycle_count_raw = get16_(data, 8);
  out.cycle_count = out.cycle_count_raw;
  out.production_date_raw = get16_(data, 10);
  out.balance_low = (data.size() >= 14) ? get16_(data, 12) : 0;
  out.balance_high = (data.size() >= 16) ? get16_(data, 14) : 0;
  out.balance_mask = (uint32_t(out.balance_low) << 16) | out.balance_high;
  out.protection_status = (data.size() >= 18) ? get16_(data, 16) : 0;
  out.software_version_raw = data[18];
  out.software_version =
      (out.software_version_raw >> 4) + ((out.software_version_raw & 0x0F) * 0.1f);
  out.soc = data[19];
  out.mos_status = data[20];
  out.strings = data[21];
  out.temperature_sensors = data[22];
  const uint8_t temp_count = (out.temperature_sensors > 6) ? 6 : out.temperature_sensors;
  for (uint8_t i = 0; i < temp_count; i++) {
    const size_t offset = 23 + (i * 2);
    if (offset + 1 >= data.size()) {
      break;
    }
    out.temperatures_raw[i] = get16_(data, offset);
    out.temperatures_c[i] = float(out.temperatures_raw[i] - 2731) * 0.1f;
  }

  mos_status_ = out.mos_status;
  nominal_capacity_ah_ = out.nominal_capacity_ah;
  nominal_known_ = true;
  return true;
}

bool Api::read_cell_info(CellInfo &out, uint32_t timeout_ms) {
  send_read_(kCmdCellInfo);
  Frame frame;
  if (!wait_for_frame(kCmdCellInfo, frame, timeout_ms)) {
    return false;
  }
  if (frame.status != 0x00 || frame.data_len == 0 || (frame.data_len % 2) != 0) {
    return false;
  }
  const auto &data = frame.data;
  const uint8_t cells = (data.size() / 2 > 32) ? 32 : uint8_t(data.size() / 2);
  out.cell_count = cells;
  out.min_v = 1000.0f;
  out.max_v = 0.0f;
  out.avg_v = 0.0f;
  out.min_cell = 0;
  out.max_cell = 0;
  for (uint8_t i = 0; i < cells; i++) {
    out.cell_raw[i] = get16_(data, i * 2);
    out.cell_v[i] = out.cell_raw[i] * 0.001f;
    out.avg_v += out.cell_v[i];
    if (out.cell_v[i] < out.min_v) {
      out.min_v = out.cell_v[i];
      out.min_cell = i + 1;
    }
    if (out.cell_v[i] > out.max_v) {
      out.max_v = out.cell_v[i];
      out.max_cell = i + 1;
    }
  }
  out.avg_v = out.avg_v / cells;
  out.delta_v = out.max_v - out.min_v;
  return true;
}

bool Api::read_model(ModelInfo &out, uint32_t timeout_ms) {
  send_read_(kCmdHwVersion);
  Frame frame;
  if (!wait_for_frame(kCmdHwVersion, frame, timeout_ms)) {
    return false;
  }
  if (frame.status != 0x00 || frame.data_len == 0) {
    return false;
  }
  out.length = frame.data_len;
  out.model = "";
  for (uint8_t b : frame.data) {
    if (b >= 32 && b <= 126) {
      out.model += char(b);
    }
  }
  return true;
}

bool Api::read_error_counts(ErrorCounts &out, uint32_t timeout_ms) {
  send_read_(kCmdErrorCounts);
  Frame frame;
  if (!wait_for_frame(kCmdErrorCounts, frame, timeout_ms)) {
    return false;
  }
  if (frame.status != 0x00 || frame.data_len < 22) {
    return false;
  }
  const auto &data = frame.data;
  out.short_circuit = get16_(data, 0);
  out.charge_overcurrent = get16_(data, 2);
  out.discharge_overcurrent = get16_(data, 4);
  out.cell_overvoltage = get16_(data, 6);
  out.cell_undervoltage = get16_(data, 8);
  out.charge_overtemperature = get16_(data, 10);
  out.charge_undertemperature = get16_(data, 12);
  out.discharge_overtemperature = get16_(data, 14);
  out.discharge_undertemperature = get16_(data, 16);
  out.battery_overvoltage = get16_(data, 18);
  out.battery_undervoltage = get16_(data, 20);
  out.reset_count = frame.data_len >= 24 ? get16_(data, 22) : 0;
  return true;
}

bool Api::read_history_data(std::vector<uint8_t> &out, uint32_t timeout_ms) {
  send_read_(kCmdHistoryData);
  Frame frame;
  if (!wait_for_frame(kCmdHistoryData, frame, timeout_ms)) {
    return false;
  }
  if (frame.status != 0x00) {
    return false;
  }
  out = frame.data;
  return true;
}

bool Api::clear_history_data(uint32_t timeout_ms) {
  const uint8_t payload[2] = {0x00, 0x00};
  send_action_(kCmdWrite, kCmdHistoryData, 2, payload);
  Frame frame;
  if (!wait_for_frame(kCmdHistoryData, frame, timeout_ms)) {
    return false;
  }
  return frame.status == 0x00;
}

bool Api::set_rtc(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute,
                  uint8_t second, uint32_t timeout_ms) {
  if (year < 2000 || year > 2127 || month < 1 || month > 12 || day < 1 || day > 31 ||
      hour > 23 || minute > 59 || second > 59) {
    return false;
  }
  const uint8_t payload[7] = {uint8_t(year >> 8), uint8_t(year >> 0), month, day,
                              hour,             minute,            second};
  send_action_(kCmdWrite, kCmdRtc, sizeof(payload), payload);
  Frame frame;
  if (!wait_for_frame(kCmdRtc, frame, timeout_ms)) {
    return false;
  }
  return frame.status == 0x00;
}

bool Api::read_rtc(uint16_t &year, uint8_t &month, uint8_t &day, uint8_t &hour, uint8_t &minute,
                   uint8_t &second, uint32_t timeout_ms) {
  send_read_(kCmdRtc);
  Frame frame;
  if (!wait_for_frame(kCmdRtc, frame, timeout_ms)) {
    return false;
  }
  if (frame.status != 0x00 || frame.data_len < 7) {
    return false;
  }
  year = (uint16_t(frame.data[0]) << 8) | uint16_t(frame.data[1]);
  month = frame.data[2];
  day = frame.data[3];
  hour = frame.data[4];
  minute = frame.data[5];
  second = frame.data[6];
  return true;
}

bool Api::mos_action(uint8_t action, uint32_t timeout_ms) {
  const uint8_t payload[2] = {0x00, action};
  send_action_(kCmdWrite, kCmdMos, 2, payload);
  Frame frame;
  if (!wait_for_frame(kCmdMos, frame, timeout_ms)) {
    return false;
  }
  return frame.status == 0x00;
}

bool Api::set_charge(bool enabled) {
  if (mos_status_ == 0xFF) {
    BasicInfo basic;
    if (!read_basic_info(basic)) {
      return false;
    }
  }
  const bool charge_on = enabled;
  const bool discharge_on = (mos_status_ & kMosDischarge) != 0;
  mos_status_ = (charge_on ? kMosCharge : 0) | (discharge_on ? kMosDischarge : 0);
  return mos_action(mos_action_for_states_(charge_on, discharge_on));
}

bool Api::set_discharge(bool enabled) {
  if (mos_status_ == 0xFF) {
    BasicInfo basic;
    if (!read_basic_info(basic)) {
      return false;
    }
  }
  const bool charge_on = (mos_status_ & kMosCharge) != 0;
  const bool discharge_on = enabled;
  mos_status_ = (charge_on ? kMosCharge : 0) | (discharge_on ? kMosDischarge : 0);
  return mos_action(mos_action_for_states_(charge_on, discharge_on));
}

bool Api::set_charge_discharge(bool enabled) {
  mos_status_ = enabled ? (kMosCharge | kMosDischarge) : 0x00;
  return mos_action(enabled ? 0x00 : 0x03);
}

bool Api::set_mos_mask(uint8_t mask) {
  mos_status_ = mask;
  const bool charge_on = (mask & kMosCharge) != 0;
  const bool discharge_on = (mask & kMosDischarge) != 0;
  return mos_action(mos_action_for_states_(charge_on, discharge_on));
}

bool Api::set_balancer(bool enabled) {
  return set_balance_control(enabled ? 0x0001 : 0x0003);
}

bool Api::set_balance_control(uint16_t mode) { return write_register(kCmdBalancer, mode); }

bool Api::set_balance_start_v(float v) { return write_register(kRegBalStart, to_mv_1(v)); }

bool Api::set_balance_window_v(float v) { return write_register(kRegBalWindow, to_mv_1(v)); }

bool Api::set_balance_enable(bool enabled) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  if (enabled) {
    cfg |= (1 << 2);
  } else {
    cfg &= ~(1 << 2);
  }
  return write_register(kRegFuncConfig, cfg);
}

bool Api::set_charge_balance_enable(bool enabled) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  if (enabled) {
    cfg |= (1 << 3);
  } else {
    cfg &= ~(1 << 3);
  }
  return write_register(kRegFuncConfig, cfg);
}

bool Api::read_balance_control(uint16_t &out) { return read_u16(kCmdBalancer, out); }

bool Api::read_balance_start_v(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegBalStart, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_balance_window_v(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegBalWindow, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_balance_enable(bool &out) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  out = (cfg & (1 << 2)) != 0;
  return true;
}

bool Api::read_charge_balance_enable(bool &out) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  out = (cfg & (1 << 3)) != 0;
  return true;
}

bool Api::set_capacity_remaining_ah(float ah) {
  if (ah < 0.0f) {
    return false;
  }
  const uint16_t value = static_cast<uint16_t>(ah * 100.0f + 0.5f);
  return write_register(kCmdCapRem, value);
}

bool Api::read_capacity_remaining_ah(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kCmdCapRem, raw)) {
    return false;
  }
  out = static_cast<float>(raw) / 100.0f;
  return true;
}

bool Api::set_capacity_remaining_mah(float mah) {
  if (mah < 0.0f) {
    return false;
  }
  return set_capacity_remaining_ah(mah / 1000.0f);
}

bool Api::read_capacity_remaining_mah(float &out) {
  float ah = 0.0f;
  if (!read_capacity_remaining_ah(ah)) {
    return false;
  }
  out = ah * 1000.0f;
  return true;
}

bool Api::reset_capacity() {
  if (!nominal_known_ || nominal_capacity_ah_ <= 0.0f) {
    return false;
  }
  return set_capacity_remaining_ah(nominal_capacity_ah_);
}

bool Api::set_soc_percent(float percent) {
  if (!nominal_known_ || percent < 0.0f || percent > 100.0f) {
    return false;
  }
  const float ah = nominal_capacity_ah_ * (percent / 100.0f);
  return set_capacity_remaining_ah(ah);
}

bool Api::soc_reset() { return write_register(kCmdForceSocReset, 0x0001); }

bool Api::enter_factory_mode() {
  const uint8_t payload[2] = {0x56, 0x78};
  return write_register_bytes(kCmdEnterFactory, payload, 2);
}

bool Api::exit_factory_mode(bool save_and_reset) {
  const uint8_t payload[2] = {uint8_t(save_and_reset ? 0x28 : 0x00),
                              uint8_t(save_and_reset ? 0x28 : 0x00)};
  return write_register_bytes(kCmdExitFactory, payload, 2);
}

bool Api::factory_mode(bool enable) {
  return enable ? enter_factory_mode() : exit_factory_mode(false);
}

bool Api::clear_error_counts() {
  if (!enter_factory_mode()) {
    return false;
  }
  return exit_factory_mode(true);
}

bool Api::write_register(uint8_t address, uint16_t value) {
  const uint8_t payload[2] = {uint8_t(value >> 8), uint8_t(value >> 0)};
  return write_register_bytes(address, payload, 2);
}

bool Api::write_register_bytes(uint8_t address, const uint8_t *data, uint8_t length,
                               uint32_t timeout_ms) {
  send_action_(kCmdWrite, address, length, data);
  Frame frame;
  if (!wait_for_frame(address, frame, timeout_ms)) {
    return false;
  }
  return frame.status == 0x00;
}

bool Api::read_register(uint8_t address, std::vector<uint8_t> &out, uint32_t timeout_ms) {
  send_read_(address);
  Frame frame;
  if (!wait_for_frame(address, frame, timeout_ms)) {
    return false;
  }
  if (frame.status != 0x00) {
    return false;
  }
  out = frame.data;
  return true;
}

bool Api::read_u16(uint8_t address, uint16_t &out, uint32_t timeout_ms) {
  std::vector<uint8_t> data;
  if (!read_register(address, data, timeout_ms) || data.size() < 2) {
    return false;
  }
  out = (uint16_t(data[0]) << 8) | uint16_t(data[1]);
  return true;
}

bool Api::read_s16(uint8_t address, int16_t &out, uint32_t timeout_ms) {
  uint16_t raw = 0;
  if (!read_u16(address, raw, timeout_ms)) {
    return false;
  }
  out = int16_t(raw);
  return true;
}

bool Api::read_string(uint8_t address, String &out, uint32_t timeout_ms) {
  std::vector<uint8_t> data;
  if (!read_register(address, data, timeout_ms) || data.empty()) {
    return false;
  }
  const uint8_t length = data[0];
  out = "";
  for (uint8_t i = 0; i < length && (1 + i) < data.size(); i++) {
    out += char(data[1 + i]);
  }
  return true;
}

bool Api::write_string(uint8_t address, const String &value) {
  const uint8_t length = value.length() > 63 ? 63 : value.length();
  std::vector<uint8_t> payload(1 + length);
  payload[0] = length;
  for (uint8_t i = 0; i < length; i++) {
    payload[1 + i] = uint8_t(value[i]);
  }
  return write_register_bytes(address, payload.data(), payload.size());
}

bool Api::read_shunt_res_mohm(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegShuntRes, raw)) {
    return false;
  }
  out = static_cast<float>(raw) / 10.0f;
  return true;
}

bool Api::read_func_config(uint16_t &out) { return read_u16(kRegFuncConfig, out); }

bool Api::read_func_switch(bool &out) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  out = (cfg & (1 << 0)) != 0;
  return true;
}

bool Api::read_func_scrl(bool &out) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  out = (cfg & (1 << 1)) != 0;
  return true;
}

bool Api::read_func_load(bool &out) { return read_func_scrl(out); }

bool Api::read_led_enable(bool &out) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  out = (cfg & (1 << 4)) != 0;
  return true;
}

bool Api::read_led_num(bool &out) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  out = (cfg & (1 << 5)) != 0;
  return true;
}

bool Api::read_func_rtc(bool &out) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  out = (cfg & (1 << 6)) != 0;
  return true;
}

bool Api::read_func_edv(bool &out) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  out = (cfg & (1 << 7)) != 0;
  return true;
}

bool Api::read_func_chg_limit(bool &out) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  out = (cfg & (1 << 8)) != 0;
  return true;
}

bool Api::read_func_gps_enable(bool &out) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  out = (cfg & (1 << 9)) != 0;
  return true;
}

bool Api::read_func_buzzer_enable(bool &out) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  out = (cfg & (1 << 10)) != 0;
  return true;
}

bool Api::read_func_car_mode(bool &out) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  out = (cfg & (1 << 11)) != 0;
  return true;
}

bool Api::set_func_config(uint16_t value) { return write_register(kRegFuncConfig, value); }

bool Api::set_func_switch(bool enabled) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  if (enabled) {
    cfg |= (1 << 0);
  } else {
    cfg &= ~(1 << 0);
  }
  return write_register(kRegFuncConfig, cfg);
}

bool Api::set_func_scrl(bool enabled) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  if (enabled) {
    cfg |= (1 << 1);
  } else {
    cfg &= ~(1 << 1);
  }
  return write_register(kRegFuncConfig, cfg);
}

bool Api::set_func_load(bool enabled) { return set_func_scrl(enabled); }

bool Api::set_led_enable(bool enabled) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  if (enabled) {
    cfg |= (1 << 4);
  } else {
    cfg &= ~(1 << 4);
  }
  return write_register(kRegFuncConfig, cfg);
}

bool Api::set_led_num(bool enabled) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  if (enabled) {
    cfg |= (1 << 5);
  } else {
    cfg &= ~(1 << 5);
  }
  return write_register(kRegFuncConfig, cfg);
}

bool Api::set_func_rtc(bool enabled) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  if (enabled) {
    cfg |= (1 << 6);
  } else {
    cfg &= ~(1 << 6);
  }
  return write_register(kRegFuncConfig, cfg);
}

bool Api::set_func_edv(bool enabled) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  if (enabled) {
    cfg |= (1 << 7);
  } else {
    cfg &= ~(1 << 7);
  }
  return write_register(kRegFuncConfig, cfg);
}

bool Api::set_func_chg_limit(bool enabled) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  if (enabled) {
    cfg |= (1 << 8);
  } else {
    cfg &= ~(1 << 8);
  }
  return write_register(kRegFuncConfig, cfg);
}

bool Api::set_func_gps_enable(bool enabled) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  if (enabled) {
    cfg |= (1 << 9);
  } else {
    cfg &= ~(1 << 9);
  }
  return write_register(kRegFuncConfig, cfg);
}

bool Api::set_func_buzzer_enable(bool enabled) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  if (enabled) {
    cfg |= (1 << 10);
  } else {
    cfg &= ~(1 << 10);
  }
  return write_register(kRegFuncConfig, cfg);
}

bool Api::set_func_car_mode(bool enabled) {
  uint16_t cfg = 0;
  if (!read_u16(kRegFuncConfig, cfg)) {
    return false;
  }
  if (enabled) {
    cfg |= (1 << 11);
  } else {
    cfg &= ~(1 << 11);
  }
  return write_register(kRegFuncConfig, cfg);
}

bool Api::read_fet_ctrl(uint16_t &out) { return read_u16(kRegFetCtrl, out); }

bool Api::set_fet_ctrl(uint16_t value) { return write_register(kRegFetCtrl, value); }

bool Api::read_led_timer(uint16_t &out) { return read_u16(kRegLedTimer, out); }

bool Api::set_led_timer(uint16_t value) { return write_register(kRegLedTimer, value); }

static const uint16_t kScDelayUsOptions[] = {70, 100, 200, 400};
static const uint16_t kScMvOptions[] = {22, 33, 44, 56, 67, 78, 89, 100};
static const uint16_t kDsgoc2DelayMsOptions[] = {8, 20, 40, 80, 160, 320, 640, 1280};
static const uint16_t kDsgoc2MvOptions[] = {8, 11, 14, 17, 19, 22, 25, 28,
                                            31, 33, 36, 39, 42, 44, 47, 50};
static const uint16_t kCovpHighDelaySOptions[] = {1, 2, 4, 8};
static const uint16_t kCuvpHighDelaySOptions[] = {1, 4, 8, 16};

static bool map_u16_to_index(const uint16_t *options, size_t count, uint16_t value,
                             uint8_t &index) {
  for (size_t i = 0; i < count; i++) {
    if (options[i] == value) {
      index = static_cast<uint8_t>(i);
      return true;
    }
  }
  return false;
}

static bool map_u16_to_nearest_index(const uint16_t *options, size_t count, uint16_t value,
                                     uint8_t &index) {
  if (count == 0) {
    return false;
  }
  uint16_t best_diff = options[0] > value ? options[0] - value : value - options[0];
  index = 0;
  for (size_t i = 1; i < count; i++) {
    uint16_t diff = options[i] > value ? options[i] - value : value - options[i];
    if (diff < best_diff) {
      best_diff = diff;
      index = static_cast<uint8_t>(i);
    }
  }
  return true;
}

bool Api::set_sc_dsgoc2(uint8_t byte0, uint8_t byte1) {
  const uint8_t payload[2] = {byte0, byte1};
  return write_register_bytes(kRegScDsgoc2, payload, 2);
}

bool Api::set_cxvp_high_delay_sc_rel(uint8_t byte0, uint8_t byte1) {
  const uint8_t payload[2] = {byte0, byte1};
  return write_register_bytes(kRegCxvpHighDelayScRel, payload, 2);
}

bool Api::read_sc_dsgoc2_config(ScDsgoc2Config &out) {
  std::vector<uint8_t> data;
  if (!read_register(kRegScDsgoc2, data) || data.size() < 2) {
    return false;
  }
  out.raw_byte0 = data[0];
  out.raw_byte1 = data[1];
  out.x2 = (data[0] & 0x80) != 0;
  const uint8_t sc_delay_idx = (data[0] >> 3) & 0x03;
  const uint8_t sc_mv_idx = data[0] & 0x07;
  const uint8_t dsgoc2_delay_idx = (data[1] >> 4) & 0x0F;
  const uint8_t dsgoc2_mv_idx = data[1] & 0x0F;
  out.sc_delay_us = kScDelayUsOptions[sc_delay_idx];
  out.sc_mv = kScMvOptions[sc_mv_idx];
  out.dsgoc2_delay_ms =
      dsgoc2_delay_idx < (sizeof(kDsgoc2DelayMsOptions) / sizeof(uint16_t))
          ? kDsgoc2DelayMsOptions[dsgoc2_delay_idx]
          : 0;
  out.dsgoc2_mv = kDsgoc2MvOptions[dsgoc2_mv_idx];
  return true;
}

bool Api::read_cxvp_high_delay_sc_rel_config(CxvpHighDelayConfig &out) {
  std::vector<uint8_t> data;
  if (!read_register(kRegCxvpHighDelayScRel, data) || data.size() < 2) {
    return false;
  }
  out.raw_byte0 = data[0];
  out.raw_byte1 = data[1];
  const uint8_t cuvp_idx = (data[0] >> 6) & 0x03;
  const uint8_t covp_idx = (data[0] >> 4) & 0x03;
  out.cuvp_high_delay_s = kCuvpHighDelaySOptions[cuvp_idx];
  out.covp_high_delay_s = kCovpHighDelaySOptions[covp_idx];
  out.sc_release_s = data[1];
  return true;
}

bool Api::read_sc_dsgoc_x2(bool &out) {
  ScDsgoc2Config cfg{};
  if (!read_sc_dsgoc2_config(cfg)) {
    return false;
  }
  out = cfg.x2;
  return true;
}

bool Api::read_short_circuit_delay_us(uint16_t &out) {
  ScDsgoc2Config cfg{};
  if (!read_sc_dsgoc2_config(cfg)) {
    return false;
  }
  out = cfg.sc_delay_us;
  return true;
}

bool Api::read_short_circuit_mv(uint16_t &out) {
  ScDsgoc2Config cfg{};
  if (!read_sc_dsgoc2_config(cfg)) {
    return false;
  }
  out = cfg.sc_mv;
  return true;
}

bool Api::read_dsgoc2_delay_ms(uint16_t &out) {
  ScDsgoc2Config cfg{};
  if (!read_sc_dsgoc2_config(cfg)) {
    return false;
  }
  out = cfg.dsgoc2_delay_ms;
  return true;
}

bool Api::read_dsgoc2_mv(uint16_t &out) {
  ScDsgoc2Config cfg{};
  if (!read_sc_dsgoc2_config(cfg)) {
    return false;
  }
  out = cfg.dsgoc2_mv;
  return true;
}

bool Api::read_secondary_cell_overvoltage_delay_s(uint16_t &out) {
  CxvpHighDelayConfig cfg{};
  if (!read_cxvp_high_delay_sc_rel_config(cfg)) {
    return false;
  }
  out = cfg.covp_high_delay_s;
  return true;
}

bool Api::read_secondary_cell_undervoltage_delay_s(uint16_t &out) {
  CxvpHighDelayConfig cfg{};
  if (!read_cxvp_high_delay_sc_rel_config(cfg)) {
    return false;
  }
  out = cfg.cuvp_high_delay_s;
  return true;
}

bool Api::read_short_circuit_release_s(uint16_t &out) {
  CxvpHighDelayConfig cfg{};
  if (!read_cxvp_high_delay_sc_rel_config(cfg)) {
    return false;
  }
  out = cfg.sc_release_s;
  return true;
}

bool Api::set_sc_dsgoc_x2(bool enabled) {
  std::vector<uint8_t> data;
  if (!read_register(kRegScDsgoc2, data) || data.size() < 2) {
    return false;
  }
  uint8_t byte0 = data[0];
  if (enabled) {
    byte0 |= 0x80;
  } else {
    byte0 &= ~0x80;
  }
  const uint8_t payload[2] = {byte0, data[1]};
  return write_register_bytes(kRegScDsgoc2, payload, 2);
}

bool Api::set_short_circuit_delay_us(uint16_t delay_us) {
  uint8_t code = 0;
  if (!map_u16_to_index(kScDelayUsOptions, sizeof(kScDelayUsOptions) / sizeof(uint16_t),
                        delay_us, code)) {
    return false;
  }
  std::vector<uint8_t> data;
  if (!read_register(kRegScDsgoc2, data) || data.size() < 2) {
    return false;
  }
  uint8_t byte0 = data[0];
  byte0 = (byte0 & ~0x18) | ((code & 0x03) << 3);
  const uint8_t payload[2] = {byte0, data[1]};
  return write_register_bytes(kRegScDsgoc2, payload, 2);
}

bool Api::set_short_circuit_mv(uint16_t mv) {
  uint8_t code = 0;
  if (!map_u16_to_index(kScMvOptions, sizeof(kScMvOptions) / sizeof(uint16_t), mv, code)) {
    return false;
  }
  std::vector<uint8_t> data;
  if (!read_register(kRegScDsgoc2, data) || data.size() < 2) {
    return false;
  }
  uint8_t byte0 = data[0];
  byte0 = (byte0 & ~0x07) | (code & 0x07);
  const uint8_t payload[2] = {byte0, data[1]};
  return write_register_bytes(kRegScDsgoc2, payload, 2);
}

bool Api::set_short_circuit_a(float a) {
  if (a <= 0.0f) {
    return false;
  }
  uint16_t raw = 0;
  if (!read_u16(kRegShuntRes, raw) || raw == 0) {
    return false;
  }
  const float shunt_mohm = static_cast<float>(raw) / 10.0f;
  return set_short_circuit_a(a, shunt_mohm);
}

bool Api::set_short_circuit_a(float a, float shunt_mohm) {
  if (a <= 0.0f || shunt_mohm <= 0.0f) {
    return false;
  }
  std::vector<uint8_t> data;
  if (!read_register(kRegScDsgoc2, data) || data.size() < 2) {
    return false;
  }
  const bool x2 = (data[0] & 0x80) != 0;
  const float factor = x2 ? 2.0f : 1.0f;
  const float mv_actual = a * shunt_mohm;
  const uint16_t mv_base = static_cast<uint16_t>(mv_actual / factor + 0.5f);
  uint8_t code = 0;
  if (!map_u16_to_nearest_index(kScMvOptions, sizeof(kScMvOptions) / sizeof(uint16_t), mv_base,
                                code)) {
    return false;
  }
  uint8_t byte0 = data[0];
  byte0 = (byte0 & ~0x07) | (code & 0x07);
  const uint8_t payload[2] = {byte0, data[1]};
  return write_register_bytes(kRegScDsgoc2, payload, 2);
}

bool Api::set_dsgoc2_delay_ms(uint16_t delay_ms) {
  uint8_t code = 0;
  if (!map_u16_to_index(kDsgoc2DelayMsOptions, sizeof(kDsgoc2DelayMsOptions) / sizeof(uint16_t),
                        delay_ms, code)) {
    return false;
  }
  std::vector<uint8_t> data;
  if (!read_register(kRegScDsgoc2, data) || data.size() < 2) {
    return false;
  }
  uint8_t byte1 = data[1];
  byte1 = (byte1 & ~0xF0) | ((code & 0x0F) << 4);
  const uint8_t payload[2] = {data[0], byte1};
  return write_register_bytes(kRegScDsgoc2, payload, 2);
}

bool Api::set_dsgoc2_mv(uint16_t mv) {
  uint8_t code = 0;
  if (!map_u16_to_index(kDsgoc2MvOptions, sizeof(kDsgoc2MvOptions) / sizeof(uint16_t), mv, code)) {
    return false;
  }
  std::vector<uint8_t> data;
  if (!read_register(kRegScDsgoc2, data) || data.size() < 2) {
    return false;
  }
  uint8_t byte1 = data[1];
  byte1 = (byte1 & ~0x0F) | (code & 0x0F);
  const uint8_t payload[2] = {data[0], byte1};
  return write_register_bytes(kRegScDsgoc2, payload, 2);
}

bool Api::set_dsgoc2_a(float a) {
  if (a <= 0.0f) {
    return false;
  }
  uint16_t raw = 0;
  if (!read_u16(kRegShuntRes, raw) || raw == 0) {
    return false;
  }
  const float shunt_mohm = static_cast<float>(raw) / 10.0f;
  return set_dsgoc2_a(a, shunt_mohm);
}

bool Api::set_dsgoc2_a(float a, float shunt_mohm) {
  if (a <= 0.0f || shunt_mohm <= 0.0f) {
    return false;
  }
  std::vector<uint8_t> data;
  if (!read_register(kRegScDsgoc2, data) || data.size() < 2) {
    return false;
  }
  const bool x2 = (data[0] & 0x80) != 0;
  const float factor = x2 ? 2.0f : 1.0f;
  const float mv_actual = a * shunt_mohm;
  const uint16_t mv_base = static_cast<uint16_t>(mv_actual / factor + 0.5f);
  uint8_t code = 0;
  if (!map_u16_to_nearest_index(kDsgoc2MvOptions,
                                sizeof(kDsgoc2MvOptions) / sizeof(uint16_t), mv_base, code)) {
    return false;
  }
  uint8_t byte1 = data[1];
  byte1 = (byte1 & ~0x0F) | (code & 0x0F);
  const uint8_t payload[2] = {data[0], byte1};
  return write_register_bytes(kRegScDsgoc2, payload, 2);
}

bool Api::set_secondary_cell_overvoltage_delay_s(uint8_t seconds) {
  uint8_t code = 0;
  if (!map_u16_to_index(kCovpHighDelaySOptions, sizeof(kCovpHighDelaySOptions) / sizeof(uint16_t),
                        seconds, code)) {
    return false;
  }
  std::vector<uint8_t> data;
  if (!read_register(kRegCxvpHighDelayScRel, data) || data.size() < 2) {
    return false;
  }
  uint8_t byte0 = data[0];
  byte0 = (byte0 & ~0x30) | ((code & 0x03) << 4);
  const uint8_t payload[2] = {byte0, data[1]};
  return write_register_bytes(kRegCxvpHighDelayScRel, payload, 2);
}

bool Api::set_secondary_cell_undervoltage_delay_s(uint8_t seconds) {
  uint8_t code = 0;
  if (!map_u16_to_index(kCuvpHighDelaySOptions, sizeof(kCuvpHighDelaySOptions) / sizeof(uint16_t),
                        seconds, code)) {
    return false;
  }
  std::vector<uint8_t> data;
  if (!read_register(kRegCxvpHighDelayScRel, data) || data.size() < 2) {
    return false;
  }
  uint8_t byte0 = data[0];
  byte0 = (byte0 & ~0xC0) | ((code & 0x03) << 6);
  const uint8_t payload[2] = {byte0, data[1]};
  return write_register_bytes(kRegCxvpHighDelayScRel, payload, 2);
}

bool Api::set_short_circuit_release_s(uint8_t seconds) {
  std::vector<uint8_t> data;
  if (!read_register(kRegCxvpHighDelayScRel, data) || data.size() < 2) {
    return false;
  }
  const uint8_t payload[2] = {data[0], seconds};
  return write_register_bytes(kRegCxvpHighDelayScRel, payload, 2);
}

bool Api::set_secondary_cell_overvoltage_v(float v) { return write_register(kRegCovpHigh, to_mv_1(v)); }

bool Api::set_secondary_cell_undervoltage_v(float v) { return write_register(kRegCuvpHigh, to_mv_1(v)); }

bool Api::read_secondary_cell_overvoltage_v(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCovpHigh, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_secondary_cell_undervoltage_v(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCuvpHigh, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_mfg_name(String &out) { return read_string(0xA0, out); }

bool Api::read_device_name(String &out) { return read_string(0xA1, out); }

bool Api::read_barcode(String &out) { return read_string(0xA2, out); }

bool Api::set_mfg_name(const String &value) { return write_string(0xA0, value); }

bool Api::set_device_name(const String &value) { return write_string(0xA1, value); }

bool Api::set_barcode(const String &value) { return write_string(0xA2, value); }

bool Api::set_manufacture_date(uint16_t year, uint8_t month, uint8_t day) {
  if (year < 2000 || year > 2127 || month < 1 || month > 12 || day < 1 || day > 31) {
    return false;
  }
  const uint16_t packed =
      uint16_t(((year - 2000) << 9) | ((uint16_t(month) & 0x0F) << 5) | (day & 0x1F));
  return write_register(kRegMfgDate, packed);
}

bool Api::set_serial_number(uint16_t serial) { return write_register(kRegSerialNum, serial); }

bool Api::set_cycle_count(uint16_t cycles) { return write_register(kRegCycleCnt, cycles); }

bool Api::read_manufacture_date(uint16_t &year, uint8_t &month, uint8_t &day) {
  uint16_t raw = 0;
  if (!read_u16(kRegMfgDate, raw)) {
    return false;
  }
  year = 2000 + ((raw >> 9) & 0x7F);
  month = (raw >> 5) & 0x0F;
  day = raw & 0x1F;
  return true;
}

bool Api::read_serial_number(uint16_t &serial) { return read_u16(kRegSerialNum, serial); }

bool Api::read_cycle_count(uint16_t &cycles) { return read_u16(kRegCycleCnt, cycles); }

static uint16_t to_mv_10(float v) {
  if (v < 0.0f) v = 0.0f;
  return static_cast<uint16_t>(v * 100.0f + 0.5f);
}

static uint16_t to_mv_1(float v) {
  if (v < 0.0f) v = 0.0f;
  return static_cast<uint16_t>(v * 1000.0f + 0.5f);
}

static uint16_t to_kelvin_tenth(float c) {
  return static_cast<uint16_t>(c * 10.0f + 2731.0f + 0.5f);
}

static float from_mv_10(uint16_t raw) { return static_cast<float>(raw) / 100.0f; }

static float from_mv_1(uint16_t raw) { return static_cast<float>(raw) / 1000.0f; }

static float from_kelvin_tenth(uint16_t raw) { return static_cast<float>(raw) / 10.0f - 273.1f; }

bool Api::read_pack_overvoltage_v(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegPovp, raw)) {
    return false;
  }
  out = from_mv_10(raw);
  return true;
}

bool Api::read_pack_overvoltage_release_v(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegPovpRel, raw)) {
    return false;
  }
  out = from_mv_10(raw);
  return true;
}

bool Api::read_pack_undervoltage_v(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegPuvp, raw)) {
    return false;
  }
  out = from_mv_10(raw);
  return true;
}

bool Api::read_pack_undervoltage_release_v(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegPuvpRel, raw)) {
    return false;
  }
  out = from_mv_10(raw);
  return true;
}

bool Api::read_cell_overvoltage_v(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCovp, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_cell_overvoltage_release_v(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCovpRel, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_cell_undervoltage_v(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCuvp, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_cell_undervoltage_release_v(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCuvpRel, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_pack_voltage_delays(uint8_t &under_s, uint8_t &over_s) {
  std::vector<uint8_t> data;
  if (!read_register(kRegPackVDelays, data) || data.size() < 2) {
    return false;
  }
  under_s = data[0];
  over_s = data[1];
  return true;
}

bool Api::read_cell_voltage_delays(uint8_t &under_s, uint8_t &over_s) {
  std::vector<uint8_t> data;
  if (!read_register(kRegCellVDelays, data) || data.size() < 2) {
    return false;
  }
  under_s = data[0];
  over_s = data[1];
  return true;
}

bool Api::read_charge_overcurrent_a(float &out) {
  int16_t raw = 0;
  if (!read_s16(kRegChgOc, raw)) {
    return false;
  }
  out = static_cast<float>(raw) / 100.0f;
  if (out < 0.0f) {
    out = -out;
  }
  return true;
}

bool Api::read_discharge_overcurrent_a(float &out) {
  int16_t raw = 0;
  if (!read_s16(kRegDsgOc, raw)) {
    return false;
  }
  out = static_cast<float>(raw) / 100.0f;
  if (out < 0.0f) {
    out = -out;
  }
  return true;
}

bool Api::read_charge_overcurrent_delays(uint8_t &delay_s, uint8_t &release_s) {
  std::vector<uint8_t> data;
  if (!read_register(kRegChgOcDelays, data) || data.size() < 2) {
    return false;
  }
  delay_s = data[0];
  release_s = data[1];
  return true;
}

bool Api::read_discharge_overcurrent_delays(uint8_t &delay_s, uint8_t &release_s) {
  std::vector<uint8_t> data;
  if (!read_register(kRegDsgOcDelays, data) || data.size() < 2) {
    return false;
  }
  delay_s = data[0];
  release_s = data[1];
  return true;
}

bool Api::read_charge_overtemp_c(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegChgOverTemp, raw)) {
    return false;
  }
  out = from_kelvin_tenth(raw);
  return true;
}

bool Api::read_charge_overtemp_release_c(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegChgOverTempRel, raw)) {
    return false;
  }
  out = from_kelvin_tenth(raw);
  return true;
}

bool Api::read_charge_undertemp_c(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegChgUnderTemp, raw)) {
    return false;
  }
  out = from_kelvin_tenth(raw);
  return true;
}

bool Api::read_charge_undertemp_release_c(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegChgUnderTempRel, raw)) {
    return false;
  }
  out = from_kelvin_tenth(raw);
  return true;
}

bool Api::read_discharge_overtemp_c(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegDsgOverTemp, raw)) {
    return false;
  }
  out = from_kelvin_tenth(raw);
  return true;
}

bool Api::read_discharge_overtemp_release_c(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegDsgOverTempRel, raw)) {
    return false;
  }
  out = from_kelvin_tenth(raw);
  return true;
}

bool Api::read_discharge_undertemp_c(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegDsgUnderTemp, raw)) {
    return false;
  }
  out = from_kelvin_tenth(raw);
  return true;
}

bool Api::read_discharge_undertemp_release_c(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegDsgUnderTempRel, raw)) {
    return false;
  }
  out = from_kelvin_tenth(raw);
  return true;
}

bool Api::read_charge_temp_delays(uint8_t &under_rel_s, uint8_t &over_rel_s) {
  std::vector<uint8_t> data;
  if (!read_register(kRegChgTempDelays, data) || data.size() < 2) {
    return false;
  }
  under_rel_s = data[0];
  over_rel_s = data[1];
  return true;
}

bool Api::read_discharge_temp_delays(uint8_t &under_rel_s, uint8_t &over_rel_s) {
  std::vector<uint8_t> data;
  if (!read_register(kRegDsgTempDelays, data) || data.size() < 2) {
    return false;
  }
  under_rel_s = data[0];
  over_rel_s = data[1];
  return true;
}

bool Api::set_pack_overvoltage_v(float v) { return write_register(kRegPovp, to_mv_10(v)); }
bool Api::set_pack_overvoltage_release_v(float v) { return write_register(kRegPovpRel, to_mv_10(v)); }
bool Api::set_pack_undervoltage_v(float v) { return write_register(kRegPuvp, to_mv_10(v)); }
bool Api::set_pack_undervoltage_release_v(float v) { return write_register(kRegPuvpRel, to_mv_10(v)); }
bool Api::set_cell_overvoltage_v(float v) { return write_register(kRegCovp, to_mv_1(v)); }
bool Api::set_cell_overvoltage_release_v(float v) { return write_register(kRegCovpRel, to_mv_1(v)); }
bool Api::set_cell_undervoltage_v(float v) { return write_register(kRegCuvp, to_mv_1(v)); }
bool Api::set_cell_undervoltage_release_v(float v) { return write_register(kRegCuvpRel, to_mv_1(v)); }

bool Api::set_pack_voltage_delays(uint8_t under_s, uint8_t over_s) {
  const uint8_t payload[2] = {under_s, over_s};
  return write_register_bytes(kRegPackVDelays, payload, 2);
}

bool Api::set_cell_voltage_delays(uint8_t under_s, uint8_t over_s) {
  const uint8_t payload[2] = {under_s, over_s};
  return write_register_bytes(kRegCellVDelays, payload, 2);
}

bool Api::set_charge_overcurrent_a(float a) {
  if (a < 0.0f) a = -a;
  const int16_t raw = int16_t(a * 100.0f + 0.5f);
  return write_register(kRegChgOc, uint16_t(raw));
}

bool Api::set_discharge_overcurrent_a(float a) {
  if (a < 0.0f) a = -a;
  const int16_t raw = int16_t(-a * 100.0f - 0.5f);
  return write_register(kRegDsgOc, uint16_t(raw));
}

bool Api::set_charge_overcurrent_delays(uint8_t delay_s, uint8_t release_s) {
  const uint8_t payload[2] = {delay_s, release_s};
  return write_register_bytes(kRegChgOcDelays, payload, 2);
}

bool Api::set_discharge_overcurrent_delays(uint8_t delay_s, uint8_t release_s) {
  const uint8_t payload[2] = {delay_s, release_s};
  return write_register_bytes(kRegDsgOcDelays, payload, 2);
}

bool Api::set_charge_overtemp_c(float c) { return write_register(kRegChgOverTemp, to_kelvin_tenth(c)); }
bool Api::set_charge_overtemp_release_c(float c) {
  return write_register(kRegChgOverTempRel, to_kelvin_tenth(c));
}
bool Api::set_charge_undertemp_c(float c) { return write_register(kRegChgUnderTemp, to_kelvin_tenth(c)); }
bool Api::set_charge_undertemp_release_c(float c) {
  return write_register(kRegChgUnderTempRel, to_kelvin_tenth(c));
}
bool Api::set_discharge_overtemp_c(float c) { return write_register(kRegDsgOverTemp, to_kelvin_tenth(c)); }
bool Api::set_discharge_overtemp_release_c(float c) {
  return write_register(kRegDsgOverTempRel, to_kelvin_tenth(c));
}
bool Api::set_discharge_undertemp_c(float c) { return write_register(kRegDsgUnderTemp, to_kelvin_tenth(c)); }
bool Api::set_discharge_undertemp_release_c(float c) {
  return write_register(kRegDsgUnderTempRel, to_kelvin_tenth(c));
}

bool Api::set_charge_temp_delays(uint8_t under_rel_s, uint8_t over_rel_s) {
  const uint8_t payload[2] = {under_rel_s, over_rel_s};
  return write_register_bytes(kRegChgTempDelays, payload, 2);
}

bool Api::set_discharge_temp_delays(uint8_t under_rel_s, uint8_t over_rel_s) {
  const uint8_t payload[2] = {under_rel_s, over_rel_s};
  return write_register_bytes(kRegDsgTempDelays, payload, 2);
}

bool Api::read_design_capacity_ah(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegDesignCap, raw)) {
    return false;
  }
  out = static_cast<float>(raw) / 100.0f;
  return true;
}

bool Api::read_design_capacity_mah(float &out) {
  float ah = 0.0f;
  if (!read_design_capacity_ah(ah)) {
    return false;
  }
  out = ah * 1000.0f;
  return true;
}

bool Api::read_cycle_capacity_ah(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCycleCap, raw)) {
    return false;
  }
  out = static_cast<float>(raw) / 100.0f;
  return true;
}

bool Api::read_cycle_capacity_mah(float &out) {
  float ah = 0.0f;
  if (!read_cycle_capacity_ah(ah)) {
    return false;
  }
  out = ah * 1000.0f;
  return true;
}

bool Api::read_full_charge_voltage_v(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCapFull, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_charge_end_voltage_v(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCap0, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_capacity_voltage_100(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCap100Pct, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_capacity_voltage_90(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCap90, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_capacity_voltage_80(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCap80, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_capacity_voltage_70(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCap70, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_capacity_voltage_60(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCap60, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_capacity_voltage_50(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCap50, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_capacity_voltage_40(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCap40, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_capacity_voltage_30(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCap30, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_capacity_voltage_20(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCap20, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_capacity_voltage_10(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCap10, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_capacity_voltage_0(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegCap0, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_self_discharge_rate_percent(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegDsgRate, raw)) {
    return false;
  }
  out = static_cast<float>(raw) / 10.0f;
  return true;
}

bool Api::read_gps_off_voltage_v(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegGpsOff, raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_gps_off_time_s(uint16_t &seconds) { return read_u16(kRegGpsOffTime, seconds); }

bool Api::read_cell_count(uint16_t &count) { return read_u16(kRegCellCnt, count); }

bool Api::read_ntc_enable_mask(uint16_t &mask) { return read_u16(kRegNtcConfig, mask); }

bool Api::read_idle_current_calibration_raw(uint16_t &out) {
  return read_u16(kRegIdleCurrentCal, out);
}

bool Api::read_charge_current_calibration_a(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegChargeCurrentCal, raw)) {
    return false;
  }
  out = static_cast<float>(raw) / 100.0f;
  return true;
}

bool Api::read_discharge_current_calibration_a(float &out) {
  uint16_t raw = 0;
  if (!read_u16(kRegDischargeCurrentCal, raw)) {
    return false;
  }
  out = static_cast<float>(raw) / 100.0f;
  return true;
}

bool Api::read_cell_voltage_calibration(uint8_t cell, float &out) {
  if (cell == 0 || cell > 32) {
    return false;
  }
  uint16_t raw = 0;
  if (!read_u16(uint8_t(kRegCellCalBase + (cell - 1)), raw)) {
    return false;
  }
  out = from_mv_1(raw);
  return true;
}

bool Api::read_ntc_calibration(uint8_t ntc_index, float &out) {
  if (ntc_index == 0 || ntc_index > 8) {
    return false;
  }
  uint16_t raw = 0;
  if (!read_u16(uint8_t(kRegNtcCalBase + (ntc_index - 1)), raw)) {
    return false;
  }
  out = from_kelvin_tenth(raw);
  return true;
}

bool Api::read_cell_resistance_mohm(uint8_t cell, float &out, uint8_t base_reg) {
  uint16_t raw = 0;
  if (!read_cell_resistance_raw(cell, raw, base_reg)) {
    return false;
  }
  out = static_cast<float>(raw) / 10.0f;
  return true;
}

bool Api::read_cell_resistance_raw(uint8_t cell, uint16_t &out, uint8_t base_reg) {
  if (cell == 0 || cell > 32) {
    return false;
  }
  return read_u16(uint8_t(base_reg + (cell - 1)), out);
}

bool Api::set_design_capacity_ah(float ah) {
  if (ah < 0.0f) {
    return false;
  }
  const uint16_t value = static_cast<uint16_t>(ah * 100.0f + 0.5f);
  return write_register(kRegDesignCap, value);
}

bool Api::set_design_capacity_mah(float mah) {
  if (mah < 0.0f) {
    return false;
  }
  return set_design_capacity_ah(mah / 1000.0f);
}

bool Api::set_cycle_capacity_ah(float ah) {
  if (ah < 0.0f) {
    return false;
  }
  const uint16_t value = static_cast<uint16_t>(ah * 100.0f + 0.5f);
  return write_register(kRegCycleCap, value);
}

bool Api::set_cycle_capacity_mah(float mah) {
  if (mah < 0.0f) {
    return false;
  }
  return set_cycle_capacity_ah(mah / 1000.0f);
}

bool Api::set_full_charge_voltage_v(float v) { return write_register(kRegCapFull, to_mv_1(v)); }
bool Api::set_charge_end_voltage_v(float v) { return write_register(kRegCap0, to_mv_1(v)); }
bool Api::set_capacity_voltage_100(float v) { return write_register(kRegCap100Pct, to_mv_1(v)); }
bool Api::set_capacity_voltage_90(float v) { return write_register(kRegCap90, to_mv_1(v)); }
bool Api::set_capacity_voltage_80(float v) { return write_register(kRegCap80, to_mv_1(v)); }
bool Api::set_capacity_voltage_70(float v) { return write_register(kRegCap70, to_mv_1(v)); }
bool Api::set_capacity_voltage_60(float v) { return write_register(kRegCap60, to_mv_1(v)); }
bool Api::set_capacity_voltage_50(float v) { return write_register(kRegCap50, to_mv_1(v)); }
bool Api::set_capacity_voltage_40(float v) { return write_register(kRegCap40, to_mv_1(v)); }
bool Api::set_capacity_voltage_30(float v) { return write_register(kRegCap30, to_mv_1(v)); }
bool Api::set_capacity_voltage_20(float v) { return write_register(kRegCap20, to_mv_1(v)); }
bool Api::set_capacity_voltage_10(float v) { return write_register(kRegCap10, to_mv_1(v)); }
bool Api::set_capacity_voltage_0(float v) { return write_register(kRegCap0, to_mv_1(v)); }

bool Api::set_self_discharge_rate_percent(float percent) {
  if (percent < 0.0f) {
    return false;
  }
  const uint16_t value = static_cast<uint16_t>(percent * 10.0f + 0.5f);
  return write_register(kRegDsgRate, value);
}

bool Api::set_gps_off_voltage_v(float v) { return write_register(kRegGpsOff, to_mv_1(v)); }

bool Api::set_gps_off_time_s(uint16_t seconds) { return write_register(kRegGpsOffTime, seconds); }

bool Api::set_cell_count(uint16_t count) { return write_register(kRegCellCnt, count); }

bool Api::set_shunt_res_mohm(float mohm) {
  if (mohm < 0.0f) {
    return false;
  }
  const uint16_t value = static_cast<uint16_t>(mohm * 10.0f + 0.5f);
  return write_register(kRegShuntRes, value);
}

bool Api::set_ntc_enable_mask(uint16_t mask) { return write_register(kRegNtcConfig, mask); }

bool Api::set_idle_current_calibration() { return write_register(kRegIdleCurrentCal, 0x0000); }

bool Api::set_charge_current_calibration_a(float a) {
  if (a < 0.0f) {
    return false;
  }
  const uint16_t value = static_cast<uint16_t>(a * 100.0f + 0.5f);
  return write_register(kRegChargeCurrentCal, value);
}

bool Api::set_discharge_current_calibration_a(float a) {
  if (a < 0.0f) {
    return false;
  }
  const uint16_t value = static_cast<uint16_t>(a * 100.0f + 0.5f);
  return write_register(kRegDischargeCurrentCal, value);
}

bool Api::set_cell_voltage_calibration(uint8_t cell, float v) {
  if (cell == 0 || cell > 32) {
    return false;
  }
  const uint16_t value = to_mv_1(v);
  return write_register(uint8_t(kRegCellCalBase + (cell - 1)), value);
}

bool Api::set_ntc_calibration(uint8_t ntc_index, float c) {
  if (ntc_index == 0 || ntc_index > 8) {
    return false;
  }
  const uint16_t value = to_kelvin_tenth(c);
  return write_register(uint8_t(kRegNtcCalBase + (ntc_index - 1)), value);
}

bool Api::set_ntc_calibration_all(float c, uint8_t count) {
  if (count == 0 || count > 8) {
    return false;
  }
  for (uint8_t i = 1; i <= count; i++) {
    if (!set_ntc_calibration(i, c)) {
      return false;
    }
  }
  return true;
}

bool Api::set_cell_resistance_raw(uint8_t cell, uint16_t raw, uint8_t base_reg) {
  if (cell == 0 || cell > 32) {
    return false;
  }
  return write_register(uint8_t(base_reg + (cell - 1)), raw);
}

bool Api::set_cell_resistance_mohm(uint8_t cell, float mohm, uint8_t base_reg) {
  if (mohm < 0.0f) {
    return false;
  }
  const uint16_t raw = static_cast<uint16_t>(mohm * 10.0f + 0.5f);
  return set_cell_resistance_raw(cell, raw, base_reg);
}
} // namespace jbd
