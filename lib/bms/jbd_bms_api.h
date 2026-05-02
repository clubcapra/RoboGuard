#pragma once

#include <Arduino.h>
#include <vector>

namespace jbd {

struct Frame {
  uint8_t function{0};
  uint8_t status{0};
  uint8_t data_len{0};
  uint16_t checksum{0};
  std::vector<uint8_t> data;
};

struct BasicInfo {
  uint16_t total_voltage_raw{0};
  int16_t current_raw{0};
  uint16_t capacity_remaining_raw{0};
  uint16_t nominal_capacity_raw{0};
  uint16_t cycle_count_raw{0};
  float total_voltage{0.0f};
  float current{0.0f};
  float power{0.0f};
  float charging_power{0.0f};
  float discharging_power{0.0f};
  float capacity_remaining_ah{0.0f};
  float nominal_capacity_ah{0.0f};
  uint16_t cycle_count{0};
  uint16_t production_date_raw{0};
  uint16_t balance_low{0};
  uint16_t balance_high{0};
  uint32_t balance_mask{0};
  uint16_t protection_status{0};
  uint8_t software_version_raw{0};
  float software_version{0.0f};
  uint8_t soc{0};
  uint8_t mos_status{0};
  uint8_t strings{0};
  uint8_t temperature_sensors{0};
  float temperatures_c[6]{};
  uint16_t temperatures_raw[6]{};
};

struct CellInfo {
  uint8_t cell_count{0};
  uint16_t cell_raw[32]{};
  float cell_v[32]{};
  uint8_t min_cell{0};
  uint8_t max_cell{0};
  float min_v{0.0f};
  float max_v{0.0f};
  float avg_v{0.0f};
  float delta_v{0.0f};
};

struct ErrorCounts {
  uint16_t short_circuit{0};
  uint16_t charge_overcurrent{0};
  uint16_t discharge_overcurrent{0};
  uint16_t cell_overvoltage{0};
  uint16_t cell_undervoltage{0};
  uint16_t charge_overtemperature{0};
  uint16_t charge_undertemperature{0};
  uint16_t discharge_overtemperature{0};
  uint16_t discharge_undertemperature{0};
  uint16_t battery_overvoltage{0};
  uint16_t battery_undervoltage{0};
  uint16_t reset_count{0};
};

struct ModelInfo {
  String model;
  uint8_t length{0};
};

struct ScDsgoc2Config {
  bool x2{false};
  uint16_t sc_delay_us{0};
  uint16_t sc_mv{0};
  uint16_t dsgoc2_delay_ms{0};
  uint16_t dsgoc2_mv{0};
  uint8_t raw_byte0{0};
  uint8_t raw_byte1{0};
};

struct CxvpHighDelayConfig {
  uint16_t cuvp_high_delay_s{0};
  uint16_t covp_high_delay_s{0};
  uint16_t sc_release_s{0};
  uint8_t raw_byte0{0};
  uint8_t raw_byte1{0};
};

struct LogEntry {
  uint32_t ms{0};
  String message;
};

class BlackBoxLogger {
 public:
  explicit BlackBoxLogger(size_t capacity = 64);

  void set_capacity(size_t capacity);
  void set_permanent_threshold_ms(uint32_t ms);
  void record_basic(const BasicInfo &info, uint32_t now_ms);
  void record_errors(const ErrorCounts &errors, uint32_t now_ms);
  void dump(Print &out) const;
  void clear();

 private:
  size_t capacity_{64};
  std::vector<LogEntry> entries_;
  uint16_t last_protection_{0};
  uint32_t protection_since_ms_{0};
  bool permanent_logged_{false};
  ErrorCounts last_errors_{};
  bool has_last_errors_{false};
  uint32_t permanent_threshold_ms_{60000};

  void add_(uint32_t ms, const String &message);
};

class Api {
 public:
  Api(HardwareSerial &port, Print *log = nullptr);

  void begin(uint32_t baud = 9600);
  void set_debug(bool enabled);

  bool read_basic_info(BasicInfo &out, uint32_t timeout_ms = 1500);
  bool read_cell_info(CellInfo &out, uint32_t timeout_ms = 1500);
  bool read_model(ModelInfo &out, uint32_t timeout_ms = 1500);
  bool read_error_counts(ErrorCounts &out, uint32_t timeout_ms = 1500);
  bool read_history_data(std::vector<uint8_t> &out, uint32_t timeout_ms = 1500);
  bool clear_history_data(uint32_t timeout_ms = 1500);
  bool set_rtc(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute,
               uint8_t second, uint32_t timeout_ms = 1500);
  bool read_rtc(uint16_t &year, uint8_t &month, uint8_t &day, uint8_t &hour, uint8_t &minute,
                uint8_t &second, uint32_t timeout_ms = 1500);

  bool mos_action(uint8_t action, uint32_t timeout_ms = 1500);
  bool set_charge(bool enabled);
  bool set_discharge(bool enabled);
  bool set_charge_discharge(bool enabled);
  bool set_mos_mask(uint8_t mask);

  bool set_balancer(bool enabled);
  bool set_balance_control(uint16_t mode);
  bool set_balance_start_v(float v);
  bool set_balance_window_v(float v);
  bool set_balance_enable(bool enabled);
  bool set_charge_balance_enable(bool enabled);
  bool read_balance_control(uint16_t &out);
  bool read_balance_start_v(float &out);
  bool read_balance_window_v(float &out);
  bool read_balance_enable(bool &out);
  bool read_charge_balance_enable(bool &out);
  bool set_capacity_remaining_ah(float ah);
  bool read_capacity_remaining_ah(float &out);
  bool set_capacity_remaining_mah(float mah);
  bool read_capacity_remaining_mah(float &out);
  bool reset_capacity();
  bool set_soc_percent(float percent);
  bool soc_reset();
  bool enter_factory_mode();
  bool exit_factory_mode(bool save_and_reset = false);
  bool factory_mode(bool enable);
  bool clear_error_counts();
  bool write_register(uint8_t address, uint16_t value);
  bool write_register_bytes(uint8_t address, const uint8_t *data, uint8_t length,
                            uint32_t timeout_ms = 1500);
  bool read_register(uint8_t address, std::vector<uint8_t> &out, uint32_t timeout_ms = 1500);
  bool read_u16(uint8_t address, uint16_t &out, uint32_t timeout_ms = 1500);
  bool read_s16(uint8_t address, int16_t &out, uint32_t timeout_ms = 1500);
  bool read_string(uint8_t address, String &out, uint32_t timeout_ms = 1500);
  bool write_string(uint8_t address, const String &value);
  bool read_shunt_res_mohm(float &out);
  bool read_func_config(uint16_t &out);
  bool read_func_switch(bool &out);
  bool read_func_scrl(bool &out);
  bool read_func_load(bool &out);
  bool read_led_enable(bool &out);
  bool read_led_num(bool &out);
  bool read_func_rtc(bool &out);
  bool read_func_edv(bool &out);
  bool read_func_chg_limit(bool &out);
  bool read_func_gps_enable(bool &out);
  bool read_func_buzzer_enable(bool &out);
  bool read_func_car_mode(bool &out);
  bool set_func_config(uint16_t value);
  bool set_func_switch(bool enabled);
  bool set_func_scrl(bool enabled);
  bool set_func_load(bool enabled);
  bool set_led_enable(bool enabled);
  bool set_led_num(bool enabled);
  bool set_func_rtc(bool enabled);
  bool set_func_edv(bool enabled);
  bool set_func_chg_limit(bool enabled);
  bool set_func_gps_enable(bool enabled);
  bool set_func_buzzer_enable(bool enabled);
  bool set_func_car_mode(bool enabled);
  bool read_fet_ctrl(uint16_t &out);
  bool set_fet_ctrl(uint16_t value);
  bool read_led_timer(uint16_t &out);
  bool set_led_timer(uint16_t value);
  bool set_sc_dsgoc2(uint8_t byte0, uint8_t byte1);
  bool set_cxvp_high_delay_sc_rel(uint8_t byte0, uint8_t byte1);
  bool read_sc_dsgoc_x2(bool &out);
  bool read_short_circuit_delay_us(uint16_t &out);
  bool read_short_circuit_mv(uint16_t &out);
  bool set_sc_dsgoc_x2(bool enabled);
  bool set_short_circuit_delay_us(uint16_t delay_us);
  bool set_short_circuit_mv(uint16_t mv);
  bool set_short_circuit_a(float a);
  bool set_short_circuit_a(float a, float shunt_mohm);
  bool read_dsgoc2_delay_ms(uint16_t &out);
  bool read_dsgoc2_mv(uint16_t &out);
  bool set_dsgoc2_delay_ms(uint16_t delay_ms);
  bool set_dsgoc2_mv(uint16_t mv);
  bool set_dsgoc2_a(float a);
  bool set_dsgoc2_a(float a, float shunt_mohm);
  bool read_sc_dsgoc2_config(ScDsgoc2Config &out);
  bool read_cxvp_high_delay_sc_rel_config(CxvpHighDelayConfig &out);
  bool read_secondary_cell_overvoltage_delay_s(uint16_t &out);
  bool read_secondary_cell_undervoltage_delay_s(uint16_t &out);
  bool read_short_circuit_release_s(uint16_t &out);
  bool set_secondary_cell_overvoltage_delay_s(uint8_t seconds);
  bool set_secondary_cell_undervoltage_delay_s(uint8_t seconds);
  bool set_short_circuit_release_s(uint8_t seconds);
  bool set_secondary_cell_overvoltage_v(float v);
  bool set_secondary_cell_undervoltage_v(float v);
  bool read_secondary_cell_overvoltage_v(float &out);
  bool read_secondary_cell_undervoltage_v(float &out);
  bool read_mfg_name(String &out);
  bool read_device_name(String &out);
  bool read_barcode(String &out);
  bool set_mfg_name(const String &value);
  bool set_device_name(const String &value);
  bool set_barcode(const String &value);
  bool set_manufacture_date(uint16_t year, uint8_t month, uint8_t day);
  bool read_manufacture_date(uint16_t &year, uint8_t &month, uint8_t &day);
  bool set_serial_number(uint16_t serial);
  bool read_serial_number(uint16_t &serial);
  bool set_cycle_count(uint16_t cycles);
  bool read_cycle_count(uint16_t &cycles);

  // Voltage thresholds (V)
  bool read_pack_overvoltage_v(float &out);
  bool read_pack_overvoltage_release_v(float &out);
  bool read_pack_undervoltage_v(float &out);
  bool read_pack_undervoltage_release_v(float &out);
  bool read_cell_overvoltage_v(float &out);
  bool read_cell_overvoltage_release_v(float &out);
  bool read_cell_undervoltage_v(float &out);
  bool read_cell_undervoltage_release_v(float &out);
  bool set_pack_overvoltage_v(float v);
  bool set_pack_overvoltage_release_v(float v);
  bool set_pack_undervoltage_v(float v);
  bool set_pack_undervoltage_release_v(float v);
  bool set_cell_overvoltage_v(float v);
  bool set_cell_overvoltage_release_v(float v);
  bool set_cell_undervoltage_v(float v);
  bool set_cell_undervoltage_release_v(float v);

  // Voltage delays (s)
  bool read_pack_voltage_delays(uint8_t &under_s, uint8_t &over_s);
  bool read_cell_voltage_delays(uint8_t &under_s, uint8_t &over_s);
  bool set_pack_voltage_delays(uint8_t under_s, uint8_t over_s);
  bool set_cell_voltage_delays(uint8_t under_s, uint8_t over_s);

  // Current thresholds (A)
  bool read_charge_overcurrent_a(float &out);
  bool read_discharge_overcurrent_a(float &out);
  bool set_charge_overcurrent_a(float a);
  bool set_discharge_overcurrent_a(float a);

  // Current delays (s)
  bool read_charge_overcurrent_delays(uint8_t &delay_s, uint8_t &release_s);
  bool read_discharge_overcurrent_delays(uint8_t &delay_s, uint8_t &release_s);
  bool set_charge_overcurrent_delays(uint8_t delay_s, uint8_t release_s);
  bool set_discharge_overcurrent_delays(uint8_t delay_s, uint8_t release_s);

  // Temperature thresholds (C)
  bool read_charge_overtemp_c(float &out);
  bool read_charge_overtemp_release_c(float &out);
  bool read_charge_undertemp_c(float &out);
  bool read_charge_undertemp_release_c(float &out);
  bool read_discharge_overtemp_c(float &out);
  bool read_discharge_overtemp_release_c(float &out);
  bool read_discharge_undertemp_c(float &out);
  bool read_discharge_undertemp_release_c(float &out);
  bool set_charge_overtemp_c(float c);
  bool set_charge_overtemp_release_c(float c);
  bool set_charge_undertemp_c(float c);
  bool set_charge_undertemp_release_c(float c);
  bool set_discharge_overtemp_c(float c);
  bool set_discharge_overtemp_release_c(float c);
  bool set_discharge_undertemp_c(float c);
  bool set_discharge_undertemp_release_c(float c);

  // Temperature delays (s)
  bool read_charge_temp_delays(uint8_t &under_rel_s, uint8_t &over_rel_s);
  bool read_discharge_temp_delays(uint8_t &under_rel_s, uint8_t &over_rel_s);
  bool set_charge_temp_delays(uint8_t under_rel_s, uint8_t over_rel_s);
  bool set_discharge_temp_delays(uint8_t under_rel_s, uint8_t over_rel_s);

  // Capacity / calibration / config
  bool read_design_capacity_ah(float &out);
  bool read_design_capacity_mah(float &out);
  bool read_cycle_capacity_ah(float &out);
  bool read_cycle_capacity_mah(float &out);
  bool read_full_charge_voltage_v(float &out);
  bool read_charge_end_voltage_v(float &out);
  bool read_capacity_voltage_100(float &out);
  bool read_capacity_voltage_90(float &out);
  bool read_capacity_voltage_80(float &out);
  bool read_capacity_voltage_70(float &out);
  bool read_capacity_voltage_60(float &out);
  bool read_capacity_voltage_50(float &out);
  bool read_capacity_voltage_40(float &out);
  bool read_capacity_voltage_30(float &out);
  bool read_capacity_voltage_20(float &out);
  bool read_capacity_voltage_10(float &out);
  bool read_capacity_voltage_0(float &out);
  bool read_self_discharge_rate_percent(float &out);
  bool read_gps_off_voltage_v(float &out);
  bool read_gps_off_time_s(uint16_t &seconds);
  bool read_cell_count(uint16_t &count);
  bool read_ntc_enable_mask(uint16_t &mask);
  bool read_idle_current_calibration_raw(uint16_t &out);
  bool read_charge_current_calibration_a(float &out);
  bool read_discharge_current_calibration_a(float &out);
  bool read_cell_voltage_calibration(uint8_t cell, float &out);
  bool read_ntc_calibration(uint8_t ntc_index, float &out);
  bool read_cell_resistance_mohm(uint8_t cell, float &out, uint8_t base_reg);
  bool read_cell_resistance_raw(uint8_t cell, uint16_t &out, uint8_t base_reg);
  bool set_design_capacity_ah(float ah);
  bool set_design_capacity_mah(float mah);
  bool set_cycle_capacity_ah(float ah);
  bool set_cycle_capacity_mah(float mah);
  bool set_full_charge_voltage_v(float v);
  bool set_charge_end_voltage_v(float v);
  bool set_capacity_voltage_100(float v);
  bool set_capacity_voltage_90(float v);
  bool set_capacity_voltage_80(float v);
  bool set_capacity_voltage_70(float v);
  bool set_capacity_voltage_60(float v);
  bool set_capacity_voltage_50(float v);
  bool set_capacity_voltage_40(float v);
  bool set_capacity_voltage_30(float v);
  bool set_capacity_voltage_20(float v);
  bool set_capacity_voltage_10(float v);
  bool set_capacity_voltage_0(float v);
  bool set_self_discharge_rate_percent(float percent);
  bool set_gps_off_voltage_v(float v);
  bool set_gps_off_time_s(uint16_t seconds);
  bool set_cell_count(uint16_t count);
  bool set_shunt_res_mohm(float mohm);
  bool set_ntc_enable_mask(uint16_t mask);
  bool set_idle_current_calibration();
  bool set_charge_current_calibration_a(float a);
  bool set_discharge_current_calibration_a(float a);
  bool set_cell_voltage_calibration(uint8_t cell, float v);
  bool set_ntc_calibration(uint8_t ntc_index, float c);
  bool set_ntc_calibration_all(float c, uint8_t count = 8);
  bool set_cell_resistance_mohm(uint8_t cell, float mohm, uint8_t base_reg);
  bool set_cell_resistance_raw(uint8_t cell, uint16_t raw, uint8_t base_reg);

  bool poll(Frame &frame);
  bool wait_for_frame(uint8_t function, Frame &frame, uint32_t timeout_ms);

  static uint16_t checksum(const uint8_t *data, uint16_t len);
  static uint16_t checksum_china(const uint8_t *raw, uint8_t data_len);

 private:
  HardwareSerial &port_;
  Print *log_;
  bool debug_{false};
  std::vector<uint8_t> rx_buffer_;
  uint8_t mos_status_{0xFF};
  float nominal_capacity_ah_{0.0f};
  bool nominal_known_{false};

  void log_hex_frame_(const char *prefix, const uint8_t *data, size_t len);
  void send_action_(uint8_t action, uint8_t function, uint8_t data_len, const uint8_t *data);
  void send_read_(uint8_t function);
  bool parse_byte_(uint8_t byte, Frame &frame);
  static uint16_t get16_(const std::vector<uint8_t> &data, size_t index);
  static uint8_t mos_action_for_states_(bool charge_on, bool discharge_on);
};

} // namespace jbd
