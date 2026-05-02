#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace roboguard {
namespace pdu {

constexpr uint8_t kPduI2cAddress = 0x31;
constexpr uint8_t kPduRailCount = 3;
constexpr uint8_t kPduFaultHistoryCapacity = 24;
constexpr size_t kPduCommandFrameSize = 5;
constexpr size_t kPduCommandResultSize = 8;
constexpr size_t kPduPmbusBridgeRequestSize = 28;
constexpr size_t kPduPmbusBridgeResultSize = 31;
constexpr size_t kPduInfoSize = 16;
constexpr size_t kPduTelemetryAllSize = 944;

enum Register : uint8_t {
  kRegisterInfo = 0x00,
  kRegisterTelemetryAll = 0x10,
  kRegisterCommand = 0x20,
  kRegisterCommandStatus = 0x21,
  kRegisterPmbusBridge = 0x40,
  kRegisterPmbusResult = 0x41,
};

enum StatusCode : uint8_t {
  kStatusOk = 0,
  kStatusBusError = 1,
  kStatusTimeout = 2,
  kStatusParam = 3,
  kStatusNotPresent = 4,
  kStatusRange = 5,
  kStatusPecMismatch = 6,
  kStatusFault = 7,
  kStatusNotInit = 8,
  kStatusInternal = 9,
};

enum RailId : uint8_t {
  kRail48V = 0,
  kRail24V = 1,
  kRail12V = 2,
  kRailAll = 3,
};

enum LedChannel : uint8_t {
  kLedBras = 0,
  kLedAvant = 1,
  kLedArriere = 2,
  kLedExtra = 3,
};

enum LedPattern : uint8_t {
  kLedPatternOff = 0,
  kLedPatternSolid = 1,
  kLedPatternHeartbeat = 2,
  kLedPatternFaultBlink = 3,
  kLedPatternEstopStrobe = 4,
};

enum WinchMode : uint8_t {
  kWinchSleep = 0,
  kWinchDualDc = 1,
  kWinchStepper = 2,
  kWinchParallelDc = 3,
};

enum WinchMotor : uint8_t {
  kWinchMotorA = 0,
  kWinchMotorB = 1,
};

enum WinchLockChannel : uint8_t {
  kWinchLock1 = 0,
  kWinchLock2 = 1,
  kWinchLockAll = 2,
};

enum SupervisorMode : uint8_t {
  kSupervisorBoot = 0,
  kSupervisorPbit = 1,
  kSupervisorNominal = 2,
  kSupervisorDegraded = 3,
  kSupervisorFault = 4,
  kSupervisorEstop = 5,
};

enum RailState : uint8_t {
  kRailStateBoot = 0,
  kRailStateAbsent = 1,
  kRailStateReady = 2,
  kRailStateRunning = 3,
  kRailStateWarning = 4,
  kRailStateTripped = 5,
  kRailStateLatched = 6,
};

#pragma pack(push, 1)

struct ApiInfo {
  char magic[4];
  uint8_t protocol_major;
  uint8_t protocol_minor;
  uint8_t fw_major;
  uint8_t fw_minor;
  uint8_t fw_patch;
  uint8_t i2c_addr;
  uint8_t rail_count;
  uint8_t reserved[5];
};

struct ApiCommandFrame {
  uint8_t command;
  uint8_t arg0;
  uint8_t arg1;
  uint8_t arg2;
  uint8_t arg3;
};

struct ApiCommandResult {
  uint8_t sequence;
  uint8_t busy;
  uint8_t status;
  uint8_t command;
  uint8_t arg0;
  uint8_t arg1;
  uint8_t reserved[2];
};

struct ApiPmbusBridgeRequest {
  uint8_t rail_id;
  uint8_t op;
  uint8_t command;
  uint8_t length;
  uint8_t data[24];
};

struct ApiPmbusBridgeResult {
  uint8_t sequence;
  uint8_t busy;
  uint8_t status;
  uint8_t rail_id;
  uint8_t op;
  uint8_t command;
  uint8_t length;
  uint8_t data[24];
};

struct ApiWinchTelemetry {
  uint8_t mode;
  uint8_t awake;
  uint8_t fault_active;
  uint8_t lock1_on;
  uint8_t lock2_on;
  uint8_t reserved[3];
  int8_t motor_a_cmd_pct;
  int8_t motor_b_cmd_pct;
  int8_t parallel_cmd_pct;
  int8_t stepper_a_cmd_pct;
  int8_t stepper_b_cmd_pct;
};

struct ApiRailTelemetry {
  uint8_t rail_id;
  uint8_t state;
  uint8_t present;
  uint8_t output_on;
  uint8_t pgood;
  uint8_t reserved0;
  uint16_t status_word;
  uint16_t diag_word;
  uint16_t status_mfr_specific2;
  uint16_t fault_count;
  uint8_t wd_plb_timer;
  uint8_t rail_reserved[3];
  uint32_t vin_mV;
  uint32_t vout_mV;
  uint32_t vaux_mV;
  uint32_t iin_mA;
  uint32_t pin_dW;
  uint8_t peak_valid;
  uint8_t peak_reserved[3];
  uint32_t peak_vin_mV;
  uint32_t peak_iin_mA;
  uint32_t peak_pin_dW;
  int16_t die_temp_centiC;
  int16_t ntc_temp_centiC;
  uint8_t bb_valid;
  uint8_t bb_config;
  uint8_t bb_timer;
  uint8_t bb_ram_len;
  uint8_t bb_eeprom_len;
  uint8_t bb_ram_event;
  uint8_t bb_ram_timer_expired;
  uint8_t bb_ram_tick;
  uint8_t bb_eeprom_event;
  uint8_t bb_eeprom_timer_expired;
  uint8_t bb_eeprom_tick;
  uint8_t bb_reserved[2];
  uint8_t bb_ram[16];
  uint8_t bb_eeprom[16];
};

struct ApiFaultRecord {
  uint8_t valid;
  uint8_t code;
  uint8_t rail;
  uint8_t reserved;
  uint16_t sequence;
  uint16_t status_word;
  uint16_t diag_word;
  uint16_t reset_count;
  uint32_t uptime_ms;
  uint32_t unix_time_s;
  uint32_t reset_flags;
};

struct ApiTelemetryAll {
  char magic[4];
  uint8_t protocol_major;
  uint8_t protocol_minor;
  uint8_t mode;
  uint8_t estop_active;
  uint8_t pbit_passed;
  uint8_t cbit_passed;
  uint16_t pbit_failed;
  uint16_t cbit_failed;
  uint32_t uptime_ms;
  uint8_t last_fault_valid;
  uint8_t last_fault_code;
  uint8_t last_fault_rail;
  uint8_t reserved0;
  uint16_t last_fault_sequence;
  uint16_t last_fault_status_word;
  uint16_t last_fault_diag_word;
  uint16_t reset_count;
  uint32_t last_fault_uptime_ms;
  uint32_t last_fault_unix_time_s;
  uint32_t reset_flags;
  uint8_t fault_history_count;
  uint8_t fault_history_capacity;
  uint16_t fault_history_dropped;
  ApiWinchTelemetry winch;
  ApiRailTelemetry rails[kPduRailCount];
  ApiFaultRecord fault_history[kPduFaultHistoryCapacity];
};

#pragma pack(pop)

static_assert(sizeof(ApiInfo) == kPduInfoSize, "ApiInfo size mismatch");
static_assert(sizeof(ApiCommandFrame) == kPduCommandFrameSize, "ApiCommandFrame size mismatch");
static_assert(sizeof(ApiCommandResult) == kPduCommandResultSize, "ApiCommandResult size mismatch");
static_assert(sizeof(ApiPmbusBridgeRequest) == kPduPmbusBridgeRequestSize,
              "ApiPmbusBridgeRequest size mismatch");
static_assert(sizeof(ApiPmbusBridgeResult) == kPduPmbusBridgeResultSize,
              "ApiPmbusBridgeResult size mismatch");
static_assert(sizeof(ApiWinchTelemetry) == 13, "ApiWinchTelemetry size mismatch");
static_assert(sizeof(ApiRailTelemetry) == 103, "ApiRailTelemetry size mismatch");
static_assert(sizeof(ApiFaultRecord) == 24, "ApiFaultRecord size mismatch");
static_assert(sizeof(ApiTelemetryAll) == kPduTelemetryAllSize, "ApiTelemetryAll size mismatch");

inline uint8_t signedPercentToByte(int8_t value) {
  return static_cast<uint8_t>(value);
}

class Client {
 public:
  explicit Client(TwoWire &wire = Wire, uint8_t address = kPduI2cAddress);

  void setAddress(uint8_t address);
  uint8_t address() const;

  bool begin(uint32_t clock_hz = 100000);

  bool writeRegister(uint8_t reg, const void *payload, size_t length);
  bool readRegister(uint8_t reg, void *payload, size_t length);

  bool readInfo(ApiInfo &out);
  bool readTelemetry(ApiTelemetryAll &out);
  bool writeCommand(const ApiCommandFrame &frame);
  bool readCommandStatus(ApiCommandResult &out);
  bool writePmbusBridge(const ApiPmbusBridgeRequest &request);
  bool readPmbusResult(ApiPmbusBridgeResult &out);

  bool waitForCommandResult(ApiCommandResult &out, uint32_t timeout_ms = 250,
                            uint32_t poll_ms = 5);
  bool waitForPmbusResult(ApiPmbusBridgeResult &out, uint32_t timeout_ms = 250,
                          uint32_t poll_ms = 5);

  bool sendCommand(uint8_t command, uint8_t arg0 = 0, uint8_t arg1 = 0, uint8_t arg2 = 0,
                   uint8_t arg3 = 0);
  bool noop();
  bool setRailEnable(uint8_t rail_id, bool enable);
  bool setLedDuty(uint8_t channel, uint8_t duty_percent);
  bool setAllLeds(uint8_t duty_percent);
  bool setLedPattern(uint8_t pattern);
  bool setEstopLocal(bool assert_estop);
  bool setEstopVtx(bool enabled);
  bool clearRailLatch(uint8_t rail_id);
  bool clearFaultLog();
  bool resetDevice();
  bool setUnixTime(uint32_t unix_time_s);
  bool refreshHotswapBlackBox(uint8_t rail_id);
  bool eraseHotswapBlackBox(uint8_t rail_id);
  bool setWinchMode(uint8_t mode);
  bool setWinchDcMotor(uint8_t motor_id, int8_t percent);
  bool setWinchParallelDc(int8_t percent);
  bool setWinchStepperPhases(int8_t phase_a_percent, int8_t phase_b_percent);
  bool brakeWinch();
  bool clearWinchFault();
  bool setWinchLock(uint8_t lock_channel, bool enabled);

 private:
  TwoWire &wire_;
  uint8_t address_;

  bool selectRegister(uint8_t reg);
};

}  // namespace pdu
}  // namespace roboguard
