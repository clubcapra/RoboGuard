#include "pdu_i2c_api.h"

namespace roboguard {
namespace pdu {

Client::Client(TwoWire &wire, uint8_t address)
    : wire_(&wire),
      address_(address),
      use_software_i2c_(false),
      scl_pin_(0),
      sda_pin_(0),
      half_period_us_(5) {}

Client::Client(uint32_t scl_pin, uint32_t sda_pin, uint8_t address)
    : wire_(nullptr),
      address_(address),
      use_software_i2c_(true),
      scl_pin_(scl_pin),
      sda_pin_(sda_pin),
      half_period_us_(5) {}

void Client::setAddress(uint8_t address) { address_ = address; }

uint8_t Client::address() const { return address_; }

bool Client::begin(uint32_t clock_hz) {
  half_period_us_ = static_cast<uint16_t>(max<uint32_t>(2, 500000UL / max<uint32_t>(clock_hz, 1)));
  if (use_software_i2c_) {
    releaseSda();
    releaseScl();
    softDelay();
    return true;
  }
  wire_->begin();
  wire_->setClock(clock_hz);
  return true;
}

bool Client::selectRegister(uint8_t reg) { return hardwareSelectRegister(reg); }

bool Client::writeRegister(uint8_t reg, const void *payload, size_t length) {
  return use_software_i2c_ ? softwareWriteRegister(reg, payload, length)
                           : hardwareWriteRegister(reg, payload, length);
}

bool Client::readRegister(uint8_t reg, void *payload, size_t length) {
  return use_software_i2c_ ? softwareReadRegister(reg, payload, length)
                           : hardwareReadRegister(reg, payload, length);
}

bool Client::hardwareSelectRegister(uint8_t reg) {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  return wire_->endTransmission(true) == 0;
}

bool Client::hardwareWriteRegister(uint8_t reg, const void *payload, size_t length) {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  if (length != 0 && payload != nullptr) {
    wire_->write(static_cast<const uint8_t *>(payload), length);
  }
  return wire_->endTransmission(true) == 0;
}

bool Client::hardwareReadRegister(uint8_t reg, void *payload, size_t length) {
  if (payload == nullptr || length == 0) {
    return false;
  }

  if (!selectRegister(reg)) {
    return false;
  }

  const uint8_t received = static_cast<uint8_t>(wire_->requestFrom(static_cast<int>(address_),
                                                                  static_cast<int>(length)));
  if (received != length) {
    while (wire_->available() > 0) {
      (void)wire_->read();
    }
    return false;
  }

  uint8_t *out = static_cast<uint8_t *>(payload);
  for (size_t i = 0; i < length; ++i) {
    if (!wire_->available()) {
      return false;
    }
    out[i] = static_cast<uint8_t>(wire_->read());
  }

  return true;
}

void Client::releaseScl() {
  pinMode(scl_pin_, INPUT_PULLUP);
}

void Client::driveSclLow() {
  digitalWrite(scl_pin_, LOW);
  pinMode(scl_pin_, OUTPUT);
}

void Client::releaseSda() {
  pinMode(sda_pin_, INPUT_PULLUP);
}

void Client::driveSdaLow() {
  digitalWrite(sda_pin_, LOW);
  pinMode(sda_pin_, OUTPUT);
}

bool Client::readSda() const { return digitalRead(sda_pin_) != LOW; }

void Client::softDelay() const { delayMicroseconds(half_period_us_); }

bool Client::waitForSclHigh() const {
  const uint32_t start = micros();
  while (digitalRead(scl_pin_) == LOW) {
    if ((micros() - start) > 1000UL) {
      return false;
    }
  }
  return true;
}

void Client::softStart() {
  releaseSda();
  releaseScl();
  (void)waitForSclHigh();
  softDelay();
  driveSdaLow();
  softDelay();
  driveSclLow();
}

void Client::softStop() {
  driveSdaLow();
  softDelay();
  releaseScl();
  (void)waitForSclHigh();
  softDelay();
  releaseSda();
  softDelay();
}

bool Client::softWriteByte(uint8_t value) {
  for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
    if ((value & mask) != 0) {
      releaseSda();
    } else {
      driveSdaLow();
    }
    softDelay();
    releaseScl();
    if (!waitForSclHigh()) {
      driveSclLow();
      return false;
    }
    softDelay();
    driveSclLow();
  }

  releaseSda();
  softDelay();
  releaseScl();
  if (!waitForSclHigh()) {
    driveSclLow();
    return false;
  }
  softDelay();
  const bool ack = !readSda();
  driveSclLow();
  return ack;
}

uint8_t Client::softReadByte(bool ack) {
  uint8_t value = 0;
  releaseSda();
  for (uint8_t i = 0; i < 8; ++i) {
    value <<= 1;
    releaseScl();
    (void)waitForSclHigh();
    softDelay();
    if (readSda()) {
      value |= 1;
    }
    driveSclLow();
    softDelay();
  }

  if (ack) {
    driveSdaLow();
  } else {
    releaseSda();
  }
  softDelay();
  releaseScl();
  (void)waitForSclHigh();
  softDelay();
  driveSclLow();
  releaseSda();
  return value;
}

bool Client::softwareWriteRegister(uint8_t reg, const void *payload, size_t length) {
  softStart();
  bool ok = softWriteByte(static_cast<uint8_t>(address_ << 1)) && softWriteByte(reg);
  const uint8_t *bytes = static_cast<const uint8_t *>(payload);
  for (size_t i = 0; ok && i < length; ++i) {
    ok = softWriteByte(bytes[i]);
  }
  softStop();
  return ok;
}

bool Client::softwareReadRegister(uint8_t reg, void *payload, size_t length) {
  if (payload == nullptr || length == 0) {
    return false;
  }

  softStart();
  bool ok = softWriteByte(static_cast<uint8_t>(address_ << 1)) && softWriteByte(reg);
  softStop();
  if (!ok) {
    return false;
  }

  delayMicroseconds(100);
  softStart();
  ok = softWriteByte(static_cast<uint8_t>((address_ << 1) | 1U));
  if (!ok) {
    softStop();
    return false;
  }

  uint8_t *out = static_cast<uint8_t *>(payload);
  for (size_t i = 0; i < length; ++i) {
    out[i] = softReadByte(i + 1 < length);
  }
  softStop();
  return true;
}

bool Client::readInfo(ApiInfo &out) { return readRegister(kRegisterInfo, &out, sizeof(out)); }

bool Client::readTelemetry(ApiTelemetryAll &out) {
  return readRegister(kRegisterTelemetryAll, &out, sizeof(out));
}

bool Client::writeCommand(const ApiCommandFrame &frame) {
  return writeRegister(kRegisterCommand, &frame, sizeof(frame));
}

bool Client::readCommandStatus(ApiCommandResult &out) {
  return readRegister(kRegisterCommandStatus, &out, sizeof(out));
}

bool Client::writePmbusBridge(const ApiPmbusBridgeRequest &request) {
  return writeRegister(kRegisterPmbusBridge, &request, sizeof(request));
}

bool Client::readPmbusResult(ApiPmbusBridgeResult &out) {
  return readRegister(kRegisterPmbusResult, &out, sizeof(out));
}

bool Client::waitForCommandResult(ApiCommandResult &out, uint32_t timeout_ms,
                                  uint32_t poll_ms) {
  const uint32_t start = millis();
  do {
    if (!readCommandStatus(out)) {
      return false;
    }
    if (out.busy == 0) {
      return true;
    }
    delay(poll_ms);
  } while ((millis() - start) < timeout_ms);

  return false;
}

bool Client::waitForPmbusResult(ApiPmbusBridgeResult &out, uint32_t timeout_ms,
                                uint32_t poll_ms) {
  const uint32_t start = millis();
  do {
    if (!readPmbusResult(out)) {
      return false;
    }
    if (out.busy == 0) {
      return true;
    }
    delay(poll_ms);
  } while ((millis() - start) < timeout_ms);

  return false;
}

bool Client::sendCommand(uint8_t command, uint8_t arg0, uint8_t arg1, uint8_t arg2,
                         uint8_t arg3) {
  const ApiCommandFrame frame{command, arg0, arg1, arg2, arg3};
  return writeCommand(frame);
}

bool Client::noop() { return sendCommand(0x00, 0, 0, 0, 0); }

bool Client::setRailEnable(uint8_t rail_id, bool enable) {
  return sendCommand(0x01, rail_id, enable ? 1 : 0, 0, 0);
}

bool Client::setLedDuty(uint8_t channel, uint8_t duty_percent) {
  return sendCommand(0x02, channel, duty_percent, 0, 0);
}

bool Client::setAllLeds(uint8_t duty_percent) { return sendCommand(0x03, duty_percent, 0, 0, 0); }

bool Client::setLedPattern(uint8_t pattern) { return sendCommand(0x04, pattern, 0, 0, 0); }

bool Client::setEstopLocal(bool assert_estop) {
  return sendCommand(0x05, assert_estop ? 1 : 0, 0, 0, 0);
}

bool Client::setEstopVtx(bool enabled) { return sendCommand(0x06, enabled ? 1 : 0, 0, 0, 0); }

bool Client::clearRailLatch(uint8_t rail_id) { return sendCommand(0x07, rail_id, 0, 0, 0); }

bool Client::clearFaultLog() { return sendCommand(0x08, 0, 0, 0, 0); }

bool Client::resetDevice() { return sendCommand(0x09, 0, 0, 0, 0); }

bool Client::setUnixTime(uint32_t unix_time_s) {
  return sendCommand(0x0A, static_cast<uint8_t>(unix_time_s >> 0),
                     static_cast<uint8_t>(unix_time_s >> 8),
                     static_cast<uint8_t>(unix_time_s >> 16),
                     static_cast<uint8_t>(unix_time_s >> 24));
}

bool Client::refreshHotswapBlackBox(uint8_t rail_id) {
  return sendCommand(0x0B, rail_id, 0, 0, 0);
}

bool Client::eraseHotswapBlackBox(uint8_t rail_id) { return sendCommand(0x0C, rail_id, 0, 0, 0); }

bool Client::setWinchMode(uint8_t mode) { return sendCommand(0x30, mode, 0, 0, 0); }

bool Client::setWinchDcMotor(uint8_t motor_id, int8_t percent) {
  return sendCommand(0x31, motor_id, signedPercentToByte(percent), 0, 0);
}

bool Client::setWinchParallelDc(int8_t percent) {
  return sendCommand(0x32, signedPercentToByte(percent), 0, 0, 0);
}

bool Client::setWinchStepperPhases(int8_t phase_a_percent, int8_t phase_b_percent) {
  return sendCommand(0x33, signedPercentToByte(phase_a_percent),
                     signedPercentToByte(phase_b_percent), 0, 0);
}

bool Client::brakeWinch() { return sendCommand(0x34, 0, 0, 0, 0); }

bool Client::clearWinchFault() { return sendCommand(0x35, 0, 0, 0, 0); }

bool Client::setWinchLock(uint8_t lock_channel, bool enabled) {
  return sendCommand(0x36, lock_channel, enabled ? 1 : 0, 0, 0);
}

}  // namespace pdu
}  // namespace roboguard
