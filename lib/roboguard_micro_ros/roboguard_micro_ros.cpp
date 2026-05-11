/**
 * @file roboguard_micro_ros.cpp
 * @brief Implementation of Nanopb communication interface for RoboGuard
 * @author Philippe Desbiens & Benoit Malenfant
 * @date June 5, 2025
 */

#include <Arduino.h>
#include <Wire.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <math.h>
#include <string.h>

#include "roboguard_micro_ros.h"
#include "sensor_data.h"
#include "pdu_i2c_api.h"
#include "roboguard.pb.h"

/** @defgroup BatteryConstants Battery Configuration Constants
 *  @brief Constants for battery monitoring and reporting
 *  @{
 */
#define BATTERY_CAPACITY 6.5        /**< Battery capacity in Ah */
/** @} */

/** @defgroup TimingConstants Communication Timing Constants
 *  @brief Timing configuration for Nanopb communication
 *  @{
 */
#define TIMER_TIMEOUT_MS 66         /**< Publisher timer timeout in milliseconds */
#define SERIAL_BAUDRATE 115200      /**< Nanopb serial link baudrate */
/** @} */

/** @defgroup ProtobufWireTypes Nanopb Protobuf Wire Types
 *  @brief Wire type constants used for manual protobuf encoding
 *  @{
 */
#define PB_WT_VARINT ((pb_wire_type_t)0)
#define PB_WT_64BIT ((pb_wire_type_t)1)
#define PB_WT_STRING ((pb_wire_type_t)2)
#define PB_WT_32BIT ((pb_wire_type_t)5)
/** @} */

/** @defgroup FrameProtocol Framed Serial Protocol
 *  @brief Simple framing constants for protobuf payload transport
 *  @{
 */
#define FRAME_SYNC_A 0xAA
#define FRAME_SYNC_B 0x55
#define FRAME_MAX_PAYLOAD 220
#define FRAME_TYPE_TELEMETRY 0x01
#define FRAME_TYPE_ESTOP_COMMAND 0x02
#define FRAME_TYPE_PDU_INFO 0x03
#define FRAME_TYPE_PDU_COMMAND 0x04
#define FRAME_TYPE_PDU_COMMAND_RESULT 0x05
/** @} */

/** @defgroup MessageFields Telemetry Protobuf Field IDs
 *  @brief Field numbers for telemetry payload (stable on-wire schema)
 *  @{
 */
#define FIELD_BATTERY_CAPACITY 1
#define FIELD_BATTERY_DESIGN_CAPACITY 2
#define FIELD_BATTERY_CHARGE 3
#define FIELD_BATTERY_POWER_SUPPLY_TECHNOLOGY 4
#define FIELD_BATTERY_PRESENT 5
#define FIELD_BATTERY_POWER_SUPPLY_HEALTH 6
#define FIELD_BATTERY_CELL_VOLTAGE 7
#define FIELD_BATTERY_CELL_TEMPERATURE 8
#define FIELD_BATTERY_PERCENTAGE 9
#define FIELD_BATTERY_VOLTAGE 10
#define FIELD_BATTERY_CURRENT 11
#define FIELD_BATTERY_TEMPERATURE 12
#define FIELD_ESTOP_BT 13
#define FIELD_ESTOP_STM32 14
#define FIELD_AMBIANT_TEMP 15
#define FIELD_HUMIDITY 16
/** @} */

/** @defgroup CommandFields Command Protobuf Field IDs
 *  @brief Field numbers for incoming command payload
 *  @{
 */
#define CMD_FIELD_ESTOP_POWER_OUT 1
/** @} */

/** @defgroup GlobalVariables Global State Variables
 *  @brief Global variables for system state tracking
 *  @{
 */
const int estop_pin = PA12;         /**< Emergency stop output pin */
int alive = 0;                      /**< Communication status flag */
HardwareSerial Serial3(USART3);     /**< Serial interface for Nanopb transport */
static uint32_t last_publish_ms = 0;
static bool pdu_info_published = false;
static roboguard::pdu::Client pdu_client(Wire);
/** @} */

/**
 * @brief Frame parser state for incoming serial data
 */
typedef enum {
    PARSER_WAIT_SYNC_A = 0,
    PARSER_WAIT_SYNC_B,
    PARSER_READ_TYPE,
    PARSER_READ_LEN_L,
    PARSER_READ_LEN_H,
    PARSER_READ_PAYLOAD,
    PARSER_READ_CHECKSUM
} parser_state_t;

static parser_state_t parser_state = PARSER_WAIT_SYNC_A;
static uint8_t rx_type = 0;
static uint16_t rx_length = 0;
static uint16_t rx_offset = 0;
static uint8_t rx_checksum = 0;
static uint8_t rx_payload[FRAME_MAX_PAYLOAD];

struct FloatArrayEncodeContext {
    const float *values;
    size_t count;
};

struct ByteArrayEncodeContext {
    const uint8_t *values;
    size_t count;
};

struct PduInfoEncodeContext {
    const uint8_t *magic;
    size_t magic_size;
};

static bool send_frame(uint8_t type, const uint8_t *payload, uint16_t payload_length);

static bool send_pb_frame(uint8_t type, const uint8_t *payload, uint16_t payload_length) {
    return send_frame(type, payload, payload_length);
}

static uint8_t compute_checksum(uint8_t type, uint16_t length, const uint8_t *payload) {
    uint8_t checksum = type;
    checksum ^= (uint8_t)(length & 0xFF);
    checksum ^= (uint8_t)((length >> 8) & 0xFF);
    for (uint16_t i = 0; i < length; i++) {
        checksum ^= payload[i];
    }
    return checksum;
}

static bool encode_float_array(pb_ostream_t *stream, const pb_field_iter_t *field, void *const *arg) {
    const FloatArrayEncodeContext *ctx = static_cast<const FloatArrayEncodeContext *>(*arg);
    for (size_t i = 0; i < ctx->count; ++i) {
        uint32_t packed = 0;
        memcpy(&packed, &ctx->values[i], sizeof(packed));
        if (!pb_encode_tag_for_field(stream, field)) {
            return false;
        }
        if (!pb_encode_fixed32(stream, &packed)) {
            return false;
        }
    }
    return true;
}

static bool encode_bytes_field(pb_ostream_t *stream, const pb_field_iter_t *field, void *const *arg) {
    const ByteArrayEncodeContext *ctx = static_cast<const ByteArrayEncodeContext *>(*arg);
    if (!pb_encode_tag_for_field(stream, field)) {
        return false;
    }
    return pb_encode_string(stream, ctx->values, ctx->count);
}

static bool encode_pdu_info_magic(pb_ostream_t *stream, const pb_field_iter_t *field, void *const *arg) {
    const PduInfoEncodeContext *ctx = static_cast<const PduInfoEncodeContext *>(*arg);
    if (!pb_encode_tag_for_field(stream, field)) {
        return false;
    }
    return pb_encode_string(stream, ctx->magic, ctx->magic_size);
}

static bool encode_telemetry_payload(uint8_t *buffer, size_t buffer_size, size_t *out_size) {
    roboguard_Telemetry message = {};
    FloatArrayEncodeContext cell_voltage_context{sensor_data.battery_cell_voltage, N_BATTERY_CELLS};
    FloatArrayEncodeContext cell_temp_context{sensor_data.battery_temp, N_THERMISTORS};

    message.battery_capacity = sensor_data.battery_capacity;
    message.battery_design_capacity = BATTERY_CAPACITY;
    message.battery_charge = nanf("1");
    message.battery_power_supply_technology = 3U;
    message.battery_present = sensor_data.present;
    message.battery_power_supply_health = 0U;
    message.battery_cell_voltage.funcs.encode = encode_float_array;
    message.battery_cell_voltage.arg = &cell_voltage_context;
    message.battery_cell_temperature.funcs.encode = encode_float_array;
    message.battery_cell_temperature.arg = &cell_temp_context;
    message.battery_percentage = nanf("1");
    message.battery_voltage = sensor_data.battery_voltage;
    message.battery_current = sensor_data.battery_current;
    message.battery_temperature = sensor_data.bms_temp;
    message.estop_bt = false;
    message.estop_stm32 = sensor_data.estop_status_stm32;
    message.ambiant_temp = sensor_data.ambiant_temp;
    message.humidity = sensor_data.humidity;

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    if (!pb_encode(&stream, roboguard_Telemetry_fields, &message)) {
        return false;
    }

    *out_size = stream.bytes_written;
    return true;
}

static bool decode_estop_command(const uint8_t *payload, size_t payload_size, bool *estop_pwr_out) {
    roboguard_EstopCommand message = {};
    pb_istream_t stream = pb_istream_from_buffer(payload, payload_size);
    if (!pb_decode(&stream, roboguard_EstopCommand_fields, &message)) {
        return false;
    }
    *estop_pwr_out = message.estop_power_out;
    return true;
}

static bool encode_pdu_info_payload(const roboguard::pdu::ApiInfo &info, uint8_t *buffer, size_t buffer_size,
                                    size_t *out_size) {
    roboguard_PduInfo message = {};
    PduInfoEncodeContext magic_context{reinterpret_cast<const uint8_t *>(info.magic), sizeof(info.magic)};

    message.magic.funcs.encode = encode_pdu_info_magic;
    message.magic.arg = &magic_context;
    message.protocol_major = info.protocol_major;
    message.protocol_minor = info.protocol_minor;
    message.fw_major = info.fw_major;
    message.fw_minor = info.fw_minor;
    message.fw_patch = info.fw_patch;
    message.i2c_addr = info.i2c_addr;
    message.rail_count = info.rail_count;

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    if (!pb_encode(&stream, roboguard_PduInfo_fields, &message)) {
        return false;
    }

    *out_size = stream.bytes_written;
    return true;
}

static bool encode_pdu_command_result(const roboguard::pdu::ApiCommandResult &result, uint8_t *buffer,
                                      size_t buffer_size, size_t *out_size) {
    roboguard_PduCommandResult message = {};
    message.sequence = result.sequence;
    message.busy = result.busy != 0;
    message.status = static_cast<roboguard_PduStatusCode>(result.status);
    message.command = static_cast<roboguard_PduCommandCode>(result.command);
    message.arg0 = result.arg0;
    message.arg1 = result.arg1;

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    if (!pb_encode(&stream, roboguard_PduCommandResult_fields, &message)) {
        return false;
    }

    *out_size = stream.bytes_written;
    return true;
}

static bool decode_pdu_command(const uint8_t *payload, size_t payload_size, roboguard_PduCommandFrame *command) {
    pb_istream_t stream = pb_istream_from_buffer(payload, payload_size);
    return pb_decode(&stream, roboguard_PduCommandFrame_fields, command);
}

static void handle_estop_command(const uint8_t *payload, size_t payload_size) {
    bool request_estop_power = false;
    if (!decode_estop_command(payload, payload_size, &request_estop_power)) {
        return;
    }

    sensor_data.estop_pwr_out = request_estop_power ? 1 : 0;
    if (!request_estop_power) {
        digitalWrite(estop_pin, LOW);
    }
}

static bool publish_telemetry() {
    uint8_t payload[FRAME_MAX_PAYLOAD];
    size_t payload_size = 0;
    if (!encode_telemetry_payload(payload, sizeof(payload), &payload_size)) {
        return false;
    }
    return send_pb_frame(FRAME_TYPE_TELEMETRY, payload, (uint16_t)payload_size);
}

static bool publish_pdu_info_once() {
    if (pdu_info_published) {
        return true;
    }

    roboguard::pdu::ApiInfo info = {};
    if (!pdu_client.readInfo(info)) {
        return false;
    }

    uint8_t payload[FRAME_MAX_PAYLOAD];
    size_t payload_size = 0;
    if (!encode_pdu_info_payload(info, payload, sizeof(payload), &payload_size)) {
        return false;
    }

    if (!send_pb_frame(FRAME_TYPE_PDU_INFO, payload, (uint16_t)payload_size)) {
        return false;
    }

    pdu_info_published = true;
    return true;
}

static bool dispatch_pdu_command(const roboguard_PduCommandFrame &command) {
    using namespace roboguard::pdu;

    bool ok = false;
    switch (command.command) {
        case roboguard_PduCommandCode_PDU_CMD_NOOP:
            ok = pdu_client.noop();
            break;
        case roboguard_PduCommandCode_PDU_CMD_SET_RAIL_ENABLE:
            ok = pdu_client.setRailEnable((uint8_t)command.arg0, command.arg1 != 0);
            break;
        case roboguard_PduCommandCode_PDU_CMD_SET_LED_DUTY:
            ok = pdu_client.setLedDuty((uint8_t)command.arg0, (uint8_t)command.arg1);
            break;
        case roboguard_PduCommandCode_PDU_CMD_SET_ALL_LEDS:
            ok = pdu_client.setAllLeds((uint8_t)command.arg0);
            break;
        case roboguard_PduCommandCode_PDU_CMD_SET_LED_PATTERN:
            ok = pdu_client.setLedPattern((uint8_t)command.arg0);
            break;
        case roboguard_PduCommandCode_PDU_CMD_SET_ESTOP_LOCAL:
            ok = pdu_client.setEstopLocal(command.arg0 != 0);
            break;
        case roboguard_PduCommandCode_PDU_CMD_SET_ESTOP_VTX:
            ok = pdu_client.setEstopVtx(command.arg0 != 0);
            break;
        case roboguard_PduCommandCode_PDU_CMD_CLEAR_RAIL_LATCH:
            ok = pdu_client.clearRailLatch((uint8_t)command.arg0);
            break;
        case roboguard_PduCommandCode_PDU_CMD_CLEAR_FAULT_LOG:
            ok = pdu_client.clearFaultLog();
            break;
        case roboguard_PduCommandCode_PDU_CMD_RESET_DEVICE:
            ok = pdu_client.resetDevice();
            break;
        case roboguard_PduCommandCode_PDU_CMD_SET_UNIX_TIME:
            ok = pdu_client.setUnixTime((uint32_t)command.arg0 | ((uint32_t)command.arg1 << 8) |
                                        ((uint32_t)command.arg2 << 16) | ((uint32_t)command.arg3 << 24));
            break;
        case roboguard_PduCommandCode_PDU_CMD_REFRESH_HOTSWAP_BLACKBOX:
            ok = pdu_client.refreshHotswapBlackBox((uint8_t)command.arg0);
            break;
        case roboguard_PduCommandCode_PDU_CMD_ERASE_HOTSWAP_BLACKBOX:
            ok = pdu_client.eraseHotswapBlackBox((uint8_t)command.arg0);
            break;
        case roboguard_PduCommandCode_PDU_CMD_SET_WINCH_MODE:
            ok = pdu_client.setWinchMode((uint8_t)command.arg0);
            break;
        case roboguard_PduCommandCode_PDU_CMD_SET_WINCH_DC_MOTOR:
            ok = pdu_client.setWinchDcMotor((uint8_t)command.arg0, (int8_t)command.arg1);
            break;
        case roboguard_PduCommandCode_PDU_CMD_SET_WINCH_PARALLEL_DC:
            ok = pdu_client.setWinchParallelDc((int8_t)command.arg0);
            break;
        case roboguard_PduCommandCode_PDU_CMD_SET_WINCH_STEPPER_PHASES:
            ok = pdu_client.setWinchStepperPhases((int8_t)command.arg0, (int8_t)command.arg1);
            break;
        case roboguard_PduCommandCode_PDU_CMD_BRAKE_WINCH:
            ok = pdu_client.brakeWinch();
            break;
        case roboguard_PduCommandCode_PDU_CMD_CLEAR_WINCH_FAULT:
            ok = pdu_client.clearWinchFault();
            break;
        case roboguard_PduCommandCode_PDU_CMD_SET_WINCH_LOCK:
            ok = pdu_client.setWinchLock((uint8_t)command.arg0, command.arg1 != 0);
            break;
        default:
            ok = false;
            break;
    }

    if (!ok) {
        return false;
    }

    roboguard::pdu::ApiCommandResult result = {};
    if (pdu_client.waitForCommandResult(result)) {
        uint8_t payload[FRAME_MAX_PAYLOAD];
        size_t payload_size = 0;
        if (encode_pdu_command_result(result, payload, sizeof(payload), &payload_size)) {
            (void)send_pb_frame(FRAME_TYPE_PDU_COMMAND_RESULT, payload, (uint16_t)payload_size);
        }
    }

    return true;
}

static void process_rx_byte(uint8_t byte_in) {
    switch (parser_state) {
        case PARSER_WAIT_SYNC_A:
            parser_state = (byte_in == FRAME_SYNC_A) ? PARSER_WAIT_SYNC_B : PARSER_WAIT_SYNC_A;
            break;
        case PARSER_WAIT_SYNC_B:
            parser_state = (byte_in == FRAME_SYNC_B) ? PARSER_READ_TYPE : PARSER_WAIT_SYNC_A;
            break;
        case PARSER_READ_TYPE:
            rx_type = byte_in;
            rx_checksum = byte_in;
            parser_state = PARSER_READ_LEN_L;
            break;
        case PARSER_READ_LEN_L:
            rx_length = byte_in;
            rx_checksum ^= byte_in;
            parser_state = PARSER_READ_LEN_H;
            break;
        case PARSER_READ_LEN_H:
            rx_length |= ((uint16_t)byte_in << 8);
            rx_checksum ^= byte_in;
            if (rx_length > FRAME_MAX_PAYLOAD) {
                parser_state = PARSER_WAIT_SYNC_A;
                break;
            }
            rx_offset = 0;
            parser_state = (rx_length == 0) ? PARSER_READ_CHECKSUM : PARSER_READ_PAYLOAD;
            break;
        case PARSER_READ_PAYLOAD:
            rx_payload[rx_offset++] = byte_in;
            rx_checksum ^= byte_in;
            if (rx_offset >= rx_length) {
                parser_state = PARSER_READ_CHECKSUM;
            }
            break;
        case PARSER_READ_CHECKSUM:
            if (rx_checksum == byte_in) {
                if (rx_type == FRAME_TYPE_ESTOP_COMMAND) {
                    handle_estop_command(rx_payload, rx_length);
                } else if (rx_type == FRAME_TYPE_PDU_COMMAND) {
                    roboguard_PduCommandFrame command = {};
                    if (decode_pdu_command(rx_payload, rx_length, &command)) {
                        (void)dispatch_pdu_command(command);
                    }
                }
            }
            parser_state = PARSER_WAIT_SYNC_A;
            break;
        default:
            parser_state = PARSER_WAIT_SYNC_A;
            break;
    }
}

static void process_incoming_frames() {
    while (Serial3.available() > 0) {
        process_rx_byte((uint8_t)Serial3.read());
    }
}

static bool send_frame(uint8_t type, const uint8_t *payload, uint16_t payload_length) {
    if (payload_length > FRAME_MAX_PAYLOAD) {
        return false;
    }

    const uint8_t checksum = compute_checksum(type, payload_length, payload);
    const size_t expected_size = (size_t)payload_length + 6U;
    size_t written = 0;

    written += Serial3.write(FRAME_SYNC_A);
    written += Serial3.write(FRAME_SYNC_B);
    written += Serial3.write(type);
    written += Serial3.write((uint8_t)(payload_length & 0xFF));
    written += Serial3.write((uint8_t)((payload_length >> 8) & 0xFF));
    if (payload_length > 0) {
        written += Serial3.write(payload, payload_length);
    }
    written += Serial3.write(checksum);

    return written == expected_size;
}

int setup_micro_ros(){
    // Configure serial transport
    Serial3.setRx(PC11);
    Serial3.setTx(PC10);
    Serial3.begin(SERIAL_BAUDRATE);

    // Initialize PDU I2C client used by protobuf control/info messages.
    (void)pdu_client.begin();

    parser_state = PARSER_WAIT_SYNC_A;
    rx_type = 0;
    rx_length = 0;
    rx_offset = 0;
    rx_checksum = 0;
    pdu_info_published = false;
    last_publish_ms = millis();

    alive = 1;
    return alive;
}

int clean_micro_ros(){
    Serial3.end();
    alive = 0;
    return 1;
}

int update_micro_ros(){
    if (!alive) {
        return 0;
    }

    process_incoming_frames();

    (void)publish_pdu_info_once();

    if ((uint32_t)(millis() - last_publish_ms) >= TIMER_TIMEOUT_MS) {
        if (!publish_telemetry()) {
            alive = 0;
            return 0;
        }
        last_publish_ms = millis();
    }

    return alive;
}
