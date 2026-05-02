/**
 * @file roboguard_micro_ros.cpp
 * @brief Implementation of Nanopb communication interface for RoboGuard
 * @author Philippe Desbiens & Benoit Malenfant
 * @date June 5, 2025
 */

#include <Arduino.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <string.h>

#include "roboguard_micro_ros.h"
#include "sensor_data.h"

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

static bool pb_write_key(pb_ostream_t *stream, uint32_t field_number, pb_wire_type_t wire_type) {
    return pb_encode_tag(stream, wire_type, field_number);
}

static bool pb_write_float(pb_ostream_t *stream, uint32_t field_number, float value) {
    uint32_t packed = 0;
    memcpy(&packed, &value, sizeof(packed));
    return pb_write_key(stream, field_number, PB_WT_32BIT) && pb_encode_fixed32(stream, &packed);
}

static bool pb_write_bool(pb_ostream_t *stream, uint32_t field_number, bool value) {
    return pb_write_key(stream, field_number, PB_WT_VARINT) && pb_encode_varint(stream, value ? 1U : 0U);
}

static bool pb_write_uint(pb_ostream_t *stream, uint32_t field_number, uint32_t value) {
    return pb_write_key(stream, field_number, PB_WT_VARINT) && pb_encode_varint(stream, value);
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

static bool encode_telemetry_payload(uint8_t *buffer, size_t buffer_size, size_t *out_size) {
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    const float unknown_value = nan("1");

    if (!pb_write_float(&stream, FIELD_BATTERY_CAPACITY, sensor_data.battery_capacity)) {
        return false;
    }
    if (!pb_write_float(&stream, FIELD_BATTERY_DESIGN_CAPACITY, BATTERY_CAPACITY)) {
        return false;
    }
    if (!pb_write_float(&stream, FIELD_BATTERY_CHARGE, unknown_value)) {
        return false;
    }
    if (!pb_write_uint(&stream, FIELD_BATTERY_POWER_SUPPLY_TECHNOLOGY, 3U)) {
        return false;
    }
    if (!pb_write_bool(&stream, FIELD_BATTERY_PRESENT, sensor_data.present)) {
        return false;
    }
    if (!pb_write_uint(&stream, FIELD_BATTERY_POWER_SUPPLY_HEALTH, 0U)) {
        return false;
    }
    for (uint8_t i = 0; i < N_BATTERY_CELLS; i++) {
        if (!pb_write_float(&stream, FIELD_BATTERY_CELL_VOLTAGE, sensor_data.battery_cell_voltage[i])) {
            return false;
        }
    }
    for (uint8_t i = 0; i < N_THERMISTORS; i++) {
        if (!pb_write_float(&stream, FIELD_BATTERY_CELL_TEMPERATURE, sensor_data.battery_temp[i])) {
            return false;
        }
    }
    if (!pb_write_float(&stream, FIELD_BATTERY_PERCENTAGE, unknown_value)) {
        return false;
    }
    if (!pb_write_float(&stream, FIELD_BATTERY_VOLTAGE, sensor_data.battery_voltage)) {
        return false;
    }
    if (!pb_write_float(&stream, FIELD_BATTERY_CURRENT, sensor_data.battery_current)) {
        return false;
    }
    if (!pb_write_float(&stream, FIELD_BATTERY_TEMPERATURE, sensor_data.bms_temp)) {
        return false;
    }
    if (!pb_write_bool(&stream, FIELD_ESTOP_BT, false)) {
        return false;
    }
    if (!pb_write_bool(&stream, FIELD_ESTOP_STM32, sensor_data.estop_status_stm32)) {
        return false;
    }
    if (!pb_write_float(&stream, FIELD_AMBIANT_TEMP, sensor_data.ambiant_temp)) {
        return false;
    }
    if (!pb_write_float(&stream, FIELD_HUMIDITY, sensor_data.humidity)) {
        return false;
    }

    *out_size = stream.bytes_written;
    return true;
}

/**
 * @brief Decode incoming estop command payload
 *
 * Expects protobuf payload with boolean field #1.
 */
static bool decode_estop_command(const uint8_t *payload, size_t payload_size, bool *estop_pwr_out) {
    pb_istream_t stream = pb_istream_from_buffer(payload, payload_size);
    bool eof = false;
    bool has_estop_field = false;
    pb_wire_type_t wire_type = PB_WT_VARINT;
    uint32_t tag = 0;

    while (pb_decode_tag(&stream, &wire_type, &tag, &eof)) {
        if (tag == CMD_FIELD_ESTOP_POWER_OUT && wire_type == PB_WT_VARINT) {
            uint64_t value = 0;
            if (!pb_decode_varint(&stream, &value)) {
                return false;
            }
            *estop_pwr_out = value ? true : false;
            has_estop_field = true;
        } else if (!pb_skip_field(&stream, wire_type)) {
            return false;
        }
    }

    return eof && has_estop_field;
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
            if (rx_checksum == byte_in && rx_type == FRAME_TYPE_ESTOP_COMMAND) {
                handle_estop_command(rx_payload, rx_length);
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

static bool publish_telemetry() {
    uint8_t payload[FRAME_MAX_PAYLOAD];
    size_t payload_size = 0;
    if (!encode_telemetry_payload(payload, sizeof(payload), &payload_size)) {
        return false;
    }
    return send_frame(FRAME_TYPE_TELEMETRY, payload, (uint16_t)payload_size);
}

int setup_micro_ros(){
    // Configure serial transport
    Serial3.setRx(PC11);
    Serial3.setTx(PC10);
    Serial3.begin(SERIAL_BAUDRATE);

    parser_state = PARSER_WAIT_SYNC_A;
    rx_type = 0;
    rx_length = 0;
    rx_offset = 0;
    rx_checksum = 0;
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

    if ((uint32_t)(millis() - last_publish_ms) >= TIMER_TIMEOUT_MS) {
        if (!publish_telemetry()) {
            alive = 0;
            return 0;
        }
        last_publish_ms = millis();
    }

    return alive;
}
