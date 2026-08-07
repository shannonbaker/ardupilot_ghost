/*
   Native GHOST extension for MSP DisplayPort.

   The wire format is shared with Betaflight and INAV.  ArduPilot keeps its
   native mission store authoritative and exposes it as MAVLink
   MISSION_ITEM_INT records.
 */

#include "AP_MSP_config.h"

#if AP_MSP_GHOST_DP_ENABLED

#include "AP_MSP_Telem_Backend.h"
#include "msp_protocol.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Mission/AP_Mission.h>

using namespace MSP;

extern const AP_HAL::HAL &hal;

namespace {

constexpr uint8_t GHOST_SUBCOMMAND = 0x80;
constexpr uint8_t GHOST_VERSION = 0x10;
constexpr uint8_t ENDPOINT_FC = 1;
constexpr uint8_t FLAG_RESPONSE = 1U << 1;
constexpr uint8_t FLAG_ERROR = 1U << 3;

enum Message : uint8_t {
    HELLO_REQUEST = 0x01,
    HELLO_RESPONSE = 0x02,
    CATALOG_REQUEST = 0x03,
    CATALOG_RESPONSE = 0x04,
    MISSION_INFO_REQUEST = 0x30,
    MISSION_INFO_RESPONSE = 0x31,
    MISSION_ITEM_REQUEST = 0x32,
    MISSION_ITEM_RESPONSE = 0x33,
};

enum Status : uint8_t {
    OK = 0,
    BAD_LENGTH = 1,
    UNSUPPORTED_VERSION = 2,
    UNSUPPORTED_MESSAGE = 3,
    INVALID_SESSION = 4,
    INVALID_MISSION = 22,
};

enum ValueType : uint8_t {
    VALUE_U8 = 1,
    VALUE_U16 = 3,
    VALUE_I16 = 4,
    VALUE_U32 = 5,
    VALUE_I32 = 6,
    VALUE_BOOL = 11,
};

enum Unit : uint8_t {
    UNIT_NONE = 0,
    UNIT_DEGREE = 1,
    UNIT_METRE = 2,
    UNIT_METRES_PER_SECOND = 3,
    UNIT_VOLT = 4,
    UNIT_AMPERE = 5,
    UNIT_AMPERE_HOUR = 6,
    UNIT_PERCENT = 8,
    UNIT_COUNT = 10,
    UNIT_DEGREES_PER_SECOND = 11,
    UNIT_WATT_HOUR = 12,
};

constexpr uint16_t FIELD_INVALID = 1U << 2;
constexpr uint16_t FIELD_SIGNED = 1U << 3;

struct Field {
    uint16_t id;
    uint8_t type;
    uint8_t unit;
    int8_t exponent;
    uint16_t flags;
    uint16_t maximum_rate;
    uint16_t native_rate;
    const char *name;
};

#define RC_FIELD(n, id) { id, VALUE_U16, UNIT_NONE, 0, FIELD_INVALID, 50, 50, "RC" #n }
const Field fields[] = {
    { 1, VALUE_I16, UNIT_DEGREE, -1, FIELD_SIGNED, 100, 100, "PITCH" },
    { 2, VALUE_I16, UNIT_DEGREE, -1, FIELD_SIGNED, 100, 100, "ROLL" },
    { 3, VALUE_U16, UNIT_DEGREE, -1, 0, 100, 100, "HEADING" },
    { 4, VALUE_I32, UNIT_DEGREE, -7, FIELD_SIGNED | FIELD_INVALID, 10, 10, "LATITUDE" },
    { 5, VALUE_I32, UNIT_DEGREE, -7, FIELD_SIGNED | FIELD_INVALID, 10, 10, "LONGITUDE" },
    { 6, VALUE_I32, UNIT_METRE, -2, FIELD_SIGNED | FIELD_INVALID, 10, 10, "GPS_ALTITUDE" },
    { 7, VALUE_U16, UNIT_METRES_PER_SECOND, -2, FIELD_INVALID, 10, 10, "GROUND_SPEED" },
    { 8, VALUE_U16, UNIT_VOLT, -3, FIELD_INVALID, 20, 10, "BATTERY_VOLTAGE" },
    { 9, VALUE_I32, UNIT_AMPERE, -2, FIELD_SIGNED | FIELD_INVALID, 20, 10, "BATTERY_CURRENT" },
    { 10, VALUE_I32, UNIT_AMPERE_HOUR, -3, FIELD_SIGNED | FIELD_INVALID, 20, 10, "BATTERY_MAH" },
    { 11, VALUE_U8, UNIT_COUNT, 0, FIELD_INVALID, 10, 10, "GPS_SATELLITES" },
    { 12, VALUE_U16, UNIT_DEGREE, -1, FIELD_INVALID, 10, 10, "HOME_BEARING" },
    { 13, VALUE_BOOL, UNIT_NONE, 0, 0, 10, 10, "HEADING_VALID" },
    { 14, VALUE_BOOL, UNIT_NONE, 0, 0, 10, 10, "GPS_FIX" },
    { 15, VALUE_BOOL, UNIT_NONE, 0, 0, 10, 10, "HOME_VALID" },
    { 16, VALUE_I16, UNIT_DEGREES_PER_SECOND, -1, FIELD_SIGNED, 100, 100, "ANGULAR_RATE_ROLL" },
    { 17, VALUE_I16, UNIT_DEGREES_PER_SECOND, -1, FIELD_SIGNED, 100, 100, "ANGULAR_RATE_PITCH" },
    { 18, VALUE_I16, UNIT_DEGREES_PER_SECOND, -1, FIELD_SIGNED, 100, 100, "ANGULAR_RATE_YAW" },
    { 22, VALUE_U16, UNIT_VOLT, -2, FIELD_INVALID, 20, 10, "BATTERY_CELL_VOLTAGE" },
    { 23, VALUE_U8, UNIT_COUNT, 0, FIELD_INVALID, 10, 1, "BATTERY_CELL_COUNT" },
    { 24, VALUE_U32, UNIT_WATT_HOUR, -3, FIELD_INVALID, 20, 10, "BATTERY_WH" },
    { 25, VALUE_U8, UNIT_PERCENT, 0, FIELD_INVALID, 10, 10, "BATTERY_REMAINING_PERCENT" },
    { 26, VALUE_U8, UNIT_NONE, 0, 0, 10, 10, "BATTERY_STATE" },
    { 27, VALUE_I32, UNIT_DEGREE, -7, FIELD_SIGNED | FIELD_INVALID, 10, 10, "HOME_LATITUDE" },
    { 28, VALUE_I32, UNIT_DEGREE, -7, FIELD_SIGNED | FIELD_INVALID, 10, 10, "HOME_LONGITUDE" },
    { 29, VALUE_U8, UNIT_COUNT, 0, FIELD_INVALID, 10, 10, "MISSION_ACTIVE_WAYPOINT" },
    { 30, VALUE_U8, UNIT_NONE, 0, 0, 10, 10, "MISSION_STATE" },
    { 31, VALUE_U8, UNIT_NONE, 0, 0, 10, 10, "MISSION_ABORT_REASON" },
    RC_FIELD(1, 32), RC_FIELD(2, 33), RC_FIELD(3, 34), RC_FIELD(4, 35),
    RC_FIELD(5, 36), RC_FIELD(6, 37), RC_FIELD(7, 38), RC_FIELD(8, 39),
    RC_FIELD(9, 40), RC_FIELD(10, 41), RC_FIELD(11, 42), RC_FIELD(12, 43),
    RC_FIELD(13, 44), RC_FIELD(14, 45), RC_FIELD(15, 46), RC_FIELD(16, 47),
    { 50, VALUE_BOOL, UNIT_NONE, 0, 0, 10, 10, "MISSION_ACTIVE" },
};
#undef RC_FIELD

uint16_t session_id;
uint32_t boot_id;
uint32_t catalog_hash;

bool take(sbuf_t *src, void *value, uint16_t len)
{
    if (sbuf_bytes_remaining(src) < len) {
        return false;
    }
    memcpy(value, src->ptr, len);
    src->ptr += len;
    return true;
}

bool read_u8(sbuf_t *src, uint8_t &v) { return take(src, &v, 1); }
bool read_u16(sbuf_t *src, uint16_t &v)
{
    uint8_t b[2];
    if (!take(src, b, sizeof(b))) { return false; }
    v = uint16_t(b[0]) | uint16_t(b[1]) << 8;
    return true;
}

void put_u8(sbuf_t *dst, uint8_t v) { sbuf_write_data(dst, &v, 1); }
void put_u16(sbuf_t *dst, uint16_t v)
{
    const uint8_t b[] = { uint8_t(v), uint8_t(v >> 8) };
    sbuf_write_data(dst, b, sizeof(b));
}
void put_u32(sbuf_t *dst, uint32_t v)
{
    const uint8_t b[] = { uint8_t(v), uint8_t(v >> 8), uint8_t(v >> 16), uint8_t(v >> 24) };
    sbuf_write_data(dst, b, sizeof(b));
}

uint32_t fnv_byte(uint32_t hash, uint8_t v) { return (hash ^ v) * 16777619U; }
void hash_u16(uint32_t &hash, uint16_t v)
{
    hash = fnv_byte(hash, v);
    hash = fnv_byte(hash, v >> 8);
}
void hash_u32(uint32_t &hash, uint32_t v)
{
    hash_u16(hash, v);
    hash_u16(hash, v >> 16);
}

uint32_t get_catalog_hash()
{
    if (catalog_hash != 0) { return catalog_hash; }
    uint32_t hash = 2166136261U;
    for (const Field &field : fields) {
        hash_u16(hash, field.id);
        hash = fnv_byte(hash, field.type);
        hash = fnv_byte(hash, field.unit);
        hash = fnv_byte(hash, field.exponent);
        hash_u16(hash, field.flags);
        hash_u16(hash, field.maximum_rate);
        hash_u16(hash, field.native_rate);
        hash = fnv_byte(hash, 1);
        const uint8_t length = strlen(field.name);
        hash = fnv_byte(hash, length);
        for (uint8_t i = 0; i < length; i++) { hash = fnv_byte(hash, field.name[i]); }
    }
    catalog_hash = hash != 0 ? hash : 1;
    return catalog_hash;
}

struct Header {
    uint8_t version;
    uint8_t message;
    uint8_t flags;
    uint8_t source;
    uint8_t destination;
    uint16_t session;
    uint16_t exchange;
};

bool read_header(sbuf_t *src, Header &h)
{
    uint8_t subcommand;
    return read_u8(src, subcommand) && subcommand == GHOST_SUBCOMMAND &&
        read_u8(src, h.version) && read_u8(src, h.message) &&
        read_u8(src, h.flags) && read_u8(src, h.source) &&
        read_u8(src, h.destination) && read_u16(src, h.session) &&
        read_u16(src, h.exchange);
}

void write_header(sbuf_t *dst, const Header &request, uint8_t message, Status status)
{
    put_u8(dst, GHOST_SUBCOMMAND);
    put_u8(dst, GHOST_VERSION);
    put_u8(dst, message);
    put_u8(dst, FLAG_RESPONSE | (status == OK ? 0 : FLAG_ERROR));
    put_u8(dst, ENDPOINT_FC);
    put_u8(dst, request.source);
    put_u16(dst, session_id);
    put_u16(dst, request.exchange);
}

uint32_t mission_hash(AP_Mission *mission)
{
    uint32_t hash = 2166136261U;
    const uint16_t count = mission == nullptr ? 0 : mission->num_commands();
    hash_u16(hash, count);
    for (uint16_t sequence = 0; sequence < count; sequence++) {
        AP_Mission::Mission_Command command {};
        mavlink_mission_item_int_t item {};
        if (!mission->read_cmd_from_storage(sequence, command) ||
            !AP_Mission::mission_cmd_to_mavlink_int(command, item)) {
            return 0;
        }
        hash_u16(hash, sequence);
        hash = fnv_byte(hash, item.frame);
        hash_u16(hash, item.command);
        hash = fnv_byte(hash, item.autocontinue);
        for (float parameter : { item.param1, item.param2, item.param3, item.param4 }) {
            uint32_t bits;
            memcpy(&bits, &parameter, sizeof(bits));
            hash_u32(hash, bits);
        }
        hash_u32(hash, item.x);
        hash_u32(hash, item.y);
        uint32_t z;
        memcpy(&z, &item.z, sizeof(z));
        hash_u32(hash, z);
    }
    return hash != 0 ? hash : 1;
}

void put_float(sbuf_t *dst, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    put_u32(dst, bits);
}

} // namespace

MSPCommandResult AP_MSP_Telem_Backend::msp_process_ghost_dp(sbuf_t *src, sbuf_t *dst)
{
    Header request {};
    if (!read_header(src, request)) { return MSP_RESULT_ERROR; }
    const bool version_ok = (request.version >> 4) == (GHOST_VERSION >> 4);

    switch (request.message) {
    case HELLO_REQUEST: {
        const Status status = version_ok ? OK : UNSUPPORTED_VERSION;
        if (boot_id == 0) {
            boot_id = AP_HAL::micros() ^ uint32_t(reinterpret_cast<uintptr_t>(this));
            if (boot_id == 0) { boot_id = 1; }
        }
        session_id++;
        if (session_id == 0) { session_id = 1; }
        write_header(dst, request, HELLO_RESPONSE, status);
        put_u8(dst, status);
        put_u32(dst, boot_id);
        uint8_t uid[16] {};
        uint8_t uid_length = sizeof(uid);
        hal.util->get_system_id_unformatted(uid, uid_length);
        sbuf_write_data(dst, uid, sizeof(uid));
        put_u32(dst, get_catalog_hash());
        // Catalogue plus canonical read-only MISSION_ITEM_INT and opaque IDs.
        put_u32(dst, (1U << 0) | (1U << 11) | (1U << 12));
        put_u16(dst, MSP_PORT_INBUF_SIZE);
        put_u32(dst, 0); // Streaming is advertised when the subscription engine is enabled.
        put_u8(dst, 0);
        put_u8(dst, 5);
        return MSP_RESULT_ACK;
    }

    case CATALOG_REQUEST: {
        Status status = !version_ok ? UNSUPPORTED_VERSION :
            (request.session != session_id ? INVALID_SESSION : OK);
        uint16_t first_id = 0;
        uint8_t maximum_records = 0;
        if (status == OK && (!read_u16(src, first_id) || !read_u8(src, maximum_records) ||
                            sbuf_bytes_remaining(src) != 0)) {
            status = BAD_LENGTH;
        }
        write_header(dst, request, CATALOG_RESPONSE, status);
        put_u8(dst, status);
        put_u32(dst, get_catalog_hash());
        uint8_t *next_ptr = dst->ptr;
        put_u16(dst, 0);
        uint8_t *count_ptr = dst->ptr;
        put_u8(dst, 0);
        if (status != OK) { return MSP_RESULT_ACK; }
        uint8_t count = 0;
        size_t index = 0;
        while (index < ARRAY_SIZE(fields) && fields[index].id < first_id) { index++; }
        for (; index < ARRAY_SIZE(fields) && count < maximum_records; index++) {
            const Field &field = fields[index];
            const uint8_t name_length = strlen(field.name);
            const uint16_t record_size = 14U + name_length;
            if (sbuf_bytes_remaining(dst) < record_size) { break; }
            put_u8(dst, 13U + name_length);
            put_u16(dst, field.id);
            put_u8(dst, field.type);
            put_u8(dst, field.unit);
            put_u8(dst, field.exponent);
            put_u16(dst, field.flags);
            put_u16(dst, field.maximum_rate);
            put_u16(dst, field.native_rate);
            put_u8(dst, 1);
            put_u8(dst, name_length);
            sbuf_write_data(dst, field.name, name_length);
            count++;
        }
        *count_ptr = count;
        if (index < ARRAY_SIZE(fields)) {
            next_ptr[0] = fields[index].id;
            next_ptr[1] = fields[index].id >> 8;
        }
        return MSP_RESULT_ACK;
    }

    case MISSION_INFO_REQUEST: {
        uint8_t mission_type = 0;
        Status status = !version_ok ? UNSUPPORTED_VERSION :
            (request.session != session_id ? INVALID_SESSION : OK);
        if (status == OK && (!read_u8(src, mission_type) || mission_type != 0 ||
                            sbuf_bytes_remaining(src) != 0)) {
            status = BAD_LENGTH;
        }
        AP_Mission *mission = AP::mission();
        const uint32_t hash = mission_hash(mission);
        if (status == OK && (mission == nullptr || hash == 0)) { status = INVALID_MISSION; }
        write_header(dst, request, MISSION_INFO_RESPONSE, status);
        put_u8(dst, status);
        put_u8(dst, mission_type);
        put_u16(dst, status == OK ? mission->num_commands() : 0);
        put_u16(dst, status == OK ? mission->num_commands_max() : 0);
        put_u32(dst, status == OK ? hash : 0);
        put_u32(dst, status == OK ? hash : 0);
        return MSP_RESULT_ACK;
    }

    case MISSION_ITEM_REQUEST: {
        uint8_t mission_type = 0;
        uint16_t sequence = 0;
        Status status = !version_ok ? UNSUPPORTED_VERSION :
            (request.session != session_id ? INVALID_SESSION : OK);
        if (status == OK && (!read_u8(src, mission_type) || !read_u16(src, sequence) ||
                            mission_type != 0 || sbuf_bytes_remaining(src) != 0)) {
            status = BAD_LENGTH;
        }
        AP_Mission *mission = AP::mission();
        AP_Mission::Mission_Command command {};
        mavlink_mission_item_int_t item {};
        const uint32_t hash = mission_hash(mission);
        if (status == OK && (mission == nullptr || sequence >= mission->num_commands() ||
                            !mission->read_cmd_from_storage(sequence, command) ||
                            !AP_Mission::mission_cmd_to_mavlink_int(command, item))) {
            status = INVALID_MISSION;
        }
        write_header(dst, request, MISSION_ITEM_RESPONSE, status);
        put_u8(dst, status);
        put_u8(dst, mission_type);
        put_u32(dst, status == OK ? hash : 0);
        put_u16(dst, sequence);
        if (status == OK) {
            put_u8(dst, item.frame);
            put_u16(dst, item.command);
            put_u8(dst, mission->get_current_nav_index() == sequence);
            put_u8(dst, item.autocontinue);
            put_float(dst, item.param1); put_float(dst, item.param2);
            put_float(dst, item.param3); put_float(dst, item.param4);
            put_u32(dst, item.x); put_u32(dst, item.y); put_float(dst, item.z);
        }
        return MSP_RESULT_ACK;
    }

    default:
        write_header(dst, request, request.message, UNSUPPORTED_MESSAGE);
        put_u8(dst, UNSUPPORTED_MESSAGE);
        return MSP_RESULT_ACK;
    }
}

#endif // AP_MSP_GHOST_DP_ENABLED
