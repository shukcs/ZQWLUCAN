#pragma once
#include <cstdint>
#include <cstring>
#include <cmath>

#include "canard_stub.h"   // uavcan_equipment_air_data_RawAirData
#include "zqwl_ucan_global.h"

// ------------------------------------------------------------------
// 标准 CAN 帧
// ------------------------------------------------------------------
namespace dronecan {
namespace transport {

// 扩展帧标志 (写入 CAN 外设 ID 寄存器的 bit31)
static const uint32_t CAN_FRAME_EFF = 1u << 31;

inline uint32_t make_can_id(uint8_t source_node_id, uint16_t data_type_id, uint8_t priority)
{
    return ((uint32_t)(priority & 0x1F) << 24) |
           ((uint32_t)data_type_id << 8) |
           (uint32_t)(source_node_id & 0x7F);
}
inline uint8_t  can_id_source_node(uint32_t id) { return (uint8_t)(id & 0x7F); }
inline uint16_t can_id_data_type(uint32_t id)   { return (uint16_t)((id >> 8) & 0xFFFF); }
inline uint8_t  can_id_priority(uint32_t id)    { return (uint8_t)((id >> 24) & 0x1F); }

} // namespace transport

// ------------------------------------------------------------------
// uavcan.equipment.air_data.RawAirData 的序列化 / 反序列化
// ------------------------------------------------------------------
namespace raw_air_data {

static const uint16_t DATA_TYPE_ID = 1027;
static const uint16_t PAYLOAD_LEN = 17;

// IEEE754 半精度 (float16) 编码
inline uint16_t float_to_float16(float value)
{
    union { float f; uint32_t u; } v;
    v.f = value;
    const uint32_t sign = (v.u >> 16) & 0x8000;
    const int32_t exp = (int32_t)((v.u >> 23) & 0xFF) - 127 + 15;
    const uint32_t mant = v.u & 0x7FFFFF;

    if (exp <= 0) {
        if (exp < -10) {
            return (uint16_t)sign;      // 下溢为 0
        }
        // 次正规数
        const uint32_t m = (mant | 0x800000) >> (uint32_t)(1 - exp);
        return (uint16_t)(sign | ((m + 0x1000) >> 13));
    }
    if (exp >= 31) {
        return (uint16_t)(sign | 0x7C00);   // 上溢为 inf
    }
    return (uint16_t)(sign | ((uint32_t)exp << 10) | ((mant + 0x1000) >> 13));
}

inline float float16_to_float(uint16_t h)
{
    const uint32_t sign = (uint32_t)(h & 0x8000) << 16;
    const int32_t exp = (h >> 10) & 0x1F;
    const uint32_t mant = h & 0x3FF;

    uint32_t u;
    if (exp == 0) {
        if (mant == 0) {
            u = sign;                       // ±0
        } else {
            // 次正规数
            float f = (float)mant / 1024.0f * std::pow(2.0f, -14.0f);
            return sign ? -f : f;
        }
    } else if (exp == 31) {
        u = sign | 0x7F800000 | (mant << 13);
    } else {
        u = sign | ((uint32_t)(exp - 15 + 127) << 23) | (mant << 13);
    }
    union { uint32_t u; float f; } v;
    v.u = u;
    return v.f;
}

inline void put_float32_le(uint8_t *p, float f)
{
    union { float f; uint32_t u; } v;
    v.f = f;
    p[0] = (uint8_t)(v.u & 0xFF);
    p[1] = (uint8_t)((v.u >> 8) & 0xFF);
    p[2] = (uint8_t)((v.u >> 16) & 0xFF);
    p[3] = (uint8_t)((v.u >> 24) & 0xFF);
}
inline float get_float32_le(const uint8_t *p)
{
    union { uint32_t u; float f; } v;
    v.u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
          ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return v.f;
}
inline void put_float16_le(uint8_t *p, float f)
{
    const uint16_t h = float_to_float16(f);
    p[0] = (uint8_t)(h & 0xFF);
    p[1] = (uint8_t)(h >> 8);
}
inline float get_float16_le(const uint8_t *p)
{
    return float16_to_float((uint16_t)(p[0] | ((uint16_t)p[1] << 8)));
}

// 按 DSDL 声明顺序 (无填充) 打包
inline void serialize(const uavcan_equipment_air_data_RawAirData &msg, uint8_t *payload)
{
    payload[0] = msg.flags;
    put_float32_le(payload + 1,  msg.static_pressure);
    put_float32_le(payload + 5,  msg.differential_pressure);
    put_float16_le(payload + 9,  msg.static_pressure_sensor_temperature);
    put_float16_le(payload + 11, msg.differential_pressure_sensor_temperature);
    put_float16_le(payload + 13, msg.static_air_temperature);
    put_float16_le(payload + 15, msg.pitot_temperature);
}

inline void deserialize(const uint8_t *payload, uavcan_equipment_air_data_RawAirData &msg)
{
    msg.flags = payload[0];
    msg.static_pressure = get_float32_le(payload + 1);
    msg.differential_pressure = get_float32_le(payload + 5);
    msg.static_pressure_sensor_temperature = get_float16_le(payload + 9);
    msg.differential_pressure_sensor_temperature = get_float16_le(payload + 11);
    msg.static_air_temperature = get_float16_le(payload + 13);
    msg.pitot_temperature = get_float16_le(payload + 15);
}

} // namespace raw_air_data
} // namespace dronecan

// ------------------------------------------------------------------
// 消息编解码 trait 特化: RawAirData 的 data type id 与反序列化
// (对应真实固件里 DSDL 生成代码, 供 Canard::allocate_sub_arg_callback 使用)
// ------------------------------------------------------------------
template <>
struct canard_msg_codec<uavcan_equipment_air_data_RawAirData> {
    static uint16_t data_type_id() { return dronecan::raw_air_data::DATA_TYPE_ID; }
    static void deserialize(const uint8_t *payload, uavcan_equipment_air_data_RawAirData &msg)
    {
        dronecan::raw_air_data::deserialize(payload, msg);
    }
};
