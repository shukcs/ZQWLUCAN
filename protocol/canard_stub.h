/*
 * canard_stub.h — DroneCAN (libcanard) 的最小替代
 *
 * 模拟真实 libcanard 的订阅机制:
 *   Canard::allocate_sub_arg_callback<T, msgtype>() 在 libcanard 里会向
 *   协议栈注册一个"收到 msgtype 时调用 cb"的回调。这里用一个订阅注册表实现,
 *   CAN 接口层收到完整传输后按 (driver_index, data_type_id) 分发。
 *
 * 真实的 DroneCAN 消息定义来自
 *   modules/DroneCAN/DSDL/uavcan/equipment/air_data/1027.RawAirData.uavcan
 * 消息的编解码由消息定义处提供 (见 can_frame.h 里的 canard_msg_codec 特化)。
 */
#pragma once
#include <cstdint>
#include <vector>
#include <functional>

struct CanardRxTransfer {
    uint8_t source_node_id;   // 1..127 (0 = anonymous)
    uint8_t transfer_id;
};

// uavcan.equipment.air_data.RawAirData (data type id = 1027)
// ArduPilot 生成代码把 float16 字段展开为 float
struct uavcan_equipment_air_data_RawAirData {
    uint8_t flags;
    float static_pressure;
    float differential_pressure;
    float static_pressure_sensor_temperature;
    float differential_pressure_sensor_temperature;
    float static_air_temperature;
    float pitot_temperature;
};

#define KELVIN_TO_C(x) ((x) - 273.15f)
#define C_TO_KELVIN(x) ((x) + 273.15f)

// 消息编解码 trait: 由消息定义处(DSDL 生成代码 / can_frame.h)给出特化
template <typename Msg>
struct canard_msg_codec;

// 一条订阅登记项
struct CanardSubscription {
    uint8_t driver_index;
    uint16_t data_type_id;
    // payload 为原始 DroneCAN 载荷, 由 trait 反序列化后交给业务回调
    std::function<void(const CanardRxTransfer &, const uint8_t *payload)> invoke;
};

inline std::vector<CanardSubscription> &canard_subscriptions()
{
    static std::vector<CanardSubscription> subs;
    return subs;
}

// 协议栈收到一次完整传输后调用: 按 驱动索引 + 数据类型 ID 找到订阅回调
inline void canard_dispatch(uint8_t driver_index,
                            const CanardRxTransfer &transfer,
                            uint16_t data_type_id,
                            const uint8_t *payload)
{
    for (auto &sub : canard_subscriptions()) {
        if (sub.driver_index == driver_index && sub.data_type_id == data_type_id) {
            sub.invoke(transfer, payload);
        }
    }
}

class Canard {
public:
    // 与 libcanard canard/subscriber.h 的 allocate_sub_arg_callback 签名一致
    template<typename T, typename msgtype>
    static void *allocate_sub_arg_callback(T *arg,
                                           void (*cb)(T *arg, const CanardRxTransfer &transfer, const msgtype &msg),
                                           uint8_t index)
    {
        CanardSubscription sub;
        sub.driver_index = index;
        sub.data_type_id = canard_msg_codec<msgtype>::data_type_id();
        sub.invoke = [arg, cb](const CanardRxTransfer &transfer, const uint8_t *payload) {
            msgtype msg;
            canard_msg_codec<msgtype>::deserialize(payload, msg);
            cb(arg, transfer, msg);
        };
        canard_subscriptions().push_back(sub);
        return (void *)&canard_subscriptions().back();
    }
};
