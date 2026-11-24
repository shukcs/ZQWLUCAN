/*
 * demo/can_iface.h — DroneCAN CAN 接口层 (对外不暴露 CanFrame)
 *
 * 对应真实固件里的 AP_DroneCAN 驱动:
 *   - 内部维护一条 CAN 总线 + 收发线程
 *   - 收到的标准 CAN 帧在这里由"协议栈"重组为 DroneCAN 传输
 *   - 按 (驱动索引, 数据类型 ID) 把消息分发给注册的订阅回调
 *
 * 业务层 (main) 只看到 AP_DroneCAN 指针, 不接触任何 CAN 帧细节。
 * 具体实现在 can_iface.cpp (PIMPL 隐藏)。
 */
#pragma once
#include <cstdint>

class AP_DroneCAN;

class DroneCANInterface {
public:
    explicit DroneCANInterface(uint8_t driver_index);
    ~DroneCANInterface();

    DroneCANInterface(const DroneCANInterface &) = delete;
    DroneCANInterface &operator=(const DroneCANInterface &) = delete;

    // 启动 CAN 收发线程 (相当于 CAN 驱动开始工作)
    void start();
    // 停止线程
    void stop();

    // 订阅方注册回调时需要的驱动对象
    AP_DroneCAN *dronecan();
    const AP_DroneCAN *dronecan() const;

    // 已分发的完整传输数量 (诊断用)
    uint32_t rx_transfers() const;

private:
    class Impl;
    Impl *_impl;
};
