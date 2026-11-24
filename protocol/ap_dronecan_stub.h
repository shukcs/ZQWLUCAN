/*
 * ap_dronecan_stub.h — AP_DroneCAN 的最小替身
 * 只提供驱动索引, 真实实现代表一条物理 CAN 总线
 */
#pragma once
#include <cstdint>

class AP_DroneCAN {
public:
    AP_DroneCAN() = default;
    explicit AP_DroneCAN(uint8_t idx, uint8_t ch) : _idx(idx), _ch(ch){}
    uint8_t get_driver_index() const { return _idx; }
    uint8_t get_driver_channel() const { return _ch; }
private:
    uint8_t _idx = 0;
    uint8_t _ch = 0;
};
