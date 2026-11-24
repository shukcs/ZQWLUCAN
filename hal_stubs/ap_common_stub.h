/*
 * ap_common_stub.h — AP_Common / AP_HAL_Boards 的最小替身
 */
#pragma once
#include <cstdint>
#include <cstddef>
#include "hal_stub.h"

#define CONFIG_HAL_BOARD 0
#define HAL_BOARD_SITL 0

#define HAL_AIRSPEED_TYPE_DEFAULT 8    // DroneCAN
#define HAL_AIRSPEED_BUS_DEFAULT 1

#ifndef NEW_NOTHROW
#define NEW_NOTHROW new
#endif

#define CLASS_NO_COPY(cls) \
    cls(const cls &) = delete; \
    cls &operator=(const cls &) = delete;

namespace AP_HAL {
class Device {
public:
    enum BusType {
        BUS_TYPE_UNKNOWN = 0,
        BUS_TYPE_I2C     = 1,
        BUS_TYPE_SPI     = 2,
        BUS_TYPE_UAVCAN  = 3,
    };
    // 与 AP_HAL::Device::make_bus_id(bus_type, bus, address, devtype) 位布局一致
    static uint32_t make_bus_id(BusType bus_type, uint8_t bus, uint8_t address, uint8_t devtype) {
        return ((uint32_t)bus_type << 24) | ((uint32_t)bus << 16) | ((uint32_t)address << 8) | devtype;
    }
};
}
