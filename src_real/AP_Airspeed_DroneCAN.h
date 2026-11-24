#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_DRONECAN_ENABLED

#include "AP_Airspeed_Backend.h"

// 真实固件: #include <AP_DroneCAN/AP_DroneCAN.h>  (libcanard + 生成的消息结构)
// demo: 用最小替身, 消息结构来自 DSDL 1027.RawAirData
#include "canard_stub.h"
#include "ap_dronecan_stub.h"

class AP_Airspeed_DroneCAN : public AP_Airspeed_Backend {
public:

    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    bool init(void) override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // temperature not available via analog backend
    bool get_temperature(float &temperature) override;

#if AP_AIRSPEED_HYGROMETER_ENABLE
    // get hygrometer data
    bool get_hygrometer(uint32_t &last_sample_ms, float &temperature, float &humidity) override;
#endif

    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);

    static AP_Airspeed_Backend* probe(AP_Airspeed &_frontend, uint8_t _instance, uint32_t previous_devid);

    // 真实固件里这是 libcanard 的私有订阅回调; demo 在 public 里以便由 CAN 帧喂数据
    static void handle_airspeed(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_RawAirData &msg);

private:

#if AP_AIRSPEED_HYGROMETER_ENABLE
    static void handle_hygrometer(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const dronecan_sensors_hygrometer_Hygrometer &msg);
#endif

    static AP_Airspeed_DroneCAN* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id);

    float _pressure; // Pascal
    float _temperature; // Celcius
    uint32_t _last_sample_time_ms;

    // hygrometer data
    struct {
        float temperature;
        float humidity;
        uint32_t last_sample_ms;
    } _hygrometer;

    HAL_Semaphore _sem_airspeed;

    // Module Detection Registry
    static struct DetectedModules {
        AP_DroneCAN* ap_dronecan;
        uint8_t node_id;
        AP_Airspeed_DroneCAN *driver;
    } _detected_modules[AIRSPEED_MAX_SENSORS];

    static HAL_Semaphore _sem_registry;
    bool _have_temperature;
};


#endif  // AP_AIRSPEED_DRONECAN_ENABLED
