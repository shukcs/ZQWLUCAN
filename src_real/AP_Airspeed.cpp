/*
 * AP_Airspeed.cpp — 脱胎于 libraries/AP_Airspeed/AP_Airspeed.cpp
 *
 * 为了让空速计可以脱离整个 ArduPilot 固件独立运行, 这里只保留了 Frontend 的
 * 核心逻辑:
 *   - allocate()         : 按 ARSPD_TYPE 创建 backend (仅保留 DroneCAN)
 *   - read()             : 差压 -> 空速的核心换算 (IIR 滤波 + 管序处理)
 *   - select_primary()   : 多传感器主用选择
 *   - healthy()/use()/enabled() 等查询
 *
 * 被裁掉的是与载具/日志/GCS/飞控 HAL 强耦合的部分 (AP_Vehicle, AP_Logger,
 * GCS_MAVLink 等), 这些在 stub/ 里由最小实现替身。
 *
 * read()/select_primary()/healthy() 的算法与上游保持一致, 便于对照。
 */

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_ENABLED

#include "AP_Airspeed.h"
#include "AP_Airspeed_Backend.h"
#include "AP_Airspeed_DroneCAN.h"

#include <cstring>

AP_Airspeed *AP_Airspeed::_singleton;

// AP_Param 在本 demo 中被 stub, 参数表仅用于结构占位
const AP_Param::GroupInfo AP_Airspeed::var_info[] = { AP_GROUPEND };

AP_Airspeed::AP_Airspeed()
{
    // 真实固件里这些数组是静态存储 (BSS 清零), demo 中对象在堆上, 必须显式清零
    memset(sensor, 0, sizeof(sensor));
    memset(state, 0, sizeof(state));

    primary = 0;
    num_sensors = 0;
    last_user_primary = 0;
    _force_disable_use = false;
    calibration_enabled = false;
    fixed_wing_parameters = nullptr;
    _log_bit = (uint32_t)-1;

    // 与 AP_Airspeed_Params var_info 里的默认值一致 (每个实例的 ARSPD[2]_* 分组)
    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        param[i].bus.set_default(HAL_AIRSPEED_BUS_DEFAULT);
        param[i].ratio.set_default(2.0f);
        param[i].psi_range.set_default(1.0f);
        param[i].tube_order.set_default((int8_t)pitot_tube_order::PITOT_TUBE_ORDER_AUTO);
    }
    // 只有实例 0 用 HAL 给的默认类型, 第二路需要用户显式设 ARSPD2_TYPE
    param[0].type.set_default(HAL_AIRSPEED_TYPE_DEFAULT);       // DroneCAN = 8
    param[0].pin.set_default(1);

    _options.set_default((int32_t)(OptionsMask::ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE |
                                   OptionsMask::ON_FAILURE_AHRS_WIND_MAX_RECOVERY_DO_REENABLE |
                                   OptionsMask::USE_EKF_CONSISTENCY));
    _wind_gate.set_default(5.0f);

    // demo 不走开机零偏校准, 直接标记为"无需零偏", 这样 offset 允许为 0
    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        state[i].cal.state = CalibrationState::NOT_REQUIRED_ZERO_OFFSET;
        state[i].failures.health_probability = 1.0f;
    }

    _singleton = this;
}

void AP_Airspeed::set_fixedwing_parameters(const class AP_FixedWing *_fixed_wing_parameters)
{
    fixed_wing_parameters = _fixed_wing_parameters;
}

bool AP_Airspeed::add_backend(AP_Airspeed_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (num_sensors >= AIRSPEED_MAX_SENSORS) {
        AP_HAL::panic("Too many airspeed drivers");
    }
    const uint8_t i = num_sensors;
    sensor[num_sensors++] = backend;
    if (!sensor[i]->init()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Airspeed %u init failed", i + 1);
        delete sensor[i];
        sensor[i] = nullptr;
    }
    return true;
}

void AP_Airspeed::convert_per_instance()
{
    // 参数转换依赖 AP_Param 存储, demo 中不需要
}

void AP_Airspeed::init()
{
    convert_per_instance();

    // Set primary from parameter to avoid primary changed message at boot
    primary = primary_sensor.get();

    if (enabled(0)) {
        allocate();
    }
}

void AP_Airspeed::allocate()
{
    if (sensor[0] != nullptr) {
        // already initialised, periph may call allocate several times to allow CAN detection
        return;
    }

    // 只保留 DroneCAN backend; 其它 backend 已在 AP_Airspeed_config.h 里关闭
    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        // Set the enable automatically to false and set the probability that the airspeed is healhy to start with
        state[i].failures.health_probability = 1.0f;

        switch ((enum airspeed_type)param[i].type.get()) {
        case TYPE_NONE:
            // nothing to do
            break;
#if AP_AIRSPEED_DRONECAN_ENABLED
        case TYPE_UAVCAN:
            sensor[i] = AP_Airspeed_DroneCAN::probe(*this, i, uint32_t(param[i].bus_id.get()));
            break;
#endif  // AP_AIRSPEED_DRONECAN_ENABLED
        }
        if (sensor[i] && !sensor[i]->init()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Airspeed %u init failed", i + 1);
            delete sensor[i];
            sensor[i] = nullptr;
        }
        if (sensor[i] != nullptr) {
            num_sensors = i + 1;
        }
    }

#if AP_AIRSPEED_DRONECAN_ENABLED
    // we need a 2nd pass for DroneCAN sensors so we can match order by DEVID
    // the 2nd pass accepts any devid
    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (sensor[i] == nullptr && (enum airspeed_type)param[i].type.get() == TYPE_UAVCAN) {
            sensor[i] = AP_Airspeed_DroneCAN::probe(*this, i, 0);
            if (sensor[i] != nullptr) {
                num_sensors = i + 1;
            }
        }
    }
#endif // AP_AIRSPEED_DRONECAN_ENABLED

    // set DEVID to zero for any sensors not found. This allows backends to order
    // based on previous value of DEVID. This allows for swapping out sensors
    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (sensor[i] == nullptr) {
            // note we use set() not set_and_save() to allow a sensor to be temporarily
            // removed for one boot without losing its slot
            param[i].bus_id.set(0);
        }
    }
}

// get a temperature reading if possible
bool AP_Airspeed::get_temperature(uint8_t i, float &temperature) const
{
    if (!enabled(i)) {
        return false;
    }
    if (sensor[i]) {
        return sensor[i]->get_temperature(temperature);
    }
    return false;
}

// calibrate the zero offset for the airspeed
void AP_Airspeed::calibrate(bool in_startup)
{
    (void)in_startup;
    // demo 中零偏已在构造时标记为 NOT_REQUIRED_ZERO_OFFSET, 无需校准
}

/*
  update async airspeed zero offset calibration
*/
void AP_Airspeed::update_calibration(uint8_t i, float raw_pressure)
{
    if (!enabled(i) || state[i].cal.start_ms == 0) {
        return;
    }

    // consider calibration complete when we have at least 15 samples
    // over at least 1 second
    if (AP_HAL::millis() - state[i].cal.start_ms >= 1000 &&
        state[i].cal.read_count > 15) {
        if (state[i].cal.count == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Airspeed %u unhealthy", i + 1);
            state[i].cal.state = CalibrationState::FAILED;
        } else {
            float calibrated_offset = state[i].cal.sum / state[i].cal.count;
            param[i].offset.set_and_save(calibrated_offset);
            state[i].cal.state = CalibrationState::SUCCESS;
            if (_options.get() & AP_Airspeed::OptionsMask::REPORT_OFFSET) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Airspeed %u calibrated, offset = %4.0f", i + 1, calibrated_offset);
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Airspeed %u calibrated", i + 1);
            }
        }
        state[i].cal.start_ms = 0;
        return;
    }
    // we discard the first 5 samples
    if (state[i].healthy && state[i].cal.read_count > 5) {
        state[i].cal.sum += raw_pressure;
        state[i].cal.count++;
    }
    state[i].cal.read_count++;
}

// get aggregate calibration state for the Airspeed library
AP_Airspeed::CalibrationState AP_Airspeed::get_calibration_state() const
{
    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (!enabled(i)) {
            continue;
        }

        switch (state[i].cal.state) {
        case CalibrationState::SUCCESS:
        case CalibrationState::NOT_REQUIRED_ZERO_OFFSET:
            continue;
        case CalibrationState::NOT_STARTED:
            return CalibrationState::NOT_STARTED;
        case CalibrationState::IN_PROGRESS:
            return CalibrationState::IN_PROGRESS;
        case CalibrationState::FAILED:
            return CalibrationState::FAILED;
        }
    }
    return CalibrationState::SUCCESS;
}

// read one airspeed sensor
void AP_Airspeed::read(uint8_t i)
{
    if (!enabled(i) || !sensor[i]) {
        return;
    }
    state[i].last_update_ms = AP_HAL::millis();

    // try and get a direct reading of airspeed
    if (sensor[i]->has_airspeed()) {
        state[i].healthy = sensor[i]->get_airspeed(state[i].airspeed);
        state[i].raw_airspeed = state[i].airspeed;  // for logging
        return;
    }

    /*
      remember the old healthy state
     */
    bool prev_healthy = state[i].healthy;

    float raw_pressure = 0;
    state[i].healthy = sensor[i]->get_differential_pressure(raw_pressure);

    float airspeed_pressure = raw_pressure - get_offset(i);

    // remember raw pressure for logging
    state[i].corrected_pressure = airspeed_pressure;

    if (state[i].cal.start_ms != 0) {
        update_calibration(i, raw_pressure);
    }

    // filter before clamping positive
    if (!prev_healthy) {
        // if the previous state was not healthy then we should not
        // use an IIR filter, otherwise a bad reading will last for
        // some time after the sensor becomes healthy again
        state[i].filtered_pressure = airspeed_pressure;
    } else {
        state[i].filtered_pressure = 0.7f * state[i].filtered_pressure + 0.3f * airspeed_pressure;
    }

    /*
      we support different pitot tube setups so user can choose if
      they want to be able to detect pressure on the static port
     */
    switch ((enum pitot_tube_order)param[i].tube_order.get()) {
    case PITOT_TUBE_ORDER_NEGATIVE:
        state[i].last_pressure  = -airspeed_pressure;
        state[i].raw_airspeed   = sqrtf(MAX(-airspeed_pressure, 0) * param[i].ratio);
        state[i].airspeed       = sqrtf(MAX(-state[i].filtered_pressure, 0) * param[i].ratio);
        break;
    case PITOT_TUBE_ORDER_POSITIVE:
        state[i].last_pressure  = airspeed_pressure;
        state[i].raw_airspeed   = sqrtf(MAX(airspeed_pressure, 0) * param[i].ratio);
        state[i].airspeed       = sqrtf(MAX(state[i].filtered_pressure, 0) * param[i].ratio);
        break;
    case PITOT_TUBE_ORDER_AUTO:
    default:
        state[i].last_pressure  = fabsf(airspeed_pressure);
        state[i].raw_airspeed   = sqrtf(fabsf(airspeed_pressure) * param[i].ratio);
        state[i].airspeed       = sqrtf(fabsf(state[i].filtered_pressure) * param[i].ratio);
        break;
    }
}

// Select primary sensor based on user parameters and health
uint8_t AP_Airspeed::select_primary()
{
    // user selected primary from parameter, track changes
    const uint8_t user_primary = primary_sensor.get();
    const bool user_primary_changed = user_primary != last_user_primary;
    last_user_primary = user_primary;

    // If user selected instance is both healthy and set to use then it is a valid primary
    const bool user_healthy = healthy(user_primary);
    const bool user_healthy_and_use = user_healthy && use(user_primary);

    // demo 中视为地面状态 (disarmed), 允许每次重新选择
    const bool armed = false;

    if ((user_primary_changed || !armed) && user_healthy_and_use) {
        return user_primary;
    }

    // If the currently selected primary is valid there is no need to change
    const bool primary_healthy = healthy(primary);
    if (primary_healthy && use(primary)) {
        return primary;
    }

    if (user_healthy_and_use) {
        // The current primary is not valid, try the user set primary first
        return user_primary;
    }

    // Select the first sensor which is both healthy and set to use
    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if ((i == primary) || (i == user_primary)) {
            continue;
        }
        if (healthy(i) && use(i)) {
            return i;
        }
    }

    // No sensor is both healthy and set to use

    // Continue with current primary if healthy
    if (primary_healthy) {
        return primary;
    }

    // Use user selected instance if healthy
    if (user_healthy) {
        return user_primary;
    }

    // Select the first sensor which is healthy
    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if ((i == primary) || (i == user_primary)) {
            continue;
        }
        if (healthy(i)) {
            return i;
        }
    }

    // No healthy sensor, don't change primary
    return primary;
}

// read all airspeed sensors
void AP_Airspeed::update()
{
    if (!lib_enabled()) {
        return;
    }

    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        read(i);
    }

    // Check for failures possibly marking sensors and unhealthy
    check_sensor_failures();

    // Record old primary sensor for reporting
    const uint8_t old_primary = primary;

    // Select primary sensor based on user parameters and health
    primary = select_primary();

    if (primary != old_primary) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Airspeed primary changed: %i", primary + 1);
    }
}

bool AP_Airspeed::use(uint8_t i) const
{
    if (!lib_enabled()) {
        return false;
    }
    if (_force_disable_use) {
        return false;
    }
    if (!enabled(i) || !param[i].use) {
        return false;
    }
    if (param[i].use == 2 && !is_zero(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle))) {
        // special case for gliders with airspeed sensors behind the
        // propeller. Allow airspeed to be disabled when throttle is
        // running
        return false;
    }
    return true;
}

/*
  return true if all enabled sensors are healthy
 */
bool AP_Airspeed::all_healthy(void) const
{
    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (enabled(i) && !healthy(i)) {
            return false;
        }
    }
    return true;
}

bool AP_Airspeed::lib_enabled() const {
    // demo 中不实现 ARSPD_ENABLE 参数, 库始终启用
    return true;
}

// return true if airspeed is enabled
bool AP_Airspeed::enabled(uint8_t i) const {
    if (!lib_enabled()) {
        return false;
    }
    if (i < AIRSPEED_MAX_SENSORS) {
        return param[i].type.get() != TYPE_NONE;
    }
    return false;
}

// return health status of sensor
bool AP_Airspeed::healthy(uint8_t i) const {
    if (!enabled(i)) {
        return false;
    }
    bool ok = state[i].healthy && sensor[i] != nullptr;
    // sanity check the offset parameter.  Zero is permitted if we are skipping calibration.
    const bool allowZeroOffset = (state[i].cal.state == CalibrationState::NOT_REQUIRED_ZERO_OFFSET) ||
        (param[i].skip_cal.get() == AP_Airspeed_Params::SkipCalType::NoCalRequired);
    ok &= allowZeroOffset || !is_zero(param[i].offset);
    return ok;
}

// return the current airspeed in m/s
float AP_Airspeed::get_airspeed(uint8_t i) const {
    if (!enabled(i)) {
        // we can't have negative airspeed so sending an obviously invalid value
        return -1.0;
    }
    return state[i].airspeed;
}

// return the unfiltered airspeed in m/s
float AP_Airspeed::get_raw_airspeed(uint8_t i) const {
    if (!enabled(i)) {
        // we can't have negative airspeed so sending an obviously invalid value
        return -1.0;
    }
    return state[i].raw_airspeed;
}

// return the differential pressure in Pascal for the last airspeed reading
float AP_Airspeed::get_differential_pressure(uint8_t i) const {
    if (!enabled(i)) {
        return 0.0;
    }
    return state[i].last_pressure;
}

// return the current corrected pressure
float AP_Airspeed::get_corrected_pressure(uint8_t i) const {
    if (!enabled(i)) {
        return 0.0;
    }
    return state[i].corrected_pressure;
}

// return the current calibration offset
float AP_Airspeed::get_offset(uint8_t i) const {
    if (state[i].cal.state == CalibrationState::NOT_REQUIRED_ZERO_OFFSET) {
        return 0.0;
    }
    return param[i].offset;
}

// demo 不做 EKF/地速一致性失败检查, 概率恒为 1
void AP_Airspeed::check_sensor_failures()
{
}

void AP_Airspeed::Log_Airspeed()
{
}

namespace AP {
    AP_Airspeed *airspeed()
    {
        return AP_Airspeed::get_singleton();
    }
}

#endif  // AP_AIRSPEED_ENABLED
