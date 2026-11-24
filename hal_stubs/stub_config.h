/*
 * stub_config.h — 覆盖 AP_Airspeed_config.h 的默认宏
 * 只开启 DroneCAN 后端 + Frontend，其他全关，切断 HAL/I2C/ADC 依赖
 */
#pragma once

#ifndef AP_AIRSPEED_ENABLED
#define AP_AIRSPEED_ENABLED 1
#endif

#ifndef AP_AIRSPEED_BACKEND_DEFAULT_ENABLED
#define AP_AIRSPEED_BACKEND_DEFAULT_ENABLED 0
#endif

#define AP_AIRSPEED_ANALOG_ENABLED      0
#define AP_AIRSPEED_ASP5033_ENABLED     0
#define AP_AIRSPEED_AUAV_ENABLED        0
#define AP_AIRSPEED_DLVR_ENABLED        0
#define AP_AIRSPEED_DRONECAN_ENABLED    1    // 只开这个
#define AP_AIRSPEED_MS4525_ENABLED      0
#define AP_AIRSPEED_MS5525_ENABLED      0
#define AP_AIRSPEED_MSP_ENABLED         0
#define AP_AIRSPEED_NMEA_ENABLED        0
#define AP_AIRSPEED_SDP3X_ENABLED       0
#define AP_AIRSPEED_SITL_ENABLED        0
#define AP_AIRSPEED_EXTERNAL_ENABLED    0
#define AP_AIRSPEED_SCRIPTING_ENABLED   0

#define AP_AIRSPEED_AUTOCAL_ENABLE      0
#define AP_AIRSPEED_HYGROMETER_ENABLE   0

#define AIRSPEED_MAX_SENSORS 2

// 真实源码里 AP_AIRSPEED_DUMMY_METHODS_ENABLED 由载具类型推导，
// 这里预定义成 0（普通固件路径），跳过那段 #ifndef 推导
#define AP_AIRSPEED_DUMMY_METHODS_ENABLED 0

// 真实源码里用 APM_BUILD_TYPE(...) 区分载具; demo 不区分, 一律返回 0
#define APM_BUILD_TYPE(x) 0
#define APM_BUILD_COPTER_OR_HELI 0
#define HAL_PROGRAM_SIZE_LIMIT_KB 2048

// DroneCAN 驱动总开关 (真实固件在 AP_HAL_Boards.h 里给出)
#ifndef HAL_ENABLE_DRONECAN_DRIVERS
#define HAL_ENABLE_DRONECAN_DRIVERS 1
#endif

// 注意: HAL_BUILD_AP_PERIPH 故意不定义 —— 真实固件只在 AP_Periph 构建时定义它，
// 代码里用 #ifndef HAL_BUILD_AP_PERIPH 判断"普通固件"
