/*
 * gcs_stub.h — GCS / SRV_Channel 的最小替身
 */
#pragma once
#include <cstdio>
#include "ap_common_stub.h"

#define HAL_GCS_ENABLED 0
#define HAL_LOGGING_ENABLED 0

enum MAV_SEVERITY {
    MAV_SEVERITY_EMERGENCY = 0,
    MAV_SEVERITY_ALERT     = 1,
    MAV_SEVERITY_CRITICAL  = 2,
    MAV_SEVERITY_ERROR     = 3,
    MAV_SEVERITY_WARNING   = 4,
    MAV_SEVERITY_NOTICE    = 5,
    MAV_SEVERITY_INFO      = 6,
    MAV_SEVERITY_DEBUG     = 7,
};

// 真实实现会把文本发到地面站; demo 直接打印到 stderr
#define GCS_SEND_TEXT(severity, fmt, ...) \
    fprintf(stderr, "[GCS:%d] " fmt "\n", (int)(severity), ##__VA_ARGS__)

namespace SRV_Channel {
enum {
    k_throttle = 70,
};
}

namespace SRV_Channels {
inline int16_t get_output_scaled(int /*ch*/) { return 0; }
}
