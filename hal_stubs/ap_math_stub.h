/*
 * ap_math_stub.h — AP_Math 的最小替身
 */
#pragma once
#include <cmath>
#include <algorithm>

using std::isnan;
using std::isinf;

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

inline bool is_zero(float v)     { return std::fabs(v) < 1e-9f; }
inline bool is_zero(double v)    { return std::fabs(v) < 1e-9; }
inline bool is_zero(int v)       { return v == 0; }
inline bool is_zero(int16_t v)   { return v == 0; }
inline bool is_zero(uint16_t v)  { return v == 0; }
inline bool is_zero(uint32_t v)  { return v == 0; }
inline bool is_positive(float v) { return v > 1e-9f; }
inline bool is_negative(float v) { return v < -1e-9f; }
inline float constrain_float(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

template <typename T> inline T sq(T v) { return v * v; }

// AP_Airspeed.h 里 Airspeed_Calibration 的成员声明需要完整类型 (AUTOCAL 关闭时不会实例化)
struct Vector3f {
    float x, y, z;
    Vector3f() : x(0), y(0), z(0) {}
    Vector3f(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
};

struct Matrix3f {
    float m[3][3];
    Matrix3f() { for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) m[i][j] = 0; }
};
