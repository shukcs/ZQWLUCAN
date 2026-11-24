/*
 * hal_stub.h — AP_HAL 的最小替身 (时间 / 信号量 / panic)
 *
 * 真实固件里 AP_HAL::millis() 来自 HAL 层; 这里改成可外部赋值的仿真时钟,
 * 便于 demo 在不 sleep 的情况下满足驱动里的超时判断。
 */
#pragma once
#include <cstdint>
#include <mutex>

namespace hal_stub {
inline uint32_t &sim_millis() { static uint32_t v = 1000; return v; }
inline uint64_t &sim_micros() { static uint64_t v = 1000000; return v; }
}

namespace AP_HAL {
inline uint32_t millis() { return hal_stub::sim_millis(); }
inline uint32_t micros() { return (uint32_t)hal_stub::sim_micros(); }
inline uint64_t micros64() { return hal_stub::sim_micros(); }
inline void panic(const char *) {}
}

class HAL_Semaphore {
public:
    void take() { _m.lock(); }
    void give() { _m.unlock(); }
    void lock() { _m.lock(); }
    void unlock() { _m.unlock(); }
private:
    std::mutex _m;
};

// 真实宏: WITH_SEMAPHORE(sem) -> std::lock_guard<decltype(sem)>
#define WITH_SEMAPHORE(sem) std::lock_guard<HAL_Semaphore> _sm_guard(sem)
