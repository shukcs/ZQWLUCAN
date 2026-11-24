/*
 * demo/log.h — 线程安全的输出 (TX 线程与 main 线程共用同一把锁)
 */
#pragma once
#include <cstdio>
#include <cstdarg>
#include <mutex>

inline std::mutex &demo_log_mutex()
{
    static std::mutex m;
    return m;
}

inline void log_line(const char *fmt, ...)
{
    std::lock_guard<std::mutex> lk(demo_log_mutex());
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    fflush(stdout);
}
