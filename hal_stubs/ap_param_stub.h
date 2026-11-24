/*
 * ap_param_stub.h — AP_Param 的最小替身
 *
 * 只保留参数对象的值语义 (get/set/set_default/...)，不做存储/加载。
 * AP_GROUPINFO 只用于让源码里的参数表能编译通过。
 */
#pragma once
#include <cstdint>
#include <cstddef>

#define AP_PARAM_FLAG_ENABLE 1
#define AP_PARAM_FLAG_INTERNAL_USE_ONLY 2

class AP_Param {
public:
    struct GroupInfo {
        const char *name;
        int16_t idx;
        void *ptr;
        int type;
        float def;
        uint32_t flags;
    };

    static void setup_object_defaults(const void *, const GroupInfo *) {}
    static bool load_all() { return true; }
    static bool save_all() { return true; }
};

#define AP_GROUPEND { nullptr, 0, nullptr, 0, 0.0f, 0 }
#define AP_GROUPINFO(name, idx, cls, member, def) \
    { name, idx, nullptr, 0, (float)(def), 0 }
#define AP_GROUPINFO_FLAGS(name, idx, cls, member, def, flags) \
    { name, idx, nullptr, 0, (float)(def), (uint32_t)(flags) }
#define AP_GROUPINFO_FRAME(name, idx, cls, member, def, frame) \
    { name, idx, nullptr, 0, (float)(def), 0 }
#define AP_SUBGROUPINFO(member, name, idx, cls, subcls) \
    { name, idx, nullptr, 0, 0.0f, 0 }

template <typename T>
class AP_Param_Base {
public:
    T get() const { return _v; }
    void set(T v) { _v = v; _configured = true; }
    void set_and_save(T v) { _v = v; _configured = true; }
    void set_and_notify(T v) { _v = v; _configured = true; }
    void set_default(T v) { _v = v; }
    bool configured() const { return _configured; }
    void save() {}
    operator T() const { return _v; }
protected:
    T _v = T(0);
    bool _configured = false;
};

class AP_Float : public AP_Param_Base<float> {};
class AP_Int8  : public AP_Param_Base<int8_t> {};
class AP_Int16 : public AP_Param_Base<int16_t> {};
class AP_Int32 : public AP_Param_Base<int32_t> {};

template <typename E>
class AP_Enum : public AP_Param_Base<int8_t> {
public:
    E get() const { return (E)this->_v; }
    void set(E v) { this->_v = (int8_t)v; this->_configured = true; }
    void set_and_save(E v) { this->_v = (int8_t)v; this->_configured = true; }
    void set_default(E v) { this->_v = (int8_t)v; }
};
