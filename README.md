# ZQWL_UCAN — DroneCAN 空速计对接 Demo

Qt5 + MinGW 工程，演示 **ZQWL 串口转 CAN 驱动** 对接 **DroneCAN (UAVCAN) 协议栈**，
驱动 **ArduPilot AP_Airspeed** 前端读取 OneASP 品牌 DroneCAN 空速计。

工程真实还原 ArduPilot 固件里 AP_DroneCAN → AP_Airspeed 的调用链路，
CAN 帧收发、重组、CRC 校验、订阅分发全部在 demo 内闭环完成。

---

## 1. 构建环境

| 项 | 值 |
|---|---|
| 编译器 | MinGW GCC 5.3.0 (x86) |
| Qt | Qt 5.11.3 (mingw53_32) |
| CMake | ≥ 3.16 |
| 构建系统 | CMake + MinGW Makefiles |

## 2. 构建步骤

```powershell
# 环境变量 (一次性)
$env:Path = "D:/Qt/Qt5.11.3/Tools/mingw530_32/bin;" + $env:Path

# Configure — 替换成本机 Qt / MinGW 路径
cmake -S . -B build -G "MinGW Makefiles" `
    -DCMAKE_PREFIX_PATH="D:/Qt/Qt5.11.3/5.11.3/mingw53_32" `
    -DCMAKE_CXX_COMPILER="D:/Qt/Qt5.11.3/Tools/mingw530_32/bin/g++.exe" `
    -DCMAKE_MAKE_PROGRAM="D:/Qt/Qt5.11.3/Tools/mingw530_32/bin/mingw32-make.exe"

# Build
cmake --build build

# 产物
build/airspeed_demo.exe    # 可执行 demo
build/libdriver_zqwl.a     # ZQWL 驱动静态库
build/libairspeed_backend.a # AP_Airspeed 静态库
```

> **VS / QtCreator 注意**：顶层 `CMakeLists.txt` 已启用 `USE_FOLDERS ON`，
> 在 VS Solution Explorer 和 QtCreator Projects 面板里 target 会自动归组。

---

## 3. 目录结构

按职责分 4 层，见下图：

```
ZQWL_UCAN/
│
├── CMakeLists.txt                 # 顶层构建文件
│
├── driver/                        # 硬件驱动层 — ZQWL 串口转 CAN
│   ├── zqwl_ucan.h / .cpp         #   ZQWL_UCAN: 单设备管理 (QObject)
│   ├── zqwl_ucan_global.h         #   CanFrame 结构体 + 导出宏
│   ├── ucanMgr.h / .cpp           #   UCanMgr: 多设备单例管理器 (QObject)
│   └── zqwlucan.h / .cpp          #   C 风格 API (ZQWLInit / ZQWLOpen)
│
├── protocol/                      # DroneCAN 协议栈层 (header-only)
│   ├── can_frame.h                #   帧辅助 + RawAirData 编解码
│   ├── canard_stub.h              #   订阅注册表 + 分发器 (libcanard 替身)
│   └── ap_dronecan_stub.h         #   AP_DroneCAN 最小替身 (只存索引/通道)
│
├── hal_stubs/                     # ArduPilot HAL / AP_* 最小替身 (header-only)
│   ├── hal_stub.h                 #   HAL_Semaphore / hal_stub::sim_millis
│   ├── stub_config.h              #   编译时开关
│   ├── ap_common_stub.h           #   AP_FixedWing / AP 单例宏
│   ├── ap_math_stub.h             #   Vector3f / Matrix3f / AP_Math
│   ├── ap_param_stub.h            #   AP_Int8 / AP_Float / AP_Param::GroupInfo
│   └── gcs_stub.h                 #   GCS 消息占位
│
├── src_real/                      # ArduPilot 应用层 — AP_Airspeed 系列
│   ├── AP_Airspeed.h / .cpp       #   空速计前端 (ArduPilot 风格)
│   ├── AP_Airspeed_Backend.h/.cpp #   后端抽象基类
│   ├── AP_Airspeed_DroneCAN.h/.cpp#   DroneCAN 后端实现
│   ├── AP_Airspeed_Params.cpp     #   参数表 (ARSPD_USE / ARSPD2_TYPE 等)
│   └── AP_Airspeed_config.h       #   打开/关闭各 backend 的宏 (全局入口)
│
└── demo/                          # 可执行入口
    ├── main.cpp                   #   程序入口 (原根目录 main.cpp)
    ├── can_iface.h                #   DroneCANInterface PIMPL 对外接口
    ├── can_iface.cpp              #   收发线程 + 传输重组 + DNA 分配 + NodeStatus
    └── log.h                      #   日志占位
```

---

## 4. CMake Target 依赖

```
airspeed_demo (EXECUTABLE)
  │
  ├── driver_zqwl (STATIC)
  │     └── Qt5::Core, Qt5::SerialPort
  │
  ├── protocol_dronecan (INTERFACE, header-only)
  │     └── Qt5::Core    (暴露 driver/ 给 can_frame.h 用 CanFrame)
  │
  ├── hal_stubs (INTERFACE, header-only)
  │
  └── airspeed_backend (STATIC)
        ├── protocol_dronecan
        └── hal_stubs
```

> 头文件搜索路径统一由 `target_include_directories` 注入，
> 源码里一律写 `#include "can_frame.h"` 这种短名，不写 `#include "stub/xxx.h"` 或 `#include "../stub/xxx.h"`。

---

## 5. 数据流图

```
  ┌──────────────┐   signal canFrameRecived(ch)   ┌───────────────────────────┐
  │  ZQWL_UCAN   │ ──────────────────────────────►│  DroneCANInterface::Impl  │
  │  (驱动线程)   │                                │  (主线程, 槽函数)          │
  └──────────────┘                                └──────────┬────────────────┘
      │                                                     │ ReadFrame(ch, fr[2])
      │                                                     ▼
      │                                           CanAssembler.add(frame)
      │                                                  │
      │                                                  ▼  (凑够 START...END)
      │                                           decode_transfer()
      │                                                  │
      │                                                  ▼
      │                                           canard_dispatch(
      │                                              driver_index,
      │                                              transfer,
      │                                              data_type_id,
      │                                              payload)
      │                                                  │
      │                                                  ▼
      │                           AP_Airspeed_DroneCAN::handle_airspeed()
      │                           (static 订阅回调)       │
      │                                                  ▼
      │                           airspeed.update() / get_airspeed()
      │                                                  │
      │                                                  ▼
      │                                           main 打印 / 业务消费
      │
      │ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─
      │  反向: 主机发 NodeStatus / DNA 响应
      │
      │   can_iface.cpp 里的 QTimer::timeout
      │        │
      │        ▼
      │   sendTransfer(NODE_STATUS_ID, payload)
      │        │  ── 多帧 / 单帧 encode_transfer ──► CanFrame[]
      │        ▼
      │   dev->SendFrame(ch, frame)
      │        │
      ▼
  UCanMgr::Instance().GetZQWL_UCAN(idx)
```

---

## 6. 关键接口契约

### 6.1 CanFrame — 硬件帧

```cpp
// driver/zqwl_ucan_global.h
typedef struct CanFrame {
    uint32_t id : 29;      // CAN ID (扩展帧 29-bit)
    bool bExt   : 1;       // true = 扩展帧
    bool bRemote: 1;       // false = 数据帧
    uint8_t dlc;           // 数据长度 (实际用 ≤ 8)
    uint8_t data[64];
} CanFrame;
```

### 6.2 DroneCAN 标准帧 — ID 布局

```cpp
// protocol/can_frame.h
namespace dronecan::transport {
    // 生成 CAN ID
    uint32_t make_can_id(
        uint8_t  source_node_id,   // 1..127 (0 = anonymous)
        uint16_t data_type_id,     // ≤ 0xFFFF
        uint8_t  priority          // 0..31
    );

    // 从 CAN ID 解包
    uint8_t  can_id_source_node(uint32_t id);  // bits0..6
    uint16_t can_id_data_type(uint32_t id);    // bits8..23 (16-bit)
    uint8_t  can_id_priority(uint32_t id);     // bits24..28
}
```

### 6.3 订阅机制 — libcanard 替身

```cpp
// protocol/canard_stub.h
class Canard {
public:
    template<typename T, typename msgtype>
    static void *allocate_sub_arg_callback(
        T *arg,
        void (*cb)(T *, const CanardRxTransfer &, const msgtype &),
        uint8_t index
    );
};

// 协议栈收到完整传输后调用
inline void canard_dispatch(
    uint8_t driver_index,
    const CanardRxTransfer &transfer,
    uint16_t data_type_id,
    const uint8_t *payload
);
```

### 6.4 DroneCANInterface — 对 main 的 PIMPL 接口

```cpp
// demo/can_iface.h
class DroneCANInterface {
public:
    explicit DroneCANInterface(uint8_t driver_index);
    void  start();          // 绑定驱动信号 + 启动 NodeStatus 定时
    void  stop();
    AP_DroneCAN *dronecan();    // 给 AP_Airspeed_DroneCAN::subscribe_msgs 用
    uint32_t rx_transfers();    // 已分发完整传输计数
};
```

### 6.5 RawAirData — DroneCAN 消息 (DSDL 1027)

```
字段布局 (共 17 字节):
 [0]      flags                        uint8
 [1..4]   static_pressure              float32
 [5..8]   differential_pressure       float32
 [9..10]  static_pressure_sensor_temperature  float16
[11..12]  differential_pressure_sensor_temperature float16
[13..14]  static_air_temperature       float16
[15..16]  pitot_temperature            float16
```

> `RawAirData` 的类型 ID = 1027，CRC 覆盖载荷本身（不含 CRC 字段和 tail byte），
> 多项式 0x1021、初值 0xFFFF、小端存放 — 与 libcanard `crc_add()` 同构。

---

## 7. 已知问题 / 约束

| # | 描述 | 状态 |
|---|---|---|
| K1 | **匿名帧数据类型 ID 解析**：DroneCAN 规范里 source_node_id=0 时 CAN ID bits8..9 只有 2-bit 有效，但 `can_id_data_type()` 会取完整 16-bit — 需要在上层加条件判断 | TODO |
| K2 | **CRC 未校验**：`CanAssembler::decode_transfer` 里 CRC 只捕获不比对，建议末帧时做 `crc_add(0xFFFF, payload, len) == crc_received` | TODO |
| K3 | **多帧重组 buff 越界风险**：`payload[64]` 数组硬上限，单帧 DroneCAN 理论最大传输 512 字节（72 帧），当前实现只适用于 RawAirData 这种小载荷 | 已知 |
| K4 | **zqwl_ucan.cpp 的位运算优先级**：`data[1] >> 2 & 6` 应为 `(data[1] >> 2) & 0x06`，当前能跑但有潜在误读 | TODO |
| K5 | **线程安全发送**：`SendFrame` 从主线程直接调用时，内部栈上 `bff` 经 `emit _writePort(bff, len)` 排队到驱动线程可能悬空，需 `QMetaObject::invokeMethod` 包裹 | TODO |
| K6 | **OneASP 初始化三帧硬编码**：`demo/can_iface.cpp` 里 `rspAlloc` 和 DNA 相关逻辑只覆盖标准 DroneCAN，厂商私有初始化流程未接入 | TODO |
| K7 | **`AP_Airspeed_DroneCAN` 后端字段未初始化**：`_pressure` / `_temperature` / `_last_sample_time_ms` / `_have_temperature` 在构造函数里没置零 | 已修复（构造函数里加了 = 0 初始化）|

> **变更约束**：修改 `can_frame.h` 或 `canard_stub.h` 的数据结构/接口时，
> 必须同步检查 `can_iface.cpp` 里的 `CanAssembler` 实现和 `AP_Airspeed_DroneCAN.cpp` 里的订阅回调，
> 避免协议栈层和业务层定义漂移。

---

## 8. 原 qmake → CMake 迁移记录

| 原文件 | 新位置 | 变更 |
|---|---|---|
| `zqwl_ucan.h / .cpp` | `driver/` | 成员从匿名 struct 提升为类成员（修 C++17 编译） |
| `zqwl_ucan_global.h` | `driver/` | 加 `ZQWL_UCAN_STATIC` 分支（STATIC 库不需要 Q_DECL_EXPORT） |
| `ucanMgr.h / .cpp` | `driver/` | 无改动 |
| `zqwlucan.h / .cpp` | `driver/` | 无改动 |
| `stub/can_frame.h` | `protocol/` | include 路径改短名 |
| `stub/canard_stub.h` | `protocol/` | 无改动 |
| `stub/ap_dronecan_stub.h` | `protocol/` | 无改动 |
| `stub/hal_stub.h` 等 6 文件 | `hal_stubs/` | include 路径改短名 |
| 根目录 `main.cpp` | `demo/main.cpp` | 覆盖了原 demo/main.cpp |
| `ZQWL_UCAN.pro` | — | 已被 `CMakeLists.txt` 替代，用户手动删除 |

---

## 9. VS / QtCreator 使用

### QtCreator
1. File → Open → `CMakeLists.txt`
2. 选择 Kit（已自动识别 Qt 5.11.3 / MinGW）
3. Configure → Build → Run

### Visual Studio
1. File → Open → Folder → 选项目根目录
2. 首次打开 CMake Tools 会自动 configure
3. 顶部 target 下拉选 `airspeed_demo (executable)`

---

## 10. 后续改进方向

1. **OneASP 初始化协议**：空速计发 `CAN 0x18D9F100 DLC=8 data=01 3E 00 46 00 08 51 C0` 请求地址，主机需回三帧 CAN 0x1000017F（载荷含 UID + 序列号 "509694" + 分配地址 0xFA）。目前 demo 里没接入。
2. **标准 DroneCAN DNA**：`rspAlloc` 实现了 UID 6+6+4 拼接、ALLOCATOR 一致校验、125 向下分配，但 `stub_config.h` 里没打开，空速计发的匿名帧不会被真正处理。
3. **CRC 校验补全**：见 K2。
4. **payload 动态缓冲**：见 K3。
5. **多通道支持**：目前 `DroneCANInterface` 硬编码 channel=1，需改成遍历 `dev->CountChannels()`。
6. **日志框架**：`demo/log.h` 是空的，加 `qInstallMessageHandler` 把 qDebug 同步到文件 / 控制台。
