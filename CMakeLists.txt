# ---------------------------------------------------------------------------
# ZQWL_UCAN — Qt5 + MinGW, Qt SerialPort ↔ DroneCAN ↔ AP_Airspeed demo
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)
project(ZQWL_UCAN LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Core SerialPort REQUIRED)

# ---------------------------------------------------------------------------
# 1) driver_zqwl  — ZQWL 串口转 CAN 硬件驱动 (STATIC 库)
# ---------------------------------------------------------------------------
add_library(driver_zqwl STATIC
    driver/zqwl_ucan.cpp
    driver/ucanMgr.cpp
    driver/zqwlucan.cpp
)

target_include_directories(driver_zqwl
    PUBLIC  ${CMAKE_SOURCE_DIR}/driver
)

# driver_zqwl 是 STATIC 库, 不需要 Q_DECL_EXPORT/IMPORT
target_compile_definitions(driver_zqwl PUBLIC ZQWL_UCAN_STATIC)

target_link_libraries(driver_zqwl PUBLIC Qt5::Core Qt5::SerialPort)

# ---------------------------------------------------------------------------
# 2) protocol_dronecan — DroneCAN 协议栈头文件 (INTERFACE 库, 全 header-only)
#    注意: can_frame.h 依赖 CanFrame 类型 (driver/zqwl_ucan_global.h),
#    所以 INTERFACE include 里必须同时暴露 driver/ 路径
# ---------------------------------------------------------------------------
add_library(protocol_dronecan INTERFACE)
target_include_directories(protocol_dronecan
    INTERFACE ${CMAKE_SOURCE_DIR}/protocol
    INTERFACE ${CMAKE_SOURCE_DIR}/driver
)
target_link_libraries(protocol_dronecan INTERFACE Qt5::Core)

# ---------------------------------------------------------------------------
# 3) hal_stubs — ArduPilot HAL/AP_* 最小替身 (INTERFACE 库)
# ---------------------------------------------------------------------------
add_library(hal_stubs INTERFACE)
target_include_directories(hal_stubs
    INTERFACE ${CMAKE_SOURCE_DIR}/hal_stubs
)

# ---------------------------------------------------------------------------
# 4) airspeed_backend — AP_Airspeed 应用层 (STATIC 库)
# ---------------------------------------------------------------------------
add_library(airspeed_backend STATIC
    src_real/AP_Airspeed.cpp
    src_real/AP_Airspeed_Backend.cpp
    src_real/AP_Airspeed_DroneCAN.cpp
    src_real/AP_Airspeed_Params.cpp
)

target_include_directories(airspeed_backend
    PUBLIC  ${CMAKE_SOURCE_DIR}/src_real
)

target_link_libraries(airspeed_backend
    PUBLIC  protocol_dronecan
    PUBLIC  hal_stubs
)

# ---------------------------------------------------------------------------
# 5) airspeed_demo — 可执行 demo
# ---------------------------------------------------------------------------
add_executable(airspeed_demo
    demo/main.cpp
    demo/can_iface.cpp
)

target_include_directories(airspeed_demo
    PRIVATE ${CMAKE_SOURCE_DIR}/demo
)

target_link_libraries(airspeed_demo
    PRIVATE driver_zqwl
    PRIVATE protocol_dronecan
    PRIVATE hal_stubs
    PRIVATE airspeed_backend
    PRIVATE Qt5::Core
    PRIVATE Qt5::SerialPort
)

# ---------------------------------------------------------------------------
# VS / QtCreator source_group — 让 IDE 按文件夹显示
# ---------------------------------------------------------------------------
if(MSVC)
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)
endif()
