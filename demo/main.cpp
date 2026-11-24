#include "zqwl_ucan.h"
#include "ucanMgr.h"
#include <QCoreApplication>
#include <QTimer>
#include <QThread>
#include "can_iface.h"
#include "AP_Airspeed_DroneCAN.h"

#pragma execution_character_set("utf-8")

class DemoAirspeedBackend : public AP_Airspeed_DroneCAN {
public:
    using AP_Airspeed_DroneCAN::AP_Airspeed_DroneCAN;
    void set_use_param(int8_t u) { set_use(u); }
    void set_type_param(int8_t t) { set_type(t); }
};
int opened = 0;
DroneCANInterface can_bus(0);
// 3) 空速计 Frontend
AP_Airspeed airspeed;

static void print_airspeed(const AP_Airspeed &airspeed, uint32_t rx_transfers)
{
    printf("\n--- AP_Airspeed::update()  (CAN rx transfers=%u) ---\n", (unsigned)rx_transfers);

    for (uint8_t i = 0; i < airspeed.get_num_sensors(); i++) {
        float temperature = 0.0f;
        const bool have_temp = airspeed.get_temperature(i, temperature);

        printf("  instance %u:\n", (unsigned)i);
        printf("    ARSPD_RATIO        = %.2f\n", airspeed.get_airspeed_ratio(i));
        printf("    differential press = %.2f Pa\n", airspeed.get_differential_pressure(i));
        printf("    corrected pressure = %.2f Pa\n", airspeed.get_corrected_pressure(i));
        printf("    raw airspeed       = %.2f m/s\n", airspeed.get_raw_airspeed(i));
        printf("    filtered airspeed  = %.2f m/s\n", airspeed.get_airspeed(i));
        if (have_temp) {
            printf("    temperature        = %.2f C\n", temperature);
        }
        else {
            printf("    temperature        = <unavailable>\n");
        }
        printf("    enabled=%s use=%s healthy=%s\n",
            airspeed.enabled(i) ? "yes" : "no",
            airspeed.use(i) ? "yes" : "no",
            airspeed.healthy(i) ? "yes" : "no");
    }

    printf("  primary sensor = %u, all_healthy = %s\n",
        (unsigned)airspeed.get_primary(),
        airspeed.all_healthy() ? "yes" : "no");
}

int main(int argc, char *argv[])
{
    // 2) 空速计驱动向协议栈订阅 RawAirData
    if (!AP_Airspeed_DroneCAN::subscribe_msgs(can_bus.dronecan())) {
        printf("DroneCAN RawAirData subscription failed\n");
        return 1;
    }

    {
        DemoAirspeedBackend setter0(airspeed, 0);
        setter0.set_use_param(1);

        DemoAirspeedBackend setter1(airspeed, 1);
        setter1.set_type_param((int8_t)AP_Airspeed::TYPE_UAVCAN);
        setter1.set_use_param(1);
    }

    airspeed.init();

    QCoreApplication a(argc, argv);
    QTimer t;
    t.setInterval(1000);
    QObject::connect(&t, &QTimer::timeout, &a, [=] {
        auto ls = UCanMgr::Instance().AllDevices();
        if (ls.isEmpty())
            return;

        auto dev = UCanMgr::Instance().GetZQWL_UCAN(ls.first());
        if (opened < dev->CountChannels())
        {
            for (int i = 0; i < dev->CountChannels(); ++i)
            {
                if (!dev->OpenCan(i, ZQWL_UCAN::Baut_1M, ZQWL_UCAN::Baut_1M))
                {
                    opened = 0;
                    return;
                }
                opened++;
            }
            can_bus.start();
            return;
        }
        if (airspeed.get_num_sensors() == 0)
            airspeed.init();
        airspeed.update();
        print_airspeed(airspeed, can_bus.rx_transfers());
    });
    t.start();

    return a.exec();
}