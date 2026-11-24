#include "ucanMgr.h"
#include <QEvent>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QThread>
#include <QTimer>
#include <QVector>
#include <QMap>
#include <mutex>
#include <chrono>
//#include <unique_lock>
#include "zqwl_ucan.h"

struct  UCanDev {
    QString portName;
    uint16_t id;
    uint8_t chs;
    uint8_t openChns; ///bit0~~7: 0~7通道是否打开
    QSerialPort *port;
    QByteArray recvs;
    QString devName;
    QString fwVer;
    QString sn;
    QString btVer;
    QMap<int, QList<CanFrame> >  readFrames;
    uint8_t bff[72]; ///发送打包缓存
};

static std::mutex s_m;
static bool IsCanDlc(uint32_t len)
{
    static QVector<uint8_t> sAcptDlc = { 12, 16,20, 24, 32, 48, 64 };
    return (len < 9 || sAcptDlc.contains(len));
}

/****************************************************************
* UCanMgr
*****************************************************************/
UCanMgr::UCanMgr(QObject *p) : QObject (p), m_thread(new QThread)
{
    m_idTimer = startTimer(2000);
    moveToThread(m_thread);
    m_thread->start();
}

UCanMgr::~UCanMgr()
{
    for (auto ptr : m_canPorts)
    {
        delete ptr;
    }
    m_thread->quit();
    m_thread->deleteLater();
}

ZQWL_UCAN *UCanMgr::get(const QString &port)
{
    static uint16_t s_devId = 0;
    for (auto itr : m_canPorts)
    {
        if (itr->portName() == port)
            return itr;
    }
    auto ret = new ZQWL_UCAN(port, s_devId++, this);
    m_canPorts << ret;
    return ret;
}

ZQWL_UCAN * UCanMgr::get(uint16_t devId)const
{
    ZQWL_UCAN *ret = nullptr;
    s_m.lock();
    for (auto itr : m_canPorts)
    {
        if (itr->id() == devId)
        {
            ret = itr;
            break;
        }
    }
    s_m.unlock();
    return ret;
}

QList<uint16_t> UCanMgr::AllDevices()const
{
    QList<uint16_t> ret;
    s_m.lock();
    for (auto ptr : m_canPorts)
    {
        ret << ptr->id();
    }
    s_m.unlock();
    return ret;
}


ZQWL_UCAN * UCanMgr::GetZQWL_UCAN(uint8_t index) const
{
    return get(index);
}

UCanMgr & UCanMgr::Instance()
{
    static UCanMgr sIns(nullptr);
    return sIns;
}

void UCanMgr::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == m_idTimer)
    {
        updateCanDev();
        return;
    }
    QObject::timerEvent(e);
}

void UCanMgr::updateCanDev()
{
    s_m.lock();
    for (auto &itr : QSerialPortInfo::availablePorts())
    {
        auto vid = itr.vendorIdentifier();
        if (vid != 0x3562 || itr.isBusy())
            continue;
        if (auto canDev = get(itr.portName()))
            canDev->update(itr.productIdentifier());
    }
    s_m.unlock();
}
