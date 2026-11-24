#ifndef __UCanMgr_H__
#define __UCanMgr_H__

#include <QObject>
#include <QList>
#include "zqwl_ucan_global.h"


class ZQWL_UCAN;
class ZQWL_UCANSHARED_EXPORT UCanMgr : public QObject
{
    Q_OBJECT
public:
    enum BuatRate {
        Baut_5M =   5000000,
        Baut_4M =   4000000,
        Baut_2M =   2000000,
        Baut_1M =   1000000,
        Baut_800K = 800000,
        Baut_500K = 500000,
        Baut_400K = 400000,
        Baut_250K = 250000,
        Baut_200K = 200000,
        Baut_125K = 125000,
        Baut_100K = 100000,
        Baut_50K =  50000,
        Baut_40K =  40000,
        Baut_25K =  25000,
        Baut_20K =  20000,
        Baut_15K =  15000,
        Baut_10K =  10000,
        Baut_5K =   5000,
    };
public:
    ~UCanMgr();

    static UCanMgr &Instance();
    QList<uint16_t> AllDevices()const;
    ZQWL_UCAN *GetZQWL_UCAN(uint8_t index)const;
protected:
    void timerEvent(QTimerEvent *e)override;
    void updateCanDev();
private:
    UCanMgr(QObject *p = nullptr);
    ZQWL_UCAN *get(const QString &portName);
    ZQWL_UCAN *get(uint16_t devId)const;
private:
    int             m_idTimer;
    QThread         *m_thread;
    QList<ZQWL_UCAN*> m_canPorts;
};
#endif // ZQWL_UCAN_H
