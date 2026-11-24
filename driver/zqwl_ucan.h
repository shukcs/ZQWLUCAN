#ifndef __ZQWL_UCAN_H__
#define __ZQWL_UCAN_H__

#include <QObject>
#include <QList>
#include <QMap>
#include "zqwl_ucan_global.h"

class QSerialPort;
class ZQWL_UCANSHARED_EXPORT ZQWL_UCAN : public QObject
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
    ZQWL_UCAN(const QString &name, int index, QObject *p = nullptr);
    ~ZQWL_UCAN();

    uint8_t CountChannels()const;
    int id()const;
    QString portName()const;
    ///baut: 数据域波特率； jud: 仲裁域波特率
    bool OpenCan(uint16_t ch, int baut = Baut_500K, int jud = Baut_125K);
    void SendFrame(uint16_t ch, const CanFrame &fr);
    int ReadFrame(uint16_t ch, CanFrame *fr, int sz=1);
protected:
    void update(uint16_t id);
signals:
    void canFrameRecived(int ch);
private:
    void initSerialPort();
    void onWritePort(const QByteArray &arr);
    void parseRead();
    uint8_t toHfBaut(int baut, int jud);
    void openCan(int ch, int baut, int jud);
    void _sndBuff(const QByteArray &arr);
    
    void _prcsMsg(const uint8_t *msg);
    int _prcsData(const uint8_t *data, uint16_t len);
    int _prcsFrame(const uint8_t *data, uint16_t len);
    void close();
signals:
    void _writePort(const QByteArray &arr);
    void _openCan(int ch, int baut, int jud);
private:
    friend class UCanMgr;
    QString         m_portName;
    uint16_t        m_id;
    uint8_t         m_chs = 0;
    uint8_t         m_openChns = 0; ///bit0~~7: 0~7通道是否打开
    QSerialPort    *m_port = nullptr;
    QByteArray      m_recvs;
    QString         m_devName;
    QString         m_fwVer;
    QString         m_sn;
    QString         m_btVer;
    QMap<int, QList<CanFrame>>  m_readFrames;
};

Q_DECLARE_METATYPE(CanFrame)
#endif // ZQWL_UCAN_H
