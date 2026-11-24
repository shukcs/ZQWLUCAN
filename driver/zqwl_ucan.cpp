#include "zqwl_ucan.h"
#include <QEvent>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QThread>
#include <QTimer>
#include <QVector>
#include <QMap>
#include <mutex>
#include <chrono>
#include <cstring>
//#include <unique_lock>

static std::mutex s_m;
static std::mutex s_open;
std::condition_variable s_cv;
/*
* Baud = 60000000 / ((SJW+1) + (TSEG1+1) + (TSEG2+1)) / BRP
*      = 60000000 / (SJW + TSEG1 + TSEG2 + 3) / BRP
*
* 约束:
*   0 <= SJW   <= 3
*   0 <= TSEG1 <= 7
*   0 <= TSEG2 <= 15   (TSEG2 < 15)
*   1 <= BRP   <= 1023 (BRP < 1024, BRP=0 会除零, 无效)
*
* 已知 Baud 求一组 (SJW, TSEG1, TSEG2, BRP); 无精确解返回 false
*/
bool customFormBaud(uint32_t Baud, uint8_t *buf , uint16_t &BRP)
{
    if (Baud == 0)
        return false;

    const uint32_t clk = 60000000u;
    if (clk % Baud != 0)
        return false;                    // Baud 必须能整除 60MHz, 否则无整数解

    const uint32_t total = clk / Baud;   // total = N * BRP, N = SJW+TSEG1+TSEG2+3
    // N 越大时间量子越多, 采样点位置更合理, 所以从大到小枚举
    for (uint32_t n = 28; n >= 3; n--)
    {
        const uint32_t brp = total / n;
        if (brp*n != total || brp < 1)
            continue;

        if (brp > 1023)
            return false;

        BRP = brp;
        if (buf)
        {
            // 分解 rem = N-3 = SJW + TSEG1 + TSEG2, 优先把 TSEG1 配大
            uint32_t rem = n - 3;
            buf[2] = rem > 7 ? 7 : rem;
            rem -= buf[2];
            buf[1] = rem > 15 ? 15 : rem;
            rem -= buf[1];
            buf[0] = rem;
        }
        return true;
    }
    return false;
}

static bool IsCanDlc(uint32_t len)
{
    static QVector<uint8_t> sAcptDlc = { 12, 16,20, 24, 32, 48, 64 };
    return (len < 9 || sAcptDlc.contains(len));
}

/*******************************************************************************
* ZQWL_UCAN
*******************************************************************************/
ZQWL_UCAN::ZQWL_UCAN(const QString &name, int index, QObject *p) : QObject (p)
, m_portName(name), m_id(index)
{
    if (p && p->thread())
        moveToThread(p->thread());
    qRegisterMetaType<CanFrame>();
    connect(this, &ZQWL_UCAN::_writePort, this, &ZQWL_UCAN::onWritePort);
    connect(this, &ZQWL_UCAN::_openCan, this, &ZQWL_UCAN::openCan);
}

ZQWL_UCAN::~ZQWL_UCAN()
{
    close();
    delete m_port;
}

uint8_t ZQWL_UCAN::CountChannels() const
{
    return m_chs;
}

int ZQWL_UCAN::id() const
{
    return m_id;
}

QString ZQWL_UCAN::portName() const
{
    return m_portName;
}

void ZQWL_UCAN::initSerialPort()
{
    if (!m_port)
    {
        auto sp = new QSerialPort(m_portName);
        sp->setBaudRate(6000000);
        sp->setStopBits(QSerialPort::OneStop);
        sp->setParity(QSerialPort::NoParity);
        sp->setDataBits(QSerialPort::Data8);
        if (sp->open(QIODevice::ReadWrite))
        {
            m_port = sp;
            connect(sp, &QSerialPort::readyRead, this, &ZQWL_UCAN::parseRead);
            connect(sp, &QSerialPort::errorOccurred, this, [=](QSerialPort::SerialPortError err) {
                if (QSerialPort::NoError != err)
                {
                    m_port->deleteLater();
                    m_port = nullptr;
                }
            });

            moveToThread(thread());
            return;
        }
        delete sp;
    }
}

void ZQWL_UCAN::onWritePort(const QByteArray &arr)
{
    if (m_port && arr.size()>0)
    {
        _sndBuff(arr);
    }
}

void ZQWL_UCAN::parseRead()
{
    if (m_port)
    {
        m_recvs += m_port->readAll();
        auto beg = (const uint8_t*)m_recvs.data();
        auto count = m_recvs.size();
        int i = 0;
        for (; i<count - 1; )
        {
            auto header = beg[i];
            if (0x49 == header && 0x3B == beg[i + 1])
            {
                if (count - i < 22)///帧不完整
                    break;

                if (beg[i + 20] == 0x45 && beg[i + 21] == 0x2E)
                {
                    _prcsMsg(beg+i+2);
                    i += 22;
                    continue;
                }
            }
            else if (0x5A == header)
            {
                auto nPrc = _prcsData(beg + i + 1, count - i - 1);
                if (nPrc==0) ///帧不完整
                    break;

                if (nPrc > 0)
                {
                    i += nPrc;
                    continue;
                }
            }
            ++i;
        }
        if (i > 0)
            m_recvs.remove(0, i);
    }
}

uint8_t ZQWL_UCAN::toHfBaut(int baut, int jud)
{
    uint8_t ret=0;
    static QVector<uint32_t> sHfJud = { Baut_5M, Baut_4M, Baut_2M, Baut_1M, Baut_800K, 
                Baut_500K,Baut_400K,Baut_250K, Baut_200K, Baut_125K, Baut_100K };
    auto idx = sHfJud.indexOf(baut);
    if (idx < 0)
        return 0xff;
    ret = idx;

    static QVector<uint32_t> sHfBaut = { Baut_1M, Baut_800K, Baut_500K,Baut_400K,Baut_250K, Baut_200K
        , Baut_125K, Baut_100K, Baut_50K, Baut_40K, Baut_25K,Baut_20K, Baut_15K,Baut_10K, Baut_5K };
    
    idx = sHfBaut.indexOf(jud);
    if (idx < 0)
        return 0xff;

    return ret | (idx<<4);
}

void ZQWL_UCAN::openCan(int ch, int baut, int jud)
{
    uint16_t BRP;
    if (!customFormBaud(baut, nullptr, BRP) || !customFormBaud(jud, nullptr, BRP))
        return;

    if (ch < m_chs)
    {
        initSerialPort();
        if (m_port)
        {
            uint8_t buff[22] = { 0x49, 0x3B, 0x42, 0x57, (uint8_t)ch, 0, toHfBaut(baut, jud), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x45, 0x2E };
            if (buff[6] == 0xff)
            {
                buff[5] = 1;
                buff[6] = 0;
                uint16_t brp;
                customFormBaud(baut, buff + 7, brp);
                buff[10] = brp >> 8;
                buff[11] = brp & 0xff;
                customFormBaud(baut, buff + 12, brp);
                buff[15] = brp >> 8;
                buff[16] = brp & 0xff;
            }
            _sndBuff(QByteArray((char*)buff, sizeof(buff)));
        }
    }
}

void ZQWL_UCAN::_sndBuff(const QByteArray &arr)
{
    if (m_port)
        m_port->write(arr);
}

void ZQWL_UCAN::_prcsMsg(const uint8_t *msg)
{
    switch (msg[0])
    {
    case 0x40:
        {
            m_devName = (const char*)msg + 2;
            m_fwVer = std::string((const char*)msg + 14, 4).c_str();
            uint8_t buf[22] = { 0x49, 0x3B, 0x41, 0x52, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x45, 0x2E };
            _sndBuff(QByteArray((char*)buf, sizeof(buf)));
        }
        break;
    case 0x41:
        m_sn = QByteArray((const char*)msg + 2, 8).toHex();
        break;
    case 0x42:
        if (0x57==msg[1]){
            if (msg[2] < 4)
            {
                uint8_t buf[22] = { 0x49, 0x3B, 0x44, 0x57, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x45, 0x2E };
                auto flag = m_openChns;
                for (int i = 0; i < 4; ++i)
                {
                    if (flag & (1 << i))
                        buf[6 + i] = 1;
                }
                buf[6 + msg[2]] = 1;
                _sndBuff(QByteArray((char*)buf, sizeof(buf)));
            }        
        }
        break;
    case 0x44:
        if (0x57 == msg[1])
        {
            m_openChns = 0;
            for (int i = 0; i < 4; ++i)
            {
                if (msg[4 + i])
                    m_openChns |= 1 << i;

            }
            std::unique_lock<std::mutex> lk(s_open);
            s_cv.notify_all();
        }
        break;
    }
}

int ZQWL_UCAN::_prcsData(const uint8_t *data, uint16_t len)
{
    if (0xff == data[0] && m_chs <= 2)
    {
        if (len >= 16)
            return data[15] == 0xA5 ? 17 : -1;

        return 0;
    }

    if (0xfe == data[0] && m_chs <= 2)
    {
        if (len >= 31)
            return data[15] == 0xA5 ? 17 : -1;

        return 0;
    }

    return _prcsFrame(data, len);
}

int ZQWL_UCAN::_prcsFrame(const uint8_t *data, uint16_t len)
{
    static QVector<uint8_t> sAcptDlc = { 12, 16,20, 24, 32, 48, 64 };
    auto dlc = data[0] & 0x7f;
    if (dlc < 9 || sAcptDlc.contains(dlc))
    {
        auto lenP = 8 + dlc;
        if (len<lenP-1)
            return 0;

        if (data[lenP - 2] == 0xA5)
        {
            uint32_t id = (data[2] << 24) + (data[3] << 16) + (data[4] << 8) + data[5];
            uint8_t ch = ((data[1] >> 2) & 6) + (data[0] >> 7);
            s_m.lock();
            auto &frames = m_readFrames[ch];
            frames << CanFrame{ id & 0x1fffffff };
            s_m.unlock();
            CanFrame &fr = frames.last();
            fr.bExt = (data[1] & 4) != 0;
            fr.bRemote = (data[1] & 2) != 0;
            fr.dlc = dlc;
            memcpy(fr.data, data + 6, dlc);
            emit canFrameRecived(ch);
            return dlc + 7;
        }
    } 
    return -1;
}

void ZQWL_UCAN::close()
{
    if (!m_port) return;
    uint8_t buff[22] = { 0x49, 0x3B, 0x44, 0x57, 0, 01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x45, 0x2E };
    _sndBuff(QByteArray((char*)buff, 22));
    m_port->close();
}

bool ZQWL_UCAN::OpenCan(uint16_t ch, int baut /*= Baur_500K*/, int jud /*= Baur_125K*/)
{
    emit _openCan(ch, baut, jud);
    ///异线程同步，与window WaitForSingleObject一样效果
    std::unique_lock<std::mutex> lk(s_open);
    if (std::cv_status::no_timeout == s_cv.wait_for(lk, std::chrono::seconds(5)))
        return (m_openChns & (1 << ch)) != 0;

    return false;
}

void ZQWL_UCAN::SendFrame(uint16_t ch, const CanFrame &fr)
{
    auto lenSnd = fr.dlc + 8;
    const uint8_t b1 = (uint8_t)(((ch & 1) << 7) | (fr.dlc & 0x0F));
    const uint8_t b2 = (uint8_t)((((ch >> 1) & 0x03) << 3) | (fr.bExt ? 0x04 : 0));  // 扩展帧、数据帧、正常发送
    uint8_t bff[72] = {0x5A, b1, b2};

    bff[3] = (uint8_t)(fr.id >> 24);
    bff[4] = (uint8_t)(fr.id >> 16);
    bff[5] = (uint8_t)(fr.id >> 8);
    bff[6] = (uint8_t)fr.id;
    std::memcpy(bff+7, fr.data, fr.dlc);
    bff[7 + fr.dlc] = 0xA5;

    emit _writePort(QByteArray((char*)bff, lenSnd));
}

int ZQWL_UCAN::ReadFrame(uint16_t ch, CanFrame *fr, int sz)
{
    if (!fr || sz < 1)
        return 0;

    int ret = 0;
    s_m.lock();
    auto itr = m_readFrames.find(ch);
    if (itr != m_readFrames.end())
    {
        for (; ret < sz && !itr->isEmpty(); ++ret)
        {
            fr[ret] = itr->takeFirst();
        }
    }
    s_m.unlock();

    return ret;
}

void ZQWL_UCAN::update(uint16_t pid)
{
    switch (pid)
    {
    case 0x0100:
    case 0x0102:
        m_chs = 2; break;
    case 0x0101:
    case 0x0103:
        m_chs = 1; break;
    case 0x0104:
    case 0x0105:
        m_chs = 4; break;
    default:
        break;
    }
}
