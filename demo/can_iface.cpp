/*
 * demo/can_iface.cpp — DroneCAN CAN 接口层实现
 *
 * 一个线程内完成:
 *   TX: can_sim 生成标准 CAN 帧 -> CanThread 队列
 *   RX: 从队列取帧 -> 按源节点组装成一次传输 -> CRC 校验 -> 反序列化
 *       -> canard_dispatch() 按 (驱动索引, data type id) 调用订阅回调
 *
 * 这与真实固件里 AP_DroneCAN::thread() 从 HAL CAN 驱动取帧、交给 libcanard、
 * 再由 libcanard 触发各库回调的流程一致。
 */

#include "can_iface.h"

#include "ap_dronecan_stub.h"
#include "can_frame.h"
#include "canard_stub.h"

#include <atomic>
#include <cstdint>
#include <map>
#include <vector>
#include <QCoreApplication>
#include <QDateTime>
#include <QTimer>

#include "zqwl_ucan.h"
#include "ucanMgr.h"

static const uint32_t ALLOC_ID = ((uint32_t)16 << 24) | (1u << 8) | 0x7f;
static const uint32_t NODE_STATUS_ID = ((uint32_t)16 << 24) | (341u << 8) | 0x7f;

// uavcan.protocol.dynamic_node_id.Allocation 的数据类型签名
// 多帧传输的 CRC 必须包含该签名（小端字节序）
static const uint8_t ALLOC_SIGNATURE[8] = { 0x40, 0x1D, 0xA1, 0x20, 0x26, 0x81, 0x2A, 0x0B };
namespace {
    // tail byte 的 SOT/EOT 标志
enum {
    SOT_EOT_SINGLE = 0xC0,
    SOT_EOT_START = 0x80,
    SOT_EOT_END = 0x40,
};

//按源节点把 CAN 帧组装成一次完整传输
struct CanAssembler {
    CanFrame frames[8];
    int count = 0;
    bool add(const CanFrame &f) {
        if (f.dlc == 0) {
            return false;
        }
        // tail byte 的高两位: START=0x80, END=0x40, SINGLE=0xC0
        const uint8_t sot_eot = f.data[f.dlc - 1] & 0xC0;
        if (sot_eot == SOT_EOT_START ||
            sot_eot == SOT_EOT_SINGLE) {
            count = 0;      // 新传输开始
        }
        if (count >= (int)(sizeof(frames) / sizeof(frames[0]))) {
            count = 0;
            return false;
        }
        frames[count++] = f;
        return sot_eot == SOT_EOT_SINGLE ||
               sot_eot == SOT_EOT_END;
    }
    /*
    * 从标准 CAN 帧重组 DroneCAN 载荷 (含 CRC 校验)
    * 返回 true 表示成功; payload_len 为重组出的字节数
    */
    bool decode_transfer(
        uint8_t &source_node_id,
        uint16_t &data_type_id,
        uint8_t *payload,
        uint16_t &payload_len)
    {
        if (count <= 0 || payload == nullptr || frames[0].dlc == 0)
            return false;

        source_node_id = dronecan::transport::can_id_source_node(frames[0].id);
        data_type_id = dronecan::transport::can_id_data_type(frames[0].id);
        payload_len = 0;

        const uint8_t first_tail = frames[0].data[frames[0].dlc - 1];
        const uint8_t first_sot_eot = first_tail & 0xC0;

        if (first_sot_eot == SOT_EOT_SINGLE)
        {
            const uint16_t len = (uint16_t)(frames[0].dlc - 1);
            memcpy(payload, frames[0].data, len);
            payload_len = len;
            return true;
        }

        uint16_t crc_received = 0;
        bool crc_captured = false;
        for (int n = 0; n < count; n++)
        {
            const CanFrame &f = frames[n];
            if (f.dlc == 0)
                return false;

            const uint8_t tail = f.data[f.dlc - 1];
            const uint8_t sot_eot = tail & 0xC0;
            const uint16_t len = (uint16_t)(f.dlc - 1);

            uint8_t start = 0;
            if (sot_eot == SOT_EOT_START) {
                if (len < 2)
                    return false;

                crc_received = (uint16_t)(f.data[0] | ((uint16_t)f.data[1] << 8));
                crc_captured = true;
                start = 2;   // 跳过 CRC
            }

            for (uint8_t i = start; i < len; i++)
            {
                payload[payload_len++] = f.data[i];
            }

            if (sot_eot == SOT_EOT_END) {
                if (!crc_captured)
                    return false;
                return true;
            }
        }
        return false;
    }
    // CRC-16/X.25, 与 libcanard 的 crcAdd() 一致: 初值 0xFFFF, 多项式 0x1021
    static uint16_t crc_add(uint16_t crc, const uint8_t *data, uint16_t len)
    {
        for (uint16_t i = 0; i < len; i++) {
            crc ^= (uint16_t)data[i] << 8;
            for (uint8_t b = 0; b < 8; b++) {
                if (crc & 0x8000) {
                    crc = (uint16_t)((crc << 1) ^ 0x1021);
                }
                else {
                    crc = (uint16_t)(crc << 1);
                }
            }
        }
        return crc;
    }
    void encode_transfer(uint32_t &frId, const uint8_t *payload, uint16_t payload_len)
    {
        static uint8_t seq = 0;
        if (payload_len <= 7)
        {
            count = 1;
            auto &cfr = frames[0];
            cfr.id = frId;
            cfr.bExt = true;
            cfr.bRemote = false;
            memcpy(cfr.data, payload, payload_len);
            cfr.data[payload_len] = SOT_EOT_SINGLE | seq;
            cfr.dlc = payload_len + 1;
        }
        else
        {
            int index = 0;
            for (count = 0; count < 4 && index < payload_len; ++count)
            {
                CanFrame &f = frames[count];
                f.bExt = true;
                f.bRemote = false;
                f.id = frId;
                auto len = payload_len - index >= 7 ? 7 : payload_len - index;
                if (count == 0)
                {
                    QByteArray crcInput((char*)ALLOC_SIGNATURE, 8);
                    crcInput.append((char *)payload, payload_len);
                    const uint16_t crc = crc_add(0xffff, (uint8_t*)crcInput.data(), crcInput.size());
                    f.data[0] = (uint8_t)(crc & 0xFF);
                    f.data[1] = (uint8_t)(crc >> 8);
                    memcpy(f.data + 2, payload+index, len - 2);
                    index += 5;
                }
                else
                {
                    memcpy(f.data, (char *)payload+index, len);
                    index += len;
                }
                uint8_t tail = seq | (count % 2 ? 0x20 : 0);
                if (index == payload_len)
                    f.data[len] = SOT_EOT_END | tail;
                else if (count == 0)
                    f.data[len] = SOT_EOT_START | tail;
                else
                    f.data[len] = tail;

                f.dlc = len + 1;
            }
        }
        seq = (seq + 1) % 0x1f;
    }
};

} // namespace

// ------------------------------------------------------------------
// PIMPL: 把 CAN 帧、协议栈、线程全部藏在实现文件里
// ------------------------------------------------------------------
class DroneCANInterface::Impl {
public:
    explicit Impl(uint8_t driver_index, uint8_t driver_ch) : dronecan(driver_index, driver_ch), m_msSend(0)
    {
    }
    void start()
    {
        if (auto dev = UCanMgr::Instance().GetZQWL_UCAN(dronecan.get_driver_index()))
        {
            QObject::connect(dev, &ZQWL_UCAN::canFrameRecived, qApp, [=](int ch) {
                CanFrame fr[2];
                while (true)
                {
                    auto nF = dev->ReadFrame(ch, fr);
                    for (int i = 0; i < nF; ++i)
                    {
                        process_frame(fr[i]);
                    }
                    if (nF < 2)
                        break;
                }
                if (!m_timer.isActive())
                {
                    m_timer.setInterval(1000);
                    m_msSend = QDateTime::currentSecsSinceEpoch();
                    QObject::connect(&m_timer, &QTimer::timeout, qApp, [=] {
                        uint32_t uptime = QDateTime::currentSecsSinceEpoch() - m_msSend;
                        QByteArray p(7, 0);  // health=OK, mode=operational, sub_mode=0, vendor=0
                        p[0] = (uint8_t)(uptime & 0xFF);
                        p[1] = (uint8_t)((uptime >> 8) & 0xFF);
                        p[2] = (uint8_t)((uptime >> 16) & 0xFF);
                        p[3] = (uint8_t)((uptime >> 24) & 0xFF);
                        sendTransfer(NODE_STATUS_ID, p);
                    });
                    m_timer.start();
                }
            });
        }
    }

    void stop()
    {
        m_timer.stop();
    }

    uint32_t rx_transfers() const { return _rx_transfers.load(); }
    AP_DroneCAN dronecan;
private:
    void process_frame(const CanFrame &frame)
    {
        const uint8_t src = dronecan::transport::can_id_source_node(frame.id);
        CanAssembler &assembler = _assemblers[src];
        if (!assembler.add(frame))
            return;

        // 一次完整传输: 重组 -> CRC 校验
        uint8_t payload[64];
        uint16_t payload_len = 0;
        uint8_t rx_src = 0;
        uint16_t data_type_id = 0;
        if (!assembler.decode_transfer(rx_src, data_type_id, payload, payload_len))
            return;

        if ((data_type_id & 0x3) == 1 && rx_src == 0)
        {
            rspAlloc(payload, payload_len);
            return;
        }
        const CanFrame &last = assembler.frames[assembler.count - 1];
        CanardRxTransfer transfer;
        transfer.source_node_id = rx_src;
        transfer.transfer_id = (uint8_t)(last.data[last.dlc - 1] & 0x1F);
        // 交给协议栈分发: 命中订阅回调后, 由回调自己反序列化并处理
        if (data_type_id == 1027)
        {
            canard_dispatch(dronecan.get_driver_index(), transfer, data_type_id, payload);
            _rx_transfers++;
        }
    }
    void rspAlloc(const uint8_t *payload,  uint16_t payload_len)
    {
        const uint8_t pref = (uint8_t)((payload[0] >> 1) & 0x7F);
        const bool first = (payload[0] & 0x01) != 0;
        const uint8_t* chunk = payload + 1;
        const size_t chunkLen = payload_len - 1;
        if (!first && !m_payload.isEmpty() && m_payload.size()+chunkLen > 16) {
            QByteArray rsp(1, '\0');
            rsp.append(m_payload);
            sendTransfer(ALLOC_ID, rsp);
            return;
        }

        if (first || m_payload.isEmpty())
            m_payload = QByteArray((char*)chunk, chunkLen);
        else
            m_payload.append((char*)chunk, chunkLen);

        if (m_payload.size() >= 16) {
            const uint8_t nodeId = (pref >= 1 && pref <= 125) ? pref : 125;
            QByteArray rsp;
            rsp.push_back((uint8_t)((nodeId << 1) & 0xFE));
            rsp.append(m_payload.left(16));
            sendTransfer(ALLOC_ID, rsp);
            m_payload.clear();
        }
        else {
            QByteArray rsp(1, '\0');
            rsp.append(m_payload);
            sendTransfer(ALLOC_ID, rsp);
        }
    }
    void sendTransfer(uint32_t id, const QByteArray &arr)
    {
        CanAssembler assm;
        memset(&assm, 0, sizeof(assm));
        assm.encode_transfer(id, (uint8_t*)arr.data(), arr.size());
        auto dev = UCanMgr::Instance().GetZQWL_UCAN(dronecan.get_driver_index());
        if (dev)
        {
            for (int i = 0; i < assm.count; ++i)
            {
                dev->SendFrame(dronecan.get_driver_channel(), assm.frames[i]);
            }
        }
    }
    int64_t m_msSend;
    //CanThread _thread;
    std::map<uint8_t, CanAssembler> _assemblers;
    std::atomic<uint32_t> _rx_transfers{0};
    QByteArray m_payload;
    QTimer m_timer;
};
/*********************************************************************
*  DroneCANInterface
**********************************************************************/
DroneCANInterface::DroneCANInterface(uint8_t driver_index)
    : _impl(new Impl(driver_index, 1))
{
}

DroneCANInterface::~DroneCANInterface()
{
    if (_impl != nullptr) {
        _impl->stop();
        delete _impl;
    }
}

void DroneCANInterface::start()
{
    _impl->start();
}

void DroneCANInterface::stop()
{
    _impl->stop();
}

AP_DroneCAN *DroneCANInterface::dronecan()
{
    return &_impl->dronecan;
}

const AP_DroneCAN *DroneCANInterface::dronecan() const
{
    return &_impl->dronecan;
}

uint32_t DroneCANInterface::rx_transfers() const
{
    return _impl->rx_transfers();
}
