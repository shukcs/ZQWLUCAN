#ifndef ZQWL_UCAN_GLOBAL_H
#define ZQWL_UCAN_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(ZQWL_UCAN_STATIC)
#  define ZQWL_UCANSHARED_EXPORT
#elif defined(ZQWL_UCAN_LIBRARY)
#  define ZQWL_UCANSHARED_EXPORT Q_DECL_EXPORT
#else
#  define ZQWL_UCANSHARED_EXPORT Q_DECL_IMPORT
#endif

typedef struct CanFrame {
    uint32_t id : 29;     ///can id
    bool bExt : 1;      ///false：标注帧；其他:扩展帧
    bool bRemote : 1;   ///false：数据帧
    uint8_t dlc;
    uint8_t data[64];
} CanFrame;

#endif // ZQWL_UCAN_GLOBAL_H
