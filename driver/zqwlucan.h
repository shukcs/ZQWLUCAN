#ifndef __ZQWLUCAN_H__
#define __ZQWLUCAN_H__
#include <stdint.h>

void ZQWLInit();
bool ZQWLOpen(uint16_t id, uint16_t ch, int baut = 500000, int jud = 125000);

#endif // __ZQWLUCAN_H__
