#include "zqwlucan.h"
#include "UCanMgr.h"

void ZQWLInit()
{
    UCanMgr::Instance();
}

bool ZQWLOpen(uint16_t id, uint16_t ch, int baut /*= 500000*/, int jud /*= 125000*/)
{
    return false;
}
