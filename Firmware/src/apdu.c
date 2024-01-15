#include "apdu.h"
#include <string.h>

int fill_capdu(CAPDU* pCAPDU, uint8_t* buff, uint16_t bufflen)
{
    if (bufflen < 4) return -1;
    pCAPDU->cla = buff[0];
    pCAPDU->ins = buff[1];
    pCAPDU->p1 = buff[2];   
    pCAPDU->p2 = buff[3];
    pCAPDU->lc = 0;
    pCAPDU->le = 0;

    if(bufflen == 4) // Case 1
        return 0;
    pCAPDU->lc = buff[4];
    if(bufflen == 5) // Case 25
    {
        pCAPDU->le = pCAPDU->lc;
        pCAPDU->lc = 0;
        if (pCAPDU->le == 0) pCAPDU->le = 0x100;
    }
    else if(pCAPDU->lc > 0 && bufflen == 5 + pCAPDU->lc) // Case 35
    {
        memmove(pCAPDU->data, buff + 5, pCAPDU->lc);
        pCAPDU->le = 0x100;
    }
    else if(pCAPDU->lc > 0 && bufflen == 6 + pCAPDU->lc) // Case 45
    {
        memmove(pCAPDU->data, buff + 5, pCAPDU->lc);
        pCAPDU->le = buff[5 + pCAPDU->lc];
        if(pCAPDU->le == 0) pCAPDU->le = 0x100;
    }
    else if(bufflen == 7) // Case 2E
    {
        if (pCAPDU->lc != 0) return -1;
        pCAPDU->le = (buff[5] << 8) | buff[6];
        if(pCAPDU->le == 0) pCAPDU->le = 0x10000;
    }
    else
    {
        if(pCAPDU->lc != 0 || bufflen < 7) return -1;
        pCAPDU->lc = (buff[5] << 8) | buff[6];
        if(pCAPDU->lc == 0) return -1;
        if (bufflen == 7 + pCAPDU->lc) // Case 3E
        {
            memmove(pCAPDU->data, buff + 7, pCAPDU->lc);
            pCAPDU->le = 0x10000;
            return 0;
        }
        else if(bufflen == 9 + pCAPDU->lc) // Case 4E
        {
            memmove(pCAPDU->data, buff + 7, pCAPDU->lc);
            pCAPDU->le = (buff[7 + pCAPDU->lc] << 8) | buff[8 + pCAPDU->lc];
            if(pCAPDU->le == 0) pCAPDU->le = 0x10000;
            return 0;
        }
        else
        {
            return -1;
        }
    }
    return 0;
}
