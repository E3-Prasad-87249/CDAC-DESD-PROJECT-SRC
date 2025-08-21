#ifndef __PCHAR_IOCTL_H
#define __PCHAR_IOCTL_H

#include <linux/ioctl.h>

typedef struct devinfo 
{
    short sGreenLed, sOrangeLed , sRedLed, sBlueLed;
}devinfo_t;

#define DEV_GETINFO    _IOR('x', 1, devinfo_t)

#endif

