// user space application to test our device driver.
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include "cdc_ioctl.h"

int main(int argc, char *argv[]) 
{
    int fd, ret;
    if(argc < 2) {
        printf("ERROR: invalid cmd line args.\n");
        return 1;
    }
    fd = open("/dev/stm32cdc", O_RDWR);
    if(fd < 0) {
        perror("open() failed");
        _exit(1);
    }
    if(strcmp(argv[1], "GETINFO") == 0) {
        devinfo_t info;
        ret = ioctl(fd, DEV_GETINFO, &info);
        if(ret == 0)
            printf("Device FIFO info:\n\tsGreenLED = %u, sOrangeLED = %u, sRedLED = %u, sBlueLED = %u\n", info.sGreenLed, info.sOrangeLed, info.sRedLed, info.sBlueLed);
    }
    else if(strcmp(argv[1], "RESIZE") == 0) {
        // ... Assignment
    }
    else
        printf("ERROR: Invalid command.\n");
    close(fd);
    return 0;
}

/* application to test pchar ioctl.
    > gcc -o ioctl_user_test.out ioctl_user_test.c
Usage:
    > sudo ./ioctl_user_test.out GETINFO

    > sudo ./ioctl_user_test.out RESET

    > sudo ./ioctl_user_test.out RESIZE  64
*/

