#include <stdio.h>
#include <stdlib.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <errno.h>
#include <linux/types.h>
#include <pthread.h>
#include <math.h>

#define msleep(a)                      usleep(a*1000)
#define ROBOT_SET_DIST                 165.0 //mm
#define LPF_VALUE                      0.8
#define MOV_LPF_VALUE                  0.8
#define STAND_HALF_HEIGHT              400.0 //mm

#define RTLS_SET_DIST                  650.0 //mm

#pragma pack(push, 1)
typedef struct _struct_DWM1000_INFO {
    uint8_t  shtAddr_dev[2];
    uint8_t  shtAddr_tag[2];
    uint8_t  dev_tagDist[2];
    uint8_t  errCheck;
}DWM1000_INFO;
#pragma pack(pop)

typedef struct _struct_UWBxyPlane{
    float x;
    float y;
} UWBXYPlane;

typedef struct _struct_UWBrpiPlane{
    float dist;
    float angle;
} UWBRPIPlane;

void openDevice(char* dev, int* fd);
unsigned int readDWM1000Data(int* fd, DWM1000_INFO* locData);
int uwbPointSelector(DWM1000_INFO* locData);
