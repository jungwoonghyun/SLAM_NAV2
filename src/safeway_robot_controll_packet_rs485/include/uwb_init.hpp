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
#include <errno.h>
#include <linux/types.h>
#include <pthread.h>
#include <math.h>

#define MATH_H

#define _USE_MATH_DEFINES

#define UWB_MAX_TAGCNT                 3
#define msleep(a)                      usleep(a*1000)
#define LOWPASSFILTER_VEL              0.75
#define LOWPASSFILTER_PROCESSVEL       0.75

#define UWB_3TAG_XPOS                  290.0
#define UWB_3TAG_YPOS                  530.0

#define UWB_Center_X                   145.0
#define UWB_Center_Y                   100.0

#define PI_defined                     3.14159265358979323846
#define DEGtoRAD                       PI_defined/180.0
#define RADtoDEG                       180.0/PI_defined

#define TAG_ADDR_1                     0x0C
#define TAG_ADDR_2                     0x05

#define ANC_ADDR_0_1                   0xCF
#define ANC_ADDR_0_2                   0xA7

#define ANC_ADDR_1_1                   0x57
#define ANC_ADDR_1_2                   0x80

#define ANC_ADDR_2_1                   0x95
#define ANC_ADDR_2_2                   0x81

#define UWB_Curve_Angle                22.0
#define UWB_Curve_radius               1000.0
#define UWB_MaxSpdDist                 1300.0
#define UWB_MinSpdDist                 700.0
#define UWB_SpdRate                    0.17

#define UWB_CURVESPD                   35.0

#pragma pack(push, 1)
typedef struct _struct_UWB_TAG_INFO {

    uint8_t  UWBaddr[2];
    uint8_t  distance[4];
    uint8_t  distQualFac;
    uint8_t  posStanForm[13];
}UWB_TAG_INFO;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct _struct_DwmLocDataTag {// DWM 1001 Data structure

    uint8_t  type1;
    uint8_t  length1;
    uint8_t  err_code;
    uint8_t  type2;
    uint8_t  length2;
    uint8_t  position[13];

    uint8_t  type3;
    uint8_t  length3;
    uint8_t  numOfDist;

    UWB_TAG_INFO TagInfo[UWB_MAX_TAGCNT];
} DWMDATA_tag; // 101 byte packet
#pragma pack(pop)

typedef struct _struct_DwmNodeidInfo {
    uint8_t  type1;
    uint8_t  length1;
    uint8_t  err_code;
    uint8_t  type2;
    uint8_t  length2;
    uint8_t  nodeId[2];
    uint8_t  nodeAddr[6];
} DWMNode_ID;

typedef struct _struct_DwmStandardDistData{
    double  dist;
    uint8_t nodeId[2];
} DWMDistData;

typedef struct _struct_UWBxyPlane{
    double x;
    double y;
} UWBXYPlane;

typedef struct _struct_UWBrpiPlane{
    double dist;
    double angle;
} UWBRPIPlane;

// device function
void openDevice(char* dev, int* fd);
void readDWMNodeNum(int* fd, DWMNode_ID* nodeID);
unsigned int readDWMLocData(int* fd, DWMDATA_tag* locData);
int uwbPointSelector(DWMNode_ID* nodeId, DWMDATA_tag* locData);
