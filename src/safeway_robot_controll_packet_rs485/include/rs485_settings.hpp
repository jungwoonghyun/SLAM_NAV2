///////////////////////////////////////
//
//  demonstration processing methods
//

#define MCMD_METHOD_SIMPLE  0   // simple and easy, but inefficient
#define MCMD_METHOD_ASYNC1  1   // async method #1, Windows: overlapped & WaitForMultipleObjects(), Linux: select()
#define MCMD_METHOD_ASYNC2  2   // async method #2, Windows: overlapped & GetQueuedCompletionStatus(), Linux: poll()
                                // * Caution: some devices are not support the IOCP

#define MCMD_METHOD     MCMD_METHOD_ASYNC1

// Linux
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <assert.h>
#include <ctype.h>      // isprint()

#include <fcntl.h>      // Contains file controls like O_RDWR
#include <errno.h>      // Error integer and strerror() function
#include <termios.h>    // Contains POSIX terminal control definitions
#include <unistd.h>     // write(), read(), close()

typedef int COMM_HANDLE;

// device identifiers
#define COMM_ID_HOST        0   // host PC
#define COMM_ID_ROBOT       1   // robot

// ascii codes for the packet decoration
#define ASCII_SOH   0x01
#define ASCII_STX   0x02
#define ASCII_ETX   0x03
#define ASCII_EOT   0x04

#define ROBOT_AXES_COUNT    2

#define msleep(a)  usleep(a*1000)

#define CMD_MULTIPLE_DELIMITER  ';'     // multiple command delimiter
#define CMD_MNEMONIC_DELIMITER  '='     // mnemonic/arguments delimiter
#define CMD_ARG_DELIMITER       ','     // each arguments delimiter

#define MCMD_stricmp    strcasecmp
#define MCMD_strnicmp   strncasecmp

typedef enum _enum_REPLY_TYPE_FLAGS
{
    REPLY_UNKNOWN = -1,
    REPLY_NONE = 0,             // no reply (no chance to view)
    REPLY_BASIC = 0x01,         // result + axes + position + speed + stage
    REPLY_STOP_REASON = 0x02,   // basic + stop_reason
    REPLY_TORQUE = 0x04,        // basic + (stop_reason) + torque
    REPLY_UMODE = 0x08,         // basic + (stop_reason) + (torque) + umode
    REPLY_USONIC = 0x10,         // basic + (stop_reason) + (torque) + (umode) + usonic
} REPLY_TYPE_FLAGS;

typedef enum _enum_MOTION_STAGE
{
    MOTION_STAGE_UNKNOWN = -1,  // unknown or system initializing
    MOTION_STAGE_INIT = 0,  // 0, initializing or not init
    MOTION_STAGE_COAST,     // 1, coasting
    MOTION_STAGE_READY,     // 2, preparation for motion (arrangement)
    MOTION_STAGE_VOLTAGE,   // 3, voltage control
    MOTION_STAGE_AMPERE,    // 4, current control
    MOTION_STAGE_FREEZE,    // 5, freeze/stuck state (fixed, aligned)
    MOTION_STAGE_ACCEL,     // 6, speed acceleration/deceleration
    MOTION_STAGE_CONST,     // 7, speed constant
    MOTION_STAGE_DECEL,     // 8, speed deceleration to stop
    MOTION_STAGE_DWELL,     // 9, dwelling before stop
    MOTION_STAGE_STOP,      // 10, stopped
    MOTION_STAGE_ESTOP,     // 11, emergency stopped
    MOTION_STAGE_BRAKE,     // 12, coil braking
    MOTION_STAGE_OBSTF,     // 13, front-side obstacle checked and stopped
    MOTION_STAGE_OBSTB,     // 14, back-side obstacle checked and stopped
    MOTION_STAGE_OBSTA,     // 15, all-side (front and back) obstacle checked and stopped
    MOTION_STAGE_ERROR,     // 16, error state

    MOTION_STAGE_COUNT      // 17 = number of motion stages
} MOTION_STAGE;

///////////////////////////////////////
//
//  STOP_REASON
//

typedef enum _enum_MOTION_STOP_REASON
{
    STOP_REASON_UNKNOWN = -1,
    STOP_REASON_NONE = 0,   // 0, not in stopped stage
    STOP_REASON_NORMAL,     // 1, normal stop, no reason
    STOP_REASON_ESTOP,      // 2, stop by estop button
    STOP_REASON_OBSTF,      // 3, stop by front-side obstacles
    STOP_REASON_OBSTB,      // 4, stop by back-side obstacles

    STOP_REASON_COUNT,      // 5 = number of stop methods
} STOP_REASON;

typedef struct _struct_ROBOT_INFO
{
    int     nPolePair;              // number of motor pole pairs
    double  dbSensorResolution;     // sensor resolution (counter/rev)
    double  dbSensorToMillimeter;   // sensor -> millimeter (0: no wheel)
    double  dbWheelDiameter;        // wheel diameter (mm)

    int     nUserModeValue1;        // user mode value #1
    int     nUserModeValue2;        // user mode value #2
    int     nUserModeValue3;        // user mode value #3

    int     nNumberofUsonic;         // number of usonic sensor
    int     nUsonicFrontValue;      // front usonic data (mm)
    int     nUsonicBackValue;       // back usonic data (mm)
} ROBOT_INFO;

// ROBOT_STATE
typedef struct _struct_ROBOT_STATE
{
    int64_t         nPosition;  // robot position (sensor unit)
    uint64_t        nDistance;  // travel distance (sensor unit)
    double          dbSpeed;    // current speed (sensor unit per second)
    MOTION_STAGE    motionStage;   // motion stage
    STOP_REASON     stopReason; // motion stop reason - depend on response type
    double          dbTorque;   // current output torque (N.m) - depend on response type
} ROBOT_STATE;

extern ROBOT_INFO g_RobotInfo;
extern ROBOT_STATE g_RobotState[ROBOT_AXES_COUNT];

COMM_HANDLE open_comm_dev(const char* pszDevName, int nBaud);
unsigned calc_crc16(const void* pData, int nLen);
int send_comm_dev(COMM_HANDLE hComm, const void* p, int size);
int format_motion_command(COMM_HANDLE hComm, const char* pszFormat, ...);
void close_comm_dev(COMM_HANDLE hComm);
int recv_comm_dev(COMM_HANDLE hComm, void* p, int size);
int parse_recv_packet(const void* ppacket, int nRecvSize);
void dump_recv_packet(const void* pPacket, int size);

void start_robot(COMM_HANDLE hComm);
void end_robot(COMM_HANDLE hComm);
