#include <stdlib.h>
#include <stdio.h>

#include "rs485_settings.hpp"

#define MOTION_CMD_BUFFER_SIZE  256
#define MOTION_CMD_MAX_LEN      (MOTION_CMD_BUFFER_SIZE - 10)

// buffer for receciving the reply
static char s_szCmdRBuffer[MOTION_CMD_BUFFER_SIZE];

// buffer for seding the commands
static char s_szCmdWBuffer[MOTION_CMD_BUFFER_SIZE];

static int s_nSendIndex = 0;

int format_motion_commandv(COMM_HANDLE hComm, const char* pszFormat, va_list arg)
{
    unsigned char* p = (unsigned char*)s_szCmdWBuffer;
    char* pCmd = (char*)p + 6;  // header is 6 byte

    // fill motion commands
    int nLen = vsnprintf(pCmd, MOTION_CMD_MAX_LEN, pszFormat, arg);

    if (nLen < 0 || nLen > 0x7f)
    {
        return -1;
    }

    // build the packet header
    p[0] = ASCII_SOH;
    p[1] = COMM_ID_ROBOT;  // robot id as a receiver
    p[2] = COMM_ID_HOST;   // host id as a sender
    p[3] = (unsigned char) nLen;
    p[4] = (unsigned char) ((++s_nSendIndex) & 0xff);
    p[5] = ASCII_STX;

    // build the packet tail
    p[6 + nLen] = ASCII_ETX;
    p[7 + nLen] = 0;    // CRC16 still not calculated
    p[8 + nLen] = 0;
    p[9 + nLen] = ASCII_EOT;

    // calc CRC16
    unsigned crc16 = calc_crc16(p + 1, nLen + 5);
    p[7 + nLen] = (unsigned char)(crc16 & 0xff);
    p[8 + nLen] = (unsigned char)((crc16 >> 8) & 0xff);;

    // send the motion commands
    int nSentLen = send_comm_dev(hComm, p, 10 + nLen);

    if (nSentLen < 0)
    {
        printf("Fail to send the command\n");
    }

    printf("written %d bytes: len=%d, crc16=%04X, '%.*s'\n",
           nSentLen,
           nLen,
           crc16,
           nLen,
           pCmd);

    return nLen;
}

int format_motion_command(COMM_HANDLE hComm, const char* pszFormat, ...)
{
    va_list arg;
    int nSentLen;

    va_start(arg, pszFormat);
    nSentLen = format_motion_commandv(hComm, pszFormat, arg);
    va_end(arg);

    return nSentLen;
}

void start_robot(COMM_HANDLE hComm)
{
    // motion control mode: speed control
    // motion standstill(SS) start with zero speed (stopped) initially
    // format_motion_command(hComm, "REPTYPE=BU;CONMODE=SPEED;SPEED=0;START=SS");
    format_motion_command(hComm, "REPTYPE=BUW;NOP");
    msleep(10);
}

void end_robot(COMM_HANDLE hComm)
{
    // motion released (coasting)
    format_motion_command(hComm, "COAST");
    msleep(10);
}
