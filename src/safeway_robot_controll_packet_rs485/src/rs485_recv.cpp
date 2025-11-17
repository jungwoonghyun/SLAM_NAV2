#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include "rs485_settings.hpp"

///////////////////////////////////////
//
//  configurations
//

#define USE_CHECK_SWICHING_NOISE    1   // TRUE/FALSE, check RS485 TX/RX switching noise
#define DUMP_SIZE           16          // 16*3 + 16 = 48 + 16 = 64

// [0]: type - "B", "BU", ...                       ----+
// [1]: previous command execution result               |   info (3)
// [2]: number of axes                              ----+
// [3]: motor pole pairs                            ----+
// [4]: motor sensor resolution                         |
// [5]: motor sensor rate (mm/pulse)                    |   motor/wheel info (4)
// [6]: wheel diameter                              ----+
// ...
// [7 + 0]: position (motor sensor based)           ----+
// [7 + 1]: travel distance (motor sensor based)        |
// [7 + 2]: speed (motor sensor based)                  |
// [7 + 3]: motion stage                                |   repeat by axes (4 + 2)
// [7 + 4]: (optional) stop reason ("R" type)           |
// [7 + 5]: (optional) torque ("T" Type)            ----+
// ...
// [-3]: umode value 1                              ----+
// [-2]: umode value 2                                  |   user mode value (3)
// [-1]: umode value 3                              ----+
//
#define REPLY_CMD_ARG_COUNT (3 + 4 + ROBOT_AXES_COUNT*6 + 3)

ROBOT_INFO g_RobotInfo;
ROBOT_STATE g_RobotState[ROBOT_AXES_COUNT];

///////////////////////////////////////
//
//  s_rgpszMotionStages[]
//

static const char* s_rgpszMotionStageNames[] =
{
    "INIT",     // 0, initializing or not init
    "COAST",    // 1, coasting
    "READY",    // 2, preparation for motion (arrangement)
    "VOLTAGE",  // 3, voltage control
    "AMPERE",   // 4, current control
    "FREEZE",   // 5, freeze/stuck state (fixed, aligned)
    "ACCEL",    // 6, speed acceleration/deceleration
    "CONST",    // 7, speed constant
    "DECEL",    // 8, speed deceleration to stop
    "DWELL",    // 9, dwelling before stop
    "STOP",     // 10, stopped
    "ESTOP",    // 11, emergency stopped
    "BRAKE",    // 12, coil braking
    "OBSTF",    // 13, front-side obstacle checked and stopped
    "OBSTB",    // 14, back-side obstacle checked and stopped
    "OBSTA",    // 15, all-side(front/back both) obstacle checked and stopped
    "ERROR",    // 16, error state
    "BLOCK",    // 17, motion blocked by other module (ex: joystick)
};

///////////////////////////////////////
//
//  s_rgpszStopReasonNames[]
//

static const char* s_rgpszMotionStopReasonNames[] =
{
    "NONE",     // 0, no in stopped stage
    "NORMAL",   // 1, normal stop, no reason
    "ESTOP",    // 2, stopped by estop button
    "OBSTF",    // 3, stopped by front-side obstacles
    "OBSTB",    // 4, stopped by back-side obstacles
    "BLOCK",    // 5, motion blocked by other module (ex: joystick)
};

///////////////////////////////////////
//
//  get_motion_stage_name()
//  get motion stage name string
//

MOTION_STAGE parse_motion_stage(const char* pMotionStage, int nLen)
{
    for (register int iStage = 0; iStage < sizeof(s_rgpszMotionStageNames) / sizeof(s_rgpszMotionStageNames[0]); iStage++)
    {
        if (MCMD_strnicmp(pMotionStage, s_rgpszMotionStageNames[iStage], nLen) == 0)
        {
            return (MOTION_STAGE)iStage;
        }
    }

    return MOTION_STAGE_UNKNOWN;
}

const char* get_motion_stage_name(MOTION_STAGE motionStage)
{
    if (motionStage >= 0 && motionStage < sizeof(s_rgpszMotionStageNames) / sizeof(s_rgpszMotionStageNames[0]))
    {
        return s_rgpszMotionStageNames[motionStage];
    }

    // unknown motion stage;
    return "UNKOWN";
}

STOP_REASON parse_stop_reason(const char* pStopReason, int nLen)
{
    for (register int iReason = 0; iReason < sizeof(s_rgpszMotionStopReasonNames) / sizeof(s_rgpszMotionStopReasonNames[0]); iReason++)
    {
        if (MCMD_strnicmp(pStopReason, s_rgpszMotionStopReasonNames[iReason], nLen) == 0)
        {
            return (STOP_REASON)iReason;
        }
    }

    return STOP_REASON_UNKNOWN;
}
//extern "C"
int parse_reply_command(const char* pCmd, int nPacketLen)
{
#if 0
    // find next command to know the length of the command
    // find command-command delimiter (';')
    // (not occurred in REPLY )
    // ex: "SPEED=30,mmps;START=SS"
    const char* pCmdDelimiter = memchr(pCmd, CMD_MULTIPLE_DELIMITER, nPacketLen);
    int nCmdLen = (pCmdDelimiter == NULL)
                ? nPacketLen
                : (int)(pCmdDelimiter - pCmd);
#else
    int nCmdLen = nPacketLen;
#endif

    // separate mnemonic and arguments
    // find mnemonic with the delimiter equal sign('=')
    // ex: "REPLY=B,...."
    const char* pMnemonicDelimiter = (const char *)memchr(pCmd, CMD_MNEMONIC_DELIMITER, nCmdLen);

    int nMnemonicLen = (pMnemonicDelimiter == NULL)
                     ? nCmdLen
                     : (int)(pMnemonicDelimiter - pCmd);

    // check the mnemonic is "REPLY"
    if (nMnemonicLen <= 0 || MCMD_strnicmp(pCmd, "REPLY", nMnemonicLen) != 0)
    {
        printf("unsupported mnemonic: '%.*s'\n", nMnemonicLen, pCmd);
        return -1;
    }

    // separate each arguments (separated by comma(,))
    int nArgRemainLen = nCmdLen - nMnemonicLen - 1;

    const char* rgpArgs[REPLY_CMD_ARG_COUNT] = { NULL };    // needless but 'lnt-uninitialized-local' rule
    int rgnArgLens[REPLY_CMD_ARG_COUNT] = { 0 };            // needless but 'lnt-uninitialized-local' rule

    const char* pszNextArg;
    const char* pszArgDelimiter;
    int nArgCount;

    register int iArg;

    // argument parsing start
    pszNextArg = pMnemonicDelimiter + 1;

    for (iArg = 0; nArgRemainLen > 0 && iArg < sizeof(rgpArgs)/sizeof(rgpArgs[0]); iArg++)
    {
        // separate the argument with comma(',')
        pszArgDelimiter = (const char *)memchr(pszNextArg, CMD_ARG_DELIMITER, nArgRemainLen);

        if (pszArgDelimiter == NULL)
        {
            // last argument
            rgpArgs[iArg] = pszNextArg;
            rgnArgLens[iArg] = nArgRemainLen;

            iArg++;
            break;
        }

        // store argument
        rgpArgs[iArg] = pszNextArg;
        rgnArgLens[iArg] = (int)(pszArgDelimiter - pszNextArg);

        // next argument
        nArgRemainLen -= (int)(pszArgDelimiter - pszNextArg) + 1;
        pszNextArg = pszArgDelimiter + 1;
    }

    // number of arguments
    nArgCount = iArg;

    // number of arguments are valid?
    if (nArgCount < 3)
    {
        printf("number of arguments are to small: %d\n",
               nArgCount);

        return -1;
    }

    // reply type
//    REPLY_TYPE_FLAGS replyFlags = REPLY_NONE;
    int replyFlags = REPLY_NONE;

    if (memchr(rgpArgs[0], 'B', rgnArgLens[0]))
    {
        replyFlags |= REPLY_BASIC;
    }

    if (memchr(rgpArgs[0], 'R', rgnArgLens[0]))
    {
        replyFlags |= REPLY_STOP_REASON;
    }

    if (memchr(rgpArgs[0], 'T', rgnArgLens[0]))
    {
        replyFlags |= REPLY_TORQUE;
    }

    if (memchr(rgpArgs[0], 'U', rgnArgLens[0]))
    {
        replyFlags |= REPLY_UMODE;
    }

    if (memchr(rgpArgs[0], 'W', rgnArgLens[0]))
    {
        replyFlags |= REPLY_USONIC;
    }

    // 'basic' info must be included always
    if ((replyFlags & REPLY_BASIC) == 0)
    {
        printf("Reply basic info is not included\n");
    }

    // nymber of arguments for a each axis
    int nAxisArgCount = 4;  // pos, dist, speed, stage

    // stop reason included?
    if (replyFlags & REPLY_STOP_REASON)
    {
        nAxisArgCount++;
    }

    // torque included?
    if (replyFlags & REPLY_TORQUE)
    {
        nAxisArgCount++;
    }

    // end of string for the function strto...()
    char* pEndStr;

    // result of previous transferred command
    int nCmdResult = (int)strtol(rgpArgs[1], &pEndStr, 0);

    if (nCmdResult != 0)
    {
        printf("\nprevious command was failed: error=%d\n",
               nCmdResult);
    }

    // number of axes
    const int nAxesCount = (int)strtol(rgpArgs[2], &pEndStr, 0);

    // robot info available?
    if (nArgCount >= 7)
    {
        // polepair
        g_RobotInfo.nPolePair = (int)strtol(rgpArgs[3], &pEndStr, 0);
        // <sres>
        g_RobotInfo.dbSensorResolution = strtod(rgpArgs[4], &pEndStr);
        // <srate>
        g_RobotInfo.dbSensorToMillimeter = strtod(rgpArgs[5], &pEndStr);
        // <dia>
        g_RobotInfo.dbWheelDiameter = strtod(rgpArgs[6], &pEndStr);

        // we already used 7 args
        int nArgPos = 7;

        for (int iAxis = 0; iAxis < ROBOT_AXES_COUNT && iAxis < nAxisArgCount; iAxis++)
        {
            // [0] position
            if (nArgCount > nArgPos + 0)
            {
                g_RobotState[iAxis].nPosition = strtoll(rgpArgs[nArgPos + 0], &pEndStr, 0);
            }

            // [1] travel disance
            if (nArgCount > nArgPos + 1)
            {
                g_RobotState[iAxis].nDistance = strtoull(rgpArgs[nArgPos + 1], &pEndStr, 0);
            }

            // [2] current speed
            if (nArgCount > nArgPos + 2)
            {
                g_RobotState[iAxis].dbSpeed = strtod(rgpArgs[nArgPos + 2], &pEndStr);
            }

            // [3] motion stage
            if (nArgCount > nArgPos + 3)
            {
                // parse the motion stage
                g_RobotState[iAxis].motionStage = parse_motion_stage(rgpArgs[nArgPos + 3], rgnArgLens[nArgPos + 3]);

                // show motion stage
#if 1
                MOTION_STAGE stageOld = g_RobotState[iAxis].motionStage;

                // motion stage changed?
                if (stageOld == MOTION_STAGE_UNKNOWN && g_RobotState[iAxis].motionStage != MOTION_STAGE_UNKNOWN)
                {
                    printf("\naxis[%d] motion stage is: '%s'\n",
                           iAxis,
                           get_motion_stage_name(g_RobotState[iAxis].motionStage));
                }
                else if (stageOld != MOTION_STAGE_UNKNOWN && g_RobotState[iAxis].motionStage != stageOld)
                {
                    printf("\naxis[%d] motion stage changed: '%s' -> '%s'\n",
                           iAxis,
                           get_motion_stage_name(stageOld),
                           get_motion_stage_name(g_RobotState[iAxis].motionStage));
                }
#endif
            }

            // variable args
            int nAxisArgPos = 4;

            // [ ] motion stop reason
            if ((replyFlags & REPLY_STOP_REASON) && (nArgCount > nArgPos + nAxisArgPos))
            {
                g_RobotState[iAxis].stopReason = parse_stop_reason(rgpArgs[nArgPos + nAxisArgPos], rgnArgLens[nArgPos + nAxisArgPos]);
                nAxisArgPos++;
            }

            // [ ] current torque
            if ((replyFlags & REPLY_TORQUE) && (nArgCount > nArgPos + nAxisArgPos))
            {
                g_RobotState[iAxis].dbTorque = strtod(rgpArgs[nArgPos + nAxisArgPos], 0);
                nAxisArgPos++;
            }

            // axis all arguments checked
            assert(nAxisArgPos == nAxisArgCount);

            // next axis arguments
            nArgPos += nAxisArgCount;
        }

        // [ ] umode three values
        if ((replyFlags & REPLY_UMODE) && (nArgCount > nArgPos))
        {
            g_RobotInfo.nUserModeValue1 = atoi(rgpArgs[nArgPos]);
            nArgPos++;
        }

        if ((replyFlags & REPLY_UMODE) && (nArgCount > nArgPos))
        {
            g_RobotInfo.nUserModeValue2 = atoi(rgpArgs[nArgPos]);
            nArgPos++;
        }

        if ((replyFlags & REPLY_UMODE) && (nArgCount > nArgPos))
        {
            g_RobotInfo.nUserModeValue3 = atoi(rgpArgs[nArgPos]);
            nArgPos++;
        }

        if ((replyFlags & REPLY_USONIC) && (nArgCount > nArgPos))
        {
            g_RobotInfo.nNumberofUsonic = atoi(rgpArgs[nArgPos]);
            nArgPos++;
        }

        if(g_RobotInfo.nNumberofUsonic == 1)
        {
            if ((replyFlags & REPLY_USONIC) && (nArgCount > nArgPos))
            {
                g_RobotInfo.nUsonicFrontValue = atoi(rgpArgs[nArgPos]);
                nArgPos++;
            }
        }
        else if(g_RobotInfo.nNumberofUsonic == 2)
        {
            if ((replyFlags & REPLY_USONIC) && (nArgCount > nArgPos))
            {
                g_RobotInfo.nUsonicFrontValue = atoi(rgpArgs[nArgPos]);
                nArgPos++;
            }

            if ((replyFlags & REPLY_USONIC) && (nArgCount > nArgPos))
            {
                g_RobotInfo.nUsonicBackValue = atoi(rgpArgs[nArgPos]);
                nArgPos++;
            }
        }
        else
        {

        }

        // all arguments checked
        assert(nArgPos == nArgCount);
    }

    //printf("found reply command: '%.*s'\n", nMnemonicLen, pCmd);

    return 0;
}

///////////////////////////////////////
//
//  dump_recv_packet()
//  dump received packet
//

void dump_recv_packet(const void* pPacket, int size)
{
    const unsigned char* p = (const unsigned char*)pPacket;
    int i, j;

    for (i = 0; i < size; i += DUMP_SIZE)
    {
        // index
        printf("[%02d] ", i);

        // dump hex
        for (j = 0; j < DUMP_SIZE && i + j < size; j++)
        {
            printf("%02X ", p[i + j]);
        }

        for (; j < DUMP_SIZE; j++)
        {
            printf("   ");
        }

        // spaces between hex & char
        printf("    ");

        // dump char
        for (j = 0; j < DUMP_SIZE && i + j < size; j++)
        {
            // printable character?
            if (isprint(p[i + j]))
            {
                // yes => print the character
                printf("%c", p[i + j]);
            }
            else
            {
                // not printable => print dot(.) instead of the character
                printf(".");
            }
        }

        printf("\n");
    }
}

int parse_recv_packet(const void* ppacket, int nRecvSize)
{
    const unsigned char* p = (const unsigned char*)ppacket;

    if (nRecvSize <= 0)
    {
        printf("recv packet is empty\n");
        return 0;
    }

    // ----- header
    // [0] = SOH
    // [1] = my id
    // [2] = dev id
    // [3] = length
    // [4] = index
    // [4] = STX
    // ----- data
    // [5] ~ [5 + len - 1] = command
    // ----- tail
    // [size - 4] = [5 + len + 0] = ETX
    // [size - 3] = [5 + len + 1] = crc16-lo
    // [size - 2] = [5 + len + 2] = crc16-hi
    // [size - 1] = [5 + len + 3] = EOT

    // header check [0] == SOH
#if USE_CHECK_SWICHING_NOISE
    // check noise of first byte
    if (p[0] == 0xff && nRecvSize > 1 && p[1] == ASCII_SOH)
    {
        // first byte is TX/RX switching noise
        // [0] = 0xff (switching noise)
        // [1] = SOH
        // ...

        nRecvSize--;
        p++;

        printf("first byte garbage skipped\n");
    }
    else
#endif
    if (p[0] != ASCII_SOH)
    {
        // first byte is not SOH
        printf("recv[0] is not SOH: %d != %d\n",
                p[0],
                ASCII_SOH);

        return -1;
    }

    // header + tail = 9 bytes
    if (nRecvSize < 9)
    {
        printf("recv packet size too small: %d < 9\n", nRecvSize);
        return 9 - nRecvSize;   // more data required
    }

    // header check [1] == my(host) id as receiver
    if (p[1] != COMM_ID_HOST)
    {
        printf("recv[1] mismatch my identifier: %d != %d\n",
               p[1],
               COMM_ID_HOST);

        return -1;
    }

    // header check [2] == device id of sender (robot controller)
    if (p[2] != COMM_ID_ROBOT)
    {
        printf("recv[1] mismatch device identifier: %d != %d\n",
               p[2],
               COMM_ID_ROBOT);

        return -1;
    }

    // header check [5] == STX
    if (p[5] != ASCII_STX)
    {
        printf("recv[4] is not STX: %d != %d\n",
               p[4],
               ASCII_STX);

        return -1;
    }

    // check packet length
    int nLen = p[3];
    int nExpSize = 6 + nLen + 4;

#if USE_CHECK_SWICHING_NOISE
    // check tail byte TX/RX switching noise
    if (nRecvSize == nExpSize + 1 &&
        p[nRecvSize - 1] == 0xff &&
        p[nRecvSize - 2] == ASCII_EOT &&
        p[nRecvSize - 5] == ASCII_ETX)
    {
        printf("last byte is switching noise\n");
        nRecvSize--;
    }
    else
#endif
    if (nLen <= 0 || nRecvSize != nExpSize)
    {
        printf("recv packet size mismatch: %d(recv) != %d(info)\n",
               nRecvSize,
               nExpSize);

        return (nLen <= 0 || nRecvSize > nExpSize)
             ? (-1)
             : (nExpSize - nRecvSize);  // more data required
    }

    // user specified index
    int nUserIndex = p[4];
#if 0
    printf("User Index: %d\n", nUserIndex);
#else
    (void)nUserIndex;
#endif

    // tail check [-4] = ETX
    if (p[nRecvSize - 4] != ASCII_ETX)
    {
        printf("recv[-4] is not ETX: %d != %d\n",
               p[nRecvSize - 4],
               ASCII_ETX);

        return -1;
    }

    // tail check [-1] = EOT
    if (p[nRecvSize - 1] != ASCII_EOT)
    {
        printf("recv[-1] is not ETX: %d != %d\n",
            p[5 + nLen + 0],
            ASCII_EOT);

        return -1;
    }

    // check CRC16
    unsigned nCrc16Recv = ((unsigned)p[nRecvSize - 3]) + (((unsigned)p[nRecvSize - 2]) << 8) & 0xffff;
    unsigned nCrc16Calc = calc_crc16(p + 1, nLen + 5);

    if (nCrc16Recv != nCrc16Calc)
    {
        printf("recv[-3&-2] CRC16 mismatch: %u != %u\n",
               nCrc16Recv,
               nCrc16Calc);

        return -1;
    }

    // parse "REPLY" command only
    return parse_reply_command((const char *)p + 6, nLen);
}
