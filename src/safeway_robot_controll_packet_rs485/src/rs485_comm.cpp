#include <stdlib.h>
#include <stdio.h>

#include "rs485_settings.hpp"

speed_t conv_baud_linux(int nBaud)
{
    speed_t baud;

    switch (nBaud)
    {
        // POSIX
    case 50: baud = B50; break;
    case 75: baud = B75; break;
    case 110: baud = B110; break;
    case 134: baud = B134; break;
    case 150: baud = B150; break;
    case 200: baud = B200; break;
    case 300: baud = B300; break;
    case 600: baud = B600; break;
    case 1200: baud = B1200; break;
    case 1800: baud = B1800; break;
    case 2400: baud = B2400; break;
    case 4800: baud = B4800; break;
    case 9600: baud = B9600; break;
    case 19200: baud = B19200; break;
    case 38400: baud = B38400; break;
        // non-POSIX
    case 57600: baud = B57600; break;
    case 115200: baud = B115200; break;
    case 230400: baud = B230400; break;
    case 460800: baud = B460800; break;
    case 500000: baud = B500000; break;
    case 576000: baud = B576000; break;
    case 921600: baud = B921600; break;
    case 1000000: baud = B1000000; break;
    case 1152000: baud = B1152000; break;
    case 1500000: baud = B1500000; break;
    case 2000000: baud = B2000000; break;
    case 2500000: baud = B2500000; break;
    case 3000000: baud = B3000000; break;
    case 3500000: baud = B3500000; break;
    case 4000000: baud = B4000000; break;
    default:
        baud = B0;
    }

    return baud;
}

int send_comm_dev(COMM_HANDLE hComm, const void* p, int size)
{
#if defined(_WIN32)
    DWORD dwBytesWritten;
    BOOL bSuccess = WriteFile(hComm,
                              p,
                              (DWORD)size,
                              &dwBytesWritten,
                              g_povCommWrite);

    if (!bSuccess && GetLastError() != ERROR_IO_PENDING)
    {
        // writing failure...
        printf("Cannot send data: req=%d, error=%d\n",
               size,
               GetLastError());

        return -1;
    }
    else if (!bSuccess)
    {
        // overlapped...
        dwBytesWritten = (DWORD)size;
    }

    return (int)dwBytesWritten;

#else
    return write(hComm, p, size);
#endif
}

COMM_HANDLE open_comm_dev(const char* pszDevName, int nBaud)
{
    /////////////////////////
    //
    //  Windows
    //
#if defined(_WIN32)
    // open the serial port
    HANDLE hComm = CreateFile(pszDevName,
                              GENERIC_READ | GENERIC_WRITE, // dwDesiredAccess
                              0,                            // dwShareMode
                              NULL,                         // lpSecurityAttributes

                              OPEN_EXISTING,                // dwCreationDisposition
#if MCMD_METHOD == MCMD_METHOD_ASYNC1 || MCMD_METHOD == MCMD_METHOD_ASYNC2
                              FILE_FLAG_OVERLAPPED,         // dwFlagsAndAttributes: use overrlapped for asynchronous reading
#else
                              0,                            // dwFlagsAndAttributes - no overlapped
#endif
                              NULL);                        // hTemplateFile

    if (hComm == INVALID_HANDLE_VALUE)
    {
        printf("Cannot open serial device: '%s', error=0x%08X\n",
               pszDevName,
               GetLastError());

        return INVALID_COMM_HANDLE;
    }

    // set communication buffer
    if (!SetupComm(hComm, 4096, 4096))
    {
        printf("Cannot set the serial buffer: error=0x%08uX\n", GetLastError());
        goto ON_ERROR;
    }

    // setup the communication attributes
    DCB dcb;

    ZeroMemory(&dcb, sizeof(dcb));

    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hComm, &dcb))
    {
        printf("Cannot read serial attributes: error=0x%08X\n", GetLastError());
        goto ON_ERROR;
    }

    dcb.DCBlength = sizeof(DCB);
    dcb.BaudRate = (DWORD)nBaud;
    dcb.fBinary = TRUE;
    dcb.fOutxCtsFlow = FALSE;   // no CTS output flow control
    dcb.fOutxDsrFlow = FALSE;   // no DSR output flow control
    dcb.fDtrControl = DTR_CONTROL_DISABLE;  // no DTR control
    dcb.fDsrSensitivity = FALSE;    // no DSR sensitivity
    dcb.fTXContinueOnXoff = FALSE;  // don't case
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fErrorChar = FALSE; // no error character
    dcb.fNull = FALSE;      // null character used
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fAbortOnError = FALSE;
    dcb.XonLim = 4096;  // don't care
    dcb.XoffLim = 4096; // don't care
    dcb.ByteSize = 8;   // byte size = 8 bits
    dcb.Parity = NOPARITY;  // no parity used
    dcb.StopBits = ONESTOPBIT;  // one stop bit used
    dcb.XonChar = 0;
    dcb.XoffChar = 0;
    dcb.ErrorChar = 0;
    dcb.EofChar = 0;
    dcb.EvtChar = 0;

    if (!SetCommState(hComm, &dcb))
    {
        printf("Cannot set serial attributes: error=0x%08X\n", GetLastError());
        goto ON_ERROR;
    }

    // set timeout
    COMMTIMEOUTS to;

    ZeroMemory(&to, sizeof(to));

    to.ReadIntervalTimeout = 0;         // 1ms interval timeout
    to.ReadTotalTimeoutMultiplier = 0;  // no multiplier
    to.ReadTotalTimeoutConstant = 1;    // 1 ms constant read timeout
    to.WriteTotalTimeoutMultiplier = 0; // no multiplier
    to.WriteTotalTimeoutConstant = 0;   // no constant write timeout

    if (!SetCommTimeouts(hComm, &to))
    {
        printf("Cannot set timeout: error=0x%08uX\n", GetLastError());
        goto ON_ERROR;
    }

    // overlapped
#if MCMD_METHOD == MCMD_METHOD_ASYNC1 || MCMD_METHOD == MCMD_METHOD_ASYNC2
    ZeroMemory(&s_ovCommRead, sizeof(s_ovCommRead));
    ZeroMemory(&s_ovCommWrite, sizeof(s_ovCommWrite));
    ZeroMemory(&s_ovCommEvent, sizeof(s_ovCommEvent));
    s_ovCommEvent.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    g_povCommRead = &s_ovCommRead;
    g_povCommWrite = &s_ovCommWrite;
    g_povCommEvent = &s_ovCommEvent;
#else
    g_povCommRead = NULL;
    g_povCommWrite = NULL;
    g_povCommEvent = NULL;
#endif

    // event mask: only serial data arrival event
    SetCommMask(hComm, EV_RXCHAR | EV_ERR);

    // success
    return hComm;

    // case of error: close the handle and error return
    ON_ERROR:
    CloseHandle(hComm);
    return INVALID_COMM_HANDLE;

    ///////////////////////////////////////
    //
    //  Linux
    //
#else
    // open serial port
    int fd = open(pszDevName, O_RDWR);

    if (fd == -1)
    {
        printf("Cannot open serial device: '%s'\n", pszDevName);
        return -1;
    }

    // convert the number to actual baud value
    speed_t baud = conv_baud_linux(nBaud);

    if (baud == B0)
    {
        printf("Unsupported baud rate: %d\n", nBaud);
        printf("Please use one of follow:\n");
        printf("    2400/4800/9600/38400/57600/115200/230400/460800/500000/576000/921600\n");
        printf("    1000000/1152000/1500000/2000000/2500000/3000000/3500000/4000000\n");
        goto ON_ERROR;
    }

    // setting tty attributes
    struct termios termios;

    if (tcgetattr(fd, &termios) != 0)
    {
        printf("Error %d from tcgetattr: %s\n", errno, strerror(errno));
        goto ON_ERROR;
    }

    termios.c_cflag &= ~CBAUD;      // clear baud
    termios.c_cflag &= ~PARENB;     // clear parity bit, disabling parity (most common)
    termios.c_cflag &= ~CSTOPB;     // clear stop field, only one stop bit used in communication (most common)
    termios.c_cflag &= ~CSIZE;      // clear all bits that set the data size
    termios.c_cflag |= CS8;         // 8 bits per byte (most common)
    termios.c_cflag &= ~CRTSCTS;    // disable RTS/CTS hardware flow control (most common)
    termios.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines (CLOCAL = 1)

    termios.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL);   // disable canonical and echos
    termios.c_lflag &= ~ISIG;   // disable interpretation of INTR, QUIT and SUSP

    termios.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    termios.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // disable any special handling of received bytes

    termios.c_oflag &= ~OPOST;  // prevent special interpretation of output bytes (e.g. newline chars)
    termios.c_oflag &= ~ONLCR;  // prevent conversion of newline to carriage return/line feed
    //termios.c_oflag &= ~OXTABS; // prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    //termios.c_oflag &= ~ONOEOT; // prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    termios.c_cc[VTIME] = 10;   // wait for up to 1s (0.1 sec unit), returning as soon as any data is received.
    termios.c_cc[VMIN] = 0;     // number of minimum characters

    // baud rate
    cfsetispeed(&termios, baud);
    cfsetospeed(&termios, baud);

    // save tty settings, also checking for error
    if (tcsetattr(fd, TCSANOW, &termios) != 0)
    {
        printf("Error %d from tcsetattr: %s\n", errno, strerror(errno));
        goto ON_ERROR;
    }

    // by testing, it needs time to apply the attributes
    msleep(100);

    // flush pre-cached data
    tcflush(fd, TCIOFLUSH);

    return fd;

    /////////////////////////
    //
    // case of error
    //

    ON_ERROR:

    close(fd);
    return -1;
#endif
}

void close_comm_dev(COMM_HANDLE hComm)
{
#if defined(_WIN32)
    CancelIo(hComm);
    CloseHandle(hComm);

#else
    close(hComm);
#endif
}

int recv_comm_dev(COMM_HANDLE hComm, void* p, int size)
{
#if defined(_WIN32) && (MCMD_METHOD == MCMD_METHOD_ASYNC1 || MCMD_METHOD == MCMD_METHOD_ASYNC2)
    DWORD dwBytesRead;

    // try read
    if (!ReadFile(hComm,
                  p,
                  (DWORD)size,
                  &dwBytesRead,
                  g_povCommRead) &&
        GetLastError() != ERROR_IO_PENDING) // overlapped
    {
        printf("Cannot recv data: req=%d, error=%d\n",
               size,
               GetLastError());

        return -1;
    }

    // check overlapped result
    //if (!GetOverlappedResult(hComm, g_povCommRead, &dwBytesRead, TRUE))
    if (!GetOverlappedResultEx(hComm, g_povCommRead, &dwBytesRead, 500, FALSE))
    {
        COMMTIMEOUTS to;
        GetCommTimeouts(hComm, &to);

        printf("overlapped error: error=%u\n", GetLastError());
        return -1;
    }

    // successfully received
    return (int)dwBytesRead;

#elif defined(_WIN32)
    DWORD dwBytesRead;

    // try read
    if (!ReadFile(hComm,
        p,
        (DWORD)size,
        &dwBytesRead,
        NULL)) // overlapped
    {
        printf("Cannot recv data: req=%d, error=%d\n",
            size,
            GetLastError());

        return -1;
    }

    // successfully received
    return (int)dwBytesRead;

#else
    return read(hComm, p, size);
#endif
}
