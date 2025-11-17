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

void openEsp32Device(char* dev, int *fd);
void Esp32ReadData(int *fd, uint8_t* buf);
void Esp32WriteData(int *fd, uint8_t* buf);
