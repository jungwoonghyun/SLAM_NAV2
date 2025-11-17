#include <termios.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <errno.h>
#include <linux/types.h>
#include <math.h>
#include <functional>

void init_keyboard();
void close_keyboard();
int _kbhit();
int _getch();
int _putch(int c);
