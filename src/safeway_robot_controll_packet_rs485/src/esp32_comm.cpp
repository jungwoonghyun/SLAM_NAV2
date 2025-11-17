#include "esp32_init.hpp"

void openEsp32Device(char* dev, int *fd)
{
  int i = 0;
  struct termios newtio;
	*fd = open(dev, O_RDWR | O_NOCTTY);
    //fd = open( "/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK );
	for(int j = 0;j < 10;j++)
	{
    if(*fd < 0)
    {
      fprintf(stderr, "Can't open file '%s': %s\r\n", dev, strerror(errno));
      i++;
    }
    else
    {
      printf("file open '%s'\r\n", dev);
      break;
    }
		// exit(EXIT_FAILURE);
    sleep(2);
    *fd = open(dev, O_RDWR | O_NOCTTY);
	}
  if(i == 10)
  {
    fprintf(stderr, "file open fail, system down '%s': %s\r\n", dev, strerror(errno));
    exit(EXIT_FAILURE);
  }

  memset(&newtio,0,sizeof(newtio));

  newtio.c_cflag = B115200;
  newtio.c_cflag |= CS8;
  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~(PARENB | PARODD);
  newtio.c_cflag &= ~CSTOPB;
  newtio.c_cflag &= ~CRTSCTS;

  newtio.c_iflag &= ~(IXON | IXOFF | IXANY);
  newtio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  newtio.c_oflag &= ~OPOST;

  newtio.c_cc[VTIME] = 1;
  newtio.c_cc[VMIN] = 1;


  // newtio.c_cflag = B115200;
  // newtio.c_cflag |= CS8;
  // newtio.c_cflag |= CLOCAL;
  // newtio.c_cflag |= CREAD;
  // newtio.c_iflag = IGNPAR;
  // //newtio.c_iflag = ICRNL;
  // newtio.c_oflag = 0;
  // newtio.c_lflag = 0;
  // newtio.c_cc[VTIME] = 0;
  // newtio.c_cc[VMIN] = 0;

  tcflush(*fd,TCIFLUSH);
  tcsetattr(*fd,TCSANOW,&newtio);
}

void Esp32ReadData(int *fd, uint8_t* buf)
{
  int set;
  set = read(*fd, buf, sizeof(buf));
  // return set;
}

void Esp32WriteData(int *fd, uint8_t* buf)
{
  int set;
  set = write(*fd, buf, sizeof(buf));
  // return set;
}
