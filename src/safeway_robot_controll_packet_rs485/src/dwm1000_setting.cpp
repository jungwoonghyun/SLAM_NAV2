// UWB 설정을 위한 헤더파일
#include "dwm1000_init.hpp"

#define ANC_ADDR_0_0  0xDA
#define ANC_ADDR_0_1  0x8C

#define ANC_ADDR_1_0  0xAE
#define ANC_ADDR_1_1  0xD4

#define ANC_ADDR_2_0  0x4E
#define ANC_ADDR_2_1  0x57


void openDevice(char* dev, int* fd){
  struct termios newtio;
	*fd = open(dev, O_RDWR | O_NOCTTY);
    //fd = open( "/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK );
	if (*fd < 0)
	{
		fprintf(stderr, "Can't open file '%s': %s\r\n", dev, strerror(errno));
		exit(EXIT_FAILURE);
	}

    memset(&newtio,0,sizeof(newtio));

    newtio.c_cflag = B115200;
    newtio.c_cflag |= CS8;
    newtio.c_cflag |= CLOCAL;
    newtio.c_cflag |= CREAD;
    newtio.c_iflag = IGNPAR;
    //newtio.c_iflag = ICRNL;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    tcflush(*fd,TCIFLUSH);
    tcsetattr(*fd,TCSANOW,&newtio);
}

unsigned int readDWM1000Data(int* fd, DWM1000_INFO* locData){
  int read_num = 0;
  unsigned int distance;
  uint16_t sum = 0;
  uint16_t errCheckbit = 0;

  read_num = read(*fd, locData, sizeof(DWM1000_INFO));
  sum = locData->shtAddr_dev[0] + locData->shtAddr_dev[1]
      + locData->shtAddr_tag[0] + locData->shtAddr_tag[1]
      + locData->dev_tagDist[0] + locData->dev_tagDist[1];

  errCheckbit = (sum + (uint16_t)(locData->errCheck)) % 256; // mm
  if(errCheckbit == 0)
  {
    distance = locData->dev_tagDist[0] * 256 + locData->dev_tagDist[1];
  }
  else
  {
    distance = 0;
  }

  return distance;
}

int uwbPointSelector(DWM1000_INFO* locData)
{
//  printf("%X, %X\r\n", locData->shtAddr_dev[0], locData->shtAddr_dev[1]);
	if(locData->shtAddr_dev[0] == 0xDA && locData->shtAddr_dev[1] == 0x8C)
  {
    return 0;
	}
	else if(locData->shtAddr_dev[0] == 0xAE && locData->shtAddr_dev[1] == 0xD4)
  {
    return 1;
	}
	else if(locData->shtAddr_dev[0] == 0x4E && locData->shtAddr_dev[1] == 0x57)
  {
    return 2;
	}
  else
  {
    return -1;
  }
}
