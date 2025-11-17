#include "uwb_init.hpp"

const uint8_t node_command[2] = {0x30, 0x00};
const uint8_t loc_command[2] = {0x0c, 0x00};

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

void readDWMNodeNum(int* fd, DWMNode_ID* nodeID){
    int id_read = 0;
    int cnt = 0;

    while(id_read < 13  && cnt < 10){
        write(*fd, node_command,2);
        id_read = read(*fd, nodeID,sizeof(DWMNode_ID));

        usleep(100000);
        cnt++;
    }
}

unsigned int readDWMLocData(int* fd, DWMDATA_tag* locData){
    int read_num = 0;
    int distance;

    write(*fd,loc_command,2);
    read_num = read(*fd, locData, sizeof(DWMDATA_tag));

    distance =
    + locData->TagInfo[0].distance[3] * 256 * 256 * 256
    + locData->TagInfo[0].distance[2] * 256 * 256
    + locData->TagInfo[0].distance[1] * 256
    + locData->TagInfo[0].distance[0];

    return distance;
}

int uwbPointSelector(DWMNode_ID* nodeId, DWMDATA_tag* locData){
	if(locData->TagInfo[0].UWBaddr[1] == TAG_ADDR_1 && locData->TagInfo[0].UWBaddr[0] == TAG_ADDR_2){
		if(nodeId->nodeId[1] == ANC_ADDR_0_1 && nodeId->nodeId[0] == ANC_ADDR_0_2){ //first point
			return 0;
		}
		else if(nodeId->nodeId[1] == ANC_ADDR_1_1 && nodeId->nodeId[0] == ANC_ADDR_1_2){ //second point
			return 1;
		}
		else if(nodeId->nodeId[1] == ANC_ADDR_2_1 && nodeId->nodeId[0] == ANC_ADDR_2_2){ //third point
			return 2;
		}
		else{
			return -1;
		}
	}
	else{
		return -1;
	}

}
