#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <stdio.h>

#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

// Termios
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <string.h>

#include "Sensor.h"

#define DEV_NAME "/dev/ttyUSB0"

// Global variable
#define STX 8
#define LFCR 4

#define MAX_LENGTH 39

unsigned char _tehuOn[]="AT+TEHU=1\n";
unsigned char _connect[] = "AT+DEST=00006D\n";

int _fd;
int write_bit = 0;
int i = 0;

unsigned char _readBuf[MAX_LENGTH]={0};
unsigned char _temp[MAX_LENGTH];
unsigned char _data[8]="AT+DATA=";
unsigned char _ETX[1] = {0x0D};

long int tmp = 0;
float tmp2 = 0;

JNIEXPORT jstring JNICALL Java_Sensor_sense
(JNIEnv *env, jobject obj){
	struct termios tio;
	long _buadrate = 115200;

	if((_fd = open(DEV_NAME, O_RDWR | O_NOCTTY | O_NDELAY)) < 0 )
	{
		printf("cannot open device %s (%d)\n", DEV_NAME, _fd);
		printf("errno = %d, %s \n",errno, strerror(errno));
		return;
	}

#if 1
	tio.c_cflag = _buadrate|CS8|CREAD|CLOCAL;
	tio.c_cflag &= ~HUPCL;
	tio.c_lflag = 0;
	tio.c_iflag = IGNPAR|ICRNL;
	tio.c_oflag = 0;

	tcflush(_fd,TCIFLUSH);
	tcsetattr(_fd, TCSANOW, &tio);
#endif


#if 0
	tio.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	tio.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
	tio.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);;

	tio.c_cflag &= ~(CSIZE | PARENB);
	tio.c_cflag |= CS8;

	tio.c_oflag &= ~OLCUC;
	
	tio.c_cc[VMIN] = 1;
	tio.c_cc[VTIME] = 10;
	
	cfsetispeed(&tio, B115200);
	cfsetospeed(&tio, B115200);

	tcflush(_fd, TCIFLUSH);
	tcsetattr(_fd, TCSAFLUSH, &tio);
#endif

	fcntl(_fd, F_SETFL, FNDELAY);

    unsigned char _result[4][5];
    for(i=0;i<5;i++){
        strcpy(_result[i],"");
    }

	while(1)
	{
		read(_fd, _readBuf, MAX_LENGTH);
		if(_readBuf[0]==0x7e && _readBuf[1]==0x45 && _readBuf[2] == 0x00 && _readBuf[7] == 0x1A)
        {
            sprintf(_result[0], "%d", _readBuf[14]);
            sprintf(_result[1], "%d", _readBuf[29]);
            sprintf(_result[2], "%d", _readBuf[30]);
            tmp = _readBuf[31]*256 + _readBuf[32];
            tmp2 = tmp/10.0;
            sprintf(_result[3], "%.0f", tmp2);
            usleep(300000);
            break;
		}
		else{
			usleep(100000);
		}
	}
    unsigned char result[20] = "";
    for(i=0; i<4; i++){
        if(i<3){
            strcat(_result[i], ":");
        }
        strcat(result, _result[i]);
    }
    jstring jresult;
    jresult = (*env)->NewStringUTF(env, result);
	close(_fd);
    return jresult;
}
