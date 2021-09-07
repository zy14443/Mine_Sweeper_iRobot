#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "createoi.h"
#include "createoi_patch.h"
#include "control.h"
#include <fcntl.h>

int user_start(); 

//Input event data format
typedef struct {
	struct timeval time;
	unsigned short type;
	unsigned short code;
	unsigned int value;
} input_event;

int main()
{

	if(user_start()==1)
	{
		startOI("/dev/ttyUSB0");
		enterFullMode();
		startControl();
	
		stopControl();
		stopOI();
	}
	
	return 0;
}

int user_start(){
	int fd;
	char *file_path = "/dev/input/event0";
	input_event *buffer;
	long pressed = -2;
	size_t nbytes;
	nbytes = sizeof(input_event);
	fd=open(file_path, O_RDONLY);//Blocking read
	if(fd >= 0) printf("File opened ..\n");
	pressed = read(fd,buffer,nbytes);//Poll for key-press
	close(fd);
	
	if(buffer->code)
		return 1;
	else
		return 0;
}
