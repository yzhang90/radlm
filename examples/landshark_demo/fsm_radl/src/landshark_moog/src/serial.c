/************************************************************************


Copyright (c) 2012 by Black-I Robotics, Inc.

Black-I Robotics Inc. provides the Air Force Research Laboratory full and
open redistribution and use rights in source and binary forms, with or
without modification, provided the following conditions are met:

-    The purpose is to comply with and meet all needs of contract
     FA8751-12-C-0036 as determined by the Air Force Research Laboratory.

-    The purpose is for research and non-commercial use.

-    Redistribution of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

-    Redistribution in binary form must reproduce the above copyright 
     notice, this list of conditions and the following disclaimer in 
     the documentation and/or other materials provided with the 
     distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR THE CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
SUCH DAMAGE.


*************************************************************************/
// File:		serial.c 
// Version:		3.3b
// Written by:		Marty Cosgrove
// Date:		09/04/12
// Description:	

#include <stdio.h>   /* Standard input/output definitions */
#include <stdlib.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <ctype.h>
#include <sys/select.h>
#include "serial.h"

/* display debug info */
//#define D1 1
#define HIGHSPEED_SERIAL_PORT 1
// varable to watch amount of read failure on serial ports for each
// file descriptor used to tell caller to reset serial port
 int read_errors[255];
int serialportequalnull[255];
char tempdata[5000]="data";


/*
 ***************** NOTES *************************** 
NOT TRUE MARTY 12-27-09 -> Landshark board needs hardware flow control enabled 
landshark board  echoes back chars sent to it so they end up in the input buffer 
*/


int open_port(char * serial_port){
    /*  Open serial port 1.
        Returns the file descriptor on success or -1 on error. */

	int fd=-1; /* File descriptor for the port */
 
	fd = open(serial_port, O_RDWR | O_NOCTTY | O_NDELAY);
	if(strcmp(serial_port,"/dev/null")==0)
	{
		serialportequalnull[fd]=1;
	}
	else
	{
		serialportequalnull[fd]=0;

	}

	if (fd == -1){
		/* Could not open the port*/
		printf("open_port: Unable to open %s - \n",serial_port);
	}
        
	else
		/* normal (blocking) behavior for port reads */
		fcntl(fd, F_SETFL, 0);
	// init read_errors to zero
	read_errors[fd]=0;
	return (fd);
}

int close_port(int fd){
	/* close serial port */
	close(fd);
return 1;
}

int initport(int fd,speed_t speed) {
	/* init serial port to correct settings */

	struct termios options;
	int error=0;

	if(serialportequalnull[fd]==1)
		return 0;
	
	/* Get the current options for the port...*/
	error=tcgetattr(fd, &options);
	if(error==-1)
	{
		printf("serial error with tcgetattr fd= %d\n",fd);
		return error;
	}
	/* Set the baud rates to speed...*/
	error=cfsetispeed(&options, speed);
	if(error==-1)
	{
		printf("serial error with cfsetispeed\n");
		return error;
	}
	error=cfsetospeed(&options, speed);
	if(error==-1)
	{
		printf("serial error with cfsetospeed\n");
		return error;
	}
	/* Enable the receiver and set local mode...*/
	options.c_cflag |= (CLOCAL | CREAD);

        /* set for no parity (8N1) */
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	//options.c_cflag &= ~HUPCL;
	options.c_cflag |= CS8;

	/* disable hardware flow control */
	options.c_cflag &= ~CRTSCTS;

	/* disable software flow control */
	options.c_iflag &= ~(IXON | IXOFF | IXANY);

	/*Choosing Raw Input as opposed to Canonical  */
	/*Canonical input is line-oriented. 
	Input characters are put into a buffer which can be edited interactively 
	by the user until a CR (carriage return) or LF (line feed) character is received.*/
	/*Raw input is unprocessed. Input characters are passed 
	through exactly as they are received, when they are received.*/
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	
	/* turn output processing off */
	options.c_oflag &= ~(OPOST);

	/* Set the new options for the port...*/
	/* TCSAFLUSH -Flush input and output buffers and make the change*/
	/* TCSANOW - Make changes now without waiting for data to complete */
	/* TCSADRAIN - Wait until everything has been transmitted */
	error=tcsetattr(fd, TCSANOW, &options);
	if(error==-1)
	{
		printf("serial error with tcsetattr\n");
		return error;
	}
	/* The FNDELAY option causes the read function to return 0 
	if no characters are available on the port*/
	error=fcntl(fd, F_SETFL, FNDELAY);
	if(error==-1)
	{
		printf("serial error with fcntl\n");
		return error;
	}
	return 0;
}
#ifdef meep
int writeport(int fd, char *chars) {
    int len = strlen(chars);
    // stick a <CR> after the command if not X command
	if(chars[0]!='X'){
            chars[len] = CR; 
            chars[len+1] = 0x00; // terminate the string properly
            len=len+1;
        }
        int n = write(fd, chars, len);
	if (n < 0) {
#ifdef D1            
		fputs("write failed!\n", stderr);
                printf("%d\n",n);
                printf("SERIAL write error %d %s\n", errno, strerror(errno));
#endif
		return 0;
	}
        else{
#ifdef D1
            printf("START write\niOut=%d\nresult=%s\nEND WRITE\n",n,chars);
#endif
        }

	return 1;
}

int readport(int fd, char *result) {
	
	// check to see if fd is valid
	// if not return
	if ((fd == -1) || (serialportequalnull[fd] == 1))
		return -2;

	int iIn = read(fd, result, 254);
	// null terminates data read
	result[iIn] = 0x00;
	if (iIn < 0) {
		if (errno == EAGAIN) {
#ifdef D1                    
			printf("SERIAL EAGAIN ERROR : %s\n",strerror(errno));
#endif
			return 0;
		}
		else {
			printf("SERIAL read error %d %s\n", errno, strerror(errno));
			return 0;
		}
	}
	else
#ifdef D1
            printf("START READ\niIn=%d\nresult=%sEND READ\n",iIn,result);
#endif
	return iIn;
}
#endif
int readport_variable(int fd, char *result, int size) {
	
	//return 0;
	fd_set readfs;
	int maxfd=fd+1;
	struct timeval tv;
	// check to see if fd is valid
	// if not return
	if ((fd == -1) || (serialportequalnull[fd] == 1))
		return -2;

	tv.tv_sec=0;
	tv.tv_usec=10000;
	//tv.tv_usec=0;
 
	FD_ZERO(&readfs);
	FD_SET(fd,&readfs);
	select(maxfd,&readfs,NULL,NULL,&tv);
	if(FD_ISSET(fd,&readfs)==0)
	{
		//printf("serial port empty \n");
		read_errors[fd]++;
		if(read_errors[fd]>3)
			return -3;
		else
			return -1;
	}
	int iIn = read(fd,(void*) result,(size_t) size);
	// null terminates data read in
	if (iIn < 0) {
		if (errno == EAGAIN) {
#ifdef D1
                    printf("SERIAL EAGAIN ERROR : %s\n",strerror(errno));
#endif
                    return 0;
		} 
       		 else {
			printf("SERIAL read variable error return=%d err=%d %s\n",iIn, errno, strerror(errno));
			return 0;
		}
	}
	else
	{
		result[iIn] = 0x00;
#ifdef D1
		printf("START READ\niIn=%d\nresult=%sEND READ\n",iIn,result);
#endif
		read_errors[fd]=0;

		return iIn;
	}
	return 0;
}

int getbaud(int fd) {
	struct termios termAttr;
	int inputSpeed = -1;
	speed_t baudRate;

	// check to see if fd is valid
	// if not return
	if ((fd == -1) || (serialportequalnull[fd] == 1))
		return -2;


	tcgetattr(fd, &termAttr);
	/* Get the input speed.                              */
	baudRate = cfgetispeed(&termAttr);
	switch (baudRate) 
	{
		case B0:      inputSpeed = 0; break;
		case B50:     inputSpeed = 50; break;
		case B110:    inputSpeed = 110; break;
		case B134:    inputSpeed = 134; break;
		case B150:    inputSpeed = 150; break;
		case B200:    inputSpeed = 200; break;
		case B300:    inputSpeed = 300; break;
		case B600:    inputSpeed = 600; break;
		case B1200:   inputSpeed = 1200; break;
		case B1800:   inputSpeed = 1800; break;
		case B2400:   inputSpeed = 2400; break;
		case B4800:   inputSpeed = 4800; break;
		case B9600:   inputSpeed = 9600; break;
		case B19200:  inputSpeed = 19200; break;
		case B38400:  inputSpeed = 38400; break;
		case B115200:  inputSpeed = 115200; break;
	}
	return inputSpeed;
}
// this will do a raw write to the serial port and won't return after
// 5 write error
int serialRawWrite(int fd,char* data,int len)
{
	//printf("data string size=%d data=%s\n",len,data);
	int n =0,i=0;
	int length=len;
	int timeout_counter=0;

	// check to see if fd is valid
	// if not return
	if ((fd == -1)||(serialportequalnull[fd]==1))
		return -2;
	
	for(n=0,i=0;i<len;i=i+n,length=length-n){
		
		n=write(fd, &data[i], length);	
		if (n < 0) {
            		printf("SERIAL write error %d %s\n", errno, strerror(errno));
			n=0;
			// wait for buffer to clear
			if(getbaud(fd)==115200)
				usleep(10000);
			else
				usleep(100000);
	
			timeout_counter++;
			
		}
		if(timeout_counter>5)
		{
			printf("serial write:  timeout: cmd= %s\n",data);
			//usleep(100000);			
			//exit(0);
			return 0;
		}
		
		//printf("n=%d\n",n);			
		
		// need a write delay for data to get out
		// 

	}
	//printf("wrote %d chars to serial port \n",i);
	return i;

}


int serialReadStarttoEndString(int fd,char* data,char* startString, char* termString)
{
	size_t maxdataread =5000;
	char* start=NULL;
	char* end=NULL;
	int timeout_counter=0;

	// check to see if fd is valid
	// if not return
	if ((fd == -1) || (serialportequalnull[fd] == 1))
		return -2;
	//printf("startstr=%s \nendstr=%s\n",startString,termString);

	// read  serial port 
	int iIn = read(fd,(void *)tempdata, maxdataread);
	// null terminates data read
	if(iIn>0)
	{
		//printf("iIn=%d : %s \n",iIn,tempdata);
		tempdata[iIn] = 0x00;
		//printf("1 = %s\n",tempdata);
		// search for start string
		start=strstr(tempdata,startString);
		//printf("start=%s\n",start);
	}
	else
	{
		//printf("iIn =%d\n",iIn);
		iIn=0;  
	}

	while(start==NULL)
	{
		int bytes_read=0;

		// haven't found start string sleep for a little and
		// read in more
		if(getbaud(fd)==115200)
			usleep(100);
		else
			usleep(100000);

		// read  serial port 
		bytes_read = read(fd, (void*)(tempdata+iIn),(size_t)(maxdataread-iIn));
		if(bytes_read>0)
		{
			iIn+=bytes_read;
			// null terminates data read
			tempdata[iIn] = 0x00;
			start=strstr(tempdata,startString);
			//printf("S = %s\n",tempdata);
		}
		else
		{
			//printf("last_data = %s\n",tempdata);
			//printf("count-iIn=%d\n",maxdataread-iIn);
			//printf("SERIAL start read error %d %s\n", errno, strerror(errno));
			timeout_counter++;

		}

		if(timeout_counter>7)
		{
			printf("serial read: start timeout\n");
			printf("count-iIn=%zu\n",maxdataread-iIn);
			printf("READ_TIMEOUT start exceeded -- cmd: %s :: last_data = %s\n",data,tempdata);
			read_errors[fd]++;
			if(read_errors[fd]>2)
			{
				printf("SERIAL_READ_ERRORS fd=%d: start exceeded - resetting serial port!!!\n",fd);
				//tcflush(fd,TCIOFLUSH);
				//exit(0);
				return -3;
			}
			return 0;
		}

	}
	//printf("meep\n");
	// search for end string after start
	end=strstr(start,termString);
	//printf("2 = %s\n",tempdata);
	// reset timeout_counter
	timeout_counter=0;
	while(end==NULL)
	{
		int bytes_read=0;

		// haven't found end string sleep for a little and 
		// read in more
		if(getbaud(fd)==115200)
			usleep(100);
		else
			usleep(100000);

		// read  serial port 
		bytes_read= read(fd, (void*)(tempdata+iIn),(size_t)(maxdataread-iIn));
		if(bytes_read>0)
		{
			iIn+=bytes_read;
			// null terminates data read
			tempdata[iIn] = 0x00;
			end=strstr(start,termString);
			//printf("E = %s\n",tempdata);
			
		}
		else
		{
			//printf("last_data = %s\n",tempdata);
			//printf("count-iIn=%d\n",maxdataread-iIn);
			//printf("SERIAL end read error %d %s\n", errno, strerror(errno));
			timeout_counter++;

		}
		if(timeout_counter>7)
		{
			printf("serial read: end timeout\n");
			printf("count-iIn=%zu\n",maxdataread-iIn);
			printf("READ_TIMEOUT fd=%d end exceeded -- cmd: %s :: last_data = %s\n",fd,data,tempdata);
			read_errors[fd]++;
			if(read_errors[fd]>2)
			{
				printf("SERIAL_READ_ERRORS end exceeded - resetting serial port!!!\n");
				//tcflush(fd,TCIOFLUSH);
				//usleep(300000);				
				//exit(0);
				return -3;
			}
			return 0;
		}
	}

	memcpy(data,start,abs(start-end));
	//printf("rcvdata = %s\n",data);
	//printf("bytes_read= %d\n",abs(start-end));
	// if you make it thru set the read errors to zero you got a read from fd
	read_errors[fd]=0;
	return (int)(abs(start-end));
}
