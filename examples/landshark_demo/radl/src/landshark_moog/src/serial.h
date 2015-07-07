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
// File:		serial.h 
// Version:		3.3b
// Written by:		Marty Cosgrove
// Date:		09/04/12
// Description:	

#ifndef SERIAL_H
#define SERIAL_H

#include <stdio.h>   /* Standard input/output definitions */
#include <stdlib.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <ctype.h>

/* hex codes for ascii chars */
#define CR 0x0d
#define ESC 0x1b



/*
serial io functions*/

int open_port(char * serial_port);
int close_port(int fd);
int initport(int fd, speed_t speed); 
int writeport(int fd, char *chars);
int readport(int fd, char *result) ;
int readport_variable(int fd, char *result, int size);
int getbaud(int fd) ;
int serialRawWrite(int fd,char* data,int len);
int serialReadStarttoEndString(int fd,char* data,char* startString,char* termString);
#endif

