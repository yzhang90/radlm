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
// File:		usdigital.h
// Version:		3.3b
// Written by:		Marty Cosgrove
// Date:		09/04/12
// Description:

#ifndef USDIGITAL_H
#define USDIGITAL_H

#include <stdio.h>   /* Standard input/output definitions */
#include <stdlib.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <ctype.h>



int init_landshark(char * port_name,int reset);
int shutdown_landshark();
/* Configuration Commands */
int Configure_GPIO(int config);
/* Action Commands */
int Set_GPIO(int value);
int Set_High_Power_Output(int value);
int Set_Pwm(char pwm_no, int duration);
int Set_Dac(char dac_no, int value);
int Set_LED(char led);
int Set_Motor_Speed(int motor, int direction, int speed );
int Set_Both_Motor_Speeds( int direction_L, int speed_L,int direction_R, int speed_R );
int Set_Motor_Timeout(int timeout);
int Stop_all_Motors();
int Set_Serial_Port_Speed_High();
int Set_Serial_Port_Speed_Low();
/*  Status Commands */
int Read_ADC_Values(int* adc);
int Read_GPIO(int* gpio);
int Read_Encoders(unsigned int* enc);
int Read_JoysticK_Buttons(int* joy);
int Read_Timeout(int* tim);
/* Mode Commands */
int JoyStick_Mode();
/* Added by Thomas */
int Get_SerialNumber(unsigned long* pSerialNumber);
#endif

