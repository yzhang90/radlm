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
// File:		usdigital.c
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
#include "usdigital.h"
#include "/usr/include/libusdusb4.h"

/* display debug info */
//#define D1 1
#define FALSE 0
#define TRUE  1



// Pause between reads of the USB4 counter
// in milliseconds.
//
const int COUNT_READ_PAUSE = 50000;

int init_landshark(char * port_name, int reset)
{
  int return_val=0;
  // Initialize the USB4 driver.
  short deviceCount = 0;
  printf("PD:USD:Init: start\n");

  int result = USB4_Initialize(&deviceCount);
  if (result != USB4_SUCCESS)
  {
    printf("PD:Failed to initialize USB4 driver!  Result code = %d.\n", result);
  }
  else
  {
    // Caution! The reset of the example is implemented without any error checking.


    // Configure encoder channel 0 (left side).
    // Set the preset register to the CPR-1 (max counter value 24 bit we want 16 bit to match old stuff)
    USB4_SetPresetValue(0,0,65535);
    USB4_SetMultiplier(0,0,3);              // Set quadrature mode to X4.
    USB4_SetCounterMode(0,0,3);             // Set counter mode to modulo-N.
    USB4_SetForward(0,0,TRUE);              // Optional: determines the direction of counting.
    USB4_SetCounterEnabled(0,0,TRUE);       // Enable the counter.**IMPORTANT**
    USB4_ResetCount(0,0);                    // Reset the counter to 0
    // Configure encoder channel 1 (right side).
    USB4_SetPresetValue(0,1,65535);           // Set the preset register to the CPR-1 (rollover value)
    USB4_SetMultiplier(0,1,3);              // Set quadrature mode to X4.
    USB4_SetCounterMode(0,1,3);             // Set counter mode to modulo-N.
    USB4_SetForward(0,1,TRUE);              // Optional: determines the direction of counting.
    USB4_SetCounterEnabled(0,1,TRUE);       // Enable the counter.**IMPORTANT**
    USB4_ResetCount(0,1);                    // Reset the counter to 0

    // USB4_SetControlMode(0,0,0xFC000);
    // You may replace the previous five lines with
    // one call to USB4_SetControlMode using to correct
    // control mode value.

    // output ports are controlled by register 46
    BOOL bTriggerOutSignalDrivesOutputPin[5] = {0, 0, 0, 0, 0};
    unsigned char ucTriggerSignalLengthCode = 1;
    USB4_SetOutputPortConfig(0, bTriggerOutSignalDrivesOutputPin,ucTriggerSignalLengthCode);
    // set output ports to zero
    USB4_WriteOutputPortRegister(0, 0);
    // set a/d sampling frequency
    USB4_SetA2DSamplingFrequency(0,0); // Sample at 11.111 kHz
  }

  printf("PD:USD:Init: end\n");

  return_val=1;
  return(return_val);
}

int shutdown_landshark()
{

  int return_val=0;
  // Close all open connections to the USB4 devices.
  USB4_Shutdown();

  return_val=1;
  return(return_val);
}

/* Configuration Commands */

int Configure_GPIO(int config)
{

  return 1;
}

/* Action Commands */

int Set_GPIO(int value)
{

  return 1;
}

int Set_High_Power_Output(int value)
{
  //printf("PD:SHPO:val=%d\n",value);
  if((value && 0x007F)==0)
    Set_LED(0);
  else
    Set_LED(1);
  return 1;
}

int Set_Pwm(char pwm_no, int duration)
{

  return 1;
}

int Set_Dac(char dac_no, int value)
{

  return 1;
}

int Set_LED(char led)
{
  unsigned char ucVal = 0x03; // MOSFET on for lowest 2 bits.

  USB4_ReadOutputPortRegister(0, &ucVal);
  //printf("pd:usdigital:before led output =%d  reg=%d\n",led,ucVal);
  if(led==0)
    ucVal=ucVal&0xef; // turn off led
  else
    ucVal=ucVal|0x10; // turn on led

  USB4_WriteOutputPortRegister(0, ucVal);
  //printf("pd:usdigital:after led output =%d  reg=%d\n",led,ucVal);

  return 1;
}

int Set_Motor_Speed(int motor, int direction, int speed )
{
  return 1;
}

int Set_Both_Motor_Speeds( int direction_L, int speed_L,int direction_R, int speed_R )
{
  /* set both motors, driver control */
  int L_uiD2Avalue=0;
  int R_uiD2Avalue=0;
  L_uiD2Avalue=abs(((float)speed_L/100) * 4095);
  R_uiD2Avalue=abs(((float)speed_R/100) * 4095);

  unsigned char ucVal = 0x03; // MOSFET on for lowest 2 bits.

  USB4_ReadOutputPortRegister(0, &ucVal);
  if(direction_L<0)
  {
    ucVal=ucVal&0xfc;
    ucVal=ucVal|0x02;
  }
  else
  {
    ucVal=ucVal&0xfc;
    ucVal=ucVal|0x01;
  }
  if(direction_R<0)
  {
    ucVal=ucVal&0xf3;
    ucVal=ucVal|0x08;
  }
  else
  {
    ucVal=ucVal&0xf3;
    ucVal=ucVal|0x04;
  }

  USB4_WriteOutputPortRegister(0, ucVal);

  USB4_SetD2A(0, 0, L_uiD2Avalue, FALSE);
  USB4_SetD2A(0, 1, R_uiD2Avalue, TRUE);

  //usleep(COUNT_READ_PAUSE);

  //printf("PD:USD: l=%d r=%d \n",L_uiD2Avalue,R_uiD2Avalue);


  //printf("matilda speed dir:l=%d r=%d\n",direction_L,direction_R);

  //printf("matilda speed cmd:l=%f r=%f\n",voltageL,voltageR);



  return 1;
}

int Set_Motor_Timeout(int timeout)
{
  /* set timout for motors */


  return 1;
}

int Stop_all_Motors()
{
  /* stop all motors */
  USB4_SetD2A(0, 0, 0, FALSE);
  USB4_SetD2A(0, 1, 0, TRUE);

  return 1;
}

int Set_Serial_Port_Speed_High()
{


  return 1;
}

int Set_Serial_Port_Speed_Low()
{
  return 1;
}

/*  Status Commands */



int Read_ADC_Values(int* adc)
{
  int a_bytes_read=10;
  unsigned short a2d=0;
  USB4_GetA2D (0, 0, &a2d);
  //printf("PD:adc: val=%f\n",((((double)a2d/4095)*5)));

  adc[4]=a2d;
  return a_bytes_read;
}

int Read_GPIO(int* gpio)
{


  return 1;
}

int Read_Encoders(unsigned int* enc)
{
  unsigned long left_currentCount=0;
  unsigned long right_currentCount=0;
  int bytes_read=10;

  USB4_GetCount(0,0,&left_currentCount);
  USB4_GetCount(0,1,&right_currentCount);

  enc[3]=(int)left_currentCount;enc[2]=(int)right_currentCount;enc[1]=0;enc[0]=0;
  //printf("PD:USD: el=%d er=%d \n",enc[3],enc[2]);
  //usleep(COUNT_READ_PAUSE);

  return bytes_read;

}

int Read_JoysticK_Buttons(int* joy)
{
  return 1;
}

int Read_Timeout(int* tim)
{
  int bytes_read=0;
  tim[0]=2;
  return bytes_read;

}

/* Mode Commands */

int JoyStick_Mode()
{
  return 1;
}

int Get_SerialNumber(unsigned long* pSerialNumber)
{
  if (pSerialNumber == NULL)
  {
    return -1;
  }

  short deviceNo = 0;
  unsigned short model = 0;
  unsigned short version = 0;
  unsigned char month = 0;
  unsigned char day = 0;
  unsigned short year = 0;

  int result = USB4_GetFactoryInfo(deviceNo, &model, &version, pSerialNumber, &month, &day, &year);
  if (result != USB4_SUCCESS)
  {
    return -1;
  }

  return 1;
}

