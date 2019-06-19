/*
 * MotorDefines.h
 *
 * Created: 3/6/2018 11:20:59 PM
 *  Author: Zac
 */ 


#ifndef MOTORDEFINES_H_
#define MOTORDEFINES_H_

#define F_CPU 16000000
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <stdbool.h>
#include "../SDO.h"
#include "../can.h"
#include "../AT90CAN_UART.h"
#include "../general.h"
#include "../assert.h"


#define CAN_PACKET_SIZE 8
#define RECEIVE_MASK 0x7ff

#define BUCKETMOTORID 0x7F
#define BUCKETMOTOR_MOB 3

#define LEFTMOTORID 0x7D
#define LEFTMOTOR_MOB 0

#define RIGHTMOTORID 0x7E
#define RIGHTMOTOR_MOB 1

#define CONVEYORMOTORID 0x7C
#define CONVEYORMOTOR_MOB 2
#define MAXCURRENTCONVEYOR 5000

//safety constants for motor controllers
//strictly speaking the motor controllers should
//self protect, but better safe than sorry.
#define MAXRPM		 6000 //3080 is safe value printed on motors, can probably push this up to 4000 without worry if needed.
#define ACCEL_CONST 1000000 //in rev/min^2, value should be between 100k and 10k *this value will probably have to be changed under load.

//maximum current allowed through BG65x25
#define MAXCURRENTBG65 20000  //seting current limit to 10A, max is actually 20A.


//from msb to lsb, LeftMotor, RightMotor, ConveyorMotor, BucketMotor
#define  LEFTMOTOR_STATUS 7
#define  RIGHTMOTOR_STATUS 6
#define  CONVEYORMOTOR_STATUS 5
#define  BUCKETMOTOR_STATUS 4

circular_buffer LeftMotor_Buffer;
#define LeftMotor_Buffer_Size 4
circular_buffer RightMotor_Buffer;
#define RightMotor_Buffer_Size 4
circular_buffer ConveyorMotor_Buffer;
#define ConveyorMotor_Buffer_Size 4
circular_buffer BucketMotor_Buffer;
#define BucketMotor_Buffer_Size 4

#define COMMSTIMEOUT 100 //ms, longest delay I have seen between message and reply is 68ms
#define COMMSCHECKTIME 1 //ms, how long between checking comms during timeout period





#endif /* MOTORDEFINES_H_ */