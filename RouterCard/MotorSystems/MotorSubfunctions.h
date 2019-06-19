/*
 * MotorSubfunctions.h
 *
 * Created: 3/6/2018 11:13:20 PM
 *  Author: Zac
 */ 


#ifndef MOTORSUBFUNCTIONS_H_
#define MOTORSUBFUNCTIONS_H_

#include "MotorDefines.h"
#include "LeftMotorFunctions.h"
#include "RightMotorFunctions.h"

void ReceiveMotor3( CAN_packet *p, unsigned char mob);
void ReceiveMotor4( CAN_packet *p, unsigned char mob);

//communication functions to protect comms to motors.
BOOL SendNoVerify(SDO_packet Packet, circular_buffer* buffer, int MotorNumber);
BOOL SendandVerify(SDO_packet Packet, circular_buffer* buffer, int MotorNumber);
BOOL ReadandVerify(SDO_packet Packet, circular_buffer* buffer, int MotorNumber, void *item);
long ArrayToLong(char temp[8]);

void setMotorStatus(uint8_t stat);
uint8_t getMotorStatus ( void );

#endif /* MOTORSUBFUNCTIONS_H_ */