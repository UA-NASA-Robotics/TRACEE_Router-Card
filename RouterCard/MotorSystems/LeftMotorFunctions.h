/*
 * LeftMotorFunctions.h
 *
 * Created: 3/6/2018 11:15:48 PM
 *  Author: Zac
 */ 


#ifndef LEFTMOTORFUNCTIONS_H_
#define LEFTMOTORFUNCTIONS_H_

#include "../can.h"
#include "MotorDefines.h"
#include "../assert.h"
#include "MotorSubfunctions.h"

long LeftMotor_GetPos();
char LeftMotor_GetVoltage();
char LeftMotor_GetTemperature();
void LeftMotor_SetVelNoCommsSafety( int Vel);
void LeftMotor_SetVel( int Vel);
BOOL LeftMotor_Status();
void ReceiveMotor1( CAN_packet *p, unsigned char mob);
void LeftMotor_VelMode();
void LeftMotor_Init();
bool LeftMotor_GetPosReached();



#endif /* LEFTMOTORFUNCTIONS_H_ */