/*
 * RightMotorFunctions.h
 *
 * Created: 3/6/2018 11:15:18 PM
 *  Author: Zac
 */ 


#ifndef RIGHTMOTORFUNCTIONS_H_
#define RIGHTMOTORFUNCTIONS_H_


#include "../can.h"
#include "MotorDefines.h"
#include "../assert.h"
#include "MotorSubfunctions.h"

BOOL RightMotor_Status();
void RightMotor_Init();
void RightMotor_VelMode();
void RightMotor_SetVel(int Vel);
void RightMotor_SetVelNoCommsSafety(int Vel);
char RightMotor_GetTemperature();
char RightMotor_GetVoltage();
long RightMotor_GetPos();
void RightMotor_ReEstablishComms();
bool RightMotor_GetPosReached();

void ReceiveMotor2( CAN_packet *p, unsigned char mob);




#endif /* RIGHTMOTORFUNCTIONS_H_ */