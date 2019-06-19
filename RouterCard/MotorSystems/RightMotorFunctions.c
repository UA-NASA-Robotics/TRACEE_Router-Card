/*
 * RightMotorFunctions.c
 *
 * Created: 3/6/2018 11:15:34 PM
 *  Author: Zac
 */ 

#include "RightMotorFunctions.h"


void RightMotor_Init()
{
	SDO_packet ClearErrors = { RIGHTMOTORID, 0x3000, 0x00, 0x01};
	SDO_packet SetMotor = { RIGHTMOTORID, 0x3900, 0x00, 0x01};
	SDO_packet SetPoles = { RIGHTMOTORID, 0x3910, 0x00, 10};
	SDO_packet MotorPolarity = { RIGHTMOTORID, 0x3911, 0x00, 0x02};
	SDO_packet SetFeedBack = {RIGHTMOTORID, 0x3350, 0x00, 2410}; //2410 for encoder feedback, 2378 for hall feedback
	SDO_packet SetSVELFeedBack = {RIGHTMOTORID, 0x3550, 0x00, 2410};
	SDO_packet SetFeedBackResolution = {RIGHTMOTORID, 0x3962, 0x00, 500};
	SDO_packet CurrentLimitPos = {RIGHTMOTORID, 0x3221, 0, MAXCURRENTBG65};
	SDO_packet CurrentLimitNeg = {RIGHTMOTORID, 0x3223, 0, MAXCURRENTBG65};
	SDO_packet VelocityLimitPos = {RIGHTMOTORID, 0x3321, 0x00, MAXRPM};
	SDO_packet VelocityLimitNeg = {RIGHTMOTORID, 0x3323, 0x00, MAXRPM};
	SDO_packet PowerEnable = { RIGHTMOTORID, 0x3004, 0x00, 0x01};
	SDO_packet PowerDisable = { RIGHTMOTORID, 0x3004, 0x00, 0x00};
	//initialize circular buffer for right motor
	cb_init(&RightMotor_Buffer, RightMotor_Buffer_Size, CAN_PACKET_SIZE);
	//prepare RX receiving mob for motor.
	BOOL ret;
	ret=prepare_rx( RIGHTMOTOR_MOB, 0x580 + RIGHTMOTORID, RECEIVE_MASK, ReceiveMotor2); //all 0s forces comparison
	ASSERT( ret==0);

	//wipe errors. This first attempt at communication also verifies the controller is in the system.
	SendandVerify(ClearErrors,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
	//disable controller so some commands actually work (Feed back commands)
	SendandVerify(PowerDisable,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
	//set resolution of encoder (500 count encoder * 4 for how the controllers work)
	SendandVerify(SetFeedBack,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
	SendandVerify(SetSVELFeedBack,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
	SendandVerify(SetFeedBackResolution,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
	//configure motor, number of magnet poles and motor type.
	SendandVerify(SetMotor,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
	SendandVerify(SetPoles,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
	//set limits on current on velocity to protect motor
	_delay_ms(2); //not sure why this is needed, but was having a problem communicating at start up.
	SendandVerify(CurrentLimitPos,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
	SendandVerify(CurrentLimitNeg,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
	SendandVerify(VelocityLimitPos,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
	SendandVerify(VelocityLimitNeg,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
	//actually critical to get right if you are using encoders
	SendandVerify(MotorPolarity,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
	//re-enable motor controller.
	SendandVerify(PowerEnable,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
	
}

void RightMotor_VelMode()
{
	SDO_packet ModeVel	 = {RIGHTMOTORID, 0x3003, 0x00, 0x3};
	SDO_packet VEL_Acc = {RIGHTMOTORID, 0x3380, 0x00, ACCEL_CONST}; //value should be between 100k and 10k
	SDO_packet VEL_Dec = {RIGHTMOTORID, 0x3381, 0x00, ACCEL_CONST};

	SendandVerify(ModeVel,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
	SendandVerify(VEL_Acc,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
	SendandVerify(VEL_Dec,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
}

void ReceiveMotor2( CAN_packet *p, unsigned char mob) // interrupt callback
{
	cb_push_back(&RightMotor_Buffer, p->data);
}

BOOL RightMotor_Status()
{
	if((getMotorStatus() & 1<<RIGHTMOTOR_STATUS) == 0)
	return FALSE;
	else
	return TRUE;
}



void RightMotor_SetVel( int Vel)
{
	if (Vel > MAXRPM)
	{
		Vel = MAXRPM;
	}
	if(Vel < -MAXRPM)
	{
		Vel = -MAXRPM;
	}

	SDO_packet DesiredVelocity = {RIGHTMOTORID, 0x3300, 0x0, Vel};

	SendandVerify(DesiredVelocity,  &RightMotor_Buffer, RIGHTMOTOR_STATUS);
}

void RightMotor_SetVelNoCommsSafety( int Vel)
{
	if (Vel > MAXRPM)
	{
		Vel = MAXRPM;
	}
	if(Vel < -MAXRPM)
	{
		Vel = -MAXRPM;
	}
	char temp[8];
	SDO_packet DesiredVelocity = {RIGHTMOTORID, 0x3300, 0x0, Vel};
	while(cb_size(&RightMotor_Buffer)>0)
	{
		cb_pop_front(&RightMotor_Buffer, temp);
	}
	//write data to motor
	while(!SDOWritePacket(DesiredVelocity));

}


char RightMotor_GetTemperature()
{
	char temp[8];
	SDO_packet ReadTemperature = {RIGHTMOTORID, 0x3114, 0x00, 0x00};

	if(ReadandVerify(ReadTemperature, &RightMotor_Buffer, RIGHTMOTOR_STATUS, temp))
	{
		long result = ArrayToLong(temp);
		return result/10;
	}
	else
	{
		return 0;
	}
}

char RightMotor_GetVoltage()
{
	char temp[8];
	SDO_packet ReadVoltage = {RIGHTMOTORID, 0x3110, 0x00, 0x00};

	if(ReadandVerify(ReadVoltage, &RightMotor_Buffer, RIGHTMOTOR_STATUS, temp))
	{
		long result = ArrayToLong(temp);
		return (result/1000 +1); //get voltage in mV from controller, add 1v to compensate for suspected diode drop to input.
	}
	else
	{
		return 0;
	}
}

long RightMotor_GetPos()
{
	char temp[8];
	SDO_packet ReadPosition ={RIGHTMOTORID, 0x396A, 0x00, 0x00};// {RIGHTMOTORID, 0x396A, 0x00, 0x00};

	if(ReadandVerify(ReadPosition, &RightMotor_Buffer, RIGHTMOTOR_STATUS, temp))
	{
		long result = ArrayToLong(temp);
		return (result/4); //returns encoder counts
	}
	else
	{
		return 0;
	}
}

bool RightMotor_GetPosReached()
{
	char temp[8];
	SDO_packet ReadPositionReached ={RIGHTMOTORID, 0x3002, 0x00, 0x00};// {RIGHTMOTORID, 0x396A, 0x00, 0x00};

	if(ReadandVerify(ReadPositionReached, &RightMotor_Buffer, RIGHTMOTOR_STATUS, temp))
	{
		//long result = ArrayToLong(temp);
		return (temp[4]&0x10); //returns position Reached
	}
	else
	{
		return 0;
	}
}

void RightMotor_ReEstablishComms()
{
	SDO_packet ClearErrors = { RIGHTMOTORID, 0x3000, 0x00, 0x01};
	SDO_packet SetMotor = { RIGHTMOTORID, 0x3900, 0x00, 0x01};
	SDO_packet SetPoles = { RIGHTMOTORID, 0x3910, 0x00, 10};
	SDO_packet MotorPolarity = { RIGHTMOTORID, 0x3911, 0x00, 0x02};
	SDO_packet SetFeedBack = {RIGHTMOTORID, 0x3350, 0x00, 2410}; //2410 for encoder feedback, 2378 for hall feedback
	SDO_packet SetSVELFeedBack = {RIGHTMOTORID, 0x3550, 0x00, 2410};
	SDO_packet SetFeedBackResolution = {RIGHTMOTORID, 0x3962, 0x00, 2000};
	SDO_packet CurrentLimitPos = {RIGHTMOTORID, 0x3221, 0, 2000};
	SDO_packet CurrentLimitNeg = {RIGHTMOTORID, 0x3223, 0, 2000};
	SDO_packet VelocityLimitPos = {RIGHTMOTORID, 0x3321, 0x00, MAXRPM};
	SDO_packet VelocityLimitNeg = {RIGHTMOTORID, 0x3323, 0x00, MAXRPM};
	SDO_packet PowerEnable = { RIGHTMOTORID, 0x3004, 0x00, 0x01};
	SDO_packet PowerDisable = { RIGHTMOTORID, 0x3004, 0x00, 0x00};
	
	setMotorStatus(getMotorStatus() | (1<<RIGHTMOTOR_STATUS));

	//wipe errors. This first attempt at communication also verifies the controller is in the system.
	SendandVerify(ClearErrors,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
	//disable controller so some commands actually work (Feed back commands)
	SendandVerify(PowerDisable,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
	//set resolution of encoder (500 count encoder * 4 for how the controllers work)
	SendandVerify(SetFeedBack,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
	SendandVerify(SetSVELFeedBack,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
	SendandVerify(SetFeedBackResolution,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
	//configure motor, number of magnet poles and motor type.
	SendandVerify(SetMotor,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
	SendandVerify(SetPoles,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
	//set limits on current on velocity to protect motor
	_delay_ms(2); //not sure why this is needed, but was having a problem communicating at start up.
	SendandVerify(CurrentLimitPos,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
	SendandVerify(CurrentLimitNeg,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
	SendandVerify(VelocityLimitPos,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
	SendandVerify(VelocityLimitNeg,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
	//actually critical to get right if you are using encoders
	SendandVerify(MotorPolarity,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);
	//re-enable motor controller.
	SendandVerify(PowerEnable,  &LeftMotor_Buffer, RIGHTMOTOR_STATUS);

}
