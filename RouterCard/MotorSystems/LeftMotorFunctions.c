/*
 * LeftMotorFunctions.c
 *
 * Created: 3/6/2018 11:15:05 PM
 *  Author: Zac
 */ 

#include "LeftMotorFunctions.h"


void LeftMotor_Init()
{
	SDO_packet ClearErrors = { LEFTMOTORID, 0x3000, 0x00, 0x01};
	SDO_packet SetMotor = { LEFTMOTORID, 0x3900, 0x00, 0x01};
	SDO_packet SetPoles = { LEFTMOTORID, 0x3910, 0x00, 10};
	SDO_packet MotorPolarity = { LEFTMOTORID, 0x3911, 0x00, 0x02};
	SDO_packet SetFeedBack = {LEFTMOTORID, 0x3350, 0x00, 2410}; //2410 for encoder feedback, 2378 for hall feedback
	SDO_packet SetSVELFeedBack = {LEFTMOTORID, 0x3550, 0x00, 2410};
	SDO_packet SetFeedBackResolution = {LEFTMOTORID, 0x3962, 0x00, 500};
	SDO_packet CurrentLimitPos = {LEFTMOTORID, 0x3221, 0, MAXCURRENTBG65};
	SDO_packet CurrentLimitNeg = {LEFTMOTORID, 0x3223, 0, MAXCURRENTBG65};
	SDO_packet VelocityLimitPos = {LEFTMOTORID, 0x3321, 0x00, MAXRPM};
	SDO_packet VelocityLimitNeg = {LEFTMOTORID, 0x3323, 0x00, MAXRPM};
	SDO_packet PowerEnable = { LEFTMOTORID, 0x3004, 0x00, 0x01};
	SDO_packet PowerDisable = { LEFTMOTORID, 0x3004, 0x00, 0x00};
	//initialize circular buffer for left motor
	cb_init(&LeftMotor_Buffer, LeftMotor_Buffer_Size, CAN_PACKET_SIZE);
	//prepare RX receiving mob for motor.
	BOOL ret;
	ret=prepare_rx( LEFTMOTOR_MOB, 0x580 + LEFTMOTORID, RECEIVE_MASK, ReceiveMotor1); //all 0s forces comparison
	ASSERT( ret==0);

	//wipe errors. This first attempt at communication also verifies the controller is in the system.
	SendandVerify(ClearErrors,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	//disable controller so some commands actually work (Feed back commands)
	SendandVerify(PowerDisable,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	//set resolution of encoder (500 count encoder * 4 for how the controllers work)
	SendandVerify(SetFeedBack,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	SendandVerify(SetSVELFeedBack,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	SendandVerify(SetFeedBackResolution,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	//configure motor, number of magnet poles and motor type.
	SendandVerify(SetMotor,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	SendandVerify(SetPoles,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	//set limits on current on velocity to protect motor
	_delay_ms(2); //not sure why this is needed, but was having a problem communicating at start up.
	SendandVerify(CurrentLimitPos,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	SendandVerify(CurrentLimitNeg,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	SendandVerify(VelocityLimitPos,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	SendandVerify(VelocityLimitNeg,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	//actually critical to get right if you are using encoders
	SendandVerify(MotorPolarity,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	//re-enable motor controller.
	SendandVerify(PowerEnable,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	
}

void LeftMotor_VelMode()
{
	SDO_packet ModeVel	 = {LEFTMOTORID, 0x3003, 0x00, 0x3};
	SDO_packet VEL_Acc = {LEFTMOTORID, 0x3380, 0x00, ACCEL_CONST};
	SDO_packet VEL_Dec = {LEFTMOTORID, 0x3381, 0x00, ACCEL_CONST};
	
	SendandVerify(ModeVel,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	SendandVerify(VEL_Acc,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	SendandVerify(VEL_Dec,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
}

void ReceiveMotor1( CAN_packet *p, unsigned char mob) // interrupt callback
{
	cb_push_back(&LeftMotor_Buffer, &p->data);
}

BOOL LeftMotor_Status()
{
	if((getMotorStatus() & 1<<LEFTMOTOR_STATUS) == 0)
	return FALSE;
	else
	return TRUE;
}


void LeftMotor_SetVel( int Vel)
{
	if (Vel > MAXRPM)
	{
		Vel = MAXRPM;
	}
	if(Vel < -1*MAXRPM)
	{
		Vel = -1*MAXRPM;
	}

	SDO_packet DesiredVelocity = {LEFTMOTORID, 0x3300, 0x0, Vel};
	
	SendandVerify(DesiredVelocity,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
}

void LeftMotor_SetVelNoCommsSafety( int Vel)
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
	SDO_packet DesiredVelocity = {LEFTMOTORID, 0x3300, 0x0, Vel};
	while(cb_size(&LeftMotor_Buffer)>0)
	{
		cb_pop_front(&LeftMotor_Buffer, temp);
	}
	//write data to motor
	while(!SDOWritePacket(DesiredVelocity));
}


//returns temp of controller to nearest degree C

char LeftMotor_GetTemperature()
{
	char temp[8];
	SDO_packet ReadTemperature = {LEFTMOTORID, 0x3114, 0x00, 0x00};

	if(ReadandVerify(ReadTemperature, &LeftMotor_Buffer, LEFTMOTOR_STATUS, temp))
	{
		long result = ArrayToLong(temp);
		return result/10;
	}
	else
	{
		return 0;
	}
}


//returns voltage to nearest volt

char LeftMotor_GetVoltage()
{
	char temp[8];
	SDO_packet ReadVoltage = {LEFTMOTORID, 0x3110, 0x00, 0x00};

	if(ReadandVerify(ReadVoltage, &LeftMotor_Buffer, LEFTMOTOR_STATUS, temp))
	{
		long result = ArrayToLong(temp);
		return (result/1000 +1); //get voltage in mV from controller, add 1v to compensate for suspected diode drop to input.
	}
	else
	{
		return 0;
	}
}

long LeftMotor_GetPos()
{
	char temp[8];
	SDO_packet ReadPosition = {LEFTMOTORID, 0x396A, 0x00, 0x00};

	if(ReadandVerify(ReadPosition, &LeftMotor_Buffer, LEFTMOTOR_STATUS, temp))
	{
		long result = ArrayToLong(temp);
		return (result/4); //returns enocer counts
	}
	else
	{
		return 0;
	}
}

bool LeftMotor_GetPosReached()
{
	char temp[8];
	SDO_packet ReadPositionReached ={LEFTMOTORID, 0x3002, 0x00, 0x00};// {RIGHTMOTORID, 0x396A, 0x00, 0x00};

	if(ReadandVerify(ReadPositionReached, &LeftMotor_Buffer, LEFTMOTOR_STATUS, temp))
	{
		//long result = ArrayToLong(temp);
		return (temp[4]&0x10); //returns position Reached
	}
	else
	{
		return 0;
	}
}


void LeftMotor_ReEstablishComms()
{
	SDO_packet ClearErrors = { LEFTMOTORID, 0x3000, 0x00, 0x01};
	SDO_packet SetMotor = { LEFTMOTORID, 0x3900, 0x00, 0x01};
	SDO_packet SetPoles = { LEFTMOTORID, 0x3910, 0x00, 10};
	SDO_packet MotorPolarity = { LEFTMOTORID, 0x3911, 0x00, 0x02};
	SDO_packet SetFeedBack = {LEFTMOTORID, 0x3350, 0x00, 2378}; //2410 for encoder feedback, 2378 for hall feedback
	SDO_packet SetSVELFeedBack = {LEFTMOTORID, 0x3550, 0x00, 2378};
	SDO_packet SetFeedBackResolution = {LEFTMOTORID, 0x3962, 0x00, 2000};
	SDO_packet CurrentLimitPos = {LEFTMOTORID, 0x3221, 0, 2000};
	SDO_packet CurrentLimitNeg = {LEFTMOTORID, 0x3223, 0, 2000};
	SDO_packet VelocityLimitPos = {LEFTMOTORID, 0x3321, 0x00, MAXRPM};
	SDO_packet VelocityLimitNeg = {LEFTMOTORID, 0x3323, 0x00, MAXRPM};
	SDO_packet PowerEnable = { LEFTMOTORID, 0x3004, 0x00, 0x01};
	SDO_packet PowerDisable = { LEFTMOTORID, 0x3004, 0x00, 0x00};

	setMotorStatus(getMotorStatus() | (1<<LEFTMOTOR_STATUS));
	//wipe errors. This first attempt at communication also verifies the controller is in the system.
	SendandVerify(ClearErrors,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	//disable controller so some commands actually work (Feed back commands)
	SendandVerify(PowerDisable,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	//set resolution of encoder (500 count encoder * 4 for how the controllers work)
	SendandVerify(SetFeedBack,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	SendandVerify(SetSVELFeedBack,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	SendandVerify(SetFeedBackResolution,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	//configure motor, number of magnet poles and motor type.
	SendandVerify(SetMotor,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	SendandVerify(SetPoles,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	//set limits on current on velocity to protect motor
	_delay_ms(2); //not sure why this is needed, but was having a problem communicating at start up.
	SendandVerify(CurrentLimitPos,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	SendandVerify(CurrentLimitNeg,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	SendandVerify(VelocityLimitPos,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	SendandVerify(VelocityLimitNeg,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	//actually critical to get right if you are using encoders
	SendandVerify(MotorPolarity,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
	//re-enable motor controller.
	SendandVerify(PowerEnable,  &LeftMotor_Buffer, LEFTMOTOR_STATUS);
}