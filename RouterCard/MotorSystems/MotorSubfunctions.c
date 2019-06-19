/*
 * MotorSubfunctions.c
 *
 * Created: 3/6/2018 11:13:08 PM
 *  Author: Zac
 */ 

#include "MotorSubfunctions.h"

//motor status tracking
int MOTORSTATUS = 0xff; //assume all motors are innocent until proven guilty

BOOL SendNoVerify(SDO_packet Packet, circular_buffer* buffer, int MotorNumber)
{
	char temp[8];
	//if motor is already in error skip comms
	if((MOTORSTATUS & 1<<MotorNumber) == 0)
	{
		return FALSE;
	}
	//clear buffer of any already received messages (such as power up message).
	while(cb_size(buffer)>0)
	{
		cb_pop_front(buffer, temp);
	}
	//write data to motor
	while(!SDOWritePacket(Packet));
	return true;
}

//communication functions to verify motors are working as intended.
BOOL SendandVerify(SDO_packet Packet, circular_buffer* buffer, int MotorNumber)
{
	BOOL status = FALSE;
	char temp[8];
	//if motor is already in error skip comms
	if((MOTORSTATUS & 1<<MotorNumber) == 0)
	{
		return FALSE;
	}
	//clear buffer of any already received messages (such as power up message).
	while(cb_size(buffer)>0)
	{
		cb_pop_front(buffer, temp);
	}
	//write data to motor
	while(!SDOWritePacket(Packet));
	//check repeatedly for reply from motor controller
	for(int i = 0; i<COMMSTIMEOUT/COMMSCHECKTIME; i++)
	{
		if(cb_size(buffer)!=0)
		{
			cb_pop_front(buffer, temp);
			if(SDOVerifyReply(temp, Packet))
			{
				status = TRUE;
				break;
			}
		}
		_delay_ms(COMMSCHECKTIME);
	}
	//if nothing received before comms timeout declare motor bad.
	if (!status)
	{
		MOTORSTATUS &= ~(1<<MotorNumber);
	}
	return status;
}

BOOL ReadandVerify(SDO_packet Packet, circular_buffer* buffer, int MotorNumber, void *item)
{
	BOOL status = FALSE;
	char temp[8];
	//if motor is already in error skip comms
	if((MOTORSTATUS & 1<<MotorNumber) == 0)
	{
		return FALSE;
	}
	//clear buffer of any already received messages (such as power up message).
	while(cb_size(buffer)>0)
	{
		cb_pop_front(buffer, temp);
	}
	//write data to motor
	while(!SDOReadRequest(Packet));
	//check repeatedly for reply from motor controller
	for(int i = 0; i<COMMSTIMEOUT/COMMSCHECKTIME; i++)
	{
		if(cb_size(buffer)!=0)
		{
			cb_pop_front(buffer, temp);
			if(SDOVerifyRead(temp, Packet))
			{
				status = TRUE;
				memcpy(item, temp, sizeof(temp));
				break;
			}
		}
		_delay_ms(COMMSCHECKTIME);
	}
	//if nothing received before comms timeout declare motor bad.
	if (!status)
	{
		MOTORSTATUS &= ~(1<<MotorNumber);
	}
	return status;
}

union{
	char byted[4];
	long combined;
	} assembled;
long ArrayToLong(char temp[8])
{
	assembled.byted[3]=temp[4];
	assembled.byted[2]=temp[5];
	assembled.byted[1]=temp[6];
	assembled.byted[0]=temp[7];
// 	long result = ((((long)temp[5])<<8) + (((long)temp[6]) << 16) + (((long)temp[7]) << 24))+ temp[4] ; //turn LSB format into a long
// 	return result;
return assembled.combined;
}


BOOL ConveyorMotor_Status()
{
	if((MOTORSTATUS & 1<<CONVEYORMOTOR_STATUS) == 0)
	return FALSE;
	else
	return TRUE;
}

BOOL BucketMotor_Status()
{
	if((MOTORSTATUS & 1<<BUCKETMOTOR_STATUS) == 0)
	return FALSE;
	else
	return TRUE;
}

//functions to control motors




void ReceiveMotor3( CAN_packet *p, unsigned char mob) // interrupt callback
{
	cb_push_back(&ConveyorMotor_Buffer, p->data);
}

void ConveyorMotor_Init()
{
	SDO_packet ClearErrors = {CONVEYORMOTORID, 0x3000, 0x00, 0x01};
	SDO_packet SetMotor = {CONVEYORMOTORID, 0x3900, 0x00, 0x01}; //0 for brushed, 1 for brushless
	SDO_packet SetPoles = { CONVEYORMOTORID, 0x3910, 0x00, 10};
	SDO_packet MotorPolarity = { CONVEYORMOTORID, 0x3911, 0x00, 0x02};
	SDO_packet SetFeedBack = {CONVEYORMOTORID, 0x3350, 0x00, 2378}; //2410 for encoder feedback, 2378 for hall feedback
	SDO_packet SetSVELFeedBack = {CONVEYORMOTORID, 0x3550, 0x00, 2378};
	SDO_packet SetFeedBackResolution = {CONVEYORMOTORID, 0x3962, 0x00, 10};
	//SDO_packet CurrentLimitPos = {CONVEYORMOTORID, 0x3221, 0, MAXCURRENTBG65};
	//SDO_packet CurrentLimitNeg = {CONVEYORMOTORID, 0x3223, 0, MAXCURRENTBG65};
	//SDO_packet VelocityLimitPos = {CONVEYORMOTORID, 0x3321, 0x00, MAXRPM};
	//SDO_packet VelocityLimitNeg = {CONVEYORMOTORID, 0x3323, 0x00, MAXRPM};
	//SDO_packet VelocityFeedback = {CONVEYORMOTORID, 0x3350, 0, 0x0112}		//Set the velocity feedback to EMF (Brushed motor) - DEFAULT
	//SDO_packet RatedVoltage = {CONVEYORMOTORID, 0x3902, 0x00, 24000}; //not used, since default is 24000.
	//SDO_packet RatedSpeed = {CONVEYORMOTORID, 0x3901, 0x00, 129};
	SDO_packet PowerEnable = {CONVEYORMOTORID, 0x3004, 0x00, 0x01};

	cb_init(&ConveyorMotor_Buffer, ConveyorMotor_Buffer_Size, CAN_PACKET_SIZE);

	BOOL ret;
	ret=prepare_rx( CONVEYORMOTOR_MOB, 0x580 + CONVEYORMOTORID, RECEIVE_MASK, ReceiveMotor3); //all 0s forces comparison
	ASSERT( ret==0);

	SendandVerify(ClearErrors,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	SendandVerify(SetMotor,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	SendandVerify(SetPoles,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	SendandVerify(MotorPolarity,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	_delay_ms(2); //not sure why this is needed, but was having a problem communicating at start up.
	SendandVerify(SetFeedBack,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	SendandVerify(SetSVELFeedBack,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	SendandVerify(SetFeedBackResolution,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	//SendandVerify(RatedSpeed,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	
	//Set the conveyor max currents (velocity control)
	SDO_packet CurrentLimitPos = {CONVEYORMOTORID, 0x3221, 0, 10000};
	SDO_packet CurrentLimitNeg = {CONVEYORMOTORID, 0x3223, 0, 10000};

	SendandVerify(CurrentLimitNeg,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	SendandVerify(CurrentLimitPos,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	SendandVerify(PowerEnable,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	
	//while(!SDOWritePacket(MotorPolarity));
	//while(cb_size(&ConveyorMotor_Buffer)==0);
	//cb_pop_front(&ConveyorMotor_Buffer,temp);
}

void ReceiveMotor4( CAN_packet *p, unsigned char mob) // interrupt callback
{
	cb_push_back(&BucketMotor_Buffer, p->data);
}

void BucketMotor_Init()
{
	SDO_packet ClearErrors = {BUCKETMOTORID, 0x3000, 0x00, 0x01};
	SDO_packet SetMotor = {BUCKETMOTORID, 0x3900, 0x00, 1};
	SDO_packet SetPoles = {BUCKETMOTORID, 0x3910, 0x00, 8};
	SDO_packet CurrentLimitPos = {BUCKETMOTORID, 0x3221, 0, 10000}; //10A
	SDO_packet CurrentLimitNeg = {BUCKETMOTORID, 0x3223, 0, 10000}; //10A
	SDO_packet VelocityLimitPos = {BUCKETMOTORID, 0x3321, 0x00, MAXRPM};
	SDO_packet VelocityLimitNeg = {BUCKETMOTORID, 0x3323, 0x00, MAXRPM};
	//SDO_packet MotorPolarity = {BUCKETMOTORID, 0x3911, 0x00, 0x01};
	SDO_packet SetFeedBack = {BUCKETMOTORID, 0x3350, 0x00, 2410}; //encoder feedback
	SDO_packet SetEncoderResolution = {BUCKETMOTORID, 0x3962, 0x00, 4096}; //bg75pi has a resolution of 4096 counts/rev
	SDO_packet PowerEnable = {BUCKETMOTORID, 0x3004, 0x00, 0x01};

	cb_init(&BucketMotor_Buffer, BucketMotor_Buffer_Size, CAN_PACKET_SIZE);

	BOOL ret;
	ret=prepare_rx( BUCKETMOTOR_MOB, 0x580 + BUCKETMOTORID, RECEIVE_MASK, ReceiveMotor4); //all 0s forces comparison
	ASSERT( ret==0);

	SendandVerify(ClearErrors,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(SetMotor,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(SetPoles,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	_delay_ms(2); //not sure why this is needed, but was having a problem communicating at start up.
	SendandVerify(CurrentLimitNeg,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(CurrentLimitPos,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(VelocityLimitPos,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(VelocityLimitNeg,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(SetFeedBack,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(SetEncoderResolution,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(PowerEnable,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);

	//while(!SDOWritePacket(MotorPolarity));
	//while(cb_size(&BucketMotor_Buffer)==0);
	//cb_pop_front(&BucketMotor_Buffer,temp);
}



void ConveyorMotor_VelMode()
{
	SDO_packet ModeVel	 = {CONVEYORMOTORID, 0x3003, 0x00, 0x3};
	SDO_packet VEL_Acc = {CONVEYORMOTORID, 0x3380, 0x00, 100000}; //value should be between 100k and 10k
	SDO_packet VEL_Dec = {CONVEYORMOTORID, 0x3381, 0x00, 100000};

	SendandVerify(ModeVel,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	SendandVerify(VEL_Acc,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	SendandVerify(VEL_Dec,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
}

void ConveyorMotor_SetCurrent(int curr)
{
	if (curr > MAXCURRENTCONVEYOR)
	{
		curr = MAXCURRENTCONVEYOR;
	}
	if(curr < -1*MAXCURRENTCONVEYOR)
	{
		curr = -1*MAXCURRENTCONVEYOR;
	}
	
	SDO_packet DesiredCurrentPos = {CONVEYORMOTORID, 3221, 0x0, curr};
	SDO_packet DesiredCurrentNeg = {CONVEYORMOTORID, 3223, 0x0, curr};
	
	SendandVerify(DesiredCurrentPos,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	SendandVerify(DesiredCurrentNeg,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
}

void BucketMotor_VelMode()
{
	SDO_packet ModeVel	 = {BUCKETMOTORID, 0x3003, 0x00, 0x3};
	SDO_packet VEL_Acc = {BUCKETMOTORID, 0x3380, 0x00, ACCEL_CONST}; //value should be between 100k and 10k
	SDO_packet VEL_Dec = {BUCKETMOTORID, 0x3381, 0x00, ACCEL_CONST};
	
	SendandVerify(ModeVel,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(VEL_Acc,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(VEL_Dec,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
}

void BucketMotor_PosMode()
{
	SDO_packet ModePos = {BUCKETMOTORID,				0x3003, 0x00, 0x7};
	SDO_packet SetPositionWindow = {BUCKETMOTORID,		0x373A, 0x00, 1000};
	SDO_packet DesiredVelocity1000 = {BUCKETMOTORID,	0x3300, 0x0,  1000};
	SDO_packet ResetPosition = {BUCKETMOTORID,			0x3762, 0x00, 0x00};
	
	SendandVerify(ModePos,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(DesiredVelocity1000,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(SetPositionWindow,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(ResetPosition,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
}

void ConveyorMotor_SetVel( int Vel)
{
	if (Vel > MAXRPM)
	{
		Vel = MAXRPM;
	}
	if(Vel < -MAXRPM)
	{
		Vel = -MAXRPM;
	}

	SDO_packet DesiredVelocity = {CONVEYORMOTORID, 0x3300, 0x0, Vel};

	SendandVerify(DesiredVelocity,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
}

void ConveyorMotor_SetVelNoCommsSafety( int Vel)
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
	SDO_packet DesiredVelocity = {CONVEYORMOTORID, 0x3300, 0x0, Vel};
	while(cb_size(&ConveyorMotor_Buffer)>0)
	{
		cb_pop_front(&ConveyorMotor_Buffer, temp);
	}
	//write data to motor
	while(!SDOWritePacket(DesiredVelocity));
}

void BucketMotor_SetVel( int Vel)
{
	if (Vel > MAXRPM)
	{
		Vel = MAXRPM;
	}
	if(Vel < -MAXRPM)
	{
		Vel = -MAXRPM;
	}

	SDO_packet Continue = {BUCKETMOTORID, 0x3000, 0x00, 4};
	SDO_packet DesiredVelocity = {BUCKETMOTORID, 0x3300, 0x0, Vel};

	SendandVerify(Continue,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(DesiredVelocity,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
}

void BucketMotor_SetVelNoCommsSafety( int Vel)
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
	SDO_packet Continue = {BUCKETMOTORID, 0x3000, 0x00, 4};
	SDO_packet DesiredVelocity = {BUCKETMOTORID, 0x3300, 0x0, Vel};
	while(cb_size(&BucketMotor_Buffer)>0)
	{
		cb_pop_front(&BucketMotor_Buffer, temp);
	}
	//write data to motor
	while(!SDOWritePacket(Continue));
	while(!SDOWritePacket(DesiredVelocity));
}

void BucketMotor_MoveCounts( long Counts)
{
	SDO_packet Move = {BUCKETMOTORID, 0x3791, 0x00, Counts};
	SendandVerify(Move,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
}

void BucketMotor_MoveCountsNoCommsSafety( long Counts)
{
	SDO_packet Move = {BUCKETMOTORID, 0x3791, 0x00, Counts};
	char temp[8];
	while(cb_size(&BucketMotor_Buffer)>0)
	{
		cb_pop_front(&BucketMotor_Buffer, temp);
	}
	//write data to motor
	while(!SDOWritePacket(Move));
}

void BucketMotor_SetLimit(void)
{
	SDO_packet PositiveLimit = {BUCKETMOTORID, 0x3055, 0x00, 0x130}; //activates digital input 0 as positive limit switch high active
	SDO_packet NegativeLimit = {BUCKETMOTORID, 0x3056, 0x00, 0x132}; //activates digital input 1 as negative limit switch high active
	SDO_packet HomeLimit = {BUCKETMOTORID, 0x3056, 0x00, 0x132}; //activates digital input 2 as home limit switch high active
	//SDO_packet Home_Method = {BUCKETMOTORID, 0x37B2, 0x00, 2}; //homes by turning CW to positive limit, should be what we want.
	
	SendandVerify(PositiveLimit,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(NegativeLimit,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(HomeLimit,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	//SendandVerify(Home_Method,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
}



char ConveyorMotor_GetTemperature()
{
	char temp[8];
	SDO_packet ReadTemperature = {CONVEYORMOTORID, 0x3114, 0x00, 0x00};

	if(ReadandVerify(ReadTemperature, &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS, temp))
	{
		long result = ArrayToLong(temp);
		return result/10;
	}
	else
	{
		return 0;
	}
}

char BucketMotor_GetTemperature()
{
	char temp[8];
	SDO_packet ReadTemperature = {BUCKETMOTORID, 0x3114, 0x00, 0x00};

	if(ReadandVerify(ReadTemperature, &BucketMotor_Buffer, BUCKETMOTOR_STATUS, temp))
	{
		long result = ArrayToLong(temp);
		return result/10;
	}
	else
	{
		return 0;
	}
}



char ConveyorMotor_GetVoltage()
{
	char temp[8];
	SDO_packet ReadVoltage = {CONVEYORMOTORID, 0x3110, 0x00, 0x00};

	if(ReadandVerify(ReadVoltage, &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS, temp))
	{
		long result = ArrayToLong(temp);
		return (result/1000 +1); //get voltage in mV from controller, add 1v to compensate for suspected diode drop to input.
	}
	else
	{
		return 0;
	}
}

char BucketMotor_GetVoltage()
{
	char temp[8];
	SDO_packet ReadVoltage = {BUCKETMOTORID, 0x3110, 0x00, 0x00};

	if(ReadandVerify(ReadVoltage, &BucketMotor_Buffer, BUCKETMOTOR_STATUS, temp))
	{
		long result = ArrayToLong(temp);
		return (result/1000 +1); //get voltage in mV from controller, add 1v to compensate for suspected diode drop to input.
	}
	else
	{
		return 0;
	}
}



long ConveyorMotor_GetPos() {
	char temp[8];
	SDO_packet ReadVoltage = {CONVEYORMOTORID, 0x396A, 0x00, 0x00};

	if(ReadandVerify(ReadVoltage, &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS, temp))
	{
		long result = ArrayToLong(temp);
		return (result/4); //returns enocer counts
	}
	else
	{
		return 0;
	}
}

long BucketMotor_GetPos()
{
	char temp[8];
	SDO_packet ReadVoltage = {BUCKETMOTORID, 0x396A, 0x00, 0x00};

	if(ReadandVerify(ReadVoltage, &BucketMotor_Buffer, BUCKETMOTOR_STATUS, temp))
	{
		long result = ArrayToLong(temp);
		return (result/4); //returns enocer counts
	}
	else
	{
		return 0;
	}
}

char BucketMotor_Inputs()
{
	char temp[8];
	SDO_packet ReadPort0 = {BUCKETMOTORID, 0x3120, 0x00, 0x00};

	ReadandVerify(ReadPort0, &BucketMotor_Buffer, BUCKETMOTOR_STATUS, temp);
	long result = ArrayToLong(temp);
	return result;
}


void ConveyorMotor_ReEstablishComms()
{
	SDO_packet ClearErrors = {CONVEYORMOTORID, 0x3000, 0x00, 0x01};
	SDO_packet SetMotor = {CONVEYORMOTORID, 0x3900, 0x00, 0x00}; //0 for brushed, 1 for brushless
	SDO_packet CurrentLimitPos = {CONVEYORMOTORID, 0x3221, 0, 10000};
	SDO_packet CurrentLimitNeg = {CONVEYORMOTORID, 0x3223, 0, 10000};
	//SDO_packet RatedVoltage = {CONVEYORMOTORID, 0x3902, 0x00, 24000}; //not used, since default is 24000.
	SDO_packet RatedSpeed = {CONVEYORMOTORID, 0x3901, 0x00, 129};
	SDO_packet PowerEnable = {CONVEYORMOTORID, 0x3004, 0x00, 0x01};

	MOTORSTATUS |= (1<<CONVEYORMOTOR_STATUS);

	SendandVerify(ClearErrors,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	SendandVerify(SetMotor,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	_delay_ms(2); //not sure why this is needed, but was having a problem communicating at start up.
	SendandVerify(CurrentLimitNeg,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	SendandVerify(CurrentLimitPos,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	SendandVerify(RatedSpeed,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
	SendandVerify(PowerEnable,  &ConveyorMotor_Buffer, CONVEYORMOTOR_STATUS);
}

void BucketMotor_ReEstablishComms()
{
	SDO_packet ClearErrors = {BUCKETMOTORID, 0x3000, 0x00, 0x01};
	SDO_packet SetMotor = {BUCKETMOTORID, 0x3900, 0x00, 1};
	SDO_packet SetPoles = {BUCKETMOTORID, 0x3910, 0x00, 8};
	SDO_packet CurrentLimitPos = {BUCKETMOTORID, 0x3221, 0, 10000}; //10A
	SDO_packet CurrentLimitNeg = {BUCKETMOTORID, 0x3223, 0, 10000}; //10A
	SDO_packet VelocityLimitPos = {BUCKETMOTORID, 0x3321, 0x00, MAXRPM};
	SDO_packet VelocityLimitNeg = {BUCKETMOTORID, 0x3323, 0x00, MAXRPM};
	//SDO_packet MotorPolarity = {BUCKETMOTORID, 0x3911, 0x00, 0x01};
	SDO_packet SetFeedBack = {BUCKETMOTORID, 0x3350, 0x00, 2410}; //encoder feedback
	SDO_packet SetEncoderResolution = {BUCKETMOTORID, 0x3962, 0x00, 4096}; //bg75pi has a resolution of 4096 counts/rev
	SDO_packet PowerEnable = {BUCKETMOTORID, 0x3004, 0x00, 0x01};

	MOTORSTATUS |= (1<<BUCKETMOTOR_STATUS);

	SendandVerify(ClearErrors,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(SetMotor,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(SetPoles,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	_delay_ms(2); //not sure why this is needed, but was having a problem communicating at start up.
	SendandVerify(CurrentLimitNeg,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(CurrentLimitPos,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(VelocityLimitPos,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(VelocityLimitNeg,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(SetFeedBack,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(SetEncoderResolution,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
	SendandVerify(PowerEnable,  &BucketMotor_Buffer, BUCKETMOTOR_STATUS);
}

void setMotorStatus(uint8_t stat)
{
	MOTORSTATUS = stat;
	
}

uint8_t getMotorStatus ( void )
{
	
	return MOTORSTATUS;
}