/*
 * CANFastTransfer.h
 *
 * Created: 10/26/2016 10:55:15 PM
 *  Author: reed
 */ 


#ifndef CANFASTTRANSFER_H_
#define CANFASTTRANSFER_H_

#define CANFAST_MOB 4
#define CONTROL_MOB 5
#define BEACON_MOB 6
#define TRANSMITMOB 13
#define BUFFER_SIZECAN 40
#define BUFFER_SIZETRANSMIT 90 //should be multiple of 3

 
//Init function
void initCANFastTransfer(void);
void beginCANFast(volatile int * ptr, unsigned int maxSize, unsigned char givenAddress); 

//RX functions
void SetReceiveMode(int input); 
int  ReceiveDataCAN(void); 

//TX functions
void ToSendCAN( unsigned int where, unsigned int what);
void ToSendCAN_Control(unsigned char where, unsigned int what);
void ToSendCAN_Beacon(unsigned char where, unsigned int what);
void sendDataCAN( unsigned int whereToSend);
void sendDataCAN_Control( unsigned int whereToSend);
void sendDataCAN_Beacon( unsigned int whereToSend);
int GetTransmitErrorCount(void); 

volatile int * receiveArrayAddressCAN; // this is where the data will go when it is received
unsigned char moduleAddressCAN; // the address of this module
unsigned int MaxIndex; 


//Circular buffer stuff
struct ringBufSCAN { // this is where the send data is stored before sending
	int buf[BUFFER_SIZECAN];
	int head;
	int tail;
	int count;
};
struct ringBufSCAN ring_buffer_CAN;

struct ringBufTRANSMIT { // this is where the send data is stored before sending
	int buf[BUFFER_SIZETRANSMIT];
	int head;
	int tail;
	int count;
};
struct ringBufTRANSMIT TransmitBuffer;



void Send_buffer_put(struct ringBufSCAN *_this, const unsigned int towhere, const unsigned int towhat);
void Transmit_buffer_put(struct ringBufTRANSMIT* _this, unsigned int dest, unsigned int index, unsigned int value);
unsigned int Send_buffer_get(struct ringBufSCAN* _this);
unsigned int Transmit_buffer_get(struct ringBufTRANSMIT* _this);
unsigned int Transmit_buffer_peek(struct ringBufTRANSMIT* _this);
void Send_buffer_flush(struct ringBufSCAN* _this, const int clearBuffer);
void Transmit_buffer_flush(struct ringBufTRANSMIT* _this, const int clearBuffer);
unsigned int Send_buffer_modulo_inc(const unsigned int value, const unsigned int modulus);
unsigned int Send_buffer_GetCount(struct ringBufSCAN* _this); 
unsigned int Transmit_buffer_GetCount(struct ringBufTRANSMIT* _this);

#endif /* CANFASTTRANSFER_H_ */