#ifndef XXPROTOH
#define XXPROTOH

#pragma pack(1)

#include <stdbool.h>

#include "..\\..\\xxTypes.h"
#include "..\\..\\xxTcp.h"

#include "xxsystem.h"
#include "Pins.h"

#define stPacketDone		0x8000

#define Swapb(x,y)  (x)^=(y);(y)^=(x);(x)^=(y)

#define naX     0
#define naY     1
#define naZ     2

#define USARTRS485	USART2

extern short int XYZ[3];

//
// Protocols
#define conNet  0
#define conCom  1
#define conWiFi 2
#define con485  3

extern TMessageHeader DHI, DHO, *dh;
extern TStateInfo si;
extern TModeInfo mi;
extern TToolParams tp, *pp;
extern TRtcTime rt;
extern TNetInfo ni;

extern int MsCounter;
extern int TsmTimeout, RcvTimeout, FlashTimeout, SpiTimeout, SendTimeout, NetTimeout; 
extern char NetInBuffer[1600];
extern short int XY[120], FlashOffset, offset;
extern char FlashInBuffer[4100], *FlashPtr;
extern long int LastBlock;

extern bool FlagBreak, ReLoad, FlagAxel, NetSleepPin;
extern bool FlagNet, FlagRtc, FlagRecord, FlagAdc;
extern bool FlagNetTsm, FlagNetRcv, NoEE;

extern int StepTimeCounter, SecCounter, SecNetCounter, SecExecute;

void MsDelay( long int ms );
void MksDelay( float mks );

// handler
#define ssWaitStay      1
#define ssStayStay      2
#define ssWaitMove      3

extern int StayStatus;

extern short int PrevAx[3], MinAx[3], MaxAx[3], dAx[3];
extern int ExecuteGo, RecordStayCounter, SecStayCounter;

bool HandlerInit( void );
void HandlerWait( void );
void HandlerWork( void );
void HandlerStatus( void ); 
void HandlerNets( void );
void HandlerStop( void );
void SetWorkMode( int mode );
void SendStatus( int  send );
int  IsMoving ( int value, int n );
void AverageAxel( void );
int  IncPage( bool ask );

int AskTool( int ask );
int FindFreeMarker( void );

// INIT
void InitPorts(void);
void InitUart(void);
void InitMCO(void);
void InitTimers( void );
void InitIntr( void );

// UART
void SendUart( void *data, int len );
int SendDataCom( void *Data, int size,
    int command, int commandEx,
    int param1, int param2 );

void InitReceiveUart( void *data, int maxcount, int timeout );
int CheckReceiveUart( void );

int SendUsByte( char byte );

// SD
#define sdReady 		0x80
#define sdHighCapacity		0x40
#define sdHighPerfomance	0x01

extern bool NoSD, FlagSDResponse, FlagSDError, FlagSDRead;
extern bool FlagSDComplete, FlagSDDone, FlagSDCommand, FlagSDReserv;
extern int *InSD, *OutSD, CountSD;

void SDInitRegs( void );
int ResetSD( void );
int InitSD( void );
int GetInterfaceSD( char pattern );
int GetStatusSD( void );
int SetAppSD( void );
int SetCRCSD( char crconoff );
int InitializeSD( char mode );
int GetOCRSD( void );
int ReadPage( int addr, int bytes, char *buffer );
int WritePage( int addr, int bytes, char *buffer );
int EraseSD( int beg, int num );

int ReadCardData( void );

// IPUTILS
int HandlerNetMessage( void );
int SendDataEx( void *Data, int size, int command, int commandEx, int param1, int param2 );
int ReplayData( void *data, int len );

extern TMessageHeaderEx mh, *pmh;
extern int TypeOfMessage;
extern int MaxLenPacket;
extern int HaveAddress;
extern char *NetRcvData;

// Encode
int EncodeCommand( void );

// RTC
int InitRtc( void );
int IsStartTime( void );
int SetRtc( TRtcTime *tt );
int GetRtc( void );

// Axel
int  AxelWhoMe( void );
void InitAxel( void );
void AxelReadXYZ( void *buf );

// Tool
void EmptyTool( void );

// 485
#define EnableRcv485()   USART2->CR1 |= (USART_CR1_RXNEIE)
#define DisableRcv485()  USART2->CR1 &= (~USART_CR1_RXNEIE)

#define stErrRtc	0x01
#define stErrSD 	0x02
#define stErrPower	0x04
#define stErrAx 	0x08
#define stErrMem	0x10
#define stErrTool	0x60
#define stErrBusy	0x80

extern char Status485;

void SetBaudRate485( int baud );
void Handler485( void );
void InitReceive485( int timeout );
int CheckReceive485( void );
int SendToTls( void *data, int len );
int Send485Packet( char *data, int lendata, char *head, int lenhead );

extern int ToolAddress;

extern char DOut[1512];
extern int HaveTlsCommand, ImTls, From485;

// EEPROM
/* Private define ------------------------------------------------------------*/

#define EE_START_ADDR	0

bool EEInit( void );
int EERead( int addr, void *data, int numdata );
int EEWrite( int addr, void *data, int numdata );

int FindSaveBlock( void );
int IncSaveBlock( int block );
int InitSaveBlock( int save );

int ReadSystemData( void *data );
int ReadMainTp( void *data );
int ReadSecTp( void *data );
int ReadCalb( void *data, int len );
int WriteMainTp( void *data );
int WriteSecTp( void *data );
int WriteCalb( void *data, int len );

int WriteStartParams( void *data );
int WriteBaudRate( void *data );
int ReadBaudRate( void *data );

int ReadRuns( void *data );
int WriteRuns( void *data, int offs, int len );
int ClearRuns( void );

int IsBlockBad( int block ) ;
int SetBadBlock( int block );
int InitBadBlocks( void );
int WriteBadBlocks( void *data );
int ReadBadBlocks( void *data );

#endif