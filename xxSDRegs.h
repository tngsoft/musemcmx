#ifndef XXSDREGSH
#define XXSDREGSH

#pragma pack(1)

#include <stdbool.h>

typedef struct {
  char Command;
  long int Param;
  char CheckSum;
} TSDCommand;

typedef struct {
  char Command;
  char Params[4];
  char CheckSum;
} TSDCharCommand;

typedef struct {
  char Manufacturer;
  char OEMID[2];
  char Name[5];
  char Revision;
  long int Serial;
  short int Date;
  char CRC7;
} TCardIdent;

typedef struct {
  char Version;
  unsigned char TAAC;
  unsigned char NAAC;
  unsigned char Speed;
  short int MaxReadBlockLength;
  long  int DeviceSize;
  short int Reserved[2];
  unsigned char Protect;
  char CRC7;
} TCardData;

extern TSDCommand *SD;
extern TCardData CD;
extern TCardIdent CI;

#define sdcReset	 0
#define sdcInit		41
#define sdcGetOCR	58
#define sdcCRCOff	59
#define sdcRead		17
#define sdcWrite	24
#define sdcEraseBeg	32
#define sdcEraseEnd	33
#define sdcErase	38

#define sdcCardSpecific	9
#define sdcCardIdent	10

#define SDIO_CLK_MASK		0xFFFF8000
#define SDIO_CMD_MASK		0xFFFF8000
#define SDIO_CLKDIV_MASK	0xFFFFFF00
#define SDIO_CLKENA_MASK	0xFFFFFEFF

// SDIO CONTROL CLOCK
#define sdclkHWFLOW	0x4000
#define sdclkNegEdge	0x2000
#define sdclkBus1	0x0000
#define sdclkBus4	0x0800
#define sdclkBus7	0x1000
#define sdclkBypass	0x0400
#define sdclkPowerSave	0x0200
#define sdclkEnable	0x0100

#define sdclkDiv2	0x0000
#define sdclkDiv200	(198)

#define sdPowerOff	0x0000
#define sdPowerOn	0x0003
 
#define respNone	0x0000
#define respShort	0x0040
#define respLong	0x00C0

// SDIO COMMAND CONTROL
#define sdcmdComplete	0x1000
#define sdcmdCPSMEna	0x0400
#define sdcmdWaitEnd	0x0200
#define sdcmdWaitInt	0x0100

// SDIO STATUS
#define stIrqReceived	0x00400000
#define stRxDataInFIFO	0x00200000
#define stTxDataInFIFO	0x00100000

#define stRcvEmpty	0x00080000
#define stTsmEmpty	0x00040000
#define stRcvFull	0x00020000
#define stTsmFull	0x00010000

#define stRcvHalfFull	0x8000
#define stTsmHalfFull	0x4000
#define stRcvProgress	0x2000
#define stTsmProgress	0x1000

#define stDataEnd	0x0100
#define stStartError	0x0200
#define stDataBlockEnd	0x0400
#define stCmdProgress	0x0800

#define stCmdSent	0x0080
#define stHaveCmdResp	0x0040
#define stRcvFifoError	0x0020
#define stTsmFifoError	0x0010

#define stDataTimeout	0x0008
#define stRespTimeout	0x0004
#define stDataCrcFail	0x0002
#define stCmdCrcFail	0x0001

#define stDataError     (stDataCrcFail+stRcvFifoError+stTsmFifoError+stDataTimeout)
#define stCommandError  (stCmdCrcFail+stRespTimeout+stStartError)

// SDIO DATA CONTROL
#define dcrTxRxEnable	0x0001
#define dcrRead		0x0002
#define dcrWrite	0x0000
#define dcrStream	0x0004
#define dcrEnaDMA	0x0008
#define dcrEnaDPSM	0x0800
#define dcr512Bytes	0x0090

#define dmaPriorityLow	0x0000	 
#define dmaPriorityMed	0x1000	 
#define dmaPriorityHigh	0x2000	 
#define dmaPriorityVery	0x3000	 

#define dmaMSizeByte	0x0000
#define dmaMSizeWord	0x0400
#define dmaMSizeLong	0x0800

#define dmaPSizeByte	0x0000
#define dmaPSizeWord	0x0100
#define dmaPSizeLong	0x0200

#define dmaMemoryInc	0x0080
#define dmaPeriphInc	0x0040
#define dmaCircular	0x0020
#define dmaFromMemory	0x0010
#define dmaFromPeriph	0x0000

#define dmaEnable	0x0001

#endif