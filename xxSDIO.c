#include "..\\inc\\stm32F4xx.h"

#pragma pack(1)

#include "xxProto.h"
#include "xxSDRegs.h"

TSDCommand *SD;
TSDCommand SDC;
TCardIdent CI;
TCardData  CD;

bool NoSD, FlagSDResponse = 0, FlagSDError = 0, FlagSDRead = 1;
bool FlagSDComplete, FlagSDDone, FlagSDCommand, FlagSDReserv;

int *InSD, *OutSD, CountSD;

int RCA;
int SDHC = 0, SDPattern = 0, SDVer = 0, SDReady = 0;
int SDSizePage = 512, CRCErr;
short int SDCRC, SDRetval;
char SData[16];

unsigned char CRC7( char *chr, int cnt ) 
{
int i,a;
unsigned char crc, data;
 
    crc=0;
    for( a=0; a<cnt; a++ ) {
        data=chr[a];
        for( i=0; i<8; i++ ) {
            crc <<= 1; 
            if ((data & 0x80)^(crc & 0x80)) crc ^=0x09;
            data <<= 1; 
        }
    }
    crc=(crc<<1)|1;
    return(crc);
} 

/*  [..]

    (#) Program the Clock Edge, Clock Bypass, Clock Power Save, Bus Wide,  
        hardware, flow control and the Clock Divider using the SDIO_Init() 
        function. 
    (#) Enable the Power ON State using the SDIO_SetPowerState(SDIO_PowerState_ON)  
        function.           
    (#) Enable the clock using the SDIO_ClockCmd() function. 
    (#) Enable the NVIC and the corresponding interrupt using the function  
        SDIO_ITConfig() if you need to use interrupt mode.  
    (#) When using the DMA mode
        (++) Configure the DMA using DMA_Init() function.
        (++) Active the needed channel Request using SDIO_DMACmd() function.
    (#) Enable the DMA using the DMA_Cmd() function, when using DMA mode.
    (#) To control the CPSM (Command Path State Machine) and send commands to the
        card use the SDIO_SendCommand(), SDIO_GetCommandResponse() and 
        SDIO_GetResponse() functions. First, user has to fill the command 
        structure (pointer to SDIO_CmdInitTypeDef) according to the selected 
        command to be sent. The parameters that should be filled are: 
        (++) Command Argument.
        (++) Command Index.
        (++) Command Response type.
        (++) Command Wait.
        (++) CPSM Status (Enable or Disable).
        To check if the command is well received, read the SDIO_CMDRESP register 
        using the SDIO_GetCommandResponse(). The SDIO responses registers 
        (SDIO_RESP1 to SDIO_RESP2), use the SDIO_GetResponse() function. 
    (#) To control the DPSM (Data Path State Machine) and send/receive  
        data to/from the card use the SDIO_DataConfig(), SDIO_GetDataCounter(),
        SDIO_ReadData(), SDIO_WriteData() and SDIO_GetFIFOCount() functions.

*/

static int ReadPageSD( int page, int bytes, char *buffer )
{       
int *in;
int retval = stErRd;

    SetIrq( SDIO_IRQn, 0, 1 );
    NVIC_ClearPendingIRQ( SDIO_IRQn );
    NVIC_EnableIRQ( SDIO_IRQn );  
    SetBit( PortTest, TestPoint4 );

    SDIO->CLKCR &= 0xFFFFFF00;
    SDIO->CLKCR |= 14;

    CRCErr = 0;
    in = (int *)buffer;
    InSD = in;
    CountSD = 0;
    SDIO->DTIMER = 0x1FFFFFFF;
    SDIO->DLEN = 512;
    SDIO->DCTRL = dcrRead | dcr512Bytes; // | (1<<10);
    SDIO->DCTRL |= dcrTxRxEnable;
    SDIO->ICR = 0x7FF;
    SDIO->ARG = page;  
    FlagSDResponse = 0;
    FlagSDError = 0;
    FlagSDRead = 1;
    FlagSDComplete = 0;
    FlagSDDone = 0;
    FlagSDCommand = 1;
    SDIO->MASK = stHaveCmdResp | stCommandError;
    SDIO->CMD = sdcRead | sdcmdComplete | respShort | sdcmdCPSMEna; 
    
    FlashTimeout = 0;
    while( 1 ) {
        if( FlagSDResponse ) break;
        if( FlagSDError ) { MsDelay( 50 ); goto ret; }
        if( FlashTimeout > 50 ) goto ret;
    }

    FlashTimeout = 0;
    while( 1 ) {
        if( FlagSDComplete ) break;
        if( FlagSDError ) { MsDelay( 50 ); goto ret; }
        if( FlashTimeout > 50 ) goto ret;
    }

    FlashTimeout = 0;
    while( 1 ) {
        if( FlagSDDone ) break;
        if( FlagSDError ) { MsDelay( 50 ); goto ret; }
        if( FlashTimeout > 50 ) goto ret;
    }  
    
    ClrBit( PortTest, TestPoint4 );
    retval = stOK;
ret:
    SDIO->ICR = 0x7FF;
    SDIO->MASK = 0;
    ClrBit( PortTest, TestPoint4 );
    
    NVIC_DisableIRQ( SDIO_IRQn );  
    return retval;
}

static int WritePageSD( int page, int bytes, char *buffer )
{       
int *in;
int retval = stErWr;

    SetBit( PortTest, TestPoint4 );

    SetIrq( SDIO_IRQn, 0, 1 );
    NVIC_ClearPendingIRQ( SDIO_IRQn );
    NVIC_EnableIRQ( SDIO_IRQn );  

    CRCErr = 0;
    in = (int *)buffer;
    SDIO->CLKCR &= 0xFFFFFF00;
    SDIO->CLKCR |= 14;
    
    OutSD = in;
    CountSD = 0;
    SDIO->DTIMER = 0x1FFFFFFF;
    SDIO->DLEN = bytes;
    SDIO->ICR = 0x7FF;
    SDIO->ARG = page;  
    SDIO->DCTRL = dcrWrite | dcr512Bytes;// | dcrEnaDMA | (1<<10);
    SDIO->DCTRL |= dcrTxRxEnable;
    FlagSDResponse = 0;
    FlagSDError = 0;
    FlagSDRead = 0;
    FlagSDComplete = 0;
    FlagSDDone = 0;
    FlagSDCommand = 1;
    SDIO->MASK = stHaveCmdResp | stCommandError;
                 //stTsmEmpty | stTxDataInFIFO | stTsmFifoError | stDataTimeout | stDataCrcFail |
                 //stDataBlockEnd;
      
    SDIO->CMD = sdcWrite | sdcmdComplete | sdcmdCPSMEna | respShort; 

    FlashTimeout = 0;
    while( 1 ) {
        if( FlagSDResponse ) break;
        if( FlagSDError ) { MsDelay( 50 ); goto ret; }
        if( FlashTimeout > 50 ) goto ret; 
    }
    
    FlashTimeout = 0;
    while( 1 ) {
        if( FlagSDComplete ) break;
        if( FlagSDError ) { MsDelay( 50 ); goto ret; }
        if( FlashTimeout > 50 ) goto ret; 
    }

    FlashTimeout = 0;
    while( 1 ) {
        if( FlagSDDone ) break;
        if( FlagSDError ) { MsDelay( 50 ); goto ret; }
        if( FlashTimeout > 50 ) goto ret; 
    }
    ClrBit( PortTest, TestPoint4 );
    retval = stOK;
ret:
    ClrBit( PortTest, TestPoint4 );
    SDIO->ICR = 0x7FF;
    SDIO->MASK = 0;
    NVIC_DisableIRQ( SDIO_IRQn );  
    return retval;
}

int ReadPage( int page, int bytes, char *buffer )
{
int repeat = 4, st;

    if( NoSD ) return stErRd;
    while( repeat > 0 ) {
        st = ReadPageSD( page, bytes, buffer );
        if( st == stOK ) break;
        repeat--;
    }
    return st;   
}

int WritePage( int page, int bytes, char *buffer )
{
int repeat = 4, st;

    if( NoSD ) return stErWr;
    while( repeat > 0 ) {
        st = WritePageSD( page, bytes, buffer );
        if( st == stOK ) break;
        repeat--;
    }
    return st;   
}


int SDSendCommand( int cmd, int arg, int respidx, int *resp )
{
int status;

    CRCErr = 0;
    SDIO->ICR = 0x7FF;
    SDIO->ARG = arg;  
    SDIO->CMD = cmd | respidx | sdcmdComplete | sdcmdCPSMEna; 

    TsmTimeout = 0;
    while( 1 ) {
        status = SDIO->STA;
        if( ((status & (stHaveCmdResp | stCmdCrcFail)) != 0) && (respidx != 0) ) break;
        if( ((status & stCmdSent) != 0) && (respidx == 0) ) break;
	if( (status & stRespTimeout) != 0 ) goto err;
        if( TsmTimeout > 500 ) return status;
    }
    resp[0] = SDIO->RESP1;
    resp[1] = SDIO->RESP2;
    resp[2] = SDIO->RESP3;
    resp[3] = SDIO->RESP4;
    CRCErr = status & stCmdCrcFail;
    //if( CRCErr == 0 ) 
      return 0;
err:
    //for( i=0; i<16; i++ ) ToggleBit( PortTest, TestPoint5 );
    //rep--;
    //if( rep > 0 ) goto once; 
    return status;
}

int InitSD( void )
{
int result;
int RESP[4];

    /* Set the SDIO CLKCR value */
    SDIO->POWER = sdPowerOn;
    SDIO->CLKCR = 32;//sdclkDiv200 + sdclkHWFLOW;
    SDIO->CLKCR |= (sdclkEnable);// + 0x4000);
    MsDelay( 250 );

    result = SDSendCommand(0, 0, 0, RESP);     //Посылаем CMD0, обнулить карту
    if (result != 0) return result;      
    MsDelay( 5 );

    //Шлем ACMD41 c аргументом 0
    result = SDSendCommand( 55, 0, respShort, RESP);      
    if (result != 0) return 1002;
    result = SDSendCommand( 41, 0, respShort, RESP );     
    if (result != 0) return 1003;
    MsDelay( 5 );

    result = SDSendCommand(8, 0x1AA, respShort, RESP);      //Посылаем CMD8 с аргументом 110101010
    if ( (result != 0) || (RESP[0] != 0x1AA) ) return 1001;      
    MsDelay( 5 );

    RESP[0] = 0;
    while( !(RESP[0]&(1<<31) ) ) {     //Ждем, пока флаг busy 
        result = SDSendCommand(55, 0, respShort, RESP);      //Шлем CMD55, тем самым, говоря, что потом будет ACMD
        if (result != 0) return 1002;
        result = SDSendCommand( 41, 0x40020000, respShort, RESP );     //Шлем ACMD41
        if (result != 0) return 1003;
        MsDelay( 5 );
    }
    
    result = SDSendCommand(2, 0, respLong, RESP);     //Шлем CMD2 и получаем инфо о карте
    if (result != 0) return result;
        MsDelay( 5 );

    result = SDSendCommand(3, 0, respShort, RESP);     //Шлем CMD3 и получаем RCA номер
    if (result != 0) return result;
        MsDelay( 5 );

    RCA = ( RESP[0] & (0xFFFF0000) );      //Маскируем отклик и получаем RCA

    result = SDSendCommand(7, RCA, respShort, RESP);     //Выбираем нашу карту				
    if (result != 0) return result;
        MsDelay( 5 );


    result = SDSendCommand(55, RCA, respShort, RESP);      //Шлем CMD55, тем самым, говоря, что потом будет ACMD
    if (result != 0) return result;

    result = SDSendCommand(6, 0x02, respShort, RESP);      //Шлем ACMD6 c аргументом 0x02, установив 4-битный режим
    if (result != 0) return result;
    
// 14 HFlow
// 11-12 - bus wide  00 - 1 01-4  10-8
// 10 - Divider On =0
// 8 - CLK ON
//9 - power save    
    SDIO->CLKCR = (2<<0)|(1<<11)|(1<<8)|(1<<14)|(0<<9)|(0<<13);     //Наращиваем клок (в части 2 - подробнее)
//    if (RESP[0] != 0x700) return 1;     //Убеждаемся, что карта находится в готовности работать с трансфером
    if (RESP[0] != 0x920) return 1;     //Убеждаемся, что карта находится в готовности работать с трансфером
    return 0;
}

int SendSD( char *out, char *in, int bytes )
{
    return 0; //SendSpi( SpiSD, out, in, bytes );
}

char SendSDByte( char byte )
{
    return 0; //SendSpiByte( SpiSD, byte );
}    

void DummyBytes( void )
{
    SendSDByte( 0xFF );
    SendSDByte( 0xFF );
    SendSDByte( 0xFF );
    SendSDByte( 0xFF );
    SendSDByte( 0xFF );
    SendSDByte( 0xFF );
    SendSDByte( 0xFF );
    SendSDByte( 0xFF );
}

int ResetSD( void )
{       
    return SDRetval;
}

int GetInterfaceSD( char pattern )
{       
    return SDRetval;
}

int GetStatusSD( void )
{       
    return SDRetval;
}

int SetCRCSD( char crconoff )
{       
    return SDRetval;
}


int SetAppSD( void )
{       
    return SDRetval;
}

int InitializeSD( char mode )
{       
    return SDRetval;
}

int GetOCRSD( void )
{       
    return SDRetval;
}

int SendSDCommand( char cmd, int num )
{       
    return SDRetval;
}

/*
    *** Read Operations *** 
    ----------------------- 
      [..]
      (#) First, user has to fill the data structure (pointer to 
          SDIO_DataInitTypeDef) according to the selected data type to be received. 
          The parameters that should be filled are:
          (++) Data TimeOut.
          (++) Data Length.
          (++) Data Block size.
          (++) Data Transfer direction: should be from card (To SDIO).
          (++) Data Transfer mode.
          (++) DPSM Status (Enable or Disable).
      (#) Configure the SDIO resources to receive the data from the card 
          according to selected transfer mode (Refer to Step 8, 9 and 10).
      (#) Send the selected Read command (refer to step 11).
      (#) Use the SDIO flags/interrupts to check the transfer status.
 
*/

int ReadCardData( void )
{       
int retval = 0xFF, i=0;
char *buffer = (char *)(&CD);

//    ClrBit( PortSD, CSSD );
    
    SData[0] = 0x40 | sdcCardSpecific;
    SData[1] = 0x00;
    SData[2] = 0x00;
    SData[3] = 0x00;
    SData[4] = 0x00;
    SData[5] = CRC7( SData, 5 );
    SendSD( SData, SData, 6 );
    TsmTimeout = 0;
    while( 1 ) {
      // Card response
      retval = SendSDByte( 0xFF );
      if( (retval) == 0x00 ) break;
      if( TsmTimeout > 200 ) return retval | 0x1000;
    }
    SDRetval = retval;

    TsmTimeout = 0;
    while( 1 ) {
      // SData response
      retval = SendSDByte( 0xFF );
      if( (retval) == 0xFE ) break;
      if( TsmTimeout > 200 ) return retval | 0x2000;
    }
    SDRetval = retval;

    for( i=0; i<16; i++ ) buffer[i] = SendSDByte( 0xFF );
    SDCRC = (SendSDByte( 0xFF )<<8);
    SDCRC |= SendSDByte( 0xFF );

    DummyBytes();
//    SetBit( PortSD, CSSD );
    return 1;
}

int SetFirstEraseSD( int page )
{       
int retval = 0xFF;
long int addr;

    SData[0] = 0x40 | sdcEraseBeg;
    if( SDHC ) addr = page;
    else addr = page * SDSizePage;
    SData[4] = addr&0xFF;
    SData[3] = (addr>>8)&0xFF;
    SData[2] = (addr>>16)&0xFF;
    SData[1] = (addr>>24)&0xFF;
    SData[5] = CRC7( SData, 5 );
    SendSD( SData, SData, 6 );
    TsmTimeout = 0;
    while( 1 ) {
      // Card response
      retval = SendSDByte( 0xFF );
      if( (retval) == 0x00 ) break;
      if( TsmTimeout > 200 ) return retval | 0x1000;
    }
    DummyBytes();
    return 1;
}

int SetLastEraseSD( int page )
{       
int retval = 0xFF;
long int addr;

    SData[0] = 0x40 | sdcEraseEnd;
    if( SDHC ) addr = page;
    else addr = page * SDSizePage;
    SData[4] = addr&0xFF;
    SData[3] = (addr>>8)&0xFF;
    SData[2] = (addr>>16)&0xFF;
    SData[1] = (addr>>24)&0xFF;
    SData[5] = CRC7( SData, 5 );
    SendSD( SData, SData, 6 );
    TsmTimeout = 0;
    while( 1 ) {
      // Card response
      retval = SendSDByte( 0xFF );
      if( (retval) == 0x00 ) break;
      if( TsmTimeout > 200 ) return retval | 0x1000;
    }
    DummyBytes();
    return 1;
}

int EraseSD( int beg, int num )
{
 int retval;
    num--;
    if( num < 0 ) return 1;
    
//    ClrBit( PortSD, CSSD );
    retval = SetFirstEraseSD( beg ); 
    retval = SetLastEraseSD( beg+num ); 
    
    SData[0] = 0x40 | sdcErase;
    SData[1] = 0x00;
    SData[2] = 0x00;
    SData[3] = 0x00;
    SData[4] = 0x00;
    SData[5] = CRC7( SData, 5 );
    SendSD( SData, SData, 6 );
    TsmTimeout = 0;
    while( 1 ) {
      // Card response
      retval = SendSDByte( 0xFF );
      if( (retval) == 0x00 ) break;
      if( TsmTimeout > 200 ) return retval | 0x1000;
    }
    SDRetval = retval;

    while( 1 ) {
      retval = SendSDByte( 0xFF );
      if( (retval) == 0xFF ) break;
      if( TsmTimeout > 500 ) return 0xFF;
    }
    SDRetval = retval;

    DummyBytes();
//    SetBit( PortSD, CSSD );
    return 1;
}

void SDIO_IRQHandler( void )
{
int status;
    status = SDIO->STA;
    SDIO->ICR |= stIrqReceived;

    // command error
    if( (status & stCommandError) != 0 ) { 
            SDIO->ICR |= stCommandError;
	    SDIO->MASK = 0;
            FlagSDError = 1;
	    return;
    }
    
    if( (status & stCmdProgress) != 0 ) {
        SDIO->ICR |= stCmdProgress;
	//return;
    }        
    // Response command
    if( (status & stHaveCmdResp) != 0 ) {
        SDIO->ICR |= stHaveCmdResp;
	SDIO->MASK &= (~stHaveCmdResp);
        if( FlagSDRead ) 
            SDIO->MASK = stRxDataInFIFO | stRcvFifoError | stDataTimeout | stDataCrcFail;
        else 
            SDIO->MASK = stTsmEmpty | stTxDataInFIFO | stTsmFifoError | stDataTimeout | stDataCrcFail | stDataBlockEnd;
        FlagSDResponse = 1;
	return;
    }        

    // Read data
    if( (SDIO->STA & (stRxDataInFIFO)) != 0 ) {
	if( CountSD < 128 ) InSD[CountSD++] = SDIO->FIFO;
	if( SDIO->DCOUNT == 0 ) {
	    FlagSDComplete = 1;
            SDIO->MASK = stDataBlockEnd | stDataError;
	    return;
        }
    }
    
    // Write data
    if( (SDIO->STA & (/*stTxDataInFIFO | */stTsmEmpty) ) != 0 ) {
       if( CountSD < 128 ) {
            //if( SDIO->FIFOCNT < 20 ) 
              SDIO->FIFO = OutSD[CountSD++];
        }
        SDIO->ICR |= (stTsmEmpty + stTxDataInFIFO );
	if( SDIO->DCOUNT == 0 ) {
            FlagSDComplete = 1;
            SDIO->MASK = stDataBlockEnd | stDataError;
	    return;
        }
    }
    
    // Error data
    if( (SDIO->STA  & stDataError) != 0 ) {
        SDIO->ICR |= stDataError;
        FlagSDError = 1;
  	return;
    }

    // Block end
    if( (SDIO->STA & stDataBlockEnd) != 0 || (SDIO->DCOUNT == 0) ) {
        SDIO->ICR |= stDataBlockEnd;
        FlagSDDone = 1;
        SDIO->MASK = 0;
	return;
    }
    SDIO->ICR |= status;
    
/*
    if( FlagSDRead ) {
      if( (status & stDataBlockEnd) != 0 || SDIO->DCOUNT == 0 ) {            
          if( SDIO->DCOUNT ==  0 ) return;
          else { 
              SDIO->ICR |= stDataBlockEnd;
	      FlagSDDone = 1;
              SDIO->MASK = 0;
              if( SDIO->DCOUNT == 0 ) return;
          }
      }
    }
    else {
    }
*/
}
  
