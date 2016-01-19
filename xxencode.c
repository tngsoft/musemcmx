#include <string.h>
#include <math.h>
#include "..\\inc\\stm32F4xx.h"

#pragma pack(1)

#include "xxProto.h"
#include "..\\..\\xCommand.h"
#include "xxRtc.h"
#include "xxNet.h"
#include "xxCustom.h"

//extern int SecCounter, ExecuteGo, SecExecute;
void RunSendPageGt( void );
void RunSendPageLt( void ); 

void SendLongData( void *Data, int size );
void InitSendLongData( int size,
    int command, int commandEx,
    int param1, int param2 );

extern unsigned int BadBlocks[SizeBadBl/4];

int ReadDataCounter = 0;
extern int RxFlag;
long int Pattern = 0xE0EFEEE6, *LI;

int CalcBaudRate( int baud, USART_TypeDef* USARTx )
{
float fraq;
int div;
    // Get APB1 prescaler
    if( USARTx == USART1 ) div = ((RCC->CFGR>>13)&7) - 3;
    else div = ((RCC->CFGR>>10)&7) - 3;
    if( div < 0 ) div = 0;
    div = 1<<div;
    //-------------------
    Calb.BaudRate485 = baud;
    Calb.Div485 = (SystemCoreClock/div)/(Calb.BaudRate485*16);
    fraq = (SystemCoreClock/div)/(Calb.BaudRate485*16.0);
    fraq -= Calb.Div485;
    fraq *= 16;
    Calb.Div485 = (Calb.Div485<<4) + round( fraq );
    return Calb.Div485;
}

int RunTlsCmd( void )
{
int bytes;
    ImTls = 1;
    bytes = SendToTls( mh.WParam, -mh.LParam[0] );
    ReplayData( DOut, bytes );
    return 0;
}

int RunReadCmd( void )
{
    if( HaveAddress <= 0 ) return 0;
    switch( mh.Command[1] ) {
        case cmdData:
	    // LParam[0] - block number
	    // WParam[0] - num pages read
  	    MaxLenPacket = 1024;
            if( MaxLenPacket >= tp.SizePage ) RunSendPageGt();
            else RunSendPageLt();
            break;
        case cmdPage:
	    // LParam[0] - block number
            ReadPage( mh.LParam[0], 512, NetRcvData );
	    ReplayData( NetRcvData, 512 );
            break;
    }
    return 1;
}

int CompareBuffers( int *buf1, int *buf2, int len )
{
int i = 0;
    for( i=0; i<len; i++ ) if( (buf1[i]^buf2[i]) != 0 ) return 0;
    return 1;
}

int RunWriteCmd( void )
{
int i, k, num, *out;

    if( HaveAddress <= 0 ) return 0;
    switch( mh.Command[1] ) {
        case cmdPage:
	    // LParam[0] - page number
	    // LParam[1] - Pattern
	    out = (int *)NetRcvData;
	    for( i=0; i<128; i++ ) out[i] = mh.LParam[1];
            WritePage( mh.LParam[0], 512, NetRcvData );
            // if( mh.LParam[0] == 0 ) EEErase( EE_START_ADDR, SizeFtp );
	    ReplayData( &Pattern, 2 );
            break;
        case cmdData:
	    // LParam[0] - page number
	    // WParam[0] - size data for write
	    // WParam[1] - address in page
            ReadPage( mh.LParam[0], 512, NetRcvData + 512 );
	    i = mh.WParam[1]+512;
	    for( k=0; k<mh.WParam[0]; k++ ) {
	        NetRcvData[i++] = NetRcvData[k];
	    }
            WritePage( mh.LParam[0], 512, NetRcvData + 512 );
	    ReplayData( &Pattern, 2 );
            break;
        case cmdBlock:            
	    // LParam[0] - block number
	    // LParam[1] - how many blocks erase
	    //EraseSD( (mh.LParam[0]<<8), mh.WParam[0] );
	    memset( FlashInBuffer+1024, 0xFF, 512 );
            si.Block = 8;
	    mh.Status &= ~(stErEr | feError);
	    num = mh.LParam[1];
            for( i=0; i<num; i++ ) {
	        WritePage( mh.LParam[0]+i, 512, FlashInBuffer+1024 );
	        ReadPage( mh.LParam[0]+i, 512, FlashInBuffer );
		k = CompareBuffers( (int *)(FlashInBuffer+1024), (int *)FlashInBuffer, 128 );
		if( k == 0 ) {
		    SetBadBlock( mh.LParam[0]+i );
                    mh.Status |= (stErEr | feError);
		    mh.LParam[1] = mh.LParam[0]+i;
		}
	    }
	    ReplayData( NetRcvData, 2 );
	    InitSaveBlock( 1 );
	    break;
    }
    return 1;
}

int RunSetCmd( void )
{
    if( HaveAddress <= 0 ) return 0;
    switch( mh.Command[1] ) {
        case cmdReset:
            ReLoad = 1;
            memset( NetRcvData, 0xFF, 80 );
            if( mh.WParam[0] != 0 ) WriteMainTp( NetRcvData );
            ReplayData( NetRcvData, 2 );
            break;
        case cmdDateTime:
            SetRtc( (TRtcTime *)NetRcvData );
            GetRtc();
            ReplayData( &rt, sizeof( rt ) );
            break;
        case cmdCalb:
	    if( mh.LenData != mh.WParam[0] ) break;
            memcpy( &Calb, NetRcvData, sizeof( TCalibrate) );
            WriteCalb( &Calb, sizeof( Calb )  );
            CalcBaudRate( Calb.BaudRate485, USARTRS485 );
            tp.BaudRate485 = Calb.BaudRate485;
            tp.Div485 = Calb.Div485;
            WriteBaudRate( &(tp.BaudRate485)  );
            SendDataEx( NetRcvData, mh.WParam[0], cmdSet, cmdCalb, Calb.BaudRate485, Calb.Div485 );
            SetBaudRate485( Calb.Div485 );
            break;
        case cmdBaudRate:
            Calb.BaudRate485 = mh.LParam[0];
            Calb.Div485 = CalcBaudRate( Calb.BaudRate485, USARTRS485 );
            WriteCalb( &Calb, sizeof( Calb ) );
            tp.BaudRate485 = Calb.BaudRate485;
            tp.Div485 = Calb.Div485;
            WriteBaudRate( &(tp.BaudRate485)  );
            SendDataEx( FlashInBuffer, 2, cmdSet, cmdBaudRate, Calb.BaudRate485, Calb.Div485 );
            SetBaudRate485( Calb.Div485 );
            break;
        case cmdStart:
	    if( mh.LenData != sizeof( mi ) ) break;
            memcpy( &mi, NetRcvData, sizeof( mi ) );
	    WriteStartParams( &mi );
            ReplayData( NetRcvData, sizeof( mi ) );
	    ClearRuns();
	    SetWorkMode( wmWait );
	    //InitValues();
            //FindFreeMarker();
            break;
        case cmdStop:
            ReplayData( NetRcvData, sizeof( mi ) );
	    SetWorkMode( wmStop );
            break;

        case cmdToolInfo:
	    if( mh.LenData != SizeFtp ) break;
            if( mh.WParam[0] == 0x3110 ) 
               WriteSecTp( NetRcvData  );
            else {
                memcpy( &tp, NetRcvData, mh.LenData );
                WriteMainTp( &tp );
	    }
            ReplayData( &(tp.StartParam), sizeof( mi ) );
            if( mh.WParam[1] != 0x1963 ) {
	        ClearRuns();
                memcpy( &mi, &(tp.StartParam), sizeof( mi ) );
	        SetWorkMode( wmWait );
	        InitValues();
            }
            break;
    }
    return 1;
}

int RunGetCmd( void )
{
    if( HaveAddress <= 0 ) return 0;
    switch( mh.Command[1] ) {
        case cmdEcho:
            //MaxLenPacket = mh.Param[3];
            return ReplayData( NetRcvData, mh.LenData );
        case cmdToolInfo:
            MaxLenPacket = 1024;
	    // WParam[0] - size info for reply
	    ReadSystemData( FlashInBuffer );
            SendDataEx( FlashInBuffer, mh.WParam[0], cmdGet, cmdToolInfo, Calb.BaudRate485, Calb.Div485 );            
            break;
        case cmdRuns:
	    // WParam[0] - size info for reply
	    ReadRuns( FlashInBuffer );
            SendDataEx( FlashInBuffer, SizeRuns, cmdGet, cmdRuns, 0, 0 );            
            break;
        case cmdBadBlocks:
	    // WParam[0] - size info for reply
	    ReadBadBlocks( FlashInBuffer );
            SendDataEx( FlashInBuffer, SizeBadBl, cmdGet, cmdBadBlocks, 0, 0 );            
            break;
        case cmdCalb:
	    // WParam[0] - size calb for reply
            memcpy( FlashInBuffer, &Calb, sizeof( Calb ) );	    
            SendDataEx( FlashInBuffer, mh.WParam[0], cmdGet, cmdCalb, Calb.BaudRate485, Calb.Div485 );
            break;
        case cmdBaudRate:
	    // WParam[0] - size calb for reply
            // memcpy( FlashInBuffer, &Calb, sizeof( Calb ) );	    
            SendDataEx( FlashInBuffer, 0, cmdGet, cmdBaudRate, Calb.BaudRate485, Calb.Div485 );
            break;
        case cmdStateInfo:
            //MaxLenPacket = mh.Param[3];
            //ReplayData( &si, sizeof( si ) );
            SendStatus( 1 );
            break;
        case cmdDateTime:
            GetRtc();
            ReplayData( &rt, sizeof( rt ) );
            break;
    }
    return 1;
}

int EncodeCommand( void )
{
    MaxLenPacket = mh.WParam[2];
    TypeOfMessage = tomReply;
    if( mh.Command[0] == '00' ) {
        if( From485 ) SendStatus( 1 );
	InitNetReceive();
	USART2->DR = 0x33;
          //SendDataEx( &si, sizeof(si), cmdGet, cmdStateInfo, 0, 0 );            
        return 0;
    }
    switch( mh.Command[0] ) {
        case (short int)(cmdWrite):
            RunWriteCmd();
            break;
        case cmdRead:
            RunReadCmd();
            break;
        case cmdSet:
            RunSetCmd();
            break;
        case cmdGet:
            RunGetCmd();
            break;
        case cmdTls:
            RunTlsCmd();
            break;
        case '00':
	    if( mh.Command[1] == '00' ) {
                //RunReadCmd();	      
	    }
            break;
    }
    InitNetReceive();
    return 0;
}

