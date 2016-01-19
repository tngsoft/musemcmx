#include "..\\inc\\stm32F4xx.h"
#pragma pack(1)

// Fuart = Fclk/(n+1)*16
// N = (Fclk/16*Fuart) - 1

// USARTDIV = DIV_Mantissa + (DIV_Fraction / 8 * (2 – OVER8))

#include <string.h>

#include "xxProto.h"
#include "..\\..\\xxTcp.h"
#include "..\\..\\xCommand.h"

int  TsmCount = -1, OutCount = 0;
static int RcvCounter = 0, MaxCount = 1, Timeout = 1;
static char *OutBuffer;
static char *InBuffer;
unsigned char value, CheckSum=0;

void USART1_IRQHandler(void)
{
short int Status;

    Status = USART1->SR;
    //NVIC_ClearPendingIRQ( USART1_IRQn );

    if( (Status & USART_FLAG_RXNE) != 0 ) {	// Receive
        value = USART1->DR;
        if( RcvCounter <= MaxCount  ) InBuffer[ RcvCounter ] = value;
        RcvCounter++;
        RcvTimeout = 0;
	return;
    }

    if( (Status & USART_FLAG_TXE) != 0 ) {	// Transmit       
        if( TsmCount > 0 ) {
	    if( TsmCount == 1 ) {
	        USART1->DR = CheckSum;
                USART1->CR1 &= ~(USART_CR1_TXEIE);
                USART1->CR1 |= (USART_CR1_TCIE);
	    }
            else USART1->DR = OutBuffer[OutCount];         
            TsmCount--;
	    OutCount++;
            return;
        }
	else {
            TsmCount = -1;
            USART1->CR1 &= ~( USART_CR1_TE | USART_CR1_TXEIE );
	    USART1->DR = 0x41;
	}
    }
    if( (Status & USART_FLAG_TC) != 0 ) {	// Transmit       
        TsmCount = -1;
        USART1->CR1 &= ~( USART_CR1_TE | USART_CR1_TXEIE );
	USART1->DR = 0x41;
    }    
}


// if len < 0 send with check sum

void SendUart( void *data, int len )
{
int i;
    if( len == 0 ) return; 
    OutBuffer = (char *)data;
    if( len < 0 ) { 
        len = -len-1;
        CheckSum = OutBuffer[len];
    }
    else {
        CheckSum = 0;
        for( i=0; i<len; i++ ) CheckSum += OutBuffer[i];
    }
    TsmCount = len;   
    OutCount = 1;
    USART1->DR = OutBuffer[0];     // sending the 1-st byte
    USART1->CR1 |= ( USART_CR1_TE | USART_CR1_TXEIE | USART_CR1_RE | USART_CR1_RXNEIE );
    while( TsmCount >= 0 );
//    USART1->CR1 &= ~( USART_CR1_TE | USART_CR1_TCIE );
//    USART1->DR = 0x41;
}


char ReceiveToolData( void *data, int maxcount )
{
int i; 
    CheckSum = 0;
    MaxCount = maxcount;
    InBuffer = (char *)data;
    RcvCounter = 0;
    RcvTimeout = 0;
    while( 1 ) {
        if( RcvCounter == 0 ) {
            if( RcvTimeout > 150 ) break;
        }
        else {
            if( RcvTimeout > 3 ) break;
        }
    }
    if( RcvCounter == 0 ) return 0x01;
    if( RcvCounter != MaxCount+1 ) return 0x02;
    for( i=0; i<RcvCounter-1; i++ ) CheckSum += InBuffer[i];
    if( InBuffer[RcvCounter-1] != CheckSum ) return 0x03;
    return 0;
}

int MaxComPacket = 1024;

TMessageHeader DHI, DHO, *dh;

static short int Unical = 1;

int SendDataCom( void *Data, int size,
    int command, int commandEx,
    int param1, int param2 )
{
int len;
char *data, *Pchar;

    data = (char *)Data;
    DHO.Cmd.Command[0] = command;
    DHO.Cmd.Command[1] = commandEx;
    Pchar = (char *)(DHO.Cmd.Command);
    Swapb( Pchar[0], Pchar[1] );
    Swapb( Pchar[2], Pchar[3] );
    DHO.FrameNumber = 0;
    DHO.WParam[0] = param1;
    DHO.WParam[1] = param2;
    DHO.Status &= (~stPacketDone); 
    DHO.mbRead = size;
    
    if( size > MaxComPacket ) DHO.UnicalNumber = Unical++;
    else DHO.UnicalNumber = 0;
    
    while( size > 0 ) {
        if( size > MaxComPacket ) len = MaxComPacket;
	else len = size;
        if( (size-len) <= 0 ) DHO.Status |= stPacketDone;	
	DHO.LenData = len;
        SendUart( &DHO, -sizeof( TMessageHeader) );
        SendUart( data, len );
        size -= len;
        data += len;
        DHO.FrameNumber++;
    }
    DHO.Status = 0;
    return 0;
}

int ReplyDataCom( void *Data, int size )
{
int len;
char *data, *Pchar;

    data = (char *)Data;
    Pchar = (char *)(DHO.Cmd.Command);
    Swapb( Pchar[0], Pchar[3] );
    Swapb( Pchar[1], Pchar[2] );
    DHO.FrameNumber = 0;
    DHO.Status &= (~stPacketDone); 
    DHO.mbRead = size;
    
    while( size > 0 ) {
        if( size > MaxComPacket ) len = MaxComPacket;
	else len = size;
        if( (size-len) <= 0 ) DHO.Status |= stPacketDone;	
	DHO.LenData = len;
        SendUart( &DHO, -sizeof( TMessageHeader) );
        SendUart( data, len );
        size -= len;
        data += len;
        DHO.FrameNumber++;
    }
    DHO.Status = 0;
    return 0;
}

void InitReceiveUart( void *data, int maxcount, int timeout )
{
    CheckSum = 0;
    MaxCount = maxcount;
    InBuffer = (char *)data;
    RcvCounter = 0;
    RcvTimeout = 0;
    Timeout = timeout;
}


int CheckReceiveUart( void )
{
    if( RcvCounter != 0 ) {
        if( RcvTimeout > Timeout ) return RcvCounter;
    }
    return 0;
}

int EncodeHost( void *data )
{
int bytes;
char *Pchar;

    Pchar = (char *)data;
    Swapb( Pchar[0], Pchar[3] );
    Swapb( Pchar[1], Pchar[2] );
    dh = (TMessageHeader *)data;
    memcpy( &DHO, data, sizeof( DHO ) );
    //bytes = DHO.Cmd.LCommand - cmdTlsCmd;
    switch( DHO.Cmd.LCommand ) {
        case cmdReadData:
	    // LParam[0] - first sector number
	    // WParam[0] - sector number
	    // WParam[1] - offset
	    // WParam[2] - bytes number
	    // if bytes number != 0 read bytes number from offset in one sector
	    // WParam[0] ignore
	    // else read needed sectors (WParam[0])
            ReadPage( dh->LParam[0], 512, NetInBuffer );
	    ReplyDataCom( NetInBuffer, 512 );
            break;
        case cmdGetToolInfo:
	    ReplyDataCom( &tp, sizeof( tp ) );
	    break;
    }
    return 0;
}
