#include "..\\inc\\stm32F4xx.h"

#pragma pack(1)

// Fuart = Fclk/(n+1)*16
// N = (Fclk/16*Fuart) - 1

// USARTDIV = DIV_Mantissa + (DIV_Fraction / 8 * (2 Ц OVER8))

#include <string.h>

#include "xxProto.h"
#include "..\\..\\xxTcp.h"
#include "..\\..\\xCommand.h"

#include "xxCustom.h"

int ToolAddress = 0;
int HaveTlsCommand = 0;
int ImTls = 0;

static int value;
static char *OutBuffer;

#define cmdTlsReset	0
#define cmdTlsWakeUp	1
#define cmdTlsStart	2
#define cmdTlsGetStatus	3
#define cmdTlsGetInfo	4
#define cmdTlsGetData	5
#define cmdTlsSetParams	6
#define cmdTlsLinkTest	7

#define stErrRtc	0x01
#define stErrSD 	0x02
#define stErrPower	0x04
#define stErrAx 	0x08
#define stErrMem	0x10
#define stErrTool	0x60
#define stErrBusy	0x80

/*
D8 = 0
D7 Ц готовность прибора 1-готов, 0-прибор зан€т (идет тестирование или ре-гистраци€ данных).
D6 Ц D5 Ц код ошибки, завис€щей от конкретного прибора
	00 Ц нет ошибок
	01, 10, 11 Ц коды ошибок.  онкретный смысл зависит от прибора.
D4 Ц пам€ть прибора полностью заполнена
D3 Ц ошибка работы акселерометров
D2 Ц ресурс аккумул€торов по питанию меньше 50%
D1 Ц ошибка пам€ти (SD карта и т.п.)
D0 Ц ошибка часов реального времени (питание или частота) 
*/

extern float B123[8];
extern 
int cnt;
static char Address, Command, Param, CheckSum;
char Status485 = 0;
char DOut[1512];
char Test485[3] = {55,56,57};

static int WaitCommand = 0, WaitData = 0, WaitOneByte = 0, TlsCounter = 0, Timeout, MaxCount;

void USART2_IRQHandler(void)
{
short int Status;

    Status = USART2->SR;
    NVIC_ClearPendingIRQ( USART2_IRQn );
    
    if( (Status & 0x0E) != 0 ) goto ret;	// Errors

    if( (Status & USART_FLAG_RXNE) != 0 ) {	// Receive
        value = USART2->DR;
        if( (value & 0x100) != 0 ) {
            Address = value & 0xFF;
	    if( (Address == TOOLADDRESS) || (Address == ANYADDRESS) ) {
	        WaitCommand = 1; 
                WaitData = 0;
                WaitOneByte = 0;
	        RcvTimeout = 0;
	    }
        }
        else {
            if( WaitCommand ) {
  	        Command = value;  
	        RcvTimeout = 0;
                WaitCommand = 0;
                if( Command == 0x77 ) WaitData = 1;
                else {
                    switch( Command ) {
                        case cmdTlsReset :	  
                        case cmdTlsGetStatus :
                        case cmdTlsWakeUp :
                            Status485 = 0x80;
                            if( Address != ANYADDRESS ) 
                                SendToTls( &Status485, 1 );
                            RcvTimeout = 100;
                            WaitCommand = 0; 
                            WaitData = 0;
                            WaitOneByte = 0;
                            HaveTlsCommand = 1;
                            break;
                        case cmdTlsStart :
                            RcvTimeout = 100;
                            WaitCommand = 0; 
                            WaitData = 0;
                            WaitOneByte = 0;
                            HaveTlsCommand = 1;
                            FlagBreak = 0;
                            break;
                        case cmdTlsGetData :
                            RcvTimeout = 100;
                            WaitCommand = 0; 
                            WaitData = 0;
                            WaitOneByte = 0;
                            HaveTlsCommand = 1;
                            if( Address != ANYADDRESS )
                            {
                              if ( si.Mode != wmStop)
                                SendToTls( Reply485, 3 );
                              else
                              {
                                SendToTls( Test485, 3 );
                                Test485[0]++;
                                Test485[1]++;
                                Test485[2]++;
                              }
                            }
                            break;
                        case cmdTlsGetInfo :
                            DOut[0] = TOOLADDRESS;
                            DOut[1] = tp.Serial;
                            DOut[2] = Status485;
                            if( Address != ANYADDRESS ) SendToTls( DOut, 3 );
                            break;
                        case cmdTlsSetParams :
                        case cmdTlsLinkTest :
                            WaitOneByte = 1;
                        default:
                            WaitData = 1;
                            break;
                    }                  
                }
            }
            else {
                if( WaitOneByte ) {
                  Param = value;
                  if( Command == cmdTlsSetParams ) {
                      if( Address != ANYADDRESS ) SendToTls( &Param, 1 );
                  } 
                  if( Command == cmdTlsLinkTest ) {
                      Param ^= 0xFF;
                      if( Address != ANYADDRESS ) SendToTls( &Param, 1 );
                  } 
                  goto ret;
              }
              if( WaitData ) {
                  if( TlsCounter < MaxCount ) DOut[TlsCounter++] = value;
	          RcvTimeout = 0;
              }
            }
        }
    }
ret:
    value = USART2->DR;
    return;
}

void SetBaudRate485( int baud )
{
    NVIC_DisableIRQ( USART2_IRQn );  
    USART2->CR1 = 0;
    USART2->BRR = baud;		// 208.3333 - 38400 - 8 MHz
    USART2->CR1 = (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE  | USART_CR1_M ); // ¬ключение USART1 и разрешение прерываний по приему.
    NVIC_ClearPendingIRQ( USART2_IRQn );
    NVIC_EnableIRQ( USART2_IRQn );  
}

void InitReceive485( int timeout )
{
    CheckSum = 0;
    MaxCount = 1500;
    TlsCounter = 0;
    RcvTimeout = 0;
    Timeout = timeout;
    Timeout = (int)(12900./19200); //tp.BaudRate485;
    if( Timeout <= 10 ) Timeout = 10;
    HaveTlsCommand = 0;
    EnableRcv485();
}

int CheckReceive485( void )
{
  if( HaveTlsCommand ) { return 1; }
    if( TlsCounter != 0 ) {
      if( RcvTimeout > Timeout ) { return TlsCounter; }
    }
    return 0;
}

int SendToTls( void *data, int len )
{
int i, status;

    if( len == 0 ) return 2; 
    
    if( len < 0 ) len = -len;
    OutBuffer = (char *)data;

    // Direct to out R/D = 1
    SetBit( PortDir485, PinDirect485 );    
    
    i = 0;
    while( len > 0 ) {        
        USART2->DR = OutBuffer[i];     // send byte
	while( 1 ) {
	    status = USART2->SR;
	    if( (status & USART_SR_TC) != 0 ) break;
	}
	i++; len--;
    }
    // Direct to In R/D = 0
    ClrBit( PortDir485, PinDirect485 );    
    return len;
}

int Encode485Command( void );
int From485;

/*
D8 = 0
D7 Ц готовность прибора 1-готов, 0-прибор зан€т (идет тестирование или ре-гистраци€ данных).
D6 Ц D5 Ц код ошибки, завис€щей от конкретного прибора
	00 Ц нет ошибок
	01, 10, 11 Ц коды ошибок.  онкретный смысл зависит от прибора.
D4 Ц пам€ть прибора полностью заполнена
D3 Ц ошибка работы акселерометров
D2 Ц ресурс аккумул€торов по питанию меньше 50%
D1 Ц ошибка пам€ти (SD карта и т.п.)
D0 Ц ошибка часов реального времени (питание или частота) 
*/
int TestCheckSum( char *in, int num )
{
int i;
    CheckSum = 0;
    for(i=0; i<num-1; i++ ) CheckSum += in[i];
    if( CheckSum == in[num-1] ) return 1;
    else return 0;
}

void Handler485( void )
{
  HaveTlsCommand = CheckReceive485();
  if( HaveTlsCommand  == 0 ) return;
  HaveTlsCommand = 0;
  // DisableRcv485();
  switch( Command ) {
      case cmdTlsStart :
          if( si.Mode != wmStop )
            AskTool( 777 );
          break;
      case cmdTlsGetData :
          break;
      case 0x77:
          if( TestCheckSum( DOut, TlsCounter ) == 0 ) break;
          memcpy( NetInBuffer, DOut, TlsCounter-1 );
          Encode485Command();
          break;

  }   
  InitReceive485( 20 ); 
}

#include "xxNet.h"

static char *Pchar;

int Encode485Command( void )
{
int count;
    Pchar = (char *)(&mh);
    if( (NetInBuffer[12] == 0x08) && (NetInBuffer[13] == 0x00) && (NetInBuffer[23] == 0x11) ) {     // UDP
	pmh = (TMessageHeaderEx *)NetInBuffer;
	for( count=0; count<sizeof( TMessageHeaderEx ); count++ ) Pchar[count] = NetInBuffer[count];
        if( pmh->udp.rPort != SrcPort ) return 0; 
        HaveAddress = 1;
	for( count=0; count<sizeof( TMessageHeaderEx ); count++ ) Pchar[count] = NetInBuffer[count];
        for( count=0; count<6; count++ ) HostMAC[count] = pmh->ss[count];
        for( count=0; count<4; count++ ) HostIP[count] = pmh->ip.sIP[count];
	NetRcvData = NetInBuffer + sizeof( TMessageHeaderEx );
        Pchar = (char *)(mh.Command);
	Swapb( Pchar[0], Pchar[1] );
	Swapb( Pchar[2], Pchar[3] );
        From485 = 1;
        ToolAddress = mh.FrameNumber;
        if( ToolAddress == TOOLADDRESS ) EncodeCommand();
        //From485 = 0;
	return 1;
    }
    return 0;
}

int Send485Packet( char *data, int lendata, char *head, int lenhead )
{
int i, status;

    CheckSum = 0;
    for( i=0; i<lenhead; i++ ) CheckSum += head[i];
    for( i=0; i<lendata; i++ ) CheckSum += data[i];
    
    // Direct to out R/D = 1
    SetBit( PortDir485, PinDirect485 );    

    // Send header
    i = 0;
    while( lenhead > 0 ) {        
        USART2->DR = head[i];     // send next byte
	while( 1 ) {
            // Check shift reg empty
	    status = USART2->SR;
	    if( (status & USART_SR_TXE) != 0 ) break;
	}
	i++; lenhead--;
    }
    
    // Send data
    i = 0;
    while( lendata > 0 ) {        
        USART2->DR = data[i];     // send next byte
	while( 1 ) {
            // Check shift reg empty
	    status = USART2->SR;
	    if( (status & USART_SR_TXE) != 0 ) break;
	}
	i++; lendata--;
    }

    USART2->DR = CheckSum;     // send CheckSum
    while( 1 ) {
        // Check tsm complete
	status = USART2->SR;
	if( (status & USART_SR_TC) != 0 ) break;
    }
    
    // Direct to In R/D = 0
    ClrBit( PortDir485, PinDirect485 );    

    return 0x77;
}
