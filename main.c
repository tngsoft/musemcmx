#include <string.h>
#include "..\\inc\\stm32F4xx.h"
#pragma pack(1)

#include "xxProto.h"
#include "xxNet.h"
#include "xxCustom.h"

TStateInfo si;
TModeInfo mi;
TRtcTime rt;
TNetInfo ni;

TToolParams *pp;
TToolParams tp;

int MsCounter; 
int TsmTimeout, RcvTimeout, FlashTimeout, SpiTimeout, SendTimeout, NetTimeout; 

bool FlagBreak = 0, ReLoad = 0, FlagAxel = 0, NetSleepPin;
bool FlagNet = 0, FlagRtc = 0, FlagRecord = 0, FlagAdc = 0;
bool FlagNetTsm = 0, FlagNetRcv = 0, NoEE = false, bRes;

short int XY[120], FlashOffset=0, offset;
long int LastBlock=1;
int StepTimeCounter;

unsigned int BadBlocks[SizeBadBl/4];
char FlashInBuffer[4100], *FlashPtr;
char NetInBuffer[1600], *NetRcvData;

short int NumHost = 0;

// Delay on ms msec
void MsDelay( long int ms )
{
    MsCounter = 0;
    while( MsCounter <= ms );
}

static int Mks;

// Delay on ms msec
void MksDelay( float mks )
{
  __disable_interrupt();  
    Mks = (int)(mks * 18);
    while( Mks-- );
  __enable_interrupt();
}

// Delay while no TLS command
void MsBreakDelay( long int ms )
{
    MsCounter = 0;
    while( MsCounter <= ms ) {
      if( FlagBreak ) return;
    }
}

static int AxCounter = 0;

int main()
{  
once:
    HandlerInit();
    ReLoad = 0;

    InitReceive485( 20 ); 

    FlagBreak = 0;    
    while(1) {
	switch( (unsigned short int)(si.Mode) ) {
	    case wmWait:	
	    case wmTest:	
		HandlerWait();
                FlagBreak = 0;    
	        break;
	    case wmWork:
		HandlerWork();
		break;
	    case wmStop:
		HandlerStop();
                FlagBreak = 0;    
		break;    	
	}
	//Handler485();
        HandlerNets();
        if( ReLoad ) goto once;
        CheckLink(); 
	//Handler485();
	HandlerStatus();
	
	//Handler485();
        if( FlagAxel ) {
	    FlagAxel = 0;
            AxelReadXYZ( &(XY[AxCounter]) );
            AxCounter += 3;
            if( AxCounter >= 120 ) AxCounter = 0;
	}
/*------ while not uart -------------------
        NumHost = CheckReceiveUart();
        if( NumHost != 0 ) {
            EncodeHost( NetInBuffer );
            NumHost = 0;
            InitReceiveUart( NetInBuffer, 256, 50 );    
	}
------------------------------------------*/
    }
}

void RtcHandler( void )
{
    FlagRtc = 1;
    SecCounter++;
    SecExecute++;
    SecNetCounter++;
    
    if( mi.Mode != wmWork ) return;
    
    CustomRtcIntr();
 
    if( mi.HowWork != wmStay ) { 
        StepTimeCounter--;
        if( StepTimeCounter <= 0 ) { 
            FlagRecord = 1;
            StepTimeCounter = mi.StepTime;
        }
    }
}

//------------------------------------------------------------------------------

// Axelerometers
void EXTI3_IRQHandler(void)
{
    EXTI->PR |= EXTI_PR_PR3;                         //Очистка Pending 
    //FlagAxel = 1;
}

//------------------------------------------------------------------------------
extern int ExtRtc;
//------------------------------------------------------------------------------

// RTC Line 6 (PE6)
void EXTI9_5_IRQHandler(void)
{
int irq;
    irq = EXTI->PR;
    if( (irq & EXTI_PR_PR6 ) != 0 ) {
        EXTI->PR |= EXTI_PR_PR6;                         //Очистка Pending 
	if( ExtRtc ) RtcHandler();
    }
}

//------------------------------------------------------------------------------
