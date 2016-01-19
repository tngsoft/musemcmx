#include <string.h>
#include "..\\inc\\stm32F4xx.h"
#pragma pack(1)

#include "xxProto.h"
#include "..\\..\\xCommand.h"
#include "xxRtc.h"
#include "xxNet.h"

#include "xxCustom.h"

short int PrevAx[3], MinAx[3], MaxAx[3], dAx[3];
int ExecuteGo = 2, RecordStayCounter = 0, SecStayCounter;
int SecCounter = 0, SecNetCounter = 0, SecExecute = 0;

void InitValues( void )
{
int i;
    StepTimeCounter = mi.StepTime;
    SecStayCounter = mi.SecStayCounter;
    StayStatus = ssWaitStay;
    SecCounter = 0;
    SecExecute = 0;
    SecNetCounter = 0;
    // Заряжаем на выполнение теста
    ExecuteGo = 10;
    // Reset axel data
    for( i=0; i<3; i++ ) {
        MinAx[i] = 0x7FFF;
        MaxAx[i] = 0x8000;
        dAx[i] = 0;
    }
}

bool HandlerInit( void )
{
int freq;

    freq = SystemCoreClock/1000;
    if( (SystemCoreClock%1000) >= 500 ) freq++;
    
    si.Status = 0;
    SysTick_Config( freq );   
    __disable_interrupt();
    InitPorts();
    InitUart();
    __enable_interrupt();

    InitTimers();
    InitCustom();
    //if( ReLoad == 0 ) 
      InitNet();
    InitAxel();
    InitIntr();
    NoEE = !EEInit();
    if( ReLoad )
    {
      RCC->BDCR|=RCC_BDCR_BDRST;
      MksDelay(50);
      RCC->BDCR&=~RCC_BDCR_BDRST;
    }
    InitRtc();
    
    AxelReadXYZ( (char *)(&si.XYZ[0]) );
    
    NoSD = InitSD();
    if( NoSD ) Status485 |= stErrSD;

    PowerOff();

    MsDelay( 10 );
   
    // Read start time, WorkMode & other params
    ReadMainTp( FlashInBuffer );
    pp = (TToolParams *)FlashInBuffer;

    if( (pp->EEBegMark != 0x3110) || (pp->EEEndMark != 0x1963) 
                       || (pp->Name[0] != 'A') || (pp->Name[1] != 'B') ) {
        EmptyTool();
        WriteMainTp( FlashInBuffer );
    }
    
    ReadMainTp( &tp );
    tp.MainFreq = SystemCoreClock;      // Frequency in MHz
    
    memcpy( &mi, &(tp.StartParam), sizeof( mi ) );
        
    si.Ident = tp.Ident;
    si.Serial = tp.Serial;
    si.Mode = tp.StartParam.Mode;
    GetRtc();
    
    // Номера страниц для записи  "BLK" N далее 0    
    si.Block = FindSaveBlock();

    // Calibration values
    ReadCalb( FlashInBuffer, sizeof( TCalibrate)  );
    if( (FlashInBuffer[0] != 'A') || 
          (FlashInBuffer[1] != 'B') || 
            (FlashInBuffer[2] != 'T') ) {
        WriteCalb( &Calb, sizeof( TCalibrate)  );
    }
    
    ReadCalb( &Calb, sizeof( TCalibrate)  );
    SetBaudRate485( Calb.Div485 );

    InitBadBlocks();
		  
    if( mi.Mode == wmWork ) PowerOn();  
    FindFreeMarker();

/*
      mi.Mode == wmStandBy ||
        mi.Mode == wmStay ||
        mi.Mode == wmQuery 
*/
    
    InitValues();

    return true;
}

void HandlerWork( void )
{
int i;
    if( FlagRtc ) {
      FlagRtc = 0;
      CustomWork( 0 );
    }
   
    if( FlagRecord ) {
        FlagRecord = 0;
        // Fix delta Axelerometers values
        dAx[naX] = MaxAx[naX] - MinAx[naX];
        dAx[naY] = MaxAx[naY] - MinAx[naY];
        dAx[naZ] = MaxAx[naZ] - MinAx[naZ];
        //Ask Tool
        if( mi.HowWork == wmStandBy ) AskTool( 0 );
        else if( mi.HowWork != wmQuery ) AskTool( 1 );  
        // Reset axel data
        for( i=0; i<3; i++ ) {
            MinAx[i] = 0x7FFF;
            MaxAx[i] = 0x8000;
            dAx[i] = 0;
        }
    }
}


void HandlerStop( void )
{
    if( FlagRtc ) {
        FlagRtc = 0;
        // Опрос прибора
        GetRtc();
        //AskTool( 0 );
    }
}

void HandlerWait( void )
{
    if( FlagRtc ) {
        
        //WaitFunction();
        
        FlagRtc = 0;
        GetRtc();

        // Если время сбилось - в режим записи	???
        if( (si.Status & stRtcBad) != 0 ) {
            PowerOn();
            SetWorkMode( wmWork );
            //FindFreeMarker();
            return;
        }
        else {
            // Если настало время включения - в режим записи
            if( IsStartTime() >= 0 ) {
                PowerOn();
                SetWorkMode( wmWork );
                //FindFreeMarker();
                return;
            }
        }

            // Ждем включения тестового режима
        if( ExecuteGo >= 2 ) {
            if( SecExecute >= mi.TestSecond ) {
                // Дождались - включаемся
                PowerOn();
                // 2 потому что тест на 2 режимах - калибровка и нормальный
                ExecuteGo = 1;
                si.Mode = wmTest;
                FindFreeMarker();
                SecExecute = 0;
                // сначала  режим калибровки
                // SendToolCommand( ecCal );
            }
        }

        if( ExecuteGo == 1 ) {
            // Опрос прибора
            AskTool( 0 );
            if( SecExecute >= mi.TestCounter ) {
                ExecuteGo = 0;
                si.Mode = wmWait;
                FindFreeMarker();
                PowerOff();
            }
        }
    }
}

void WriteNetReg( char bank, char reg, char value );

void HandlerNets( void )
{
    if( NetSleepPin ) return;
    
    if( HandlerNetMessage() ) {
        SecNetCounter = 0;
    }
    
    if( SecNetCounter > 1200 ) {
        if( NetSleepPin == 0 ) {  // is net waked up
            //WriteNetReg( 0, ControlReg2, crSleep + crIncPointer );
            NetSleepPin = 1;    // go to Sleep net
	    MsDelay( 20 );
        }
    }
    return;    
}

void SetWorkMode( int mode ) 
{
    switch( mode ) {
        case wmWait:
            memcpy( &(tp.StartParam), &mi, sizeof( mi ) );
            break;
        case wmWork:
            InitValues();
            PowerOn();
            // wm = tp.StartParam.HowWork;
            //if( !((si.Mode == wmBefore) || (si.Mode == wmTesting)) ) return;
            // Define free memory
            
            break;
        case wmStop:
            PowerOn();
            tp.StartParam.HowWork = wmStop;
            if( si.Mode == wmStop ) return;
            break;
        default:
            return;
    }
    FlashOffset = 0;
    si.Mode = mode;
    mi.Mode = mode;
    tp.StartParam.Mode = mode;
    FindFreeMarker();
    FlashPtr = (char *)(&tp);
    WriteStartParams( &mi  );
    //if( From485 == 0 ) SendDataEx( FlashPtr, SizeFtp, cmdGet, cmdToolInfo, 0, 0 );
}

int AddLen = 0x200;
char FF[4] = { 0xFF, 0xFF, 0xFF, 0xFF };

#define StatusMs  700

void SendStatus( int  send )
{
int mode;

    if( SendTimeout >= StatusMs ) send = 1;
    if( send ) {
        SendTimeout = 0;
        if( From485 ) 
          if( ToolAddress != TOOLADDRESS ) 
            return; 
        if( NoSD ) si.Status |= stSDBad;
        GetRtc();
        if( HaveAddress >= 0 ) {
            AverageAxel();
            mode = si.Mode;
	    AddLen ^= 0x200;
            SendDataEx( &si, sizeof(si), cmdGet, cmdStateInfo, 0, 0 );            
            si.Mode = mode;
   	}
    }
}

void HandlerStatus( void ) 
{
    if( From485 == 0 ) SendStatus( 0 );
}

int IncPage( bool ask )
{
    if( si.Block >= NUMBLOCKS ) {
        Status485 |= stErrMem;    
        return 0;
    }
    FlashOffset = 0;
    if( ask ) {
        WritePage( si.Block, 512, FlashPtr );
        si.Block++;
        IncSaveBlock( si.Block );
    }
    return 1;
}

// Test moving

int IsMoving ( int value, int n )
{
int ax;

    ax = value-PrevAx[n];
    if( ax < MinAx[n] ) MinAx[n] = ax;
    if( ax > MaxAx[n] ) MaxAx[n] = ax;
    // Test moving by axelerometers 
    if( mi.Mode == wmStay ) {
        // define delta Az 
        if( (value == 0) || (value==0xFFFF) )  { // axel breaked
            ax = mi.AxLevel + 100;
        }
        else {
            if( value < 0 ) value = -value;
            ax = value-PrevAx[n];
            if( ax < 0 ) ax = -ax;
        }
        // check sleep period
        if( ax < mi.AxLevel ) {
            if( SecStayCounter > 0 ) SecStayCounter--;
            else return 0;
        }
        else {
            SecStayCounter = mi.SecStayCounter;
        } 
        PrevAx[n] = value;
    }
    return 1;
}


// Axelerometers

void AverageAxel( void )
{
int i, x, y, z;  

    x = y = z = 0;
    for( i=0; i<48; i+=3 ) {
        x += XY[i+naX];
        y += XY[i+naY];
        z += XY[i+naZ];
    }
    si.XYZ[naX] = (x>>5);
    si.XYZ[naY] = (y>>5);
    si.XYZ[naZ] = (z>>5);
    
    x= y = z = 0;
    for( i=48; i<96; i+=3 ) {
        x += XY[i+naX];
        y += XY[i+naY];
        z += XY[i+naZ];
    }
    si.XYZ[naX] += (x>>5);
    si.XYZ[naY] += (y>>5);
    si.XYZ[naZ] += (z>>5);
}

int FindFreeMarker( void )
{
int n;
    ReadRuns( FlashInBuffer );
    for( n=0; n<SizeRuns; n+=sizeof( si ) ) {
	if( FlashInBuffer[n] == 0xFF ) {
	    //memcpy( FlashInBuffer+n, &si, sizeof( si ) );
	    WriteRuns( &si, n, sizeof( si ) );
	    return n;
	}
    }
    return 0;
}

