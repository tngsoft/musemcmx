#include <string.h>
#include <math.h>
#include "..\\inc\\stm32F4xx.h"
#pragma pack(1)

#include "xxProto.h"
#include "..\\..\\xCommand.h"

#include "xxCustom.h"

float B123[8];
char Reply485[8] = { 17, 18, 19, 0, 0, 0, 0, 0};

extern int cnt;

TRecord Rec;

int IncPages( void *Data, int pages, bool ask )
{
char *data = Data;
int i;
    FlashOffset = 0;
    if( ask == 0 ) return 1;
    for( i=0; i<pages; i++ ) {
        if( si.Block >= NUMBLOCKS ) {
            Status485 |= stErrMem;    
            return 0;
        }
        WritePage( si.Block, 512, data );
        data += SIZEPAGE;
        si.Block++;
    }
    IncSaveBlock( si.Block );
    return 1;
}

// Delay on mks
void _MksDelay( int mks )
{
    mks *= 18;
    while( mks-- );
}

extern float min1, min2, max1, max2;    

char Buffer[1024];
short int Sin1[ONERECORD];
short int Sin2[ONERECORD];

int AskTool( int ask )
{
float temp;
int i, ampl, numrec, size;
//TRecord *rec = &Rec;
//TRecord *rec = (TRecord *)(FlashInBuffer + FlashOffset);
TRecord *rec = (TRecord *)(Buffer + FlashOffset);

    // Start ADC
    ADC1->CR2 |= ADC_CR2_SWSTART;

    SetBit( PortTest, TestPoint3 );
    Status485 &= (~stErrBusy);
    
    offset = FlashOffset;

    GetRtc();	////////////////////////////////////////////////////////

    // Axelerometers
    rec->Z = si.XYZ[0];
    
    LastBlock = si.Block;

    rec->rt = rt;
    rec->mark = 0x1963;
    if( ask == 777 ) rec->mark = 0x777;
    
    rec->Temp = 25 + (si.Temperature/8);
    
    // Read ADC
    rec->Volt = (ADC1->DR);
    
    for( i=0; i<3; i++ ) {
    __disable_interrupt();      
        if( i == 0 ) {
            // Generation On
            SetBit( Gen1Port, PinG1 );
            _MksDelay( 30 );
            SendSpiByte( SPI1, 0x80 );
            _MksDelay( 400 );
            // Generation Off        
            ClrBit( Gen1Port, PinG1 );
        }
        
        if( i == 1 ) {
            // Generation On
            SetBit( Gen2Port, PinG2 );
            _MksDelay( 30 );
            SendSpiByte( SPI1, 0x80 );
            _MksDelay( 400 );
            // Generation Off        
            ClrBit( Gen2Port, PinG2 );
        }
        
        if( i == 2 ) {
            // Generation On
            SetBit( Gen3Port, PinG3 );
            _MksDelay( 30 );
            SendSpiByte( SPI1, 0x80 );
            _MksDelay( 400 );
            // Generation Off        
            ClrBit( Gen3Port, PinG3 );
        }
        __enable_interrupt();
    
	//ReadAdc( Sin1, Sin2, ONERECORD );
#include "sinus.inc"
        Average(Sin1,Sin1f);
        Average(Sin2,Sin2f);
/*
	ampl = max1 - min1;
	if( ampl > 900 ) ampl = 900;
	ampl /= 100;
	if( (ampl % 100) != 0 ) ampl++;
	rec->Index |= (ampl<<((i<<3)));

	ampl = max2 - min2;
	if( ampl > 900 ) ampl = 900;
	ampl /= 100;
	if( (ampl % 100) != 0 ) ampl++;
	rec->Index |= (ampl<<((i<<3)+4));
*/
        rec->dPhase[i] = GetDeltaPhase( Sin1f, Sin2f );
        memcpy( rec->Sin2+i*100, Sin1, 200 );
        memcpy( rec->Sin1+i*100, Sin2, 200 );
	temp = rec->dPhase[i] - Calb.ZondNull[i] - Calb.AirNull[i];
        rec->Rp[i] = Calb.A[i]*temp*temp + Calb.B[i]*temp + Calb.C[i];        
        temp = Calb.TA[i]*temp*temp + Calb.TB[i]*temp + Calb.TC[i];
        rec->Rp[i] -= temp;
        // Здесь можешь поставить любое значение, не обязательно 1
        if( rec->Rp[i] < 0.1 ) rec->Rp[i] = 0.1;
    }
    
    // pack for TLS
    for( i=0; i<3; i++ ) {
        Reply485[i] = (int)round( log10( rec->Rp[i] + 0.5 ) * 63.4313 );
    }
    
    LastBlock = si.Block;

    if( tp.SizeOfData <= SIZEPAGE ) {
        // Save data to SD
        if( (FlashOffset + 2*tp.SizeOfData) > SIZEPAGE ) {
            //FlashPtr = FlashInBuffer;
            FlashPtr = Buffer;
            IncPage( ask );
        }
        else FlashOffset += tp.SizeOfData;
    }
    else {
        FlashPtr = (char *)rec;
        size = sizeof( TRecord );
        numrec = size / SIZEPAGE;
        if( (size % SIZEPAGE) != 0 ) numrec++;
        IncPages( rec, numrec, ask );
    }
    
    // Clear busy flag for TLS
    Status485 |= (stErrBusy);

    ClrBit( PortTest, TestPoint3 );

    // 777 - it'+s GetData from TLS
    // if( ask == 777 ) return 1;
    
    // If have net connect - send data to host
    if( From485 ) return 1;
    if( HaveAddress > 0 ) {
      if( NetSleepPin == 0 ) {
        mh.Command[0] = cmdGet;
        mh.Command[1] = cmdTestData;
        mh.Status = 0;
        mh.UnicalNumber = 0;
        mh.LenData = tp.SizeOfData;
        mh.LParam[0] = LastBlock;
        SendDataEx( rec, tp.SizeOfData, cmdGet, cmdTestData, 0, 0 );      
      }
    }
    return 1;
}

//---------------------------------------------------------------------------
/*
int SetCross( float *Val, float *Cross, int maxcross, double *Freq )
{
int nCross=0;

    int *iCross = new int[maxcross];

    for( int i=0; i<Count-1; i++ ) {
        if( Val[i] < 0 && Val[i+1] >= 0 ) iCross[nCross++] = i;
        if( nCross >= maxcross ) break;
    }
    for( int i=0; i<nCross; i++ ) {
        Cross[i] = Step*iCross[i] + (abs( Val[iCross[i]] )*Step)/(abs( Val[iCross[i]] ) + Val[iCross[i]+1] );
    }

    for( int i=0; i<nCross-1; i++ ) {
        Cross[i] = (Cross[i+1] - Cross[i]);
    }

    if( nCross > 8 ) {
        for( int i=3; i<nCross; i++ ) iCross[i-3] = iCross[i];
        nCross -= 6;
    }

    double sum = 0;
    for( int i=0; i<nCross; i++ ) {
        sum += Cross[i];
    }
    if( nCross != 0 ) sum /= nCross;
    else sum = sum;
    *Freq = max( 0.1, sum );

    delete[] iCross;

    return nCross;
}
*/
//---------------------------------------------------------------------------
