#include "..\\inc\\stm32F4xx.h"

#pragma pack(1)

#include <string.h>
#include <math.h>

#include "xxProto.h"
#include "xxCustom.h"
#include "xxSpi.h"

//------------------------------------------------------------------------------

TCalibrate Calb = {
    "ABT",
    19200, 1200,
    0.4656, 0.4835, 0.4951,
    11.949, 6.5775, 4.2602,
    0.0338, 0.7495, 1.0837,
    0, 0, 0, 0,
    // AirNull
    0.0, 0.0, 0.0, 
    // ZondNull
    0.0, 0.0, 0.0,
    // Temperature correct coefs
    0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0    
};

float Sin1f[sinTotalPoints],
      Sin2f[sinTotalPoints];


//------------------------------------------------------------------------------

void CustomRtcIntr( void )
{
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void BreakFunction( void )
{
    // Ask Tool  
    // PowerOn();
    FlagBreak = 0;
    AskTool( 777 );
    // PowerOff();
}

//------------------------------------------------------------------------------

void PowerOn( void ) 
{
    SetBit( PortPower, PinPower );    
    ClrBit( PortPower, PinPowerInv );    
    MsDelay( 5 );
}

//------------------------------------------------------------------------------

void PowerOff( void )
{
    ClrBit( PortPower, PinPower );    
    SetBit( PortPower, PinPowerInv );    
}

//------------------------------------------------------------------------------

int StayStatus = ssWaitStay;

//------------------------------------------------------------------------------

void CustomWork( int param )
{
int move;  
    // Here set custom proc for tool
    
    AxelReadXYZ( &(XY[96]) );

    AverageAxel();
    IsMoving ( si.XYZ[naX], naX );
    IsMoving ( si.XYZ[naY], naY );
    move = IsMoving ( si.XYZ[naZ], naZ );
    
    if( mi.Mode == wmStay ) {
      switch( StayStatus ) {
          case ssWaitStay :
            if( move == 0 ) {
              FlagRecord = 1;
              StayStatus = ssStayStay;
            }
            break;
          case ssStayStay :
            if( move == 0 ) {
              RecordStayCounter++;
              if( RecordStayCounter >= mi.RecordStayCounter ) {
                  RecordStayCounter = 0;
                  StayStatus = ssWaitMove;
              }
              else {
                FlagRecord = 1;
              }
            }
            else {
              RecordStayCounter = 0;
              StayStatus = ssWaitStay;
            }
            break;
          case ssWaitMove :
            if( move == 0 ) break;
            RecordStayCounter = 0;
            StayStatus = ssWaitStay;
            break;
      }
    }
}

//------------------------------------------------------------------------------

void InitCustom( void )
{
    ConfigPin( Gen1Port, PinG1, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ClrBit( Gen1Port, PinG1 );
    ConfigPin( Gen2Port, PinG2, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ClrBit( Gen2Port, PinG2 );
    ConfigPin( Gen3Port, PinG3, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ClrBit( Gen3Port, PinG3 );
    ConfigPin( PortCtrl, CSAdc, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ClrBit( PortCtrl, CSAdc );
}

//------------------------------------------------------------------------------

void InitCalb( void )
{
    strcpy( Calb.Ident, "ABT" );
}

//------------------------------------------------------------------------------

int cnt =0;

//------------------------------------------------------------------------------

void ConvertData( void )
{
    //DOut[0] = (int)((log10(B123[0])+1)*51);  
    //DOut[1] = (int)((log10(B123[1])+1)*51);  
    //DOut[2] = (int)((log10(B123[2])+1)*51);  
    DOut[0] = cnt;
    DOut[1] = cnt;
    DOut[2] = cnt;
    cnt++;
    
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

void Average( short int *src, float *dest )
{
    int i;
    memset((void*)dest,0,4*sinTotalPoints);
    for( i=0; i<ONERECORD; i++ ) 
    {
        dest[i%sinTotalPoints] += (float)src[i];
    }
}

float getPhase(float *ptr)
{
    return 0.0f;
}

float GetDeltaPhase(float *sin1, float *sin2)
{   
    return 0.0f;
}

float min1, min2, max1, max2;    

int ReadAdc( short int *sin1, short int *sin2, int words )
{ 
int i;
short int hi, mi, lo;    

    min1 = min2 = +32767;
    max1 = max2 = -32767;
    
    SendSpiByte( SPI1, 0xE0 );
    MksDelay(10);
    SetBit( PortCtrl, CSAdc );

    SetBit( PortTest, TestPoint2 );
    for( i=0; i<words; i++ ) {

        // read 1-st byte
        SPI1->DR = 0x00;
        while( 1 ) { if( Spi1RxReady ) break; }
        hi = SPI1->DR & 0xFF;

        // read 2-nd byte
        SPI1->DR = 0x00;
        while( 1 ) { if( Spi1RxReady ) break; }
        mi = SPI1->DR & 0xFF;

        // read 3-rd byte
        SPI1->DR = 0x00;
        while( 1 ) { if( Spi1RxReady ) break; }
        lo = SPI1->DR & 0xFF;
        
        sin1[i] = (hi<<4) + (mi>>4);
        sin2[i] = ((mi<<8) + lo)& 0x0FFF;
	if( sin1[i] < min1 ) min1 = sin1[i];
	if( sin2[i] < min2 ) min2 = sin2[i];
	if( sin1[i] > max1 ) max1 = sin1[i];
	if( sin2[i] > max2 ) max2 = sin2[i];
    }

    ClrBit( PortCtrl, CSAdc );
   
    return 0;    
}

//------------------------------------------------------------------------------

void SetTool( void )
{
    strcpy( pp->Name, "ABEMCX" );
    strcpy( pp->RusName, "ЭMКПБХ-1" );
    pp->EEBegMark = 0x3110;
    pp->Ident = TOOLIDENT;
    pp->Serial = 1;
    pp->Version = VERSION;
    pp->NumBlocks = NUMBLOCKS;
    pp->NumPages = NUMPAGES;
    pp->SizePage = SIZEPAGE;
    pp->FreeBlock = 8;
    pp->DefectBlocks = 0;
    pp->SizeOfData = SIZEOFDATA;   // in bytes
    pp->NumChannel = 0;
    pp->SizeOfService = 10;
    pp->Length = 777;              // in cm
    pp->Diameter = 90;             // in mm
    pp->Weight = 40;               // in kg
    pp->Status = stDownhole + stFloatSize;         // Downhole, Float SizeOfData
    pp->MainFreq = F_Crystal;      // Frequency in Hz
    pp->LeftTicks = 0;
    pp->WorkSecCounter = 0;
    pp->uRes = 0;
    pp->SaveBlock = 8;
    pp->BaudRate485 = 19200;
    pp->Div485 = 1536;  // 16
    pp->EEEndMark = 0x1963;

    pp->StartParam.StartTime.wYear = 0x11;
    pp->StartParam.StartTime.wMonth = 0x10;
    pp->StartParam.StartTime.wDay = 0x31;
    pp->StartParam.StartTime.wHour = 0x11;
    pp->StartParam.StartTime.wMinute = 0x00;
    pp->StartParam.StartTime.wSecond = 0x00;
    pp->StartParam.StartTime.mSec = 0;
    pp->StartParam.Mode = wmStop;
    pp->StartParam.TestSecond = 60;
    pp->StartParam.TestCounter = 30;
    pp->StartParam.HowWork = wmStop;
    pp->StartParam.StepTime = 1;
    pp->StartParam.AxLevel = 10;
    pp->StartParam.RecordStayCounter = 3;
    pp->StartParam.CalbCounter = 600;
    pp->StartParam.SecStayCounter = 20;
}

void SetChannel( void )
{
TSensor * cu;
int nom = 0;
long size = 0;

    pp->RealChannel = 0;
    cu = pp->cc + nom;

    strcpy( cu->Name, "Time" );
    cu->MeasurePoint = cu->OwnMeasurePoint = 0;
    cu->Status = tsSave;
    cu->TypeOfData = todRtc;
    cu->SizeOfItem = 8;
    cu->Dimension = 1;
    cu->SizeOfData = cu->SizeOfItem * cu->Dimension;
    size += cu->SizeOfData;
    cu->Step = 0;
    cu->A = 1; cu->B = 0;
    cu->Receivers = 0x00;
    pp->RealChannel++;
    nom++;

//Mark    
    cu = pp->cc + nom;
    strcpy( cu->Name, "Mark" );
    cu->MeasurePoint = cu->OwnMeasurePoint = 0;
    cu->Status = tsSave | tsHex;
    cu->TypeOfData = todI4;
    cu->SizeOfItem = 4;
    cu->Dimension = 1;
    cu->SizeOfData = cu->SizeOfItem * cu->Dimension;
    size += cu->SizeOfData;
    cu->Step = 0;
    cu->A = 1; cu->B = 0;
    cu->Receivers = 0x00;
    pp->RealChannel++;
    nom++;

// Напряжение    
    cu = pp->cc + nom;
    strcpy( cu->Name, "Volt" );
    cu->MeasurePoint = cu->OwnMeasurePoint = 0;
    cu->Status = tsSave;
    cu->TypeOfData = todR4;
    cu->SizeOfItem = 4;
    cu->Dimension = 1;
    cu->SizeOfData = cu->SizeOfItem * cu->Dimension;
    size += cu->SizeOfData;
    cu->Step = 0;
    cu->A = 1; cu->B = 0;
    cu->Receivers = 0x00;
    pp->RealChannel++;
    nom++;
    
// Температура    
    cu = pp->cc + nom;
    strcpy( cu->Name, "Temp" );
    cu->MeasurePoint = cu->OwnMeasurePoint = 0;
    cu->Status = tsSave;
    cu->TypeOfData = todR4;
    cu->SizeOfItem = 4;
    cu->Dimension = 1;
    cu->SizeOfData = cu->SizeOfItem * cu->Dimension;
    size += cu->SizeOfData;
    cu->Step = 0;
    cu->A = 1; cu->B = 0;
    cu->Receivers = 0x00;
    pp->RealChannel++;
    nom++;
    
// Axelerometers
    cu = pp->cc + nom;
    strcpy( cu->Name, "AxelZ" );
    cu->MeasurePoint = cu->OwnMeasurePoint = 0;
    cu->Status = tsSave;
    cu->TypeOfData = todI2;
    cu->SizeOfItem = 2;
    cu->Dimension = 1;
    cu->SizeOfData = cu->SizeOfItem * cu->Dimension;
    size += cu->SizeOfData;
    cu->Step = 1;
    cu->A = 1; cu->B = 0;
    cu->Receivers = 0x00;
    pp->RealChannel++;
    nom++;
    
// Delta Fi1
    cu = pp->cc + nom;
    strcpy( cu->Name, "dPhase1" );
    cu->MeasurePoint = cu->OwnMeasurePoint = 0;
    cu->Status = tsSave;
    cu->TypeOfData = todR4;
    cu->SizeOfItem = 4;
    cu->Dimension = 1;
    cu->SizeOfData = cu->SizeOfItem * cu->Dimension;
    size += cu->SizeOfData;
    cu->Step = 1;
    cu->A = 1; cu->B = 0;
    cu->Receivers = 0x00;
    pp->RealChannel++;
    nom++;

// Delta Fi2
    cu = pp->cc + nom;
    strcpy( cu->Name, "dPhase2" );
    cu->MeasurePoint = cu->OwnMeasurePoint = 0;
    cu->Status = tsSave;
    cu->TypeOfData = todR4;
    cu->SizeOfItem = 4;
    cu->Dimension = 1;
    cu->SizeOfData = cu->SizeOfItem * cu->Dimension;
    size += cu->SizeOfData;
    cu->Step = 1;
    cu->A = 1; cu->B = 0;
    cu->Receivers = 0x00;
    pp->RealChannel++;
    nom++;

// Delta Fi3
    cu = pp->cc + nom;
    strcpy( cu->Name, "dPhase3" );
    cu->MeasurePoint = cu->OwnMeasurePoint = 0;
    cu->Status = tsSave;
    cu->TypeOfData = todR4;
    cu->SizeOfItem = 4;
    cu->Dimension = 1;
    cu->SizeOfData = cu->SizeOfItem * cu->Dimension;
    size += cu->SizeOfData;
    cu->Step = 1;
    cu->A = 1; cu->B = 0;
    cu->Receivers = 0x00;
    pp->RealChannel++;
    nom++;    
    

// Rp1
    cu = pp->cc + nom;
    strcpy( cu->Name, "Rp1" );
    cu->MeasurePoint = cu->OwnMeasurePoint = 0;
    cu->Status = tsSave;
    cu->TypeOfData = todR4;
    cu->SizeOfItem = 4;
    cu->Dimension = 1;
    cu->SizeOfData = cu->SizeOfItem * cu->Dimension;
    size += cu->SizeOfData;
    cu->Step = 1;
    cu->A = 1; cu->B = 0;
    cu->Receivers = 0x00;
    pp->RealChannel++;
    nom++;

// Rp2
    cu = pp->cc + nom;
    strcpy( cu->Name, "Rp2" );
    cu->MeasurePoint = cu->OwnMeasurePoint = 0;
    cu->Status = tsSave;
    cu->TypeOfData = todR4;
    cu->SizeOfItem = 4;
    cu->Dimension = 1;
    cu->SizeOfData = cu->SizeOfItem * cu->Dimension;
    size += cu->SizeOfData;
    cu->Step = 1;
    cu->A = 1; cu->B = 0;
    cu->Receivers = 0x00;
    pp->RealChannel++;
    nom++;
    
// Rp3
    cu = pp->cc + nom;
    strcpy( cu->Name, "Rp3" );
    cu->MeasurePoint = cu->OwnMeasurePoint = 0;
    cu->Status = tsSave;
    cu->TypeOfData = todR4;
    cu->SizeOfItem = 4;
    cu->Dimension = 1;
    cu->SizeOfData = cu->SizeOfItem * cu->Dimension;
    size += cu->SizeOfData;
    cu->Step = 1;
    cu->A = 1; cu->B = 0;
    cu->Receivers = 0x00;
    pp->RealChannel++;
    nom++;    
    
// Sin1

    cu = pp->cc + nom;
    strcpy( cu->Name, "Sin1" );
    cu->MeasurePoint = 0;
    cu->Status = tsSave;
    cu->TypeOfData = todI2;
    cu->SizeOfItem = 2;
    cu->Dimension = 100*3;
    cu->SizeOfData = cu->SizeOfItem * cu->Dimension;
    size += cu->SizeOfData;
    cu->Step = 1;
    cu->Receivers = 0x00;
    pp->RealChannel++;
    nom++;
    
// Sin2

    cu = pp->cc + nom;
    strcpy( cu->Name, "Sin2" );
    cu->MeasurePoint = 0;
    cu->Status = tsSave;
    cu->TypeOfData = todI2;
    cu->SizeOfItem = 2;
    cu->Dimension = 100*3;
    cu->SizeOfData = cu->SizeOfItem * cu->Dimension;
    size += cu->SizeOfData;
    cu->Step = 1;
    cu->Receivers = 0x00;
    pp->RealChannel++;
    nom++;
    
// Calc sizes    
    pp->SizeOfData = size;   // in bytes
    pp->NumChannel = nom;
    pp->SizeOfService = 10;
}

//------------------------------------------------------------------------------

void EmptyTool( void )
{
    pp = (TToolParams *)FlashInBuffer;
    SetTool();
    SetChannel();
}