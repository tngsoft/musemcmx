#include <string.h>
#include "..\\inc\\stm32F4xx.h"

#pragma pack(1)

#include "xxProto.h"
#include "xxRtc.h"
#include "xxSpi.h"

#include "xxCustom.h"

int RTC_CPOL = 0;
int RTC_CPHA = 0;

void RtcHandler( void );

short int flagRtcBad, flagBatBad;

void RtcReadBytes( char addr, char *in, int num );
void RtcWriteBytes( char addr, char *out, int num );

static char axData;
char *OutRtc, *InRtc;
//short int numRtcBytes = 0;

int ExtRtc = 0;

typedef struct {
    unsigned char  wSecond;
    unsigned char  wMinute;
    unsigned char  wHour;
    unsigned char  wDayOfWeek;
    unsigned char  wDay;
    unsigned char  wMonth;
    unsigned char  wYear;
    unsigned char  wNone;
} TRtc;

TRtc OwnRtc;

void _WriteRtcByte( char Byte )
{
    SendSpiByte( SpiRtc, Byte );
}

char _ReadRtcByte( void )
{
    return SendSpiByte( SpiRtc, 0x00 );
}

void RtcWriteBytes( char addr, char *out, int num )
{
int i;
    ClrBit( PortRtc, CSRtc );
    addr |= rtcWrite;  
    SetSpiPolarity( SpiRtc, RTC_CPOL, RTC_CPHA );
    SendSpiByte( SpiRtc, addr );
    for( i=0; i<num; i++ ) {
        SendSpiByte( SpiRtc, out[i] );
    }
    SetBit( PortRtc, CSRtc );
}


void RtcReadBytes( char addr, char *in, int num )
{
int i;
    ClrBit( PortRtc, CSRtc );
    addr |= rtcRead;    
    SetSpiPolarity( SpiRtc, RTC_CPOL, RTC_CPHA );
    SendSpiByte( SpiRtc, addr );
    for( i=0; i<num; i++ ) {
        in[i] = SendSpiByte( SpiRtc, addr );
    }
    SetBit( PortRtc, CSRtc );
}

// Return < 0 if current time < Start
// >0 if current time > Start
// == 0 if current time == Start

int IsStartTime( void )
{
    //GetRtc();
    rt.wSecond &= 0x7F;
    if( mi.StartTime.wYear > rt.wYear ) return -1;
    if( mi.StartTime.wYear < rt.wYear ) return +1;
    if( mi.StartTime.wMonth > rt.wMonth ) return -1;
    if( mi.StartTime.wMonth < rt.wMonth ) return 1;
    if( mi.StartTime.wDay > rt.wDay ) return -1;
    if( mi.StartTime.wDay < rt.wDay ) return 1;
    if( mi.StartTime.wHour > rt.wHour ) return -1;
    if( mi.StartTime.wHour < rt.wHour ) return 1;
    if( mi.StartTime.wMinute > rt.wMinute ) return -1;
    if( mi.StartTime.wMinute < rt.wMinute ) return 1;
    if( mi.StartTime.wSecond > rt.wSecond ) return -1;
    if( mi.StartTime.wSecond < rt.wSecond ) return 1;
    return 0;
}

int IsTestTime( void )
{
    return 0;
}

int InitExtRtc( void )
{
once:
  // Write to control registers
    axData = ctRate1Hz + ctTempConvert;
    RtcWriteBytes( arControl, &axData, 1 );
    axData = st32KHzEna;
    RtcWriteBytes( arStatus, &axData, 1 ); 
    RtcReadBytes( arControl, &axData, 1 ); 
    if( axData != 0x1C ) {//(ctRate1Hz + ctTempConvert) ) {
      MksDelay( 500 );  
       //goto once;
        //si.Status |= stRtcBad;
        //ExtRtc = 0;
    }
    return 1;
}

int SetExtRtc( TRtcTime *tt )
{
    SetBit( PortTest, TestPoint4 );
    __disable_interrupt();
    //RtcWriteRegister( arControl, ctOscDisable );

    OwnRtc.wSecond = tt->wSecond;
    OwnRtc.wMinute = tt->wMinute;
    OwnRtc.wHour = tt->wHour;
    OwnRtc.wDay = tt->wDay;
    OwnRtc.wDayOfWeek = 1;
    OwnRtc.wMonth = tt->wMonth;
    OwnRtc.wYear = tt->wYear;

    //RtcWriteRegister( arControl, ctRate1Hz );

    RtcWriteBytes( arSecond, (char *)(&OwnRtc), 7 );
    ClrBit( PortTest, TestPoint4 );
    GetRtc();
    __enable_interrupt();
    return 1;
}

int GetExtRtc( void )
{
    //si.Status &= stClrRtc;
    RtcReadBytes( arSecond, (char *)(&OwnRtc), 7 );

    rt.wSecond = OwnRtc.wSecond;
    rt.wMinute = OwnRtc.wMinute;
    rt.wHour = OwnRtc.wHour;
    rt.wDay = OwnRtc.wDay;
    rt.wMonth = OwnRtc.wMonth;
    rt.wYear = OwnRtc.wYear;
//    rt.mSec = 0;
    
//    flagBatBad = (rt.wSecond&BitBatBad);

    flagRtcBad = 0;
    if( (rt.wMinute&0x80) != 0 ) flagRtcBad |= stRtcBad;
    if( (rt.wHour&0x80) != 0 ) flagRtcBad |= stRtcBad;
    if( (rt.wDay&0xC0) != 0 ) flagRtcBad |= stRtcBad;
    if( (rt.wYear) < 12 ) flagRtcBad |= stRtcBad;
//    if( (rt.wMonth&0xE0) != 0 ) flagRtcBad |= BitRtcBad;
    
    rt.wSecond &= 0x7F;
    rt.wMinute &= 0x7F;
    rt.wHour &= 0x7F;
    rt.wDay &= 0x3F;
    rt.wMonth &= 0x1F;
    // si.Time = rt;

    //RtcWriteRegister( arControl, ctRate1Hz + ctTempConvert );
    //RtcReadRegister( arControl ); 
    return (si.Status |= flagRtcBad);
}

// For RCC BDCR
#define bitRtcSel       0x00000300
#define bitRtcHSE       0x00000300
#define bitRtcLSE       0x00000100
#define bitLSEON        0x00000001
#define bitLSERDY       0x00000002
#define bitRtcEna       0x00008000

// For RCC CFGR
#define clrRtcPre       0xFFE0FFFF

// For ISR register

// Enter into Init mode
#define bitRtcInitMode		0x0080
// Rtc is in Init mode
#define bitRtcInInitMode	0x0040
// Rtc was initialized
#define bitRtcInitState		0x0010

// Rtc wake up event
#define bitRtcWakeUp		0x0400
// Rtc wakeup timer values can be changed
#define bitRtcWakeUpAllow	0x0004

// For CR register

// WakeUp interrupt enable
#define bitRtcWakeUpIrqEna	0x4000
// WakeUp timer enable
#define bitRtcWakeUpTimerEna	0x0400
// WakeUp Clock select
#define bitRtcWakeUp1Hz		0x0004


#define INIT_RTC_TIMEOUT         (300000)
#define RtcExitInitMode()       RTC->ISR &= (~bitRtcInitMode)

void RTC_WKUP_IRQHandler( void )
{
    EXTI->PR |= (1<<22);                         //Очистка
    RTC->ISR &= ~bitRtcWakeUp;
    if( ExtRtc ) return;
    RtcHandler();
}

int RtcEnterInitMode( void )
{
//int counter = 0;
int InitStatus = 0;

    MsCounter = 0;
  /* Check if the Initialization mode is set */
    if( (RTC->ISR & bitRtcInInitMode) == 0 ) {
        /* Set the Initialization mode */
        RTC->ISR |= bitRtcInitMode;
        
        /* Wait till RTC is in INIT state and if Time out is reached exit */
        do {
            InitStatus = RTC->ISR & bitRtcInInitMode;
        } while( (MsCounter < 200) && (InitStatus == 0) );
    
        if( (RTC->ISR & bitRtcInInitMode) != 0 ) return 1;
    }
    else return 1;  
    
    return 0;  
}

static int Divs = 255, Diva = 127; 

void RtcInHSE( void )
{
    RCC->CFGR &= clrRtcPre;
    RCC->CFGR |= (8<<16);
    Diva = 127;
    Divs = ((F_Crystal/8)/(Diva+1)) - 1;
    if( RTC->PRER != (0x7F0000 + Divs) ) {
        RCC->BDCR |=  RCC_BDCR_BDRST;
        RCC->BDCR &= ~RCC_BDCR_BDRST;
    }
    RCC->BDCR &= (~bitRtcSel);
    RCC->BDCR &= (~bitRtcEna);
    RCC->BDCR |= bitRtcHSE;
    RCC->BDCR |= bitRtcEna;    
}

//------------------------------------------------------------------------------

int RtcInLSE( void )
{
int counter;
    RCC->BDCR |= bitLSEON;
    counter = 0;
    while( 1 ) {
        if( (RCC->BDCR & bitLSERDY ) != 0 ) break; 
	if( counter++ > 200000 ) return 0;
    }
    RCC->BDCR &= (~bitRtcSel);
    RCC->BDCR &= (~bitRtcEna);
    RCC->BDCR |= bitRtcLSE;
    Diva = 127;
    Divs = 255;
    RCC->BDCR |= bitRtcEna;    
    return 1;
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

int InitIntRtc( void )
{
int status = 0, extirq = 22;
int tim, dat, tmp;

    // Включим тактирование PWR
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
   
    // Разрешим доступ к управляющим регистрам энергонезависимого домена
    PWR->CR |= PWR_CR_DBP;
   
    if( RtcInLSE() == 0 ) {
        //RtcInHSE();
	ExtRtc = 1;
	return 0;
    }
    
    /* Disable the write protection for RTC registers */
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;

    EXTI->PR |= (1<<22);                         //Очистка
    RTC->ISR &= ~bitRtcWakeUp;

    RTC->ISR |= bitRtcWakeUpAllow;
    RTC->WUTR = 0;
    RTC->ISR &= (~bitRtcWakeUpAllow);
    RTC->CR |= bitRtcWakeUp1Hz;
    RTC->CR |= bitRtcWakeUpIrqEna;
    RTC->CR |= bitRtcWakeUpTimerEna;	
   
    // Если часы запущены, делать тут нечего.
    if(RTC->ISR & RTC_ISR_INITS) goto ret;

    status = RtcEnterInitMode();    
    if( status ) {  
	
        RTC->WUTR = 0;
        RTC->CR |= bitRtcWakeUp1Hz;
        RTC->CR |= bitRtcWakeUpIrqEna;
        RTC->CR |= bitRtcWakeUpTimerEna;	

	/* Clear RTC CR FMT Bit - it's 24 hour format */
        RTC->CR &= (~RTC_CR_FMT);
    /* Configure the RTC PRER */
        RTC->PRER = Divs;
        RTC->PRER |= (Diva << 16);
	
        tim = 0x00;	// sec
        tmp = 0x02;	// Minute;
        tim |= (tmp<<8);
        tmp = 0x11;	// Hour;
        tim |= (tmp<<16);
    
        dat = 0x31;	// Day;
        tmp = 0x10;	// Month;
        dat |= (tmp<<8);
        tmp = 0x63;	// Year;
        dat |= (tmp<<16);
        /* Set the RTC_TR register */
        RTC->TR = tim;
        /* Set the RTC_TR register */
        RTC->DR = dat;
    }
    
    /* Exit Initialization mode */
    RtcExitInitMode();
ret:
    /* Enable the write protection for RTC registers */
    RTC->WPR = 0xFF; 
    
    extirq = 22;
    EXTI->IMR &= ~(1<<extirq);                          
    EXTI->IMR |= (1<<extirq);                           
    //EXTI->EMR &= ~(1<<extirq);                          
    //EXTI->EMR |= (1<<extirq);                           

    EXTI->RTSR &= ~(1<<extirq);
    EXTI->FTSR &= ~(1<<extirq);
    EXTI->RTSR |= (1<<extirq);

    SetIrq( RTC_WKUP_IRQn, 10, 0 );
    NVIC_EnableIRQ( RTC_WKUP_IRQn );  
    
    return status;  
}

int SetIntRtc( TRtcTime *tt )
{
int tim, dat, tmp;
int status = 0;

    tim = tt->wSecond;
    tmp = tt->wMinute;
    tim |= (tmp<<8);
    tmp = tt->wHour;
    tim |= (tmp<<16);
    
    dat = tt->wDay;
    tmp = tt->wMonth;
    dat |= (tmp<<8);
    tmp = tt->wYear;
    dat |= (tmp<<16);
    
    /* Disable the write protection for RTC registers */
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    status = RtcEnterInitMode();    
    if( status ) {
        /* Set the RTC_TR register */
        RTC->TR = tim;
        /* Set the RTC_TR register */
        RTC->DR = dat;

        /* Exit Initialization mode */
        RtcExitInitMode();
    }

    /* Enable the write protection for RTC registers */
    RTC->WPR = 0xFF; 
    return status;  
}

int GetIntRtc( void )
{
int tim, dat;
    tim = RTC->TR;
    dat = RTC->DR;
    rt.wSecond = tim & 0xFF;
    rt.wMinute = (tim>>8) & 0xFF;
    rt.wHour = (tim>>16) & 0xFF;
    rt.wDay = (dat) & 0xFF;
    rt.wMonth = (dat>>8) & 0xFF;
    rt.wYear = (dat>>16) & 0xFF;  
    return 1;
}

int InitRtc( void)
{
    //if( ExtRtc ) InitExtRtc();
    //if( ExtRtc == 0 ) InitIntRtc();
    InitExtRtc();
    InitIntRtc();
    return 1;
}

int SetRtc( TRtcTime *tt )
{
    if( ExtRtc ) return SetExtRtc( tt );
    else return SetIntRtc( tt );
    return 1;
}

int GetRtc( void )
{
int ret;
    if( ExtRtc ) ret = GetExtRtc();
    else ret = GetIntRtc();
    memcpy( &si.Time, &rt, 8 );
    return ret;
}
