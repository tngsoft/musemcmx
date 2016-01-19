/******************************************************************************/
/*            Timer Functions				                      */
/******************************************************************************/
#include "..\\inc\\stm32F4xx.h"

#pragma pack(1)

#include "xxProto.h"
#include "xxcustom.h"
#include "xxSpi.h"

/*
       ===================================================================      
              TIM Driver: how to use it in Timing(Time base) Mode
       =================================================================== 
       To use the Timer in Timing(Time base) mode, the following steps are mandatory:
       
       1. Enable TIM clock using RCC_APBxPeriphClockCmd(RCC_APBxPeriph_TIMx, ENABLE) function
                    
       2. Fill the TIM_TimeBaseInitStruct with the desired parameters.
       
       3. Call TIM_TimeBaseInit(TIMx, &TIM_TimeBaseInitStruct) to configure the Time Base unit
          with the corresponding configuration
          
       4. Enable the NVIC if you need to generate the update interrupt. 
          
       5. Enable the corresponding interrupt using the function TIM_ITConfig(TIMx, TIM_IT_Update) 
       
       6. Call the TIM_Cmd(ENABLE) function to enable the TIM counter.
*/             
/*
 ===============================================================================
               ##### Input Capture management functions #####
 ===============================================================================
   
          *** TIM Driver: how to use it in Input Capture Mode ***
 ===============================================================================
    [..] To use the Timer in Input Capture mode, the following steps are mandatory:
         (#) Enable TIM clock using RCC_APBxPeriphClockCmd(RCC_APBxPeriph_TIMx, ENABLE) 
             function.
         (#) Configure the TIM pins by configuring the corresponding GPIO pins.
         (#) Configure the Time base unit as described in the first part of this 
             driver, if needed, else the Timer will run with the default configuration:
             (++) Autoreload value = 0xFFFF.
             (++) Prescaler value = 0x0000.
             (++) Counter mode = Up counting.
             (++) Clock Division = TIM_CKD_DIV1.
         (#) Fill the TIM_ICInitStruct with the desired parameters including:
             (++) TIM Channel: TIM_Channel.
             (++) TIM Input Capture polarity: TIM_ICPolarity.
             (++) TIM Input Capture selection: TIM_ICSelection.
             (++) TIM Input Capture Prescaler: TIM_ICPrescaler.
             (++) TIM Input CApture filter value: TIM_ICFilter.
         (#) Call TIM_ICInit(TIMx, &TIM_ICInitStruct) to configure the desired 
             channel with the corresponding configuration and to measure only 
             frequency or duty cycle of the input signal,or, Call 
             TIM_PWMIConfig(TIMx, &TIM_ICInitStruct) to configure the desired 
             channels with the corresponding configuration and to measure the 
             frequency and the duty cycle of the input signal.
         (#) Enable the NVIC or the DMA to read the measured frequency.
         (#) Enable the corresponding interrupt (or DMA request) to read 
             the Captured value, using the function TIM_ITConfig(TIMx, TIM_IT_CCx)
             (or TIM_DMA_Cmd(TIMx, TIM_DMA_CCx)).
         (#) Call the TIM_Cmd(ENABLE) function to enable the TIM counter.
         (#) Use TIM_GetCapturex(TIMx); to read the captured value.
    [..]
        (@) All other functions can be used separately to modify, if needed,
            a specific feature of the Timer. 

*/

void InitTimers( void ) 
{
    RCC->APB1ENR |= RCC_APB1Periph_TIM5;
    EnaTimer( RCC_APB1Periph_TIM5 );
    StopTimer( TIM5 );
    SetTimerCounterMode( TIM5, tmReload | tmUpdateOver | 0x10 );
    TIM5->PSC = (SystemCoreClock/10000)/4-1;
    // For 40 Hz - 10000 Hz/250 = 40
    TIM5->ARR = 10000/20 - 1;
    ItTimerConfig( TIM5, 1 );
    SetIrq( TIM5_IRQn, 1, 1 );
    NVIC_EnableIRQ( TIM5_IRQn );
    StartTimer( TIM5 );
    return;
}


void InitPorts(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  SYSCFG->CMPCR |= SYSCFG_CMPCR_CMP_PD;
  while(!(SYSCFG->CMPCR & SYSCFG_CMPCR_READY));  

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;         // разрешение тактирования порта A
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;         // разрешение тактирования порта B
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;         // разрешение тактирования порта C
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;         // разрешение тактирования порта D
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;         // разрешение тактирования порта E
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;        //разрешение тактирования контроллера конфигурации системы

    // Power Pin
    ConfigPin( PortPower, PinPower, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortPower, PinPowerInv, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ClrBit( PortPower, PinPower );    
    SetBit( PortPower, PinPowerInv );    

    // Direct RS-485
    ConfigPin( PortDir485, PinDirect485, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ClrBit( PortDir485, PinDirect485 );    

    // Test points & Leds
    ConfigPin( PortTest, TestPoint1, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortTest, TestPoint2, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortTest, TestPoint3, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortTest, TestPoint4, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortTest, TestPoint5, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortTest, TestPoint6, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortTest, TestPoint7, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortTest, TestPoint8, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ClrBit( PortTest, TestPoint1 );    
    ClrBit( PortTest, TestPoint2 );    
    ClrBit( PortTest, TestPoint3 );    
    ClrBit( PortTest, TestPoint4 );    
    ClrBit( PortTest, TestPoint5 );    
    ClrBit( PortTest, TestPoint6 );    
    ClrBit( PortTest, TestPoint7 );    
    ClrBit( PortTest, TestPoint8 );    

    // CS for RTC
    ConfigPin( PortRtc, CSRtc, mdPinOut, tpPushPull, ppPullUp, sp50M );
    SetBit( PortRtc, CSRtc );
    // CS for axelerometers
    ConfigPin( PortAxel, CSAxel, mdPinOut, tpPushPull, ppPullUp, sp50M );
    SetBit( PortAxel, CSAxel );    
    // CS for EEPROM
    ConfigPin( PortEprom, CSEprom, mdPinOut, tpPushPull, ppPullUp, sp50M );
    SetBit( PortEprom, CSEprom );

    // SpiDataOut
    ConfigPin( CommonSpiPort, SpiDataOut, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ClrBit( CommonSpiPort, SpiDataOut );
    SetBit( CommonSpiPort, SpiDataOut );
    ClrBit( CommonSpiPort, SpiDataOut );
    // SpiClkPin
    ConfigPin( CommonSpiPort, SpiDataClk, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ClrBit( CommonSpiPort, SpiDataClk );
    SetBit( CommonSpiPort, SpiDataClk );
    ClrBit( CommonSpiPort, SpiDataClk );
    // SpiDataIn
    ConfigPin( CommonSpiPort, SpiDataIn, mdPinIn, tpPushPull, ppPullUp, sp50M );

    // SDIO
    RCC->APB2ENR |= RCC_APB2Periph_SDIO;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;         // разрешение тактирования порта C
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;         // разрешение тактирования порта D
    
    PortSD->AFR[1] |= 0x000CCCCC;                  //PC8-11 D0-D3  PC12 - Clk
    PortSDCmd->AFR[0] |= 0x00000C00;                  //PD2 - SDIO CMD  

    ConfigPin( PortSD, PinSDIOData0, mdPinAf, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortSD, PinSDIOData1, mdPinAf, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortSD, PinSDIOData2, mdPinAf, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortSD, PinSDIOData3, mdPinAf, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortSD, PinSDIOClk, mdPinAf, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortSDCmd, PinSDIOCmd, mdPinAf, tpPushPull, ppPullUp, sp50M );
    
    
    // ADC
    /* Enable The HSI (16Mhz) */
    RCC->CR |= 1;
    //count = 20000;
    while( (RCC->CR & 2) == 0);
    
//  RCC->APB2ENR|=RCC_APB2ENR_ADC1EN;
//  ADC->CCR|=ADC_CCR_ADCPRE;
    RCC->APB2ENR |= RCC_APB2Periph_ADC1;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;         // разрешение тактирования порта C
    ConfigPin( PortADC, PinADCV, mdPinAn, tpPushPull, ppPullNone, sp2M );

    /*---------------------------- ADCx CR1 Configuration -----------------*/
    ADC1->CR1 = 0x0800;
    ADC1->CR2 = 0;
    ADC1->SMPR2 = 6;
    ADC1->SQR3 = 10;
    // Delay
    ADC1->CR2 = 0x52; 
    ADC1->CR2 |= 0x01;
    while ( (ADC1->SR & 0x100) != 0 ) {        
        ADC1->SQR3 = 10;
       //ADC1->CR2 |= ADC_CR2_SWSTART;
    }
}

/*
        (#) Enable peripheral clock using
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE) function for
            USART1 or using RCC_APB1PeriphClockCmd(RCC_APB1Periph_USARTx, ENABLE)
            function for USART2 and USART3.
        (#) According to the USART mode, enable the GPIO clocks using
            RCC_AHBPeriphClockCmd() function. (The I/O can be TX, RX, CTS,
            or and SCLK).
        (#) Peripheral's alternate function:
            (++) Connect the pin to the desired peripherals' Alternate
                 Function (AF) using GPIO_PinAFConfig() function.
            (++) Configure the desired pin in alternate function by:
                 GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF.
            (++) Select the type, pull-up/pull-down and output speed via
                 GPIO_PuPd, GPIO_OType and GPIO_Speed members.
            (++) Call GPIO_Init() function.
        (#) Program the Baud Rate, Word Length , Stop Bit, Parity, Hardware
               flow control and Mode(Receiver/Transmitter) using the SPI_Init()
               function.
        (#) For synchronous mode, enable the clock and program the polarity,
            phase and last bit using the USART_ClockInit() function.
        (#) Enable the NVIC and the corresponding interrupt using the function
            USART_ITConfig() if you need to use interrupt mode.
        (#) When using the DMA mode.
            (++) Configure the DMA using DMA_Init() function.
            (++) Active the needed channel Request using USART_DMACmd() function.
        (#) Enable the USART using the USART_Cmd() function.
        (#) Enable the DMA using the DMA_Cmd() function, when using DMA mode.
    [..]
*/

#include <math.h>

int GetDivUSART( int freq, USART_TypeDef* USARTx ) 
{
float fraq;
int Div;
int div;
    // Get APB1 prescaler
    if( USARTx == USART1 ) div = ((RCC->CFGR>>13)&7) - 3;
    else div = ((RCC->CFGR>>10)&7) - 3;
    if( div < 0 ) div = 0;
    div = 1<<div;

    Div = (SystemCoreClock/div)/(freq*16);
    fraq = (SystemCoreClock/div)/(freq*16.0);
    fraq -= Div;
    fraq *= 16;
    Div = (Div<<4) + (int)(round( fraq ));
    return Div;
}

void InitUart( void ) 
{    
    RCC->APB2ENR |= RCC_APB2Periph_USART1;
    RCC->APB1ENR |= RCC_APB1Periph_USART2;
    
    // UART1 UART2
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;             // разрешение тактирования порта A
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;             // разрешение тактирования порта B
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;             // разрешение тактирования порта C
    GPIOD->AFR[0] |= 0x07700000;                     // PD5 = USART2 TX; PD6 = USART2 RX  
    GPIOA->AFR[1] |= 0x00000770;                     // PA9 = USART1 TX; PA10 = USART1 RX  

    ConfigPin( PortUart2, PinTX2, mdPinAf, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortUart2, PinRX2, mdPinAf, tpPushPull, ppPullUp, sp50M );

    ConfigPin( PortUart1, PinTX1, mdPinAf, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortUart1, PinRX1, mdPinAf, tpPushPull, ppPullUp, sp50M );

    // For RS-485
    USART2->BRR = GetDivUSART( 19200, USART2 );
    USART2->CR1 = (USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE ); // Включение USART2 и разрешение прерываний по приему.
    NVIC_ClearPendingIRQ( USART2_IRQn );
    NVIC_EnableIRQ( USART2_IRQn );  

    USART1->BRR = GetDivUSART( 9600, USART1 );
    USART1->CR1 = (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE  | USART_CR1_M ); // Включение USART1 и разрешение прерываний по приему.
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    NVIC_ClearPendingIRQ( USART1_IRQn );
    NVIC_EnableIRQ( USART1_IRQn );  

// SPI 1 
//#define PortSpi1Clk	GPIOA
//#dfine PortSpi1MISO	GPIOA
//#define PortSpi1MOSI	GPIOB
//#define Spi1_CLK	5
//#define Spi1_MISO	6
//#define Spi1_MOSI	5

    // Spi1 - Common
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;         // разрешение тактирования порта A
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;         // разрешение тактирования порта B
    RCC->APB2ENR |= RCC_APB2Periph_SPI1;	// Spi1
    GPIOA->AFR[0] |= 0x05500000;              
    GPIOB->AFR[0] |= 0x00500000;              
    ConfigPin( PortSpi1Clk, Spi1_CLK, mdPinAf, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortSpi1MISO, Spi1_MISO, mdPinAf, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortSpi1MOSI, Spi1_MOSI, mdPinAf, tpPushPull, ppPullUp, sp50M );
   
    // Ext ADC 
    SPI1->CR1 = spPhase0 | spPol0 | spMaster | spDiv32 | spMSB | spData16;
    SPI1->CR2 = spSSEnable;
    SPI1->CR1 |= spEnable;

// Spi 2
//#define PortSpi2Clk	GPIOB
//#define PortSpi2MISO	GPIOC
//#define PortSpi2MOSI	GPIOC
//#define Spi2_CLK	10
//#define Spi2_MISO	2
//#define Spi2_MOSI	3

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;         // разрешение тактирования порта C
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;         // разрешение тактирования порта B
    
#ifdef PROGRAMM_SPI2
    ConfigPin( PortSpi2Clk, Spi2_CLK, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortSpi2MISO, Spi2_MISO, mdPinIn, tpPushPull, ppPullNone, sp50M );
    ConfigPin( PortSpi2MOSI, Spi2_MOSI, mdPinOut, tpPushPull, ppPullUp, sp50M );
    ClrBit( PortSpi2MOSI, Spi2_MOSI );
    ClrBit( PortSpi2Clk, Spi2_CLK );
#else
    // Spi2 - Axel EEPROM RTC
    RCC->APB1ENR |= RCC_APB1Periph_SPI2;	// Spi2
    GPIOB->AFR[1] |= 0x00000500;              
    GPIOC->AFR[0] |= 0x00005500;              
    ConfigPin( PortSpi2Clk, Spi2_CLK, mdPinAf, tpPushPull, ppPullNone, sp50M );
    ConfigPin( PortSpi2MISO, Spi2_MISO, mdPinAf, tpPushPull, ppPullNone, sp50M );
    ConfigPin( PortSpi2MOSI, Spi2_MOSI, mdPinAf, tpPushPull, ppPullNone, sp50M );
   
    // Spi2 Eprom Axel
    SPI2->CR1 = spPhase0 | spPol0 | spMaster | spDiv32 | spMSB | spData8;
    SPI2->CR2 = spSSEnable;
    SPI2->CR1 |= spEnable;
#endif
}  

void InitIntr( void )
{
  // PE6 - Int RTC
  // PA3 - Int AXEL

    ConfigPin( PortIrqRtc, PinIrqRtc, mdPinIn, tpPushPull, ppPullUp, sp50M );
    ConfigPin( PortIrqAxel, PinIrqAxel, mdPinIn, tpPushPull, ppPullUp, sp50M );

    SetPortExtIrq( PortIrqRtc, PinIrqRtc, edFail );
    //SetPortExtIrq( PortIrqAxel, PinIrqAxel, edFail );

    NVIC_EnableIRQ( IrqRtc );
    //NVIC_EnableIRQ( IrqAxel );
}
