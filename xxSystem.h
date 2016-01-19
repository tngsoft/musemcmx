#ifndef XXSYSTEMH
#define XXSYSTEMH

#include <locale.h>

#define	AF_TIM1		0x01
#define	AF_TIM2		0x01

#define	AF_TIM3		0x02
#define	AF_TIM4		0x02
#define	AF_TIM5		0x02

#define	AF_TIM8		0x03
#define	AF_TIM9		0x03
#define	AF_TIM10	0x03
#define	AF_TIM11	0x03

#define	AF_I2C1		0x04
#define	AF_I2C2		0x04
#define	AF_I2C3		0x04

#define	AF_SPI1		0x05		
#define	AF_SPI2		0x05		
#define	AF_SPI3		0x06		

#define	AF_USART1	0x07
#define	AF_USART2	0x07
#define	AF_USART3	0x07

#define	AF_USART4	0x08
#define	AF_USART5	0x08
#define	AF_USART6	0x08

#define	AF_TIM12	0x09
#define	AF_TIM13	0x09
#define	AF_TIM14	0x09

#define	AF_CAN1		0x09
#define	AF_CAN2		0x09

#define	AF_ETH		0x0B
#define	AF_FLASH	0x0C
#define	AF_SDIO		0x0C

#define ResetIrq( IRQChannel ) NVIC->ICER[ IRQChannel >> 0x05 ] = 0x01 << (IRQChannel & 0x1F)

#define StartSysTick() SysTick->CTRL |= 0x0001
#define StopSysTick()  SysTick->CTRL &= 0xFFFE
#define EnaSysTick()   SysTick->CTRL |= 0x0002
#define GetSysTick()   SysTick->STK_VALUE

//****************************************************************************************************
// TIMERS
//****************************************************************************************************
// CR1 register
#define crEnable	0x0001
#define crOneShot	0x0008
#define crDirDown	0x0010
#define crAutoReload	0x0080
#define crDiv1		0x0000
#define crDiv2		0x0100
#define crDiv4		0x0200

// CR2 register

// CCMR1 register
#define ccmOutput	0x0000

#define ccmToggle1	0x0030
#define ccmPreload1	0x0008
#define ccmToggle2	0x3000
#define ccmPreload2	0x0800

#define ccmInput1	0x0001
#define ccmInput2	0x0100
// CCMR2
#define ccmInput3	0x0001
#define ccmInput4	0x0100

#define ccmDiv1_1	0x0000
#define ccmDiv1_2	0x0004
#define ccmDiv1_4	0x0008
#define ccmDiv1_8	0x000C

#define ccmDiv2_1	0x0000
#define ccmDiv2_2	0x0400
#define ccmDiv2_4	0x0800
#define ccmDiv2_8	0x0C00

// CCMR2 register
#define ccmToggle3	0x0030
#define ccmPreload3	0x0008
#define ccmToggle4	0x3000
#define ccmPreload4	0x0800

#define ccmInput3	0x0001
#define ccmInput4	0x0100
#define ccmDiv3_1	0x0000
#define ccmDiv3_2	0x0004
#define ccmDiv3_4	0x0008
#define ccmDiv3_8	0x000C

#define ccmDiv4_1	0x0000
#define ccmDiv4_2	0x0400
#define ccmDiv4_4	0x0800
#define ccmDiv4_8	0x0C00

// Tim CCER register capture/compare enable
#define cceEnable1	0x0001
#define cceActiveLow1	0x0002
#define ccePolarRise1	0x0000
#define ccePolarFail1	0x0002
#define ccePolarBoth1	0x000A

#define cceEnable2	0x0010
#define cceActiveLow2	0x0020
#define ccePolarRise2	0x0000
#define ccePolarFail2	0x0020
#define ccePolarBoth2	0x00A0

#define cceEnable3	0x0100
#define cceActiveLow3	0x0200
#define ccePolarRise3	0x0000
#define ccePolarFail3	0x0200
#define ccePolarBoth3	0x0A00

#define cceEnable4	0x1000
#define cceActiveLow4	0x2000
#define ccePolarRise4	0x0000
#define ccePolarFail4	0x2000
#define ccePolarBoth4	0xA000

#define tmReload	0x80
#define tmDownCount	0x10
#define tmDisUpdate	0x02
#define tmOneShot	0x08
#define tmUpdateOver	0x04

// Interrupt bits TIMx_DIER
#define tiTriggerEnable	0x40
#define tiCh4Enable	0x10
#define tiCh3Enable	0x08
#define tiCh2Enable	0x04
#define tiCh1Enable	0x02
#define tiUpdateEnable	0x01

#define EnaTimer( x ) RCC->APB1ENR |= x
#define DisTimer( x ) RCC->APB1ENR &= ~x

#define StartTimer( x )     x->CR1 |= 1
#define StopTimer( x )      x->CR1 &= 0xFFFE
#define ItTimerConfig(x,i)  x->DIER |= (i)
  
#define SetTimerCounterMode( x, m )    x->CR1 &= (~(TIM_CR1_DIR | TIM_CR1_CMS)); x->CR1 |= m
#define SetTimerReloadValue( x, r )    x->ARR = r
#define SetTimerPreScaler( x, r )      x->PSC = r-1; x->EGR = 0

//****************************************************************************************************
// GPIO
//****************************************************************************************************
   
#define mdPinIn	 0
#define mdPinOut 1
#define mdPinAf	 2
#define mdPinAn	 3

#define tpPushPull	0
#define tpOpenDrain	1

#define ppPullNone	0
#define ppPullUp	1
#define ppPullDown	2

#define sp2M	0
#define sp25M	1
#define sp50M	2
#define sp100M	3

#define GetInBit( x, b ) ((x->IDR & (1<<b))>>b) 
#define GetInBitEx( x, b ) (x->IDR & (1<<b)) 

#define GetOutBit( x, b ) ((x->ODR & (1<<b))>>b) 
#define GetOutBitEx( x, b ) (x->ODR & (1<<b)) 

#define SetBit( x, b )  (x->BSRRL |= (1<<b)) 
#define ClrBit( x, b )  (x->BSRRH |= (1<<b)) 

#define ToggleBit( x, b ) x->ODR ^= (1<<b)

#define edFail	0
#define edRise	1

void SetSysTick( int Value, int Div ) ;
void SetIrq( int IRQChannel, int Priority, int SubPriority ) ;

void ConfigPin( GPIO_TypeDef* GPIOx, int pin, int mode, 
	                             int type, int pull, 
				     int speed );

void SetAF( GPIO_TypeDef* GPIOx, int pin, int af );

void SetPortExtIrq( GPIO_TypeDef* GPIOx, short int extirq, short int edge );

// USART
#define usPhase0	0x0000
#define usPhase1	0x0200

#define usPol0		0
#define usPol1		0x0400

#define usLastBit	0x0100

#define usEnaClk	0x0800

// SPI

// SPI_CR1
#define spPhase0	0
#define spPhase1	1

#define spPol0		0
#define spPol1		0x0002

#define spSlave		0
#define spMaster	0x0004

#define spDiv2		0
#define spDiv4		0x0008
#define spDiv8		0x0010
#define spDiv16		0x0018
#define spDiv32		0x0020
#define spDiv64		0x0028
#define spDiv128	0x0030
#define spDiv256	0x0038

#define spEnable	0x0040
#define spBusy		0x0080

#define spLSB		0x0080
#define spMSB		0x0000

#define spSoftSlave	0x0200
#define spSoftSS	0x0100

#define spData8		0x0000
#define spData16	0x0800

#define spCRCEnable	0x2000

#define spBiDiMode	0x8000

// SPI_CR2

#define spRxDma		0x0001
#define spTxDma		0x0002
#define spSSEnable	0x0004
#define spErrIrqEnable	0x0020
#define spRxIrqEnable	0x0040
#define spTxIrqEnable	0x0080

// SPI_SR   status reg


#define spRxReady	0x0001
#define spTxReady	0x0002

#define spErrUnder	0x0008
#define spErrCRC	0x0010
#define spErrMode	0x0020
#define spErrOver	0x0040
#define spBusy		0x0080
#define spErrFrame	0x0100

#define Spi1TxReady	((SPI1->SR & spTxReady)!=0)
#define Spi1RxReady	((SPI1->SR & spRxReady)!=0)
#define Spi1Busy	((SPI1->SR & spBusy)!=0)

#define Spi2TxReady	((SPI2->SR & spTxReady)!=0)
#define Spi2RxReady	((SPI2->SR & spRxReady)!=0)
#define Spi2Busy	((SPI2->SR & spBusy)!=0)

#define Spi3TxReady	((SPI3->SR & spTxReady)!=0)
#define Spi3RxReady	((SPI3->SR & spRxReady)!=0)

#endif