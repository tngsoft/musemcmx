/******************************************************************************/
/*            SysTick Functions				                      */
/******************************************************************************/
#include "..\\inc\\stm32F4xx.h"

#pragma pack(1)

#include <stdbool.h>
#include "xxsystem.h"

void SetSysTick( int Value, int Div ) 
{
    StopSysTick();
    SysTick->CTRL &= SysTick_CLKSource_HCLK_Div8;
    if( Div == 1 ) SysTick->CTRL |= SysTick_CLKSource_HCLK;
//    SysTick_LOAD = Value;
    StartSysTick();
    EnaSysTick();
}

//******************************************************************************

void SetIrq( int IRQChannel, int Priority, int SubPriority ) 
{
int tmppriority=0, tmppre=0, tmpsub=0x0F;

    /* Compute the Corresponding IRQ Priority --------------------------------*/    
    tmppriority = (0x0700 - ((SCB->AIRCR) & 0x0700))>> 0x08;
    tmppre = (0x4 - tmppriority);
    tmpsub = tmpsub >> tmppriority;

    tmppriority = Priority << tmppre;
    tmppriority |=  SubPriority & tmpsub;
    tmppriority = tmppriority << 0x04;
        
    NVIC->IP[ IRQChannel ] = tmppriority;
    
    /* Enable the Selected IRQ Channels --------------------------------------*/
    NVIC->ISER[ IRQChannel >> 0x05 ] = 0x01 << (IRQChannel & 0x1F);
}

//******************************************************************************

void ConfigPin( GPIO_TypeDef* GPIOx, int pin, int mode, 
	                             int type, int pull, 
				     int speed )
{
int clr;
    
    clr = ~(0x1<<pin);
    GPIOx->OTYPER &= clr;
    GPIOx->OTYPER |= ( type<<pin );        
    
    pin += pin;
    
    clr = ~(0x3<<pin);
    GPIOx->MODER &= clr;
    GPIOx->MODER |= (mode<<pin);     
    
    GPIOx->PUPDR &= clr;
    GPIOx->PUPDR |= (pull<<pin);     
    
    GPIOx->OSPEEDR &= clr;
    GPIOx->OSPEEDR |= (speed<<pin);     
}

void SetAF( GPIO_TypeDef* GPIOx, int pin, int af )
{
int n = 0;
if( pin >= 8 ) { pin -= 8; n = 1; }
    pin <<= 2;
    GPIOx->AFR[n] |= (af<<pin);	 
}

void SetPortExtIrq( GPIO_TypeDef* GPIOx, short int extirq, short int edge )
{
short int tetr, nom = extirq >> 2, clr, idx;

    tetr = (extirq%4);
    clr = ~(0x000F << tetr);
    idx = 0; 
    if( GPIOx == GPIOB ) idx = 1; 
    if( GPIOx == GPIOC ) idx = 2; 
    if( GPIOx == GPIOD ) idx = 3; 
    if( GPIOx == GPIOE ) idx = 4; 
    if( GPIOx == GPIOH ) idx = 5; 
    clr = idx<<(tetr<<2);
    SYSCFG->EXTICR[nom] &= (~clr); 
    SYSCFG->EXTICR[nom] |= clr;
    
    EXTI->IMR &= ~(1<<extirq);                          
    EXTI->IMR |= (1<<extirq);                           

    EXTI->RTSR &= ~(1<<extirq);
    EXTI->FTSR &= ~(1<<extirq);
    if( edge == edRise ) EXTI->RTSR |= (1<<extirq);
    else EXTI->FTSR |= (1<<extirq);
}
