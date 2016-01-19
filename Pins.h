#pragma pack(1)

//ETH ports
#define ETH_RX_PORT	GPIOC
#define ETH_TX_PORT	GPIOB
#define ETH_CRS_PORT	GPIOA
#define ETH_MDIO_PORT	GPIOA
#define ETH_MDC_PORT	GPIOC
#define ETH_REFCLK_PORT	GPIOA
#define ETH_RST_PORT	GPIOE

//ETH pins
#define ETH_RXD0	4
#define ETH_RXD1	5
#define ETH_TXD0	12
#define ETH_TXD1	13
#define ETH_TXEN	11
#define ETH_CRS		7
#define ETH_MDIO	2
#define ETH_MDC		1
#define ETH_REFCLK	1
#define ETH_RST		2

// Power Pins
#define PortPower	GPIOD
#define PinPower	0
#define PinPowerInv	1

#define PortTest	GPIOD
#define TestPoint1	8
#define TestPoint2	9
#define TestPoint3	10
#define TestPoint4	11
#define TestPoint5	12
#define TestPoint6	13
#define TestPoint7	14
#define TestPoint8	15

// RS-485
#define PortDir485	GPIOA
#define PinDirect485	8

// Reset Net
#define PortNet		GPIOE
#define ResetNet	2

// ADC
#define PortADC		GPIOC
#define PinADCV		0

// USART
#define PortUart1	GPIOA
#define PinTX1	9
#define PinRX1	10

#define PortUart2	GPIOD
#define PinTX2	5
#define PinRX2	6

// SPI 1
#define PortSpi1Clk	GPIOA
#define PortSpi1MISO	GPIOA
#define PortSpi1MOSI	GPIOB
#define Spi1_CLK	5
#define Spi1_MISO	6
#define Spi1_MOSI	5

// Spi 2
#define PortSpi2Clk	GPIOB
#define PortSpi2MISO	GPIOC
#define PortSpi2MOSI	GPIOC
#define Spi2_CLK	10
#define Spi2_MISO	2
#define Spi2_MOSI	3

// SDIO
#define PortSD		GPIOC
#define PinSDIOData0	8
#define PinSDIOData1	9
#define PinSDIOData2	10
#define PinSDIOData3	11
#define PinSDIOClk	12
// SDIO CMD in PortD
#define PortSDCmd	GPIOD
#define PinSDIOCmd	2

// Common serial
#define CommonSpiPort	GPIOE
#define SpiDataClk	0
#define SpiDataOut	3
#define SpiDataIn	1

// CS pins
// RTC
#define PortRtc		GPIOB
#define CSRtc		6

// Axel
#define PortAxel	GPIOA
#define CSAxel		4

// Eprom
#define PortEprom	GPIOB
#define CSEprom		7

// Interrupts
#define PortIrqRtc	GPIOE
#define PortIrqAxel	GPIOA

#define PinIrqRtc	6
#define PinIrqAxel	3

#define IrqRtc	EXTI9_5_IRQn
#define IrqAxel	EXTI3_IRQn

