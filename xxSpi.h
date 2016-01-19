#ifndef xxSpiH
#define xxSpiH

//#define PROGRAMM_SPI2

#ifdef PROGRAMM_SPI2
#define SpiEprom	NULL
#define SpiRtc		NULL
#define SpiAxel		NULL
#else
#define SpiEprom	SPI2
#define SpiRtc		SPI2
#define SpiAxel		SPI2
#endif

extern int SPI_CPOL, SPI_CPHA;

// SPI
char SendSpiByte( SPI_TypeDef* SPIx, char out );
int SendSpi( SPI_TypeDef* SPIx, char *out, char *in, int bytes );
void SetSpiPolarity( SPI_TypeDef* SPIx, int pol, int pha );

#endif