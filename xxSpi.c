#include "..\\inc\\stm32F4xx.h"

#pragma pack(1)

#include "xxProto.h"
#include "xxCustom.h"
#include "xxSpi.h"

int SPI_CPOL = 0;
int SPI_CPHA = 0;

static char SendProgrammSpiByte( char out );
static char Fin[4]; 

int SendSpi( SPI_TypeDef* SPIx, char *out, char *in, int bytes )
{
int i, k=0, inc = 1, idx = 1;

  if( in == 0 ) {
    in = Fin;
    inc = 0;
  }
  
  if( SPIx != NULL ) {  
    for( i=0; i<bytes; i++ ) {
	SpiTimeout = 0;
	while( 1 ) {
            if( (SPIx->SR & spTxReady)!=0 ) break;
	    if( SpiTimeout > 2 ) { idx = 0; break; }
	}
        SPIx->DR = out[i];
	SpiTimeout = 0;
	while( 1 ) {
            if( (SPIx->SR & spRxReady)!=0 ) { in[k] = SPIx->DR; break; }
	    if( SpiTimeout > 2 ) { idx = 0; in[k] = 0x77; break; }
	}
	k += inc;
    }
    return idx;    
  }
  else {
      for( i=0; i<bytes; i++ ) {
        in[k] = SendSpiByte( NULL, out[i] );
      }
      k += inc;
  }
  return 1;
}


char SendSpiByte( SPI_TypeDef* SPIx, char out )
{
char dr = 0x77, value;

    if( SPIx != NULL ) {
        SpiTimeout = 0;
        while( 1 ) {
            if( (SPIx->SR & spTxReady)!=0 ) break;
	    if( SpiTimeout > 5 ) goto ret;
        }
        SPIx->DR = out;
        SpiTimeout = 0;
        while( 1 ) {
            if( (SPIx->SR & spRxReady)!=0 ) break;
	    if( SpiTimeout > 5 ) goto ret;
        }
	dr = SPIx->DR;
	return dr;
    }
    else {
        // Programm SPI
        return SendProgrammSpiByte( out );
    }
ret:    
    value = SPIx->DR;
    return dr;
}

void SetSpiPolarity( SPI_TypeDef* SPIx, int pol, int pha )
{
    if( SPIx != NULL ) return;
    SPI_CPOL = pol;
    SPI_CPHA = pha;
    
    if( SPI_CPOL != 0 ) SetBit( PortSpi2Clk, Spi2_CLK );
    else ClrBit( PortSpi2Clk, Spi2_CLK );
    MksDelay( 0.5 );
}

static char inData, outData;

static char SendProgrammSpiByte( char out )
{
int i;
unsigned char mask = 0x80;

    outData = out;
    
    inData = 0;
    if( SPI_CPOL == 0 ) {
      if( SPI_CPHA == 0 ) {
        // CPOL = 0 CPHA = 0
        for( i=0; i<8; i++ ) {
            if( (outData & mask) != 0 ) SetBit( PortSpi2MOSI, Spi2_MOSI );
            else ClrBit( PortSpi2MOSI, Spi2_MOSI );
            SetBit( PortSpi2Clk, Spi2_CLK );
            __no_operation();
	    __no_operation();
	    if( GetInBit( PortSpi2MISO, Spi2_MISO ) ) inData |= mask;
	    MksDelay( 0.25 );
            ClrBit( PortSpi2Clk, Spi2_CLK );
            mask >>= 1;
	    MksDelay( 0.25 );
        }
      }
      else {
        // CPOL = 0 CPHA = 1
        for( i=0; i<8; i++ ) {
            SetBit( PortSpi2Clk, Spi2_CLK );
            if( (outData & mask) != 0 ) SetBit( PortSpi2MOSI, Spi2_MOSI );
            else ClrBit( PortSpi2MOSI, Spi2_MOSI );
            if( GetInBit( PortSpi2MISO, Spi2_MISO ) ) inData |= mask;
            ClrBit( PortSpi2Clk, Spi2_CLK );
            mask >>= 1;
        }
      }
    }
    else {
        // CPOL = 1 CPHA = 0
      if( SPI_CPHA == 0 ) {
        for( i=0; i<8; i++ ) {
            if( (outData & mask) != 0 ) SetBit( PortSpi2MOSI, Spi2_MOSI );
            else ClrBit( PortSpi2MOSI, Spi2_MOSI );
            ClrBit( PortSpi2Clk, Spi2_CLK );
            if( GetInBit( PortSpi2MISO, Spi2_MISO ) ) inData |= mask;
            SetBit( PortSpi2Clk, Spi2_CLK );
            mask >>= 1;
        }
      }
      else {
        // CPOL = 1 CPHA = 1
        for( i=0; i<8; i++ ) {
            ClrBit( PortSpi2Clk, Spi2_CLK );
            if( (outData & mask) != 0 ) SetBit( PortSpi2MOSI, Spi2_MOSI );
            else ClrBit( PortSpi2MOSI, Spi2_MOSI );
            if( GetInBit( PortSpi2MISO, Spi2_MISO ) ) inData |= mask;
            SetBit( PortSpi2Clk, Spi2_CLK );
            mask >>= 1;
        }
      }
    }
    return inData;
}
