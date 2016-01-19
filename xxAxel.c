#include "..\\inc\\stm32F4xx.h"

#pragma pack(1)

#include "xxProto.h"
#include "xxSpi.h"
#include "xxCustom.h"

#define axRead		0xC0
#define axWrite	        0x40

#define arWhoMe 	0x0F
#define arOffsX 	0x16
#define arOffsY 	0x17
#define arOffsZ 	0x18
#define arGainX 	0x19
#define arGainY 	0x1A
#define arGainZ 	0x1B

#define arControl0	0x1F
#define arControl1	0x20
#define arControl2	0x21
#define arControl3	0x22
#define arControl4	0x23
#define arControl5	0x24
#define arControl6	0x25
#define arControl7	0x26
#define srStatus	0x27

#define arFilterReset	0x23

#define arWordX 	0x28
#define arWordY 	0x2A

#define arWordXL 	0x28
#define arWordXH 	0x29
#define arWordYL 	0x2A
#define arWordYH 	0x2B
#define arWordZL 	0x2C
#define arWordZH 	0x2D

#define ct1AxRate25Hz	0x40
#define ct1AxRate50Hz	0x50
#define ct1BlockData	0x08
#define ct1EnaZ		0x04
#define ct1EnaY		0x02
#define ct1EnaX		0x01

#define ct2AxScale2g	0x00
#define ct2AxScale4g	0x08
#define ct2AxScale6g	0x10
#define ct2AxScale8g	0x18

#define ct3AxEnaInt1	0x04
#define ct4MaEnaInt1	0x02

#define ct4AxEnaInt2	0x08
#define ct4MaEnaInt2	0x04

#define ct5TempEna	0x80
#define ct5MaResHi	0x60
#define ct5MaRate25Hz	0x0C
#define ct5MaRate50Hz	0x10

#define ct6MaScale2g	0x00
#define ct6MaScale4g	0x20
#define ct6MaScale8g	0x40
#define ct6MaScale12g	0x60

int AXEL_CPOL = 0;
int AXEL_CPHA = 0;

static char axData;
char *InAx;


void AxelWriteByte( char addr, char byte )
{
    ClrBit( PortAxel, CSAxel );
    SetSpiPolarity( SpiAxel, AXEL_CPOL, AXEL_CPHA );
    addr |= axWrite;  
    SendSpiByte( SpiAxel, addr );
    SendSpiByte( SpiAxel, byte );
    SetBit( PortAxel, CSAxel );
}

char AxelReadByte( char addr )
{
    ClrBit( PortAxel, CSAxel );
    SetSpiPolarity( SpiAxel, AXEL_CPOL, AXEL_CPHA );
    addr |= axRead;  
    SendSpiByte( SpiAxel, addr );
    axData = SendSpiByte( SpiAxel, 0x77 );
    SetBit( PortAxel, CSAxel );

    return axData;
}

void AxelReadXYZ( void *buf )
{
int i;
    ClrBit( PortAxel, CSAxel );
    InAx = (char *)buf;    
    SetSpiPolarity( SpiAxel, AXEL_CPOL, AXEL_CPHA );
    SendSpiByte( SpiAxel, arWordXL | axRead );
    for( i=0; i<6; i++ ) InAx[i] = SendSpiByte( SpiAxel, 0x77 );
    SetBit( PortAxel, CSAxel );

    ClrBit( PortAxel, CSAxel );
    SendSpiByte( SpiAxel, 5 | axRead );
    si.Temperature = SendSpiByte( SpiAxel, 0x77 );
    si.Temperature |= (SendSpiByte( SpiAxel, 0x77 ))<<8;
    SetBit( PortAxel, CSAxel );
    
    return;
}

void InitAxel( void )
{
    // Write to control registers
    AxelWriteByte( arControl0, 0x00 );
    AxelWriteByte( arControl1, ct1AxRate50Hz + ct1BlockData + ct1EnaX + ct1EnaY + ct1EnaZ );
    AxelWriteByte( arControl2, ct2AxScale2g );
    AxelWriteByte( arControl3, ct3AxEnaInt1 );
    AxelWriteByte( arControl4, ct4MaEnaInt2 );
    AxelWriteByte( arControl5, ct5TempEna + ct5MaResHi + ct5MaRate50Hz );
    AxelWriteByte( arControl6, ct6MaScale2g );
    AxelWriteByte( arControl7, 0x00 );
    AxelReadByte( arFilterReset );
    AxelReadByte( arWhoMe );
    if( axData != 0x49 ) si.Status |= stAxelBad;
    return;
}

int AxelWhoMe( void )
{
    return AxelReadByte( arControl1 );
}
