#include "..\\inc\\stm32F4xx.h"
#include <string.h>

#pragma pack(1)

//#include "xxRtc.h"
#include "xxSpi.h"
#include "xxProto.h"
#include "xxCustom.h"

#define eeCmdWriteEna           0x06
#define eeCmdWriteDis           0x04
#define eeCmdWriteStatus        0x01
#define eeCmdReadStatus         0x05
#define eeCmdReadData           0x03
#define eeCmdWriteData          0x02

#define stEepromBusy            0x01
#define stEepromEnaWrite        0x02
#define stEepromNoProtect       0x00
#define stEepromQuartProtect    0x04
#define stEepromHalfProtect     0x08
#define stEepromFullProtect     0x0C

int EE_CPOL = 0;
int EE_CPHA = 0;

static unsigned char Status;
static char *EEdata, *SDdata;

static unsigned char ReadStatus( void )
{
    ClrBit( PortEprom, CSEprom );
    __no_operation();
    SendSpiByte( SpiEprom, eeCmdReadStatus );
    Status = SendSpiByte( SpiEprom, 0x00 );
    __no_operation();
    SetBit( PortEprom, CSEprom );
    return Status;
}

static void WriteEnable( void )
{
    // Enable write
    ClrBit( PortEprom, CSEprom );
    __no_operation();
    SendSpiByte( SpiEprom, eeCmdWriteEna );
    __no_operation();
    SetBit( PortEprom, CSEprom );
    // Waite done write operation
    while( 1 ) {
        if( (ReadStatus() & stEepromEnaWrite) != 0 ) break;
	if( FlashTimeout > 5 ) break;
    }
}

static void WriteDisable( void )
{
    // Disable write
    ClrBit( PortEprom, CSEprom );
    __no_operation();
    SendSpiByte( SpiEprom, eeCmdWriteDis );
    SetBit( PortEprom, CSEprom );
    __no_operation();
    while( 1 ) {
        if( (ReadStatus() & stEepromEnaWrite) == 0 ) break;
	if( FlashTimeout > 5 ) break;
    }
}

static void WriteStatus( unsigned char status )
{
    SetSpiPolarity( SpiEprom, EE_CPOL, EE_CPHA );

    WriteEnable();
    // Write status
    ClrBit( PortEprom, CSEprom );
    __no_operation();
    SendSpiByte( SpiEprom, eeCmdWriteStatus );
    SendSpiByte( SpiEprom, status );
    __no_operation();
    SetBit( PortEprom, CSEprom );
    WriteDisable();
}

bool EEInit( void )
{
int to = 20000;
    WriteStatus( 4 );
    Status = ReadStatus();
    while( --to ) {
        Status = ReadStatus();
	if( (Status & stEepromBusy) == 0 ) break;
    }
    if( Status != 4 ) return false;
    return true;
}

#define EESIZEPAGE	256

void EEWR( int addr, int num ) 
{
int i;

    WriteEnable();
    ClrBit( PortEprom, CSEprom );  
    __no_operation();

    // Write command
    SendSpiByte( SpiEprom, eeCmdWriteData );
    // Write 3 address bytes
    SendSpiByte( SpiEprom, (addr>>16)&0xFF );
    SendSpiByte( SpiEprom, (addr>>8)&0xFF );
    SendSpiByte( SpiEprom, (addr)&0xFF );
    // Write data
    for( i=0; i<num; i++ ) SendSpiByte( SpiEprom, EEdata[i] );
    __no_operation();
    SetBit( PortEprom, CSEprom );    
    FlashTimeout = 0;
    // Waite done write operation
    while( 1 ) {
        if( (ReadStatus() & stEepromBusy) == 0 ) break;
	if( FlashTimeout > 5 ) break;
    }
    WriteDisable();
}

static int addr, num;

int EEWrite( int begaddr, void *data, int numdata )
{
    SetSpiPolarity( SpiEprom, EE_CPOL, EE_CPHA );

    EEdata = (char *)data;
    
    addr = begaddr;
    num = begaddr;
    while( num >= EESIZEPAGE ) num -= EESIZEPAGE;
    num = EESIZEPAGE - num;
    
    if( num > numdata ) num = numdata;
    EEWR( addr, num );
    addr += num;
    EEdata += num;
    numdata -= num;
    
    while( numdata > 0 ) {
        if( numdata >= EESIZEPAGE ) num = EESIZEPAGE;
	else num = numdata;
        EEWR( addr, num );
        addr += num;
        EEdata += num;
        numdata -= num;
    }
    return Status;
}

void EERD( int addr, int num ) 
{
int i;

    ClrBit( PortEprom, CSEprom );
    __no_operation();
    // Read command
    SendSpiByte( SpiEprom, eeCmdReadData );
    // Write 3 address bytes
    SendSpiByte( SpiEprom, (addr>>16)&0xFF );
    SendSpiByte( SpiEprom, (addr>>8)&0xFF );
    SendSpiByte( SpiEprom, (addr)&0xFF );
    // MksDelay( 2 );
    // Read data
    for( i=0; i<num; i++ ) EEdata[i] = SendSpiByte( SpiEprom, 0x77 );
    __no_operation();
    SetBit( PortEprom, CSEprom );    
}

int EERead( int begaddr, void *data, int numdata )
{      
    SetSpiPolarity( SpiEprom, EE_CPOL, EE_CPHA );

    EEdata = (char *)data;
    
    addr = begaddr;
    num = begaddr;
    while( num >= EESIZEPAGE ) num -= EESIZEPAGE;
    num = EESIZEPAGE - num;

    if( num > numdata ) num = numdata;
    EERD( addr, num );
    addr += num;
    EEdata += num;
    numdata -= num;
    
    while( numdata > 0 ) {
        if( numdata >= EESIZEPAGE ) num = EESIZEPAGE;
	else num = numdata;
        EERD( addr, num );
        addr += num;
        EEdata += num;
        numdata -= num;
    }
    return Status;
}

#define SDSIZEPAGE	512
static int SDpage = 0;  

int SDRead( int begaddr, void *data, int numdata )
{
    SDpage = 0;  
    SDdata = (char *)data;
    
    addr = begaddr;
    num = begaddr;
    while( num >= SDSIZEPAGE ) {
      num -= SDSIZEPAGE; 
      addr -= SDSIZEPAGE; 
      SDpage++; 
    }
    num = SDSIZEPAGE - num;
    
    if( num > numdata ) num = numdata;
    Status = ReadPage( SDpage, 512, DOut );
    memcpy( SDdata, DOut + addr, num );
    addr = 0;
    SDdata += num;
    numdata -= num;
    SDpage++;
    
    while( numdata > 0 ) {
        if( numdata >= SDSIZEPAGE ) num = SDSIZEPAGE;
	else num = numdata;
        Status = ReadPage( SDpage, 512, DOut );
        memcpy( SDdata, DOut + addr, num );
        addr = 0;
        SDdata += num;
        numdata -= num;
        SDpage++;
    }
    return Status;
}

int SDWrite( int begaddr, void *data, int numdata )
{
    SDpage = 0;  
    SDdata = (char *)data;
    
    addr = begaddr;
    num = begaddr;
    while( num >= SDSIZEPAGE ) { 
      num -= SDSIZEPAGE; 
      addr -= SDSIZEPAGE; 
      SDpage++; 
    }
    num = SDSIZEPAGE - num;
    
    if( num > numdata ) num = numdata;
    Status = ReadPage( SDpage, 512, DOut );
    memcpy( DOut, SDdata + addr, num );
    Status = WritePage( SDpage, 512, DOut );
    addr = 0;
    SDdata += num;
    numdata -= num;
    SDpage++;
    
    while( numdata > 0 ) {
        if( numdata >= SDSIZEPAGE ) num = SDSIZEPAGE;
	else num = numdata;
        Status = ReadPage( SDpage, 512, DOut );
        memcpy( DOut, SDdata + addr, num );
        Status = WritePage( SDpage, 512, DOut );
        addr = 0;
        SDdata += num;
        numdata -= num;
        SDpage++;
    }
    return Status;
}

#define SIZE_SECTOR	 SizeBadBl
#define SIZE_INT_SECTOR	 (SizeBadBl/4)

/* Private variables ---------------------------------------------------------*/
static int BlockWordIndex = 1;
static int *SaveBlock;
static unsigned int FF[2];
//------------------------------------------------------------------------------

int FindSaveBlock( void )
{
int i;
    
    if( NoEE ) SDRead( EE_START_ADDR + SizeSysData, FlashInBuffer, SIZE_SECTOR );
    else EERead( EE_START_ADDR + SizeSysData, FlashInBuffer, SIZE_SECTOR );
    if( (FlashInBuffer[0] != 'B') ||
         (FlashInBuffer[1] != 'L') ||
           (FlashInBuffer[2] != 'K') ) {
        InitSaveBlock( 1 );
        return 8;
    }

    SaveBlock = (int *)FlashInBuffer;
    for( i=1; i<SIZE_INT_SECTOR; i++ ) {
        if( SaveBlock[i] != 0xFFFFFFFF ) {
            BlockWordIndex = i;
            return SaveBlock[i];
        }
    }
    InitSaveBlock( 1 );
    return 8;
}

int IncSaveBlock( int block )
{
    if( BlockWordIndex >= (SIZE_INT_SECTOR-1) ) { 
        FF[0] = 0xFFFFFFFF;
        EEWrite( EE_START_ADDR + SizeSysData + (SIZE_INT_SECTOR-1)*4, FF, 4 );
        SDWrite( OffsBadBl + (SIZE_INT_SECTOR-1)*4, FF, 4 );
	BlockWordIndex = 1;
        FF[0] = block;
        EEWrite( EE_START_ADDR + SizeSysData + BlockWordIndex*4, FF, 4 );
	SDWrite( OffsBadBl + BlockWordIndex*4, FF, 4 );
    }
    else {
        FF[0] = 0xFFFFFFFF;
        FF[1] = block;
        EEWrite( EE_START_ADDR + SizeSysData + BlockWordIndex*4, FF, 8 );
        SDWrite( OffsBadBl + BlockWordIndex*4, FF, 8 );
        BlockWordIndex++; 
    }
    return BlockWordIndex;
}

int InitSaveBlock( int block )
{
int i;
char header[8] = { 'B', 'L', 'K', 0, 0, 0, 0,  8 };
    
    if( si.Block == 8 ) return 8;
    if( block < 8 ) block = 8;
    SaveBlock = (int *)FlashInBuffer;
    for( i=0; i<SIZE_INT_SECTOR; i++ ) SaveBlock[i] = 0xFFFFFFFF;
    memcpy( SaveBlock, header, 8 );
    SaveBlock[1] = block;
    EEWrite( EE_START_ADDR + SizeSysData, SaveBlock, SIZE_SECTOR );
    SDWrite( OffsBadBl, SaveBlock, SIZE_SECTOR );
    BlockWordIndex = 1; 
    return 8;
}

extern unsigned int BadBlocks[SizeBadBl/4];
#define MAXBAD	(SizeBadBl/4-2)

int ReadBadBlocks( void *data )
{
    if( NoEE ) SDRead( OffsBadBl, data, SizeBadBl );
    else EERead( EE_START_ADDR + OffsBadBl, data, SizeBadBl );
    return Status;
}

int WriteBadBlocks( void *data )
{
    if( NoEE ) SDWrite( OffsBadBl, data, SizeBadBl );
    else EEWrite( EE_START_ADDR + OffsBadBl, data, SizeBadBl );
    return Status;
}

int InitBadBlocks( void )
{
char header[8] = { 'B', 'A', 'D', 0, 0, 0, 0, 0 };
//int num = 0; 

    if( NoEE ) return 0;
    EERead( OffsBadBl, FlashInBuffer, SizeBadBl );
    if( (FlashInBuffer[0] != 'B') ||
         (FlashInBuffer[1] != 'A') ||
           (FlashInBuffer[2] != 'D') ) {
             memcpy( FlashInBuffer, header, 8 );
             EEWrite( EE_START_ADDR + OffsBadBl, header, 8 );
             //return 0;
    }
    memcpy( BadBlocks, FlashInBuffer, SizeBadBl );
    return BadBlocks[0];
}

int SetBadBlock( int block )
{
    if( BadBlocks[1] < MAXBAD ) {
        BadBlocks[1]++;
        BadBlocks[BadBlocks[1]+1] = block;
        if( !NoEE ) EEWrite( OffsBadBl, BadBlocks, SizeBadBl );
    }
    return Status;
}

int IsBlockBad( int block ) 
{
//int i;
//    if( BadBlocks[1] <= 0 ) return 0;
//    for( i=2; i<=BadBlocks[1]; i++ ) if( block == BadBlocks[i] ) return 1;
    return 0;
}

int ReadSystemData( void *data )
{
    if( NoEE ) SDRead( 0, data, SizeSysData );
    else EERead( EE_START_ADDR, data, SizeSysData );
    return Status;
}

int ReadMainTp( void *data )
{
    if( NoEE ) SDRead( OffsInfo, data, SizeFtp );
    else EERead( EE_START_ADDR + OffsInfo, data, SizeFtp );
    return Status;
}

int ReadSecTp( void *data )
{
    if( NoEE ) SDRead( OffsSecTp, data, SizeStp );
    else EERead( EE_START_ADDR + OffsSecTp, data, SizeStp );
    return Status;
}

int ReadCalb( void *data, int len )
{
    if( NoEE ) SDRead( OffsCalb, data, len );
    else EERead( EE_START_ADDR + OffsCalb, data, len );
    return Status;
}

int ReadRuns( void *data )
{
    if( NoEE ) SDRead( OffsRuns, data, SizeRuns );
    else EERead( EE_START_ADDR + OffsRuns, data, SizeRuns );
    return Status;
}

int WriteMainTp( void *data )
{
    if( NoEE ) SDWrite( OffsInfo, data, SizeFtp );
    else EEWrite( EE_START_ADDR + OffsInfo, data, SizeFtp );
    return Status;
}

int WriteSecTp( void *data )
{
    if( NoEE ) SDWrite( OffsSecTp, data, SizeStp );
    else EEWrite( EE_START_ADDR + OffsSecTp, data, SizeStp );
    return Status;
}

int WriteCalb( void *data, int len )
{
    if( NoEE ) SDWrite( OffsCalb, data, len );
    else EEWrite( EE_START_ADDR + OffsCalb, data, len );
    return Status;
}

int ReadBaudRate( void *data )
{
char *ctp, *cmi;
int offs;
    ctp = (char *)(&tp);
    cmi = (char *)(&tp.BaudRate485);
    offs = cmi - ctp;
    if( NoEE ) SDRead( offs, data, 8 );
    else EERead( EE_START_ADDR + offs, data, 8 );
    return Status;
}

int WriteBaudRate( void *data )
{
char *ctp, *cmi;
int offs;
    ctp = (char *)(&tp);
    cmi = (char *)(&tp.BaudRate485);
    offs = cmi - ctp;
    if( NoEE ) SDWrite( offs, data, 8 );
    else EEWrite( EE_START_ADDR + offs, data, 8 );
    return Status;
}

int WriteStartParams( void *data )
{
char *ctp, *cmi;
    ctp = (char *)(&tp);
    cmi = (char *)(&tp.StartParam);
    if( NoEE ) SDWrite( (cmi-ctp) , data, sizeof( mi )  );
    else EEWrite( EE_START_ADDR + (cmi-ctp) , data, sizeof( mi )  );
    return Status;
}

int WriteRuns( void *data, int offs, int len )
{
    if( NoEE ) SDWrite( OffsRuns + offs, data, len );
    else EEWrite( EE_START_ADDR + OffsRuns + offs, data, len );
    return Status;
}

int ClearRuns( void )
{
    memset( FlashInBuffer, 0xFF, SizeRuns ); 
    if( NoEE ) SDWrite( OffsRuns, FlashInBuffer, SizeRuns );
    else EEWrite( EE_START_ADDR + OffsRuns, FlashInBuffer, SizeRuns );
    return Status;
}