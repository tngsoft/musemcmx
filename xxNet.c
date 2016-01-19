#include "..\\inc\\stm32F4xx.h"

#pragma pack(1)

//#include "xxRtc.h"
#include "xxProto.h"
#include "xxNet.h"

#pragma data_alignment=4
TEthDescriptor TxDescriptors[1];
#pragma data_alignment=4
TEthDescriptor RxDescriptors[2];

TEthDescriptor *TxDesc, *RxDesc;

bool NetLink = false, NetPowerDown = false, NetSpeed100M = true, NetFullDuplex = true;
static bool PrevLink = false, bRes[3];

short int SrcPort = 0x060D;
short int HostPort = 0x060D;
unsigned long int *HMAC, *LMAC;
unsigned char SrcMAC[8] = {0x02, 0x10, 0x19, 0x63, 0x11, 0x02, 0, 0x02  };
unsigned char SrcIP[4] = { 111, 111, 111, 11 };

unsigned char HostMAC[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
unsigned char HostIP[4] = { 111, 111, 111, 3 };

#define INTR_NET_TSM

void InitNet( void )
{
unsigned int tmp, SystemClock = (SystemCoreClock);
    
    RCC->AHB1RSTR |= 0x02000000;
    RCC->AHB1RSTR &= ~0x02000000;
//select RMII
    SYSCFG->PMC |= SYSCFG_PMC_MII_RMII_SEL;
 
    // Ethernet    
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | 
                    RCC_AHB1ENR_GPIOCEN | 
                    RCC_AHB1ENR_GPIODEN |
                    RCC_AHB1ENR_GPIOEEN;

    ETH_RX_PORT->AFR[0] |= 0x00BB00B0;                  // RX D0 D1 MDC
    ETH_TX_PORT->AFR[1] |= 0x00BBB000;                  // TX D0 D1 EN
    ETH_MDIO_PORT->AFR[0] |= 0xB0000BB0;                  // CRS MDIO REFCLK

    // ETH pins config
    ConfigPin( ETH_RX_PORT, ETH_RXD0, mdPinAf, tpPushPull, ppPullUp, sp50M );
    ConfigPin( ETH_RX_PORT, ETH_RXD1, mdPinAf, tpPushPull, ppPullUp, sp50M );
    ConfigPin( ETH_TX_PORT, ETH_TXD0, mdPinAf, tpPushPull, ppPullUp, sp50M );
    ConfigPin( ETH_TX_PORT, ETH_TXD1, mdPinAf, tpPushPull, ppPullUp, sp50M );
    ConfigPin( ETH_TX_PORT, ETH_TXEN, mdPinAf, tpPushPull, ppPullUp, sp50M );
    ConfigPin( ETH_CRS_PORT, ETH_CRS, mdPinAf, tpPushPull, ppPullUp, sp50M );
    ConfigPin( ETH_MDIO_PORT, ETH_MDIO, mdPinAf, tpPushPull, ppPullUp, sp50M );
    ConfigPin( ETH_MDC_PORT, ETH_MDC, mdPinAf, tpPushPull, ppPullUp, sp50M );
    ConfigPin( ETH_REFCLK_PORT, ETH_REFCLK, mdPinAf, tpPushPull, ppPullUp, sp50M );

    // Reset Net
    ConfigPin( PortNet, ResetNet, mdPinOut, tpPushPull, ppPullUp, sp50M );
    SetBit( PortNet, ResetNet );    

    //MAC clocks enable
    RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACEN | RCC_AHB1ENR_ETHMACRXEN | RCC_AHB1ENR_ETHMACTXEN; 

    //DMA soft reset
    ETH->DMABMR |= ETH_DMABMR_SR; 
    FlashTimeout = 0;
    while( (ETH->DMABMR & ETH_DMABMR_SR) !=0 ) {
        if( FlashTimeout >= 200 ) break; 
    }
    if( FlashTimeout >= 200 ) 
        FlashTimeout = 0;
    
    //configure SMI spe+ed
    tmp = ETH->MACMIIAR;
    tmp &= ~ETH_MACMIIAR_CR;
    if( SystemClock <= 35000000 ) tmp |= ETH_MACMIIAR_CR_Div16;
    if((SystemClock > 35000000) && (SystemClock <= 60000000)) tmp |= ETH_MACMIIAR_CR_Div26;
    if((SystemClock > 60000000) && (SystemClock <= 100000000)) tmp |= ETH_MACMIIAR_CR_Div42;
    if((SystemClock > 100000000) && (SystemClock <= 150000000)) tmp |= ETH_MACMIIAR_CR_Div62;
    if(SystemClock > 150000000) tmp |= ETH_MACMIIAR_CR_Div102;

    //tmp |= ETH_MACMIIAR_CR_Div26;
    ETH->MACMIIAR = tmp;
  
    //init PHY
    InitPHY( true );
    
    //configure MAC
    LMAC = (unsigned long int *)SrcMAC;
    HMAC = (unsigned long int *)(SrcMAC+4);
    ETH->MACA0HR = *HMAC; //MAC address
    ETH->MACA0LR = *LMAC;

    //disable timestamp
    ETH->PTPTSCR &= ~ETH_PTPTSSR_TSSIPV4FE;

    tmp = ETH->DMABMR;
    tmp &= ~ETH_DMABMR_PBL;
    tmp |= ETH_DMABMR_AAB | ETH_DMABMR_FB | ETH_DMABMR_PBL_32Beat; //address aligned beats, fixed burst, maximum burst length 32 beats
    ETH->DMABMR = tmp;

    //DMA interrupt
    ETH->DMAIER |= ETH_DMAIER_RIE | ETH_DMAIER_TIE | ETH_DMAIER_NISE;
#ifdef INTR_NET_TSM
    //ETH->DMAIER |= ETH_DMAIER_TIE;
#endif

    //configure DMA d+escriptors - 1 Tx & 1 Rx
    // first&last segment
    // next descriptor chained, 
    // auto-checksums
  
    TxDescriptors[0].Flags = txcIntrComplete | txcLastSegment | txcFirstSegment | 
                               txcCheckSumFull | txcNoPad | txcEndOfRing;// | txcSecondDesc;
    TxDescriptors[0].Len1 = 0;
    TxDescriptors[0].Len2 = 0;
    TxDescriptors[0].Address1 = NetInBuffer + 0x80;
    TxDescriptors[0].Address2 = NetInBuffer;
    //TxDescriptors[0].Address2 = &TxDescriptors[1];
/*
    TxDescriptors[1].Flags = txcIntrComplete | txcLastSegment | txcFirstSegment |
                               txcCheckSumFull | txcSecondDesc | txcNoPad;
    TxDescriptors[1].Len1 = 0;
    TxDescriptors[1].Len2 = 0;
    TxDescriptors[1].Address1 = FlashInBuffer;
    TxDescriptors[1].Address2 = &TxDescriptors[0];
*/
#ifdef INTR_NET_TSM
    //TxDescriptors[0].Flags |= txcIntrComplete; 
    //TxDescriptors[1].Flags |= txcIntrComplete; 
#endif
    
    RxDescriptors[0].Flags = rxcInDMA | rxcLastDesc | rxcFirstDesc;
    RxDescriptors[0].Len1 = (1600 | rxcEndOfRing);
    RxDescriptors[0].Len2 = 1600;
    RxDescriptors[0].Address1 = NetInBuffer;
    RxDescriptors[0].Address2 = NetInBuffer;
    
    TxDesc = TxDescriptors;
    RxDesc = RxDescriptors;

    //configure DMA
    ETH->DMATDLAR = (unsigned int)&TxDescriptors;
    ETH->DMARDLAR = (unsigned int)&RxDescriptors;
    
    //full-duplex mode, transmit&receive modules enable  
    ETH->MACCR |= ETH_MACCR_TE | ETH_MACCR_RE; 
  
    //run
    //transmit&receive srore&forward, start transmit&receive
    ETH->DMAOMR |= /*ETH_DMAOMR_OSF | */ETH_DMAOMR_RSF | ETH_DMAOMR_TSF | ETH_DMAOMR_ST | ETH_DMAOMR_SR; 
  
    NVIC_SetPriority( ETH_IRQn, 1 );
    NVIC_EnableIRQ( ETH_IRQn );  
    NVIC_ClearPendingIRQ( ETH_IRQn );
}

void InitPHY( bool reset )
{
short status;

if( reset ) {

    ClrBit( PortNet, ResetNet );
    MsDelay( 5 );
    SetBit( PortNet, ResetNet );
    MsDelay( 5 );
    // Set Mode Register 10 Mb Full duplex for follow soft reset
    //SMIWrite( PhySpecialModeReg, ecrMode10Full );
    //MsDelay( 50 );

    // Soft reset
    /*
    status = SMIRead( PhyControlReg );
    status |= ecrSoftReset;
    SMIWrite( PhyControlReg, status );
    SMIWaitResetBit( PhyControlReg, ecrSoftReset );
    */
}
    // Depend from ext setting select - AutoNeg or 10 Mb or 100 Mb
    // Now AutoNeg
    // Enable AutoNegotiation
    status = SMIRead( PhyControlReg );
    //status = ecrFullDuplexMode | ecrSetSpeed10M;
    status |= ecrAutoNegEnable;
    SMIWrite( PhyControlReg, status );    
    //SMIWrite( PhyControlReg, ecrAutoNegEnable );    
    // Wait for AutoNeg done
    SMIWaitSetBit( PhyStatusReg, estAutoNegComplete );
    SMIWaitSetBit( PhySpecialCSReg, estAutoNegDone );
    //return;
/*    
    // Restart AutoNeg
    status = SMIRead( PhyControlReg );
    status |= ecrAutoNegRestart;
    SMIWrite( PhyControlReg, status );    
    SMIWaitSetBit( PhyStatusReg, estAutoNegComplete );
*/
    status = ecrFullDuplexMode | ecrSetSpeed10M;
    //SMIWrite( PhyControlReg, status );    
    
    // Clear duplex & speed bits in MAC
    ETH->MACCR &= ~(ETH_MACCR_DM | ETH_MACCR_FES);
    status = SMIRead( PhySpecialCSReg );
    // Set 100 Mb Full -----------------------------------------
    //status = estFullDuplexBit | estSpeed100MBit;
    // Set MAC from values PHY speed & duplex
    if( status & estFullDuplexBit ) {
        // Set Full Duplex for MAC
        ETH->MACCR |= ETH_MACCR_DM;
        NetFullDuplex = true;
    }
    else {
        // Set Half Duplex for MAC
        ETH->MACCR |= ETH_MACCR_DM;
        NetFullDuplex = true;
//        ETH->MACCR &= ~ETH_MACCR_DM;
//        NetFullDuplex = false;
    }
    if( status & estSpeed100MBit ) {
        // Set speed 100 M
        ETH->MACCR |= ETH_MACCR_FES;
        NetSpeed100M = true;
    }
    else {
        // Set speed 10 M
        ETH->MACCR &= ~ETH_MACCR_FES;
        NetSpeed100M = false;
    }    
}

static int LinkCounter = 0;

bool CheckLink( void )
{
  if( ++LinkCounter < 1000 ) return NetLink;
  LinkCounter = 0;
  NetLink = SMIRead( PhyStatusReg ) & estHaveLink;

    if( (!PrevLink) & NetLink )   
        InitPHY( false );
    if( (PrevLink) & !NetLink ) 
        PrevLink = true;
        
    return (PrevLink = NetLink);
}

void NetToPowerDown( void )
{
unsigned short tmp;

    NetPowerDown = true;
    tmp = SMIRead( PhyControlReg );
    tmp &= ~ecrAutoNegEnable;
    SMIWrite( PhyControlReg, tmp );
    SMIWaitResetBit( PhyControlReg, ecrAutoNegEnable );
    SMIWrite( PhyControlReg, ecrSetPowerDownMode );
    ETH->DMAOMR &= ~(ETH_DMAOMR_ST | ETH_DMAOMR_SR);
    NVIC_DisableIRQ(ETH_IRQn);
    ETH->MACCR &= ~(ETH_MACCR_TE | ETH_MACCR_RE);
}

bool SMIWrite( char addr, short int data )
{
unsigned long int tmp;

    addr &= 0x1F;
    
    NetTimeout = 0;  
    while( ETH->MACMIIAR & ETH_MACMIIAR_MB ) {
        if( NetTimeout >= 500 ) break;
    }
    if( NetTimeout >= 500 ) 
        NetTimeout =0;
    tmp = ETH->MACMIIAR;
    tmp &= ETH_MACMIIAR_CR;
    tmp |= (addr<<6) | ETH_MACMIIAR_MW | ETH_MACMIIAR_MB;  
    ETH->MACMIIDR = data;
    ETH->MACMIIAR = tmp;
    NetTimeout = 0;  
    while( ETH->MACMIIAR & ETH_MACMIIAR_MB ) {
        if( NetTimeout >= 500 ) break;
    }
    if( NetTimeout >= 500 ) 
        NetTimeout =0;
    return true;
}

short int SMIRead( char addr )
{
unsigned long int tmp;

    addr &= 0x1F;
  
    NetTimeout = 0;  
    while( ETH->MACMIIAR & ETH_MACMIIAR_MB ) {
        if( NetTimeout >= 500 ) break;
    }
    if( NetTimeout >= 500 ) 
        NetTimeout = 0;
    tmp = ETH->MACMIIAR;
    tmp &= ETH_MACMIIAR_CR;
    tmp |= (addr<<6) | ETH_MACMIIAR_MB;
    ETH->MACMIIAR = tmp;
    NetTimeout = 0;  
    while( ETH->MACMIIAR & ETH_MACMIIAR_MB ) {
        if( NetTimeout >= 500 ) break;
    }
    if( NetTimeout >= 500 ) 
        NetTimeout = 0;
    return ETH->MACMIIDR;
}

void SMIWaitSetBit( char addr, unsigned short bit )
{
unsigned short tmp=0;
    
    FlashTimeout = 0;  
    while( 1 ) {
        tmp = SMIRead( addr );
        if( (tmp & bit) ) break;
        if( FlashTimeout++ > 500 ) break;
  }
}
  
void SMIWaitResetBit( char addr, unsigned short bit )
{
unsigned short tmp=0;
    
    FlashTimeout = 0;  
    while( 1 ) {
        tmp = SMIRead( addr );
        if( !(tmp & bit) ) break;
        if( FlashTimeout++ > 500 ) break;
  }
}
  
static int One=0, Sec=2;

int SendPacket( char *buf, int num, char *head, int sizehead )
{
TEthDescriptor *Desc = (TEthDescriptor *)ETH->DMACHTDR;

    TxDesc = Desc;
    
    // Disable transmit
    //ETH->DMAOMR &= ~ETH_DMAOMR_ST; 

    // Set descriptor numbers
    if( Desc == &TxDescriptors[0] ) { 
      One = 0; Sec = 1; 
    }
    if( Desc == &TxDescriptors[1] ) { 
      One = 1; Sec = 0; 
    }

    if( One == Sec ) return -1;
    
    // Enable Tx polling
    //ETH->DMATPDR |= 1; 
    // Wait while packets tsm done
    //FlashTimeout = 0;
/*
    while( 1 ) {
        if( (( TxDescriptors[0].Flags & txcInDMA ) == 0) && 
            (( TxDescriptors[1].Flags & txcInDMA ) == 0) ) break;
        if( FlashTimeout > 50 ) return -2;
    }    
*/
    //TxDescriptors[One].Len2 = num;
    //TxDescriptors[One].Address2 = buf;
/*
    TxDescriptors[Sec].Len1 = num;
    TxDescriptors[Sec].Address1 = buf;
    TxDescriptors[One].Flags |= txcInDMA;
    TxDescriptors[Sec].Flags |= txcInDMA;
*/
    
//    if( num == 0 ) {
      //TxDescriptors[One].Flags &= ~(txcLastSegment | txcFirstSegment);    
      //TxDescriptors[One].Flags |= (txcLastSegment | txcFirstSegment);
      TxDescriptors[One].Len1 = sizehead;
      TxDescriptors[One].Address1 = head;
      TxDescriptors[One].Len2 = num;
      TxDescriptors[One].Address2 = buf;
      TxDescriptors[One].Flags |= txcInDMA;
      //TxDescriptors[Sec].Flags |= txcInDMA;
//    }
/*
    else {
      TxDescriptors[One].Flags &= ~(txcLastSegment | txcFirstSegment);    
      TxDescriptors[One].Flags |= txcFirstSegment;
      TxDescriptors[One].Len1 = sizeheader;
      TxDescriptors[One].Address1 = header;
      TxDescriptors[One].Flags |= txcInDMA;

      TxDescriptors[Sec].Flags &= ~(txcLastSegment | txcFirstSegment);
      TxDescriptors[Sec].Flags |= txcLastSegment;
      TxDescriptors[Sec].Len1 = num;
      TxDescriptors[Sec].Address1 = buf;
      TxDescriptors[Sec].Flags |= txcInDMA;
    }
*/
    // Enable transmit
    //ETH->DMAOMR |= ETH_DMAOMR_ST; 
    
    FlagNetTsm = 0;
    // Start polling descriptors
    ETH->DMATPDR |= 1;      

    NetTimeout = 0;
//    while( !FlagNetTsm ) {
//        if( NetTimeout > 2 ) break;
//    }
    // MB wait while not set FlagNetTsm 
    
    // Wait while packets tsm done
    FlashTimeout = 0;
    while( 1 ) {
        if( ( TxDescriptors[One].Flags & txcInDMA ) == 0 ) break;
        if( FlashTimeout > 20 ) return -2;
    }    
    return One;
}

int LenPacket;

void InitNetReceive( void ) 
{ 
    RxDesc->Flags &= 0xC000FFFF;
    RxDesc->Flags |= rxcInDMA; 
    ETH->DMARPDR |= 1;
}

int ReadPacket( void )
{
TEthDescriptor *Desc = (TEthDescriptor *)ETH->DMACHRDR;
int len = 0;  
    NetTimeout = 0;
    while( 1 ) {
        if( ( Desc->Flags & rxcInDMA ) == 0 ) break;
        if( NetTimeout > 4 ) goto ret;
    }
    
    if( Desc != RxDesc ) goto ret;
    if( Desc->Flags & rxsWasError ) goto ret;
    len = (RxDesc->Flags >> 16) & 0x3FFF;
    len -= 4;
    if( len > 0 ) return len;
ret:
    LenPacket = 0;
    InitNetReceive();
    return 0;  
}

//------------------------------------------------------------------------------

void ETH_IRQHandler( void )
{
    if( ETH->DMASR & ETH_DMASR_RS ) {
        ETH->DMASR |= ETH_DMASR_RS;
	FlagNetRcv = 1;
	//ReadPacket();
        //NVIC_ClearPendingIRQ( ETH_IRQn );
    }
    if( ETH->DMASR & ETH_DMASR_TS ) {
        ETH->DMASR |= ETH_DMASR_TS;
	FlagNetTsm = 1;
	//ReadPacket();
        //NVIC_ClearPendingIRQ( ETH_IRQn );
    }
}

//------------------------------------------------------------------------------
