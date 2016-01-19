#ifndef XXNETH
#define XXNETH

typedef struct {
  unsigned long int Flags;
  short int Len1, Len2;
  void *Address1;
  void *Address2;
} TEthDescriptor;

#define PhyControlReg   0
//----- Bits for reg -----------------
#define ecrFullDuplexMode       0x0100
#define ecrAutoNegRestart       0x0200
#define ecrIsolatePhyFromRMII   0x0400
#define ecrSetPowerDownMode     0x0800
#define ecrAutoNegEnable        0x1000
#define ecrSetSpeed10M          0x0000
#define ecrSetSpeed100M         0x2000
#define ecrNormalOperation      0x0000
#define ecrLoopbackMode         0x4000
#define ecrSoftReset            0x8000
//------------------------------------

#define PhyStatusReg    1
//----- Bits for reg -----------------
#define estHaveLink             0x0004 
#define estAutoNegComplete      0x0020 
//------------------------------------

#define PhyPartnerReg   5
//----- Bits for reg -----------------
#define estHave10MFull          0x0040
#define estHave100MFull         0x0100 
#define estHaveLinkCode         0x4000 
//------------------------------------

#define PhyPartnerExReg 6
//----- Bits for reg -----------------
#define estPartnerHaveAutoNeg   0x0001 
#define estParallelFaultDetect  0x0010 
//------------------------------------

#define PhyEnergyReg    17
//----- Bits for reg -----------------
#define ecrPowerDownDetectEna   0x2000 
#define estPowerDetected        0x0002 
//------------------------------------

#define PhySpecialModeReg       18
//----- Bits for reg -----------------
#define ecrMode10Full           0x0020 
#define ecrMode100Full          0x0060 
#define ecrModeAllEnable        0x00E0 
//------------------------------------

#define PhyErrCounterReg    26
//----- Bits for reg -----------------
//------------------------------------

#define PhySpecialReg       27
//----- Bits for reg -----------------
#define ecrAutoRxTx             0x0000 
#define ecrSetRxToTx            0x2000 
#define estReversPolarity       0x0010 
//------------------------------------

#define PhySpecialCSReg     31
//----- Bits for reg -----------------
#define estAutoNegDone          0x1000 
#define estFullDuplexBit        0x0010 
#define estSpeed100MBit         0x0008 
#define estSpeed10MBit          0x0004 
//------------------------------------

#define IsSMIFree (!(ETH->MACMIIAR & ETH_MACMIIAR_MB))
#define IsSMIBusy (ETH->MACMIIAR & ETH_MACMIIAR_MB)


/*
//PHY regs
#define PHY_CR 0
#define PHY_SR 1
#define PHY_ID1 2
#define PHY_ID2 3
#define PHY_ANAR 4
#define PHY_ANLPAR 5
#define PHY_ANER 6
#define PHY_MCSR 17
#define PHY_SM 18
#define PHY_SECR 26
#define PHY_CSIR 27
#define PHY_ISR 29
#define PHY_IMR 30
#define PHY_SCSR 31

//PHY_CR bits
#define PHY_CR_SPEED 0x2000
#define PHY_CR_AUTONEGEN 0x1000
#define PHY_CR_PWRDN 0x0800
#define PHY_CR_RESTAUTONEG 0x0200
#define PHY_CR_DM 0x0100

//PHY_SR bits
#define PHY_SR_AUTONEGDONE 0x0020
#define PHY_SR_LINK 0x0004

//PHY_MCSR bits
#define PHY_MCSR_FARLOOPBACK 0x0200
*/

// Flags control bits
#define txcInDMA         0x80000000
#define txcIntrComplete  0x40000000
#define txcLastSegment   0x20000000
#define txcFirstSegment  0x10000000

#define txcDisableCRC    0x08000000
#define txcDisablePad    0x04000000
#define txcTimeStampEna  0x02000000

#define txcCheckSumNone  0x00000000
#define txcCheckSumIP    0x00400000
#define txcCheckSumIPPs  0x00800000
#define txcCheckSumFull  0x00C00000

#define txcEndOfRing     0x00200000
#define txcSecondDesc    0x00100000

#define txcNoPad         0x04000000

// Flags status bits
#define txsItsTimeStamp  0x00020000
#define txsErrIPHeader   0x00010000

#define txsWasError      0x00008000
#define txsErrJubber     0x00004000
#define txsErrFrameFlush 0x00002000
#define txsErrIPPayload  0x00001000
#define txsLossCarrier   0x00000800
#define txsErrNoCarrier  0x00000400
#define txsErrLateCols   0x00000200
#define txsErrExcesCols  0x00000100
#define txsErrDefferal   0x00000004
#define txsErrUnderFlow  0x00000002

#define txsItsVlanFrame  0x00000080

// Rx Flags bits

// Flags control bits
#define rxcInDMA         0x80000000

// Flags status bits
#define rxsDAFailed      0x40000000

#define rxsWasError      0x00008000
#define rxsErrDescriptor 0x00004000
#define rxsErrSAFailed   0x00002000
#define rxsErrLength     0x00001000
#define rxsErrOverFlow   0x00000800
#define rxsErrBigFrame   0x00000080
#define rxsErrIPHeader   0x00000080
#define rxsErrCollision  0x00000040
#define rxsErrTimeout    0x00000010
#define rxsErrReceive    0x00000008
#define rxsErrCRC        0x00000002
#define rxsErrCheckSum   0x00000001

#define rxsFrameType     0x00000020
#define rxcLastDesc      0x00000100
#define rxcFirstDesc     0x00000200
#define rxsItsVLAN       0x00000400

// Bits for DESC2 word
#define rxcIntrDisable   0x80000000
#define rxcEndOfRing     0x8000
#define rxcSecondDesc    0x4000

void InitNetReceive( void );

extern bool NetLink, NetPowerDown, NetSpeed100M, NetFullDuplex;
extern int LenPacket;

extern unsigned char SrcMAC[8];
extern unsigned char SrcIP[4];
extern short int SrcPort;

extern unsigned char HostMAC[8];
extern unsigned char HostIP[4];
extern short int HostPort;

void InitNet( void );
void InitPHY( bool reset );
bool CheckLink( void );
void NetToPowerDown( void );
void SMIWaitSetBit( char addr, unsigned short bit );
void SMIWaitResetBit( char addr, unsigned short bit );
//int  LockPacket( char *buf, int num, char *header, int sizeheader );
int  SendPacket( char *buf, int num, char *head, int sizehead );
int  ReadPacket( void );
bool SMIWrite( char addr, short int data );
short int SMIRead( char addr );

#endif