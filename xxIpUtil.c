#include <string.h>

#include "..\\inc\\stm32F4xx.h"

#pragma pack(1)

#include "xxProto.h"
#include "..\\..\\xxTcp.h"
#include "xxNet.h"

#define ntArp	0x0608
#define ntIp	0x0008

#define dhcpServerPort 0x4300
#define dhcpClientPort 0x4400
#define dhcpMagic 0x63825363
#define dhcpLeasing 84600
#define dhcpMaxlen 342

//char NetOutBuffer[1200], *NetOutData;
//TMessageHeaderEx *MH;



int Swap( unsigned int x )
{
    (x)^=((x)<<8);(x)^=((x)>>8);(x)^=((x)<<8);
    return x;
}

TMessageHeaderEx mh, *pmh;

int TypeOfMessage = 0;
int HaveAddress = -1;   // -1 no arp, =0 have arp no get art  =1 have all

//static int CountMessage = 0, CountARP = 0, CountCommand = 0;
static int LenPacket;
int MaxLenPacket = 128, LenOutNetData, LenReadFlashData;
short int *Pword;

//unsigned long sum,ChkSum;
short int Chsum;

static unsigned int count; //, LenData;
static unsigned char *Pchar;

void HandleICMPMessage( int len );
void HandleArpMessage( void );
void HandleDHCPMessage();
int RxFlag;

TIPHeader IP;
TUDPHeader UDP;

unsigned short int GetCheckSum( void *buf, int words )
{
unsigned short int *data = buf, ret;
int sum = 0;
    for( int i=0; i<words; i++ ) sum += data[i];
    ret = (sum & 0xFFFF );
    ret += ((sum>>16) & 0xFFFF);
    return ret;
}

int HandlerNetMessage( void )
{
//unsigned short int cs, ics, i;

    if( !FlagNetRcv ) return 0;
    FlagNetRcv = 0;
    LenPacket = ReadPacket();
    if (LenPacket > 1580 || LenPacket <= 0 ) {
        InitNetReceive();
        return 0;
    }
    ni.Packets++;
    ni.LenPacket = LenPacket;
    Pchar = (unsigned char *)(&mh);
    if( (NetInBuffer[12] == 0x08) && (NetInBuffer[13] == 0x06) ) { 
        ni.ArpPackets++;
        HandleArpMessage();
        //FlagFromArp = 1;  
        HaveAddress = 1;
        return 1;
    }
    if( (NetInBuffer[12] == 0x08) && (NetInBuffer[13] == 0x00) && (NetInBuffer[23] == 0x01) ) { 
        ni.IcmpPackets++; 
        HandleICMPMessage( LenPacket ); return 1; 
    }    // ICMP
    if( (NetInBuffer[12] == 0x08) && (NetInBuffer[13] == 0x00) && (NetInBuffer[23] == 0x11) ) {     // UDP
	pmh = (TMessageHeaderEx *)NetInBuffer;
	for( count=0; count<sizeof( TMessageHeaderEx ); count++ ) Pchar[count] = NetInBuffer[count];
        if(mh.udp.rPort==dhcpServerPort)
        {
          HandleDHCPMessage();
        }
        else
        {
          ni.UdpPackets++; 
          //memcpy( HostMAC, &ni.Param[4], 6 );
          //memcpy( HostIP, &ni.Param[7], 4 );
          ni.Param[9] = mh.udp.sPort;
          ni.Param[10] = mh.udp.rPort;
          if( pmh->udp.rPort != SrcPort ) {
              InitNetReceive();
              return 0; 
          }
          HaveAddress = 1;
          // if( ni.UdpPackets == 1 ) FlagFromArp = 1;
          for( count=0; count<sizeof( TMessageHeaderEx ); count++ ) Pchar[count] = NetInBuffer[count];
          for( count=0; count<6; count++ ) HostMAC[count] = pmh->ss[count];
          for( count=0; count<4; count++ ) HostIP[count] = pmh->ip.sIP[count];
          NetRcvData = NetInBuffer + sizeof( TMessageHeaderEx );
          Pchar = (unsigned char *)(mh.Command);
          Swapb( Pchar[0], Pchar[1] );
          Swapb( Pchar[2], Pchar[3] );
          From485 = mh.udp.Length;
          From485 = 0;
/*	
        memcpy( &IP, &pmh->ip, sizeof( TIPHeader ) );
	ics = IP.CheckSum;
	IP.CheckSum = 0;
	
	cs = GetCheckSum( &IP, 10 );
	cs = ~cs;

        memcpy( &UDP, &pmh->udp, sizeof( TUDPHeader ) );
	ics = UDP.CheckSum;
	UDP.CheckSum = 0;
    Pchar = (char *)(&UDP);
    Swapb( Pchar[0], Pchar[1] );
    Swapb( Pchar[2], Pchar[3] );
    Swapb( Pchar[4], Pchar[5] );
	cs = GetCheckSum( &UDP, 3 );
	cs = ~cs;
*/
          EncodeCommand();
          return 1;
        }
    }
    InitNetReceive();
    return 0;
}
/*
    char rs[6];
    char ss[6];
    short int type;
    // TIPHeader ip;		//20
    short int VersAndLen;	// 14
    short int Length;		// 16
    short int Ident;		// 18
    short int Status;		// 20
    short int Protocol;		// 22
    short int CheckSum;		// 24
    char  sIP[4];		// 26
    char  rIP[4];		// 30
    //TUDPHeader udp;		
    short int sPort;		// 34
    short int rPort;		// 36
    unsigned short int Length;	// 38
    unsigned short int CheckSum;// 40
*/

void  PrepareIp( void )
{
int i;
//unsigned short int len;
    Pchar = (unsigned char *)(&mh);
    Pword = (short int *)(&mh);
    
    // Common part of header
    for( i=0; i<6; i++ ) mh.rs[i] = HostMAC[i];
    for( i=0; i<6; i++ ) mh.ss[i] = SrcMAC[i];
    mh.type = 0x0008;
    
    // IP header
    mh.ip.VersAndLen = 0x0045;
    //len = ( LenOutNetData + sizeof( TUDPHeader ) + sizeof( TIPHeader ) + sizeof( TMessageHeader ) );
    mh.ip.Length = ( LenOutNetData + sizeof( TUDPHeader ) + sizeof( TIPHeader ) + sizeof( TMessageHeader ) );
    Pchar = (unsigned char *)(&mh.ip.Length);
    Swapb( Pchar[0], Pchar[1] );
    //Swapb( Pchar[2], Pchar[3] );
    mh.ip.Ident = 0;
    mh.ip.Status = 0;
    mh.ip.Protocol = 0x1180;
    mh.ip.CheckSum = 0;
    for( i=0; i<4; i++ ) mh.ip.sIP[i] = SrcIP[i];
    for( i=0; i<4; i++ ) mh.ip.rIP[i] = HostIP[i];
    
    mh.LenData = LenOutNetData;
    mh.ip.CheckSum = ~GetCheckSum( &(mh.ip), 10 );

    // UDP Header
    mh.udp.sPort = SrcPort;
    mh.udp.rPort = HostPort;
//    if( LenOutNetData < 196 )
        mh.udp.Length = ( LenOutNetData + sizeof( TUDPHeader ) + sizeof( TMessageHeader ) );
    Pchar = (unsigned char *)(&mh.udp.Length);
    Swapb( Pchar[0], Pchar[1] );
//    Swapb( Pchar[2], Pchar[3] );
//    else 
//        mh.udp.Length = 0x2800;
    
    mh.udp.CheckSum = 0;    
    mh.LenData = LenOutNetData;
}

int SendUdpMessage( void *message )
{
    PrepareIp();
    if( From485 ) Send485Packet( (char *)message, LenOutNetData, (char *)(&mh), sizeof( TMessageHeaderEx ) );
    else SendPacket( (char *)message, LenOutNetData, (char *)(&mh), sizeof( TMessageHeaderEx ) );
    return 1;
}

/*
int SendUdpMessage( void *message )
{
    PrepareIp();
    if( From485 ) Send485Packet( (char *)message, LenOutNetData, (char *)(&mh), sizeof( TMessageHeaderEx ) );
    else SendPacket( (char *)message, LenOutNetData + sizeof( TMessageHeaderEx ) );
    return 1;
}
*/

static short int Unical = 1;

int SendDataEx( void *Data, int size,
    int command, int commandEx,
    int param1, int param2 )
{
int len;
char *data;

    data = (char *)Data;
    mh.Command[0] = command;
    mh.Command[1] = commandEx;
    Pchar = (unsigned char *)(mh.Command);
    Swapb( Pchar[0], Pchar[1] );
    Swapb( Pchar[2], Pchar[3] );
    mh.FrameNumber = 0;
    mh.LParam[0] = param1;
    mh.LParam[1] = param2;
    mh.Status &= (~psItemDone); 
    mh.mbRead = size;
    mh.UnicalNumber = 0;
/*
    if( From485 ) {
        LenOutNetData = size;
        mh.Status = psItemDone;
        SendUdpMessage( data );
        goto ret;
    }
*/
    if( size > MaxLenPacket ) mh.UnicalNumber = Unical++;
    mh.Status |= psFirstItem;
    do {
        if( size > MaxLenPacket ) len = MaxLenPacket;
	else len = size;
        LenOutNetData = len;
        if( (size-len) <= 0 ) mh.Status |= psItemDone;
        SendUdpMessage( data );
        size -= len;
        data += len;
        mh.FrameNumber++;
        mh.Status |= psMiddleItem;
    } while( size > 0 );

//ret:
    mh.Status = 0;
    mh.UnicalNumber++;
    return 0;
}

int ReplayData( void *Data, int size )
{
int len;
char *data;

    data = (char *)Data;
    Pchar = (unsigned char *)(mh.Command);
    Swapb( Pchar[0], Pchar[1] );
    Swapb( Pchar[2], Pchar[3] );
    mh.FrameNumber = 0;
    mh.Status &= (~psItemDone); 
    mh.mbRead = size;
    mh.Status |= psFirstItem;
    do {
        if( size > MaxLenPacket ) len = MaxLenPacket;
	else len = size;
        LenOutNetData = len;
        if( (size-len) <= 0 ) mh.Status |= psItemDone;
        SendUdpMessage( data );
        size -= len;
        data += len;
        mh.FrameNumber++;
        mh.Status |= psMiddleItem;
    } while( size > 0 );
    mh.Status = 0;
    return 0;
}
/*
int SendDataEx( void *Data, int size,
    int command, int commandEx,
    int param1, int param2 )
{
int len;
char *data;

    MH = (TMessageHeaderEx *)NetOutBuffer;
    NetOutData = NetOutBuffer + sizeof( TMessageHeaderEx );
    
    data = (char *)Data;

    MH->Command[0] = command;
    MH->Command[1] = commandEx;
    Pchar = (char *)(MH->Command);
    Swapb( Pchar[0], Pchar[1] );
    Swapb( Pchar[2], Pchar[3] );
    MH->FrameNumber = 0;
    MH->LParam[0] = param1;
    MH->LParam[1] = param2;
    MH->Status &= (~psItemDone); 
    MH->Status = 0;
    MH->mbRead = size;
    MH->UnicalNumber = 0;

    if( From485 ) {
        LenOutNetData = size;
        mh.Status = psItemDone;
        SendUdpMessage( data );
        goto ret;
    }

    if( size > MaxLenPacket ) MH->UnicalNumber = Unical++;
    MH->Status |= psFirstItem;
    do {
        if( size > MaxLenPacket ) len = MaxLenPacket;
	else len = size;
        LenOutNetData = len;
        if( (size-len) <= 0 ) MH->Status |= psItemDone;
	memcpy( NetOutData, data, len );
        SendUdpMessage( NetOutBuffer );
        size -= len;
        data += len;
        MH->FrameNumber++;
        MH->Status |= psMiddleItem;
    } while( size > 0 );

//ret:
    MH->Status = 0;
    MH->UnicalNumber++;
    return 0;
}

int ReplayData( void *Data, int size )
{
int len;
char *data;

    MH = (TMessageHeaderEx *)NetOutBuffer;
    NetOutData = NetOutBuffer + sizeof( TMessageHeaderEx );
    memcpy( MH, &mh, sizeof(TMessageHeaderEx) );
    data = (char *)Data;
    Pchar = (char *)(MH->Command);
    Swapb( Pchar[0], Pchar[1] );
    Swapb( Pchar[2], Pchar[3] );
    MH->FrameNumber = 0;
    MH->Status &= (~psItemDone); 
    MH->mbRead = size;
    MH->Status |= psFirstItem;
    do {
        if( size > MaxLenPacket ) len = MaxLenPacket;
	else len = size;
        LenOutNetData = len;
        if( (size-len) <= 0 ) MH->Status |= psItemDone;
	memcpy( NetOutData, data, len );
        SendUdpMessage( NetOutBuffer );
        size -= len;
        data += len;
        MH->FrameNumber++;
        MH->Status |= psMiddleItem;
    } while( size > 0 );
    MH->Status = 0;
    return 0;
}
*/

/*
void RunSendPageLt( void ) 
{
int i, k, porcia;

    porcia = tp.SizePage/MaxLenPacket;
    mh.FrameNumber = 0;
    LenReadFlashData = MaxLenPacket;
    LenOutNetData = MaxLenPacket;
    PrepareIp();
    mh.Status = psFirstItem; 
    FlashBlock = mh.Param[0];
    FlashPage = mh.Param[1];
    for( i=0; i<mh.Param[2]; i++ ) {
      mh.Status |= ReadDataInit();
      SendPacketInit();
      ReadFlashWriteNet();
      for( k=1; k<porcia; k++ ) {
          mh.FrameNumber++;
          mh.Status |= psMiddleItem;
          if( (i == (mh.Param[2]-1)) && (k == (porcia-1)) ) mh.Status |= (psLastItem|psMiddleItem);
          SendPacketInit();
          ReadFlashWriteNet();
      }
      SendPacketDone();
      ReadDataDone();
//      if( FlagReadGo ) HandlerNets();      
//      if( FlagCancel ) return;
      FlashPage++;
      if( FlashPage >= tt.NumPages ) {
          FlashBlock++;
          FlashPage=0;
      }
    }
}
*/

static  char *RxBuf, *TxBuf;

void HandleArpMessage( void )
{
unsigned int i;

TxBuf = &(NetInBuffer[0x80]); RxBuf = NetInBuffer;
    
    for( i=0; i<6; i++ ) HostMAC[i] = RxBuf[i+22];
    //for( i=0; i<4; i++ ) HostIP[i] = RxBuf[i+28];   
    HaveAddress = 0;
    
	for (i=0;i<60;i++) TxBuf[i] = 0;
	for( i=0; i<6; i++ ) {
	    TxBuf[i] = RxBuf[i+6];
	    TxBuf[i+6] = SrcMAC[i];
	    TxBuf[i+22] = SrcMAC[i];
            TxBuf[i+32] = RxBuf[i+22];
	}
    TxBuf[12] = 0x08;
    TxBuf[13] = 0x06;
    TxBuf[14] = 0x00;
    TxBuf[15] = 0x01;
    TxBuf[16] = 0x08;
    TxBuf[17] = 0x00;
    TxBuf[18] = 0x06;
    TxBuf[19] = 0x04;
    TxBuf[20] = 0x00;
    TxBuf[21] = 0x02;

    for( i=0; i<4; i++ ) {
        TxBuf[i+28] = SrcIP[i];
        TxBuf[i+38] = RxBuf[i+28];
    }
    SendPacket( TxBuf+60, 2, TxBuf, 60 );
    InitNetReceive();
    //ni.ArpPackets++; 
    return;
}

void HandleDHCPMessage()
{
  TDHCPMessage dhcp;
  uint16_t endOfOptions=0,
           offset=0, i=0;
  uint32_t magic=0;
  
  if(!HaveAddress)
  {
    for( i=0; i<6; i++ ) HostMAC[i] = NetInBuffer[i+6];
    HaveAddress=1;
  }
  
  dhcp.oper=NetInBuffer[42];
  dhcp.xid=(NetInBuffer[46]<<24)|(NetInBuffer[47]<<16)|
           (NetInBuffer[48]<<8)|NetInBuffer[49];
  dhcp.yiaddr=(NetInBuffer[58]<<24)|(NetInBuffer[59]<<16)|
              (NetInBuffer[60]<<8)|NetInBuffer[61];
  dhcp.chaddr[0]=(NetInBuffer[70]<<8)|NetInBuffer[71];
  dhcp.chaddr[1]=(NetInBuffer[72]<<24)|(NetInBuffer[73]<<16)|
                 (NetInBuffer[74]<<8)|NetInBuffer[75];
  dhcp.dhcptype=0;
  dhcp.dnsIP=0;
  dhcp.leasing=0;
  dhcp.mask=0;
  dhcp.requestedIP=0;
  dhcp.routerIP=0;
  dhcp.serverIP=0;
  magic=(NetInBuffer[278]<<24)|(NetInBuffer[279]<<16)|
        (NetInBuffer[280]<<8)|NetInBuffer[281];
  if(magic==dhcpMagic) {
    offset=282;
    do {
      switch(NetInBuffer[offset]) {
      case 0x01: {
          offset+=2;
          dhcp.mask=(NetInBuffer[offset]<<24)|(NetInBuffer[offset+1]<<16)|
                    (NetInBuffer[offset+2]<<8)|NetInBuffer[offset+3];
          offset+=4;
          break;
        }
      case 0x03: {
          offset+=2;
          dhcp.routerIP=(NetInBuffer[offset]<<24)|(NetInBuffer[offset+1]<<16)|
                        (NetInBuffer[offset+2]<<8)|NetInBuffer[offset+3];
          offset+=4;
          break;
        }
      case 0x06: {
          offset+=2;
          dhcp.dnsIP=(NetInBuffer[offset]<<24)|(NetInBuffer[offset+1]<<16)|
                     (NetInBuffer[offset+2]<<8)|NetInBuffer[offset+3];
          offset+=4;
          break;
        }
      case 0x32: {
          offset+=2;
          dhcp.requestedIP=(NetInBuffer[offset]<<24)|(NetInBuffer[offset+1]<<16)|
                           (NetInBuffer[offset+2]<<8)|NetInBuffer[offset+3];
          offset+=4;
          break;
        }
      case 0x33: {
          offset+=2;
          dhcp.leasing=(NetInBuffer[offset]<<24)|(NetInBuffer[offset+1]<<16)|
                       (NetInBuffer[offset+2]<<8)|NetInBuffer[offset+3];
          offset+=4;
          break;
        }
      case 0x35: {
          offset+=2;
          dhcp.dhcptype=NetInBuffer[offset];
          offset++;
          break;
        }
      case 0x36: {
          offset+=2;
          dhcp.serverIP=(NetInBuffer[offset]<<24)|(NetInBuffer[offset+1]<<16)|
                        (NetInBuffer[offset+2]<<8)|NetInBuffer[offset+3];
          offset+=4;
          break;
        }
      case 0xff: {
          endOfOptions=1;
          break;
        }
      default: {
          offset++;
          offset+=NetInBuffer[offset]+1;
          break;
        }
      }
    }
    while(!endOfOptions && (offset<dhcpMaxlen));
  }
  if((dhcp.oper==1)&&((dhcp.dhcptype==1)||(dhcp.dhcptype==3))) {
    dhcp.oper=2;
    dhcp.yiaddr=(HostIP[0]<<24)|(HostIP[1]<<16)|(HostIP[2]<<8)|HostIP[3];
    dhcp.requestedIP=0;
    dhcp.dhcptype=(dhcp.dhcptype==1 ? 2 : 5);
    dhcp.mask=0xffffffe0;
    dhcp.routerIP=0;//NET_TOOL_IP;
    dhcp.serverIP=(SrcIP[0]<<24)|(SrcIP[1]<<16)|(SrcIP[2]<<8)|SrcIP[3];
    dhcp.leasing=dhcpLeasing;
    dhcp.dnsIP=0;//NET_TOOL_IP;
    
    for(i=0;i<6;i++){ 
      NetInBuffer[i]=0xff;
      NetInBuffer[i+6]=SrcMAC[i];
    }
    NetInBuffer[12]=0x08; //ethtype
    NetInBuffer[13]=0x00;
    NetInBuffer[14]=0x45; //vers&hlen
    NetInBuffer[15]=0x00;
    NetInBuffer[16]=0x01; //payload len
    NetInBuffer[17]=0x48;
    NetInBuffer[18]=0x00; //id
    NetInBuffer[19]=0x00; 
    NetInBuffer[20]=0x00; //flags
    NetInBuffer[21]=0x00; //offset
    NetInBuffer[22]=0x80; //TTL
    NetInBuffer[23]=0x11; //UDP proto
    NetInBuffer[24]=0x00; //HCS
    NetInBuffer[25]=0x00;
    for(i=0;i<4;i++) {
      NetInBuffer[i+26]=SrcIP[i];
      NetInBuffer[i+30]=0xff;
    }
    NetInBuffer[34]=dhcpServerPort&0xff;
    NetInBuffer[35]=(dhcpServerPort>>8)&0xff;
    NetInBuffer[36]=dhcpClientPort&0xff;
    NetInBuffer[37]=(dhcpClientPort>>8)&0xff;
    NetInBuffer[38]=0x01; //payload len
    NetInBuffer[39]=0x34;
    NetInBuffer[40]=0x00; //checksum
    NetInBuffer[41]=0x00;
  
    NetInBuffer[42]=dhcp.oper; //type
    NetInBuffer[43]=0x01; //HYPE
    NetInBuffer[44]=0x06; //HLEN
    NetInBuffer[45]=0x00; //hops
    NetInBuffer[46]=(dhcp.xid>>24) & 0xff;
    NetInBuffer[47]=(dhcp.xid>>16) & 0xff;
    NetInBuffer[48]=(dhcp.xid>>8)  & 0xff;
    NetInBuffer[49]=(dhcp.xid)     & 0xff;
    NetInBuffer[50]=0x00; //secs
    NetInBuffer[51]=0x00;
    NetInBuffer[52]=0x80; //broadcast flag
    NetInBuffer[53]=0x00;
    NetInBuffer[54]=0x00; //ciaddr
    NetInBuffer[55]=0x00;
    NetInBuffer[56]=0x00;
    NetInBuffer[57]=0x00;
    NetInBuffer[58]=(dhcp.yiaddr>>24) & 0xff; //yiaddr
    NetInBuffer[59]=(dhcp.yiaddr>>16) & 0xff;
    NetInBuffer[60]=(dhcp.yiaddr>>8)  & 0xff;
    NetInBuffer[61]=(dhcp.yiaddr)     & 0xff;
    NetInBuffer[62]=0x00; //siaddr
    NetInBuffer[63]=0x00;
    NetInBuffer[64]=0x00;
    NetInBuffer[65]=0x00;
    NetInBuffer[66]=0x00; //giaddr
    NetInBuffer[67]=0x00;
    NetInBuffer[68]=0x00;
    NetInBuffer[69]=0x00;
    NetInBuffer[70]=(dhcp.chaddr[0]>>8) & 0xff;
    NetInBuffer[71]=(dhcp.chaddr[0])    & 0xff;
    NetInBuffer[72]=(dhcp.chaddr[1]>>24)& 0xff;
    NetInBuffer[73]=(dhcp.chaddr[1]>>16)& 0xff;
    NetInBuffer[74]=(dhcp.chaddr[1]>>8) & 0xff;
    NetInBuffer[75]=(dhcp.chaddr[1])    & 0xff;
    for(i=76;i<278;i++)
    {
      NetInBuffer[i]=0x00;
    }
    NetInBuffer[278]=(dhcpMagic>>24) & 0xff;
    NetInBuffer[279]=(dhcpMagic>>16) & 0xff;
    NetInBuffer[280]=(dhcpMagic>>8)  & 0xff;
    NetInBuffer[281]=(dhcpMagic)     & 0xff;
  
    //dhcp options
    NetInBuffer[282]=0x35;
    NetInBuffer[283]=0x01;
    NetInBuffer[284]=dhcp.dhcptype;
    offset=285;
    if(dhcp.mask!=0)
    {
      NetInBuffer[offset++]=0x01;
      NetInBuffer[offset++]=0x04;
      NetInBuffer[offset++]=(dhcp.mask>>24) & 0xff;
      NetInBuffer[offset++]=(dhcp.mask>>16) & 0xff;
      NetInBuffer[offset++]=(dhcp.mask>>8)  & 0xff;
      NetInBuffer[offset++]=(dhcp.mask)     & 0xff;
    }
    if(dhcp.serverIP!=0)
    {
      NetInBuffer[offset++]=0x36;
      NetInBuffer[offset++]=0x04;
      NetInBuffer[offset++]=(dhcp.serverIP>>24) & 0xff;
      NetInBuffer[offset++]=(dhcp.serverIP>>16) & 0xff;
      NetInBuffer[offset++]=(dhcp.serverIP>>8)  & 0xff;
      NetInBuffer[offset++]=(dhcp.serverIP)     & 0xff;
    }
    if(dhcp.leasing!=0)
    {
      NetInBuffer[offset++]=0x33;
      NetInBuffer[offset++]=0x04;
      NetInBuffer[offset++]=(dhcp.leasing>>24) & 0xff;
      NetInBuffer[offset++]=(dhcp.leasing>>16) & 0xff;
      NetInBuffer[offset++]=(dhcp.leasing>>8)  & 0xff;
      NetInBuffer[offset++]=(dhcp.leasing)     & 0xff;    
    }
    if(dhcp.routerIP!=0)
    {
      NetInBuffer[offset++]=0x03;
      NetInBuffer[offset++]=0x04;
      NetInBuffer[offset++]=(dhcp.routerIP>>24) & 0xff;
      NetInBuffer[offset++]=(dhcp.routerIP>>16) & 0xff;
      NetInBuffer[offset++]=(dhcp.routerIP>>8)  & 0xff;
      NetInBuffer[offset++]=(dhcp.routerIP)     & 0xff;    
    }
    if(dhcp.dnsIP!=0)
    {
      NetInBuffer[offset++]=0x06;
      NetInBuffer[offset++]=0x04;
      NetInBuffer[offset++]=(dhcp.dnsIP>>24) & 0xff;
      NetInBuffer[offset++]=(dhcp.dnsIP>>16) & 0xff;
      NetInBuffer[offset++]=(dhcp.dnsIP>>8)  & 0xff;
      NetInBuffer[offset++]=(dhcp.dnsIP)     & 0xff;    
    }
    NetInBuffer[offset++]=0xff;
    for(i=offset;i<342;i++)
    {
      NetInBuffer[i]=0x00;
    }      
    SendPacket( NetInBuffer+42, 300, NetInBuffer, 42 );
    InitNetReceive();
  }
}

void AskArp( void )
{
unsigned int i;

    TxBuf = NetInBuffer;
    
    for( i=0; i<6; i++ ) TxBuf[i] = 0xFF;
    for( i=6; i<12; i++ ) TxBuf[i] = SrcMAC[i];
    TxBuf[12] = 0x08;
    TxBuf[13] = 0x06;
    TxBuf[14] = 0x00;
    TxBuf[15] = 0x01;
    TxBuf[16] = 0x08;
    TxBuf[17] = 0x00;
    TxBuf[18] = 0x06;
    TxBuf[19] = 0x04;
    TxBuf[20] = 0x00;
    TxBuf[21] = 0x01;
    for( i=22; i<28; i++ ) TxBuf[i] = SrcMAC[i];
    for( i=28; i<32; i++ ) TxBuf[i] = SrcIP[i];
    for( i=32; i<38; i++ ) TxBuf[i] = 0xFF;
    for( i=38; i<42; i++ ) TxBuf[i] = 0xFF;

    SendPacket( TxBuf, 60, TxBuf, 60 );
    return;
}

unsigned char PacketType (unsigned char *Ptr)
{
	unsigned char result;
	if (*(Ptr+12)==0x08 && *(Ptr+13)==0x06)	result = 1;
	if (*(Ptr+12)==0x08 && *(Ptr+13)==0x00 && *(Ptr+23)==0x11)	result = 2;
	if (*(Ptr+12)==0x08 && *(Ptr+13)==0x00 && *(Ptr+23)==0x01)	result = 3;
	return (result);
}


void HandleICMPMessage( int RxLength )
{
unsigned int i;

unsigned long sum,ChkSum;
unsigned int csum, temp;

        TxBuf = NetInBuffer + RxLength + 2; RxBuf = NetInBuffer;
	for (i=0; i<RxLength; i++) TxBuf[i] = RxBuf[i];

//------------------------------------------------------------------
	
	for (i=0;i<6;i++){
	    TxBuf[i]= RxBuf[i+6]; 
	    TxBuf[i+6] = SrcMAC[i];
	}
	TxBuf[24] = 0x00;		
	TxBuf[25] = 0x00;
	TxBuf[34] = 0x00;
	TxBuf[36] = 0x00;		
	TxBuf[37] = 0x00;		
	for (i=0;i<4;i++) {
	    TxBuf[i+30] = RxBuf[i+26]; 
	    TxBuf[i+26] = SrcIP[i];	
	}

//------------------------------------------------------------------
	TxBuf[24] = 0x00;		
	TxBuf[25] = 0x00;		
	sum=0;
	for (i=0;i<10;i++) { sum=sum+256*TxBuf[15+(i<<1)] + TxBuf[14+(i<<1)]; }
	
	ChkSum=sum&0xFFFF;
	ChkSum=ChkSum+(sum>>16);
	csum=(int)ChkSum;
	csum=~csum;
	TxBuf[24] = csum&0xFF;
	TxBuf[25] = (csum>>8)&0xFF;

//------------------------------------------------------------------

	TxBuf[36] = 0x00;		
	TxBuf[37] = 0x00;		
	sum=0;
	temp = ((RxLength-34)>>1);
	for (i=0;i<temp;i++) { sum=sum+256*TxBuf[35+(i<<1)] + TxBuf[34+(i<<1)]; }
	if( (RxLength&1)!=0 ) sum += (TxBuf[RxLength-1]<<8);
	
	ChkSum=sum&0xFFFF;
	ChkSum=ChkSum+(sum>>16);
	csum=(int)ChkSum;
	csum=~csum;
	TxBuf[36] = csum&0xFF;
	TxBuf[37] = (csum>>8)&0xFF;

//------------------------------------------------------------------
    SendPacket( TxBuf, RxLength, TxBuf, 0 );
    InitNetReceive();
}


int HandleIpMessage( void );
int HandleCommand(void);
int SendUdpMessage( void *message );
int SendDataEx( void *Data, int size,
    int command, int address,
    int param1, int param2 );

#include "xxCustom.h"

static int FlashBlock;

void RunSendPageLt( void ) 
{
int i, k, porcia;
char *outdata = FlashInBuffer;

    porcia = tp.SizePage/MaxLenPacket;
    LenOutNetData = MaxLenPacket;

    Pchar = (unsigned char *)(mh.Command);
    Swapb( Pchar[0], Pchar[1] );
    Swapb( Pchar[2], Pchar[3] );
    mh.FrameNumber = 0;
    mh.Status &= (~psItemDone); 
    mh.Status &= (~feError); 
    mh.mbRead = mh.WParam[0] * 512;
    mh.UnicalNumber++;
    mh.Status |= psFirstItem;     

    FlashBlock = mh.LParam[0];
    for( i=0; i<mh.WParam[0]; i++ ) {
        mh.Status |= ReadPage( FlashBlock, 512, FlashInBuffer );
        outdata = FlashInBuffer;
        for( k=0; k<porcia; k++ ) {
            SendUdpMessage( outdata );
            outdata += MaxLenPacket;
            mh.FrameNumber++;
            mh.Status |= psMiddleItem;
            if( (i == (mh.WParam[0]-1)) && (k == (porcia-2)) ) 
              mh.Status |= psItemDone;
        }
        if( FlashBlock < NUMBLOCKS ) {
            FlashBlock++;
        }   
    }
}

void RunSendPageGt( void ) 
{
int i;

    LenOutNetData = MaxLenPacket;
    Pchar = (unsigned char *)(mh.Command);
    Swapb( Pchar[0], Pchar[1] );
    Swapb( Pchar[2], Pchar[3] );
    mh.FrameNumber = 0;
    mh.Status &= (~psItemDone); 
    mh.Status &= (~feError); 
    mh.mbRead = mh.WParam[0] * 512;
    mh.UnicalNumber++;
    mh.Status |= psFirstItem;     

    FlashBlock = mh.LParam[0];
    i = 0;
    while( 1 ) {
        mh.Status |= ReadPage( FlashBlock, 512, FlashInBuffer );
        LenOutNetData = 512;  
        if( i == (mh.WParam[0]-1) ) {
	    mh.Status |= psItemDone;
            SendUdpMessage( FlashInBuffer );
	    return;
	}
	i++;
        if( FlashBlock < NUMBLOCKS ) FlashBlock++;
        LenOutNetData = 1024;  
        if( FlashBlock < NUMBLOCKS ) FlashBlock++;
        mh.Status |= ReadPage( FlashBlock, 512, FlashInBuffer+512 );
        if( i == (mh.WParam[0]-1) ) {
	    mh.Status |= psItemDone;
            SendUdpMessage( FlashInBuffer );
	    return;
	}
        i++;
        SendUdpMessage( FlashInBuffer );
	mh.FrameNumber++;
        mh.Status |= psMiddleItem;
        if( FlashBlock < NUMBLOCKS ) {
            FlashBlock++;
        }
    }
}


