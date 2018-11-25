#include "common.h"

#define NFCONF               (*(volatile unsigned long *) 0x4E000000)
#define NFCONT               (*(volatile unsigned long *) 0x4E000004)
#define NFCMD                (*(volatile unsigned long *) 0x4E000008)
#define NFADDR               (*(volatile unsigned long *) 0x4E00000C)
#define NFDATA               (*(volatile unsigned long *) 0x4E000010)
#define NFSTAT               (*(volatile unsigned long *) 0x4E000020)

#define READ_1ST        0x00
#define READ_2ND        0x30
#define RESET           0xFF

#define NFPageSize      2048
#define NFOBBSize       64
#define NFBlockSize     64
#define NFDevSize       2048

//-------------------------------------------------------------------
inline static void DevON( void ){ NFCONT &= ~0x2; }
inline static void DevOFF( void ){ NFCONT |= 0x2; }
inline static void WriteCmd( unsigned char Cmd ){ NFCMD = Cmd; }
inline static void WiteForDevReady( void ){ while( !( NFSTAT & 1 ) ); }

inline static void ResetDev( void )
{
    DevON();
    WriteCmd( RESET );
    WiteForDevReady();
    DevOFF();
}

inline static void WritePageAddress( unsigned int iBlock, unsigned int iPage )
{
    unsigned int iPageAddr = iBlock * NFBlockSize + iPage ;

    NFADDR = 0x00;
    NFADDR = 0x00;

    NFADDR = iPageAddr & 0xFF;
    NFADDR = ( iPageAddr >> 8 ) & 0xFF;
    NFADDR = ( iPageAddr >> 16 ) & 1;
}
//---------------------------------------------------------------------------------------------------------
void NF_InitDev( void )
{
    unsigned long iNFCONF;
	
    iNFCONF = NFCONF;
    iNFCONF &= 0x000F;
    iNFCONF |= ( 3 << 12 ) | ( 7 << 8 ) | ( 7 << 4 );
    NFCONF = iNFCONF;
    NFCONT = ( 1 <<  4 ) | ( 1 << 1 ) | ( 1 << 0 ) ;
	
    ResetDev();
    
    BOOT_FLAG = 1;
}
//-----------------------------------------------------------------------------------------------------------
static void NF_ReadPage( unsigned int iBlock, unsigned int iPage, unsigned long iTargeAddr )
{
    unsigned int i = 0;
    unsigned long *pTargeAddr = (unsigned long *)iTargeAddr;
    
    WriteCmd( READ_1ST );
    WritePageAddress( iBlock,  iPage );
    WriteCmd( READ_2ND );
    WiteForDevReady();

    for( i = 0 ; i < NFPageSize / 4 ; i++ )
    {
        pTargeAddr[i] = NFDATA;
    }
}
void NF_Read( unsigned int iBlock, unsigned int iPage, unsigned long iTargeAddr, int iSize )
{
    int i;
    DevON();
    for( i=0; i<iSize; i++ )
    {
        NF_ReadPage( iBlock, iPage, iTargeAddr );
        iTargeAddr += NFPageSize;
        if( NFBlockSize == ++iPage )
        {
            iPage = 0;
            iBlock++;
        }
    }
    DevOFF();
}
//-------------------------------------------------------------------------------------------------------------
void NF_CopyCode2SDRAM(void)
{
    NF_Read( 0, 0, CONFIG_SYS_TEXT_BASE, 128 );
}


