#include <Uefi.h>
#include <Library/CpldIoLib.h>
#include <Library/DebugLib.h>
#include <Library/TimerLib.h>


VOID WriteCpldReg(UINTN ulRegAddr, UINT8 ulValue)
{
    *(volatile UINT8 *)(ulRegAddr + PcdGet64(PcdCpldBaseAddress)) = ulValue;
}


UINT8 ReadCpldReg(UINTN ulRegAddr)
{    
    volatile UINT8 ulValue = 0;
    
    ulValue = *(volatile UINT8 *)(ulRegAddr + PcdGet64(PcdCpldBaseAddress));
    
    return (ulValue); 
}


VOID ReadCpldBytes(UINT16 Addr, UINT8 *Data, UINT8 Bytes)
{
    UINT8 i;

    for(i = 0;i < Bytes; i++)
    {
        *(Data + i) = ReadCpldReg(Addr + i);
    }
}

VOID WriteCpldBytes(UINT16 Addr, UINT8 *Data, UINT8 Bytes)
{
    UINT8 i;

    for(i = 0; i < Bytes; i++)
    {
        WriteCpldReg(Addr + i, *(Data + i));
    }
}
