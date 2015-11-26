#include <Uefi.h>
#include <Library/CpldIoLib.h>
#include <Library/DebugLib.h>
#include <Library/TimerLib.h>
#include <PlatformArch.h>
#include <Library/OemAddressMapLib.h>
#include <Library/OemMiscLib.h>


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


