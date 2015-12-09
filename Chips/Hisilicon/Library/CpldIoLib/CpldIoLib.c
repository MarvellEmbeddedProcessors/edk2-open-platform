/** @file
*
*  Copyright (c) 2015, Hisilicon Limited. All rights reserved.
*  Copyright (c) 2015, Linaro Limited. All rights reserved.
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

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
