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
#include <PiPei.h>
#include <Library/DebugLib.h>
#include <Library/PcdLib.h>
#include <Library/IoLib.h>
#include <Library/CacheMaintenanceLib.h>

#include <PlatformArch.h>
#include <Library/PlatformSysCtrlLib.h>

#include <Library/OemMiscLib.h>

#define TIMER_SUBCTRL_BASE   PcdGet64(PcdPeriSubctrlAddress)
#define SC_TM_CLKEN0_REG     (0x2050)

VOID PlatformTimerStart (VOID)
{
    // Timer0 clock enable
    MmioWrite32 (TIMER_SUBCTRL_BASE + SC_TM_CLKEN0_REG, 0x3);
}

EFI_STATUS
EFIAPI
EarlyConfigEntry (
  IN       EFI_PEI_FILE_HANDLE  FileHandle,
  IN CONST EFI_PEI_SERVICES     **PeiServices
  )
{
    DEBUG((EFI_D_ERROR,"SMMU CONFIG........."));
    SmmuConfigForLinux();
    DEBUG((EFI_D_ERROR,"Done\n"));

    DEBUG((EFI_D_ERROR,"ITS CONFIG........."));
	ITSCONFIG();
    DEBUG((EFI_D_ERROR,"Done\n"));

    DEBUG((EFI_D_ERROR,"AP CONFIG........."));
    MmioWrite64(FixedPcdGet64(PcdMailBoxAddress), 0x0);
    (void)WriteBackInvalidateDataCacheRange((VOID *) FixedPcdGet64(PcdMailBoxAddress), 8);
    asm("DSB SY");
    asm("ISB");

    CoreSelectBoot();

DEBUG((EFI_D_ERROR,"Done\n"));

    DEBUG((EFI_D_ERROR,"MN CONFIG........."));
    MN_CONFIG ();
    DEBUG((EFI_D_ERROR,"Done\n"));

    DEBUG((EFI_D_ERROR,"RTC CONFIG........."));
    
    MmioWrite32(0xA00021F0, 0xF);

    DEBUG((EFI_D_ERROR,"Done\n"));

    DEBUG((EFI_D_ERROR,"Tsensor CONFIG........."));
    
    MmioWrite32(0x80010000 + 0x5000, 0x1);
    *(volatile UINT32*)0xA0000A8C = 0x1f;

    DEBUG((EFI_D_ERROR,"Done\n"));

    DEBUG((EFI_D_ERROR,"Timer CONFIG........."));
    PlatformTimerStart ();
    DEBUG((EFI_D_ERROR,"Done\n"));

    return EFI_SUCCESS;
}
