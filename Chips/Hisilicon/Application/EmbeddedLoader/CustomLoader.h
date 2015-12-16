/** @file
*
*  Copyright (c) 2011-2015, ARM Limited. All rights reserved.
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
#include <Library/IoLib.h>
#include <Library/UefiLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/ArmLib.h>
#include <Guid/Fdt.h>
#include <Chipset/ArmCortexA5x.h>

#include <PlatformArch.h>
#include <Library/PlatformSysCtrlLib.h>
#include <Library/TimerLib.h>
#include <Library/OemAddressMapLib.h>
#include <Library/FdtUpdateLib.h>
#include <Library/LzmaCustomDecompressLib/LzmaDecompressLibInternal.h>

typedef VOID (*ESL_LINUX)(UINTN ParametersBase, UINTN Reserved0,
                          UINTN Reserved1, UINTN Reserved2);

EFI_STATUS
ShutdownUefiBootServices (
  VOID
  );

EFI_STATUS
PreparePlatformHardware (
  VOID
  );

VOID
ESL_Start_OS (
  );

VOID
Flash_Start_OS (
  );

