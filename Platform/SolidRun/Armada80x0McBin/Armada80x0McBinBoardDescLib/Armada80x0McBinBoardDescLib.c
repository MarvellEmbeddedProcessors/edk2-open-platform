/**
*
*  Copyright (C) 2018, Marvell International Ltd. and its affiliates.
*
*  This program and the accompanying materials are licensed and made available
*  under the terms and conditions of the BSD License which accompanies this
*  distribution. The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#include <Uefi.h>

#include <Library/ArmadaBoardDescLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>

EFI_STATUS
EFIAPI
ArmadaBoardDescGpioGet (
  IN OUT MV_I2C_IO_EXPANDER_DESC **I2cIoExpanderDesc,
  IN OUT UINTN                    *I2cIoExpanderCount
  )
{
  /* No I2C IO expanders on board */
  *I2cIoExpanderDesc = NULL;
  *I2cIoExpanderCount = 0;

  return EFI_SUCCESS;
}
