/** @file
  Copyright (C) Marvell International Ltd. and its affiliates

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/DebugLib.h>
#include <Library/MppLib.h>
#include <Library/MvComPhyLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiDriverEntryPoint.h>
#include <Library/UtmiPhyLib.h>

EFI_STATUS
EFIAPI
ArmadaPlatInitDxeEntryPoint (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  DEBUG ((DEBUG_ERROR, "\nArmada Platform Init\n\n"));

  MvComPhyInit ();
  UtmiPhyInit ();
  MppInitialize ();

  return EFI_SUCCESS;
}
