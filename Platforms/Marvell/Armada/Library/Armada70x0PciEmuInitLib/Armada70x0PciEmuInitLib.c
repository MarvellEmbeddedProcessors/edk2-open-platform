/**
*
*  Copyright (c) 2017, Linaro Ltd. All rights reserved.
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

#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MvBoardDescLib.h>
#include <Library/NonDiscoverableDeviceRegistrationLib.h>

#include <Protocol/NonDiscoverableDevice.h>

STATIC
EFI_STATUS
EFIAPI
Armada70x0InitXhciVbus (
  IN  NON_DISCOVERABLE_DEVICE       *This
  )
{
  /* I2C IO-expander GPIO pins modification should be added here */

  return EFI_SUCCESS;
}

NON_DISCOVERABLE_DEVICE_INIT
EFIAPI
GetInitializerForType (
  IN  NON_DISCOVERABLE_DEVICE_TYPE  Type,
  IN  UINTN                         Index
  )
{
  UINT8    BoardId;

  MVBOARD_ID_GET (BoardId);

  switch (BoardId) {
  case MVBOARD_ID_ARMADA7040_DB:
    if (Type == NonDiscoverableDeviceTypeXhci) {
          return Armada70x0InitXhciVbus;
    }
  default:
    return NULL;
  }
}
