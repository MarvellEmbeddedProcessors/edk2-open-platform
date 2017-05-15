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

#include "Armada70x0PciEmuInitLib.h"

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

STATIC
EFI_STATUS
EFIAPI
Armada80x0InitXhciVbus (
  IN  NON_DISCOVERABLE_DEVICE       *This
  )
{
  /* I2C IO-expander GPIO pins modification should be added here */

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
Armada8040McBinInitXhciVbus (
  IN  NON_DISCOVERABLE_DEVICE       *This
  )
{
  MmioOr32 (
      GPIO_BASE + GPIO_DIR_OFFSET (ARMADA_8040_MCBIN_VBUS_GPIO),
      GPIO_PIN_MASK (ARMADA_8040_MCBIN_VBUS_GPIO)
      );
  MmioAnd32 (
      GPIO_BASE + GPIO_ENABLE_OFFSET (ARMADA_8040_MCBIN_VBUS_GPIO),
      ~GPIO_PIN_MASK (ARMADA_8040_MCBIN_VBUS_GPIO)
      );

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
  case MVBOARD_ID_ARMADA8040_DB:
    if (Type == NonDiscoverableDeviceTypeXhci) {
          return Armada80x0InitXhciVbus;
    }
  case MVBOARD_ID_ARMADA8040_MCBIN:
    if (Type == NonDiscoverableDeviceTypeXhci && Index == 0x2) {
          return Armada8040McBinInitXhciVbus;
    }
  default:
    return NULL;
  }
}
