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
#include <Library/UefiBootServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/DevicePathLib.h>

#include <Protocol/NonDiscoverableDevice.h>
#include <Protocol/Gpio.h>

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
  EFI_STATUS              Status = EFI_SUCCESS;
  MARVELL_GPIO_PROTOCOL   *GpioProtocol;
  EFI_HANDLE              *ProtHandle = NULL;
  GPIO_PIN_DESC           McBinVbusEn = {MV_GPIO_CP0_CONTROLLER1, ARMADA_8040_MCBIN_VBUS_GPIO, TRUE};

  Status = MarvellGpioGetHandle (GPIO_DRIVER_TYPE_SOC_CONTROLLER, &ProtHandle);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to find GPIO protocol, Status: 0x%x\n", Status));
    return Status;
  }

  Status = gBS->OpenProtocol (
                  ProtHandle,
                  &gMarvellGpioProtocolGuid,
                  (void **)&GpioProtocol,
                  gImageHandle,
                  NULL,
                  EFI_OPEN_PROTOCOL_GET_PROTOCOL
                  );
  if(EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to open GPIO protocol, Status: 0x%x\n", Status));
    return Status;
  }

  GpioProtocol->DirectionOutput(
        GpioProtocol,
        McBinVbusEn.ControllerId,
        McBinVbusEn.PinNumber,
        McBinVbusEn.ActiveHigh
        );

  Status = gBS->CloseProtocol (
                  ProtHandle,
                  &gMarvellGpioProtocolGuid,
                  gImageHandle,
                  ProtHandle
                  );
  return Status;
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
