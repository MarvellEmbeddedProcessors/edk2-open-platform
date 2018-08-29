/**
*
*  Copyright (c) 2017, Linaro Ltd. All rights reserved.
*  Copyright (c) 2018, Marvell International Ltd. All rights reserved.
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
#include <Library/DevicePathLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/NonDiscoverableDeviceRegistrationLib.h>
#include <Library/UefiBootServicesTableLib.h>

#include <Protocol/MvGpio.h>
#include <Protocol/NonDiscoverableDevice.h>

#define ARMADA_80x0_CP0_CONTROLLER1_INDEX   2
#define ARMADA_80x0_MCBIN_VBUS0_PIN         15

STATIC CONST GPIO_PIN_DESC mArmada80x0McBinVbusEn = {
  ARMADA_80x0_CP0_CONTROLLER1_INDEX,
  ARMADA_80x0_MCBIN_VBUS0_PIN,
  TRUE,
};

STATIC
EFI_STATUS
EFIAPI
Armada80x0McBinInitXhciVbus (
  IN  NON_DISCOVERABLE_DEVICE       *This
  )
{
  MARVELL_GPIO_PROTOCOL   *GpioProtocol;
  EFI_STATUS              Status;

  Status = MvGpioGetProtocol (GPIO_DRIVER_TYPE_SOC_CONTROLLER, &GpioProtocol);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Unable to find GPIO protocol\n", __FUNCTION__));
    return Status;
  }

  GpioProtocol->DirectionOutput (GpioProtocol,
                  mArmada80x0McBinVbusEn.ControllerId,
                  mArmada80x0McBinVbusEn.PinNumber,
                  mArmada80x0McBinVbusEn.ActiveHigh);

  return Status;
}

NON_DISCOVERABLE_DEVICE_INIT
EFIAPI
GetInitializerForType (
  IN  NON_DISCOVERABLE_DEVICE_TYPE  Type,
  IN  UINTN                         Index
  )
{
  if (Type == NonDiscoverableDeviceTypeXhci) {
        return Armada80x0McBinInitXhciVbus;
  }

  return NULL;
}
