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

#define ARMADA_70x0_DB_IO_EXPANDER0       0
#define ARMADA_70x0_DB_VBUS0_PIN          0
#define ARMADA_70x0_DB_VBUS0_LIMIT_PIN    4
#define ARMADA_70x0_DB_VBUS1_PIN          1
#define ARMADA_70x0_DB_VBUS1_LIMIT_PIN    5

STATIC CONST GPIO_PIN_DESC mArmada70x0DbVbusEn[] = {
  {
    ARMADA_70x0_DB_IO_EXPANDER0,
    ARMADA_70x0_DB_VBUS0_PIN,
    TRUE,
  },
  {
    ARMADA_70x0_DB_IO_EXPANDER0,
    ARMADA_70x0_DB_VBUS0_LIMIT_PIN,
    TRUE,
  },
  {
    ARMADA_70x0_DB_IO_EXPANDER0,
    ARMADA_70x0_DB_VBUS1_PIN,
    TRUE,
  },
  {
    ARMADA_70x0_DB_IO_EXPANDER0,
    ARMADA_70x0_DB_VBUS1_LIMIT_PIN,
    TRUE,
  },
};

STATIC
EFI_STATUS
EFIAPI
Armada70x0DbInitXhciVbus (
  IN  NON_DISCOVERABLE_DEVICE       *This
  )
{
  CONST GPIO_PIN_DESC     *VbusEnPinDesc;
  MARVELL_GPIO_PROTOCOL   *GpioProtocol;
  EFI_STATUS              Status;
  UINTN                   Index;

  Status = MvGpioGetProtocol (GPIO_DRIVER_TYPE_PCA95XX, &GpioProtocol);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Unable to find GPIO protocol\n", __FUNCTION__));
    return Status;
  }

  VbusEnPinDesc = mArmada70x0DbVbusEn;
  for (Index = 0; Index < ARRAY_SIZE (mArmada70x0DbVbusEn); Index++) {
    GpioProtocol->DirectionOutput (GpioProtocol,
                    VbusEnPinDesc->ControllerId,
                    VbusEnPinDesc->PinNumber,
                    VbusEnPinDesc->ActiveHigh);
    VbusEnPinDesc++;
  }

  return EFI_SUCCESS;
}

NON_DISCOVERABLE_DEVICE_INIT
EFIAPI
GetInitializerForType (
  IN  NON_DISCOVERABLE_DEVICE_TYPE  Type,
  IN  UINTN                         Index
  )
{
  if (Type == NonDiscoverableDeviceTypeXhci) {
        return Armada70x0DbInitXhciVbus;
  }

  return NULL;
}
