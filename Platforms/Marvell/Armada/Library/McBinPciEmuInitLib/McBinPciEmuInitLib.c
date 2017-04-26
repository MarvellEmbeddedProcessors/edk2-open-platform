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
#include <Library/NonDiscoverableDeviceRegistrationLib.h>

#include <Protocol/NonDiscoverableDevice.h>

#define GPIO_BASE               FixedPcdGet64 (PcdChip1MppBaseAddress) + 0x100
#define GPIO_DIR_OFFSET(n)      (((n) >> 5) * 0x40)
#define GPIO_ENABLE_OFFSET(n)   (((n) >> 5) * 0x40 + 0x4)

#define GPIO_PIN_MASK(n)        (1 << ((n) & 0x1f))

#define VBUS_GPIO               47

STATIC
EFI_STATUS
EFIAPI
InitXhciVbus (
  IN  NON_DISCOVERABLE_DEVICE       *This
  )
{
  DEBUG ((DEBUG_INFO, "%a: enabling VBUS GPIO\n", __FUNCTION__));

  MmioOr32 (GPIO_BASE + GPIO_DIR_OFFSET (VBUS_GPIO),
            GPIO_PIN_MASK (VBUS_GPIO));
  MmioAnd32 (GPIO_BASE + GPIO_ENABLE_OFFSET (VBUS_GPIO),
             ~GPIO_PIN_MASK (VBUS_GPIO));

  return EFI_SUCCESS;
}

NON_DISCOVERABLE_DEVICE_INIT
EFIAPI
GetInitializerForType (
  IN  NON_DISCOVERABLE_DEVICE_TYPE  Type,
  IN  UINTN                         Index
  )
{
  if (Type == NonDiscoverableDeviceTypeXhci &&
      Index == 0x2) {
        return InitXhciVbus;
  }
  return NULL;
}
