/** @file

  Copyright (c) 2008 - 2009, Apple Inc. All rights reserved.<BR>
  Copyright (c) 2016, Marvell. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD
  License which accompanies this distribution. The full text of the license
  may be found at http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <PiDxe.h>

#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/NonDiscoverableDeviceRegistrationLib.h>
#include <Library/ParsePcdLib.h>
#include <Library/UefiBootServicesTableLib.h>

#include <Protocol/EmbeddedExternalDevice.h>

typedef enum {
  DEV_XHCI,
  DEV_AHCI,
  DEV_SDHCI,
  DEV_MAX
} DEV_TYPE;

typedef enum {
  DMA_COHERENT,
  DMA_NONCOHERENT,
  DMA_TYPE_MAX,
} DMA_TYPE;

//
// Below function is used to parse devices information from PCD strings.
// Once obtained, the resources are used for registration of
// NonDiscoverable devices.
//
EFI_STATUS
EFIAPI
PciEmulationEntryPoint (
  IN EFI_HANDLE       ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  EFI_STATUS Status;
  UINT8 i;
  UINT8 DevCount = PcdGet32 (PcdPciEDevCount);
  UINT8 *DeviceType = PcdGetPtr (PcdPciEDevType);
  UINT8 *DmaType = PcdGetPtr (PcdPciEDmaType);
  UINTN BaseAddrTable[PcdGet32 (PcdPciEDevCount)];
  NON_DISCOVERABLE_DEVICE_TYPE DeviceTypeTable[PcdGet32 (PcdPciEDevCount)];
  NON_DISCOVERABLE_DEVICE_DMA_TYPE DmaTypeTable[PcdGet32 (PcdPciEDevCount)];

  ASSERT (PcdGetSize (PcdPciEDevType) == DevCount);
  ASSERT (PcdGetSize (PcdPciEDmaType) == DevCount);

  Status = ParsePcdString ((CHAR16 *) PcdGetPtr (PcdPciEDevBaseAddress), DevCount, BaseAddrTable, NULL);
  if (EFI_ERROR(Status)) {
    DEBUG((DEBUG_ERROR, "PciEmulation: Wrong PcdPciEDevBaseAddress format\n"));
    return EFI_INVALID_PARAMETER;
  }

  for (i = 0; i < DevCount; i++) {
    switch (DeviceType[i]) {
    case DEV_XHCI:
      DeviceTypeTable[i] = NonDiscoverableDeviceTypeXhci;
      break;
    case DEV_AHCI:
      DeviceTypeTable[i] = NonDiscoverableDeviceTypeAhci;
      break;
    case DEV_SDHCI:
      DeviceTypeTable[i] = NonDiscoverableDeviceTypeSdhci;
      break;
    default:
      DEBUG((DEBUG_ERROR, "PciEmulation: Unsupported device type with ID=%d\n", i));
      return EFI_INVALID_PARAMETER;
    }

    switch (DmaType[i]) {
    case DMA_COHERENT:
      DmaTypeTable[i] = NonDiscoverableDeviceDmaTypeCoherent;
      break;
    case DMA_NONCOHERENT:
      DmaTypeTable[i] = NonDiscoverableDeviceDmaTypeNonCoherent;
      break;
    default:
      DEBUG((DEBUG_ERROR, "PciEmulation: Unsupported DMA type with ID=%d\n", i));
      return EFI_INVALID_PARAMETER;
    }

    Status = RegisterNonDiscoverableDevice (
                     BaseAddrTable[i],
                     DeviceTypeTable[i],
                     DmaTypeTable[i],
                     NULL,
                     NULL
                   );

    if (EFI_ERROR(Status)) {
      DEBUG((DEBUG_ERROR, "PciEmulation: Cannot install device with ID=%d\n", i));
    }
  }

  return Status;
}
