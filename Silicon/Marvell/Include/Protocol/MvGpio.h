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
#ifndef __MARVELL_GPIO_PROTOCOL_H__
#define __MARVELL_GPIO_PROTOCOL_H__

#include <Uefi.h>

#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>

extern EFI_GUID gMarvellGpioProtocolGuid;

typedef struct _MARVELL_GPIO_PROTOCOL MARVELL_GPIO_PROTOCOL;

typedef enum {
  GPIO_MODE_INPUT                 = 0x0,
  GPIO_MODE_OUTPUT                = 0x1
} MARVELL_GPIO_MODE;

typedef enum {
  GPIO_DRIVER_TYPE_SOC_CONTROLLER = 0x0,
  GPIO_DRIVER_TYPE_PCA95XX        = 0x1
} MARVELL_GPIO_DRIVER_TYPE;

typedef enum {
  PCA9505_ID,
  PCA9534_ID,
  PCA9535_ID,
  PCA9536_ID,
  PCA9537_ID,
  PCA9538_ID,
  PCA9539_ID,
  PCA9554_ID,
  PCA9555_ID,
  PCA9556_ID,
  PCA9557_ID,
  PCA95XX_MAX_ID,
} MARVELL_IO_EXPANDER_TYPE_PCA95XX;

typedef
EFI_STATUS
(EFIAPI *MV_GPIO_DIRECTION_OUTPUT) (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINTN ControllerIndex,
  IN  UINTN GpioPin,
  IN  BOOLEAN Value
  );

typedef
EFI_STATUS
(EFIAPI *MV_GPIO_DIRECTION_INPUT) (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINTN ControllerIndex,
  IN  UINTN GpioPin
  );

typedef
EFI_STATUS
(EFIAPI *MV_GPIO_GET_FUNCTION) (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINTN ControllerIndex,
  IN  UINTN GpioPin,
  OUT MARVELL_GPIO_MODE *Mode
  );

typedef
EFI_STATUS
(EFIAPI *MV_GPIO_GET_VALUE) (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINTN ControllerIndex,
  IN  UINTN GpioPin,
  OUT BOOLEAN *Value
  );

typedef
EFI_STATUS
(EFIAPI *MV_GPIO_SET_VALUE) (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINTN ControllerIndex,
  IN  UINTN GpioPin,
  IN  BOOLEAN Value
  );

struct _MARVELL_GPIO_PROTOCOL {
  MV_GPIO_DIRECTION_INPUT       DirectionInput;
  MV_GPIO_DIRECTION_OUTPUT      DirectionOutput;
  MV_GPIO_GET_FUNCTION          GetFunction;
  MV_GPIO_GET_VALUE             GetValue;
  MV_GPIO_SET_VALUE             SetValue;
};

typedef struct {
  VENDOR_DEVICE_PATH            Header;
  MARVELL_GPIO_DRIVER_TYPE      GpioDriverType;
  EFI_DEVICE_PATH_PROTOCOL      End;
} MV_GPIO_DEVICE_PATH;

typedef struct {
  UINTN                         ControllerId;
  UINTN                         PinNumber;
  BOOLEAN                       ActiveHigh;
} GPIO_PIN_DESC;

/*
 * Select desired protocol producer upon MARVELL_GPIO_DRIVER_TYPE
 * field of driver's MV_GPIO_DEVICE_PATH.
 */
STATIC
inline
EFI_STATUS
EFIAPI
MvGpioGetProtocol (
  IN     MARVELL_GPIO_DRIVER_TYPE     GpioDriverType,
  IN OUT MARVELL_GPIO_PROTOCOL      **GpioProtocol
  )
{
  MV_GPIO_DEVICE_PATH *GpioDevicePath;
  EFI_DEVICE_PATH     *DevicePath;
  EFI_HANDLE          *HandleBuffer;
  EFI_STATUS           Status;
  UINTN                HandleCount;
  UINTN                Index;

  /* Locate Handles of all MARVELL_GPIO_PROTOCOL producers */
  Status = gBS->LocateHandleBuffer (ByProtocol,
                  &gMarvellGpioProtocolGuid,
                  NULL,
                  &HandleCount,
                  &HandleBuffer);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Unable to locate handles\n", __FUNCTION__));
    return Status;
  }

  /* Iterate over all protocol producers */
  for (Index = 0; Index < HandleCount; Index++) {
    /* Open device path protocol installed on each handle */
    Status = gBS->OpenProtocol (HandleBuffer[Index],
                    &gEfiDevicePathProtocolGuid,
                    (VOID **)&DevicePath,
                    gImageHandle,
                    NULL,
                    EFI_OPEN_PROTOCOL_GET_PROTOCOL);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "%a: Unable to find DevicePath\n", __FUNCTION__));
      continue;
    }

    while (!IsDevicePathEndType (DevicePath)) {
      /* Check if GpioDriverType matches one found in the device path */
      GpioDevicePath = (MV_GPIO_DEVICE_PATH *)DevicePath;
      if (GpioDevicePath->GpioDriverType != GpioDriverType) {
        DevicePath = NextDevicePathNode (DevicePath);
        continue;
      }

      /*
       * Open GpioProtocol. With EFI_OPEN_PROTOCOL_GET_PROTOCOL attribute
       * the consumer is not obliged to call CloseProtocol.
       */
      Status = gBS->OpenProtocol (HandleBuffer[Index],
                      &gMarvellGpioProtocolGuid,
                      (VOID **)GpioProtocol,
                      gImageHandle,
                      NULL,
                      EFI_OPEN_PROTOCOL_GET_PROTOCOL);
      if (EFI_ERROR (Status)) {
        DEBUG ((DEBUG_ERROR,
          "%a: Unable to open GPIO protocol\n",
          __FUNCTION__));
        gBS->FreePool (HandleBuffer);
        return Status;
      }

      gBS->FreePool (HandleBuffer);

      return EFI_SUCCESS;
    }
  }

  gBS->FreePool (HandleBuffer);

  return EFI_UNSUPPORTED;
}

#endif // __MARVELL_GPIO_PROTOCOL_H__
