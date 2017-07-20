/**
*
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

#include "MvGpioDxe.h"

STATIC MV_GPIO *mGpioInstance;

STATIC MV_GPIO_DEVICE_PATH mGpioDevicePathTemplate = {
  {
    {
      HARDWARE_DEVICE_PATH,
      HW_VENDOR_DP,
      {
        (UINT8) (sizeof (VENDOR_DEVICE_PATH) +
                 sizeof (MARVELL_GPIO_DRIVER_TYPE)),
        (UINT8) ((sizeof (VENDOR_DEVICE_PATH) +
                 sizeof (MARVELL_GPIO_DRIVER_TYPE)) >> 8),
      },
    },
    EFI_CALLER_ID_GUID
  },
  GPIO_DRIVER_TYPE_SOC_CONTROLLER,
  {
    END_DEVICE_PATH_TYPE,
    END_ENTIRE_DEVICE_PATH_SUBTYPE,
    {
      sizeof(EFI_DEVICE_PATH_PROTOCOL),
      0
    }
  }
};

STATIC
EFI_STATUS
MvGpioValidate (
  IN  UINTN ControllerIndex,
  IN  UINTN GpioPin
  )
{
  if (ControllerIndex >= mGpioInstance->Desc->GpioDevCount) {
    DEBUG ((DEBUG_ERROR,
      "%a: Invalid GPIO ControllerIndex: %d\n",
      __FUNCTION__,
      ControllerIndex));
    return EFI_INVALID_PARAMETER;
  }

  if (GpioPin >= mGpioInstance->Desc->SoC[ControllerIndex].GpioPinCount) {
    DEBUG ((DEBUG_ERROR,
      "%a: GPIO pin #%d not available in Controller#%d\n",
      __FUNCTION__,
      GpioPin,
      ControllerIndex));
    return EFI_INVALID_PARAMETER;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvGpioDirectionOutput (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINTN ControllerIndex,
  IN  UINTN GpioPin,
  IN  BOOLEAN Value
  )
{
  UINTN BaseAddress;
  EFI_STATUS Status;

  Status = MvGpioValidate (ControllerIndex, GpioPin);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: Fail to set output for pin #%d\n",
      __FUNCTION__,
      GpioPin));
    return Status;
  }

  BaseAddress = mGpioInstance->Desc->SoC[ControllerIndex].GpioBaseAddress;

  MmioAndThenOr32 (BaseAddress + GPIO_DATA_OUT_REG,
    ~BIT (GpioPin),
    (Value) << GpioPin);

  MmioAnd32 (BaseAddress + GPIO_OUT_EN_REG, ~BIT (GpioPin));

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvGpioDirectionInput (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINTN ControllerIndex,
  IN  UINTN GpioPin
  )
{
  UINTN BaseAddress;
  EFI_STATUS Status;

  Status = MvGpioValidate (ControllerIndex, GpioPin);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: Fail to set input for pin #%d\n",
      __FUNCTION__,
      GpioPin));
    return Status;
  }

  BaseAddress = mGpioInstance->Desc->SoC[ControllerIndex].GpioBaseAddress;

  MmioOr32 (BaseAddress + GPIO_OUT_EN_REG, BIT (GpioPin));

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvGpioGetFunction (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINTN ControllerIndex,
  IN  UINTN GpioPin,
  OUT MARVELL_GPIO_MODE *Mode
  )
{
  UINT32 RegVal;
  UINTN BaseAddress;
  EFI_STATUS Status;

  Status = MvGpioValidate (ControllerIndex, GpioPin);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: Fail to get function of pin #%d\n",
      __FUNCTION__,
      GpioPin));
    return Status;
  }

  BaseAddress = mGpioInstance->Desc->SoC[ControllerIndex].GpioBaseAddress;

  RegVal = MmioRead32 (BaseAddress + GPIO_OUT_EN_REG);
  *Mode = ((RegVal & BIT (GpioPin)) ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT);

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvGpioGetValue (
  IN     MARVELL_GPIO_PROTOCOL *This,
  IN     UINTN ControllerIndex,
  IN     UINTN GpioPin,
  IN OUT BOOLEAN *Value
  )
{
  UINTN BaseAddress;
  EFI_STATUS Status;

  Status = MvGpioValidate (ControllerIndex, GpioPin);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: Fail to get value of pin #%d\n",
      __FUNCTION__,
      GpioPin));
    return Status;
  }

  BaseAddress = mGpioInstance->Desc->SoC[ControllerIndex].GpioBaseAddress;

  *Value = !!(MmioRead32 (BaseAddress + GPIO_DATA_IN_REG) & BIT (GpioPin));

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvGpioSetValue (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINTN ControllerIndex,
  IN  UINTN GpioPin,
  IN  BOOLEAN Value
  )
{
  UINTN BaseAddress;
  EFI_STATUS Status;

  Status = MvGpioValidate (ControllerIndex, GpioPin);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: Fail to get value of pin #%d\n",
      __FUNCTION__,
      GpioPin));
    return Status;
  }

  BaseAddress = mGpioInstance->Desc->SoC[ControllerIndex].GpioBaseAddress;

  MmioAndThenOr32 (BaseAddress + GPIO_DATA_OUT_REG,
    ~BIT (GpioPin),
    Value << GpioPin);

  return EFI_SUCCESS;
}

STATIC
VOID
MvGpioInitProtocol (
  IN MARVELL_GPIO_PROTOCOL *GpioProtocol
  )
{
  GpioProtocol->DirectionInput  = MvGpioDirectionInput;
  GpioProtocol->DirectionOutput = MvGpioDirectionOutput;
  GpioProtocol->GetFunction     = MvGpioGetFunction;
  GpioProtocol->GetValue        = MvGpioGetValue;
  GpioProtocol->SetValue        = MvGpioSetValue;
}

EFI_STATUS
EFIAPI
MvGpioEntryPoint (
  IN EFI_HANDLE       ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  MARVELL_BOARD_DESC_PROTOCOL *BoardDescProtocol;
  MV_GPIO_DEVICE_PATH *GpioDevicePath;
  MV_BOARD_GPIO_DESC *Desc;
  EFI_STATUS Status;

  GpioDevicePath = AllocateCopyPool (sizeof (MV_GPIO_DEVICE_PATH),
                     &mGpioDevicePathTemplate);
  if (GpioDevicePath == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  mGpioInstance = AllocateZeroPool (sizeof (MV_GPIO));
  if (mGpioInstance == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrGpioInstanceAlloc;
  }

  /* Obtain list of available controllers */
  Status = gBS->LocateProtocol (&gMarvellBoardDescProtocolGuid,
                  NULL,
                  (VOID **)&BoardDescProtocol);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: Cannot locate BoardDesc protocol\n",
      __FUNCTION__));
    goto ErrLocateBoardDesc;
  }

  Status = BoardDescProtocol->BoardDescGpioGet (BoardDescProtocol, &Desc);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: Cannot get GPIO board desc from BoardDesc protocol\n",
      __FUNCTION__));
    goto ErrLocateBoardDesc;
  }

  mGpioInstance->Signature = GPIO_SIGNATURE;
  mGpioInstance->Desc = Desc;

  MvGpioInitProtocol (&mGpioInstance->GpioProtocol);

  Status = gBS->InstallMultipleProtocolInterfaces (&(mGpioInstance->Handle),
                  &gMarvellGpioProtocolGuid,
                  &(mGpioInstance->GpioProtocol),
                  &gEfiDevicePathProtocolGuid,
                  (EFI_DEVICE_PATH_PROTOCOL *)GpioDevicePath,
                  NULL);
  if (EFI_ERROR (Status)) {
    goto ErrLocateBoardDesc;
  }

  return EFI_SUCCESS;

ErrLocateBoardDesc:
  gBS->FreePool (mGpioInstance);

ErrGpioInstanceAlloc:
  gBS->FreePool (GpioDevicePath);

  return Status;
}
