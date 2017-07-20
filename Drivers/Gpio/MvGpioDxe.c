/*******************************************************************************
Copyright (C) 2017 Marvell International Ltd.

Marvell BSD License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File under the following licensing terms.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

* Neither the name of Marvell nor the names of its contributors may be
  used to endorse or promote products derived from this software without
  specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/
#include <Library/MvHwDescLib.h>

#include "MvGpioDxe.h"

DECLARE_A7K8K_GPIO_TEMPLATE;

MV_GPIO *mGpioInstance;

STATIC MV_GPIO_DEVICE_PATH mGpioDevicePathTemplate = {
  {
    {
      HARDWARE_DEVICE_PATH,
      HW_VENDOR_DP,
      {
  (UINT8) (sizeof(VENDOR_DEVICE_PATH) + sizeof(MARVELL_GPIO_DRIVER_TYPE)),
  (UINT8) ((sizeof(VENDOR_DEVICE_PATH) + sizeof(MARVELL_GPIO_DRIVER_TYPE)) >> 8),
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
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin
  )
{
  if (!mGpioInstance->GpioControllerDesc[ControllerIndex].Enabled ||
      ControllerIndex >= MVHW_MAX_GPIO_DEVS) {
    DEBUG ((DEBUG_ERROR, "Invalid GPIO ControllerIndex: %d\n", ControllerIndex));
    return EFI_INVALID_PARAMETER;
  }

  if (GpioPin >= mGpioInstance->GpioControllerDesc[ControllerIndex].PinCount) {
    DEBUG ((DEBUG_ERROR, "GPIO pin number: %d not available in Controller#%d\n", GpioPin, ControllerIndex));
    return EFI_INVALID_PARAMETER;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvGpioDirectionOutput (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin,
  IN  BOOLEAN Value
  )
{
  UINTN  BaseAddress;
  EFI_STATUS Status;

  Status = MvGpioValidate (ControllerIndex, GpioPin);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Fail to execute MvGpioDirectionOutput\n"));
    return Status;
  }

  BaseAddress = mGpioInstance->GpioControllerDesc[ControllerIndex].BaseAddress;

  MmioAndThenOr32 (BaseAddress + GPIO_DATA_OUT_REG, ~BIT (GpioPin), (Value) << GpioPin);

  MmioAnd32 (BaseAddress + GPIO_OUT_EN_REG, ~BIT (GpioPin));

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvGpioDirectionInput (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin
  )
{
  UINTN  BaseAddress;
  EFI_STATUS Status;

  Status = MvGpioValidate (ControllerIndex, GpioPin);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Fail to execute MvGpioDirectionInput\n"));
    return Status;
  }

  BaseAddress = mGpioInstance->GpioControllerDesc[ControllerIndex].BaseAddress;

  MmioOr32 (BaseAddress + GPIO_OUT_EN_REG, BIT (GpioPin));

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvGpioGetFunction (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin,
  OUT MARVELL_GPIO_MODE *Mode
  )
{
  UINT32 RegVal;
  UINTN  BaseAddress;
  EFI_STATUS Status;

  Status = MvGpioValidate (ControllerIndex, GpioPin);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Fail to execute MvGpioGetFunction\n"));
    return Status;
  }

  BaseAddress = mGpioInstance->GpioControllerDesc[ControllerIndex].BaseAddress;

  RegVal = MmioRead32 (BaseAddress + GPIO_OUT_EN_REG) & BIT (GpioPin);
  if (RegVal) {
    *Mode = GPIO_MODE_INPUT;
  } else {
    *Mode = GPIO_MODE_OUTPUT;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvGpioGetValue (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin,
  OUT BOOLEAN *Value
  )
{
  UINTN  BaseAddress;
  EFI_STATUS Status;

  Status = MvGpioValidate (ControllerIndex, GpioPin);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Fail to execute MvGpioGetValue\n"));
    return Status;
  }

  BaseAddress = mGpioInstance->GpioControllerDesc[ControllerIndex].BaseAddress;

  *Value = !!(MmioRead32 (BaseAddress + GPIO_DATA_IN_REG) & BIT (GpioPin));

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvGpioSetValue (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin,
  IN  BOOLEAN Value
  )
{
  UINTN  BaseAddress;
  EFI_STATUS Status;

  Status = MvGpioValidate (ControllerIndex, GpioPin);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Fail to execute MvGpioSetValue\n"));
    return Status;
  }

  BaseAddress = mGpioInstance->GpioControllerDesc[ControllerIndex].BaseAddress;

  MmioAndThenOr32 (BaseAddress + GPIO_DATA_OUT_REG, ~BIT (GpioPin), Value << GpioPin);

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvGpioInitProtocol (
  IN MARVELL_GPIO_PROTOCOL *GpioProtocol
  )
{
  GpioProtocol->DirectionInput  = MvGpioDirectionInput;
  GpioProtocol->DirectionOutput = MvGpioDirectionOutput;
  GpioProtocol->GetFunction     = MvGpioGetFunction;
  GpioProtocol->GetValue        = MvGpioGetValue;
  GpioProtocol->SetValue        = MvGpioSetValue;

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
MvGpioEntryPoint (
  IN EFI_HANDLE       ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  MVHW_GPIO_DESC *Desc = &mA7k8kGpioDescTemplate;
  UINT8 *GpioDeviceTable, Index;
  EFI_STATUS Status;
  MV_GPIO_DEVICE_PATH *GpioDevicePath;

  GpioDevicePath = AllocateCopyPool (sizeof (MV_GPIO_DEVICE_PATH), &mGpioDevicePathTemplate);
  if (GpioDevicePath == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  /* Obtain table with enabled GPIO devices */
  GpioDeviceTable = (UINT8 *)PcdGetPtr (PcdGpioControllers);
  if (GpioDeviceTable == NULL) {
    DEBUG ((DEBUG_ERROR, "Missing PcdGpioControllers\n"));
    return EFI_INVALID_PARAMETER;
  }

  if (PcdGetSize (PcdGpioControllers) > MVHW_MAX_GPIO_DEVS) {
    DEBUG ((DEBUG_ERROR, "Wrong PcdGpioControllers format\n"));
    return EFI_INVALID_PARAMETER;
  }

  mGpioInstance = AllocateZeroPool (sizeof (MV_GPIO));
  if (mGpioInstance == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  EfiInitializeLock (&mGpioInstance->Lock, TPL_NOTIFY);

  MvGpioInitProtocol (&mGpioInstance->GpioProtocol);

  mGpioInstance->Signature = GPIO_SIGNATURE;

  /* Initialize enabled controllers */
  for (Index = 0; Index < PcdGetSize (PcdGpioControllers); Index++) {
    if (!MVHW_DEV_ENABLED (Gpio, Index)) {
      DEBUG ((DEBUG_ERROR, "Skip Gpio controller %d\n", Index));
      mGpioInstance->GpioControllerDesc[Index].Enabled = FALSE;
      continue;
    }

    mGpioInstance->GpioControllerDesc[Index].Enabled = TRUE;
    mGpioInstance->GpioControllerDesc[Index].BaseAddress = Desc->GpioBaseAddresses[Index];
    mGpioInstance->GpioControllerDesc[Index].PinCount = Desc->GpioPinCount[Index];
  }

  Status = gBS->InstallMultipleProtocolInterfaces (
        &(mGpioInstance->Handle),
        &gMarvellGpioProtocolGuid,
        &(mGpioInstance->GpioProtocol),
        &gEfiDevicePathProtocolGuid,
        (EFI_DEVICE_PATH_PROTOCOL *) GpioDevicePath,
        NULL);
  if (EFI_ERROR (Status)) {
    FreePool (mGpioInstance);
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}
