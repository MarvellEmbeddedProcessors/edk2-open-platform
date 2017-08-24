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

#include <Protocol/I2cIo.h>

#include <Pi/PiI2c.h>

#include "MvPca95xxDxe.h"

STATIC PCA95XX *mPca95xxInstance;

STATIC MV_GPIO_DEVICE_PATH mPca95xxDevicePathTemplate = {
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
  GPIO_DRIVER_TYPE_PCA95XX,
  {
    END_DEVICE_PATH_TYPE,
    END_ENTIRE_DEVICE_PATH_SUBTYPE,
    {
      sizeof(EFI_DEVICE_PATH_PROTOCOL),
      0
    }
  }
};

STATIC PCA95XX_PIN_COUNT mPca95xxPinCount[PCA95XX_MAX_ID] = {
  PCA9505_PIN_COUNT,
  PCA9534_PIN_COUNT,
  PCA9535_PIN_COUNT,
  PCA9536_PIN_COUNT,
  PCA9537_PIN_COUNT,
  PCA9538_PIN_COUNT,
  PCA9539_PIN_COUNT,
  PCA9554_PIN_COUNT,
  PCA9555_PIN_COUNT,
  PCA9556_PIN_COUNT,
  PCA9557_PIN_COUNT,
};

STATIC
EFI_STATUS
MvPca95xxValidate (
  IN UINTN ControllerIndex,
  IN UINTN GpioPin
  )
{
  UINTN ControllerId;

  if (ControllerIndex >= mPca95xxInstance->ControllerCount) {
    DEBUG ((DEBUG_ERROR,
      "%a: Invalid GPIO ControllerIndex: %d\n",
      __FUNCTION__,
      ControllerIndex));
    return EFI_INVALID_PARAMETER;
  }

  ControllerId = mPca95xxInstance->ControllerDesc[ControllerIndex].ChipId;

  if (GpioPin >= mPca95xxPinCount[ControllerId]) {
    DEBUG ((DEBUG_ERROR,
      "%a: GPIO pin #%d not available in Controller#%d\n",
      __FUNCTION__,
      GpioPin,
      ControllerIndex));
    return EFI_INVALID_PARAMETER;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
MvPca95xxGetI2c (
  IN     UINTN                 ControllerIndex,
  IN OUT EFI_I2C_IO_PROTOCOL **I2cIo
  )
{
  UINTN        I2cBus, I2cAddress;
  UINTN        HandleCount, Index;
  EFI_HANDLE  *HandleBuffer;
  EFI_STATUS   Status;

  I2cBus = mPca95xxInstance->ControllerDesc[ControllerIndex].I2cBus;
  I2cAddress = mPca95xxInstance->ControllerDesc[ControllerIndex].I2cAddress;

  /* Locate Handles of all EfiI2cIoProtocol producers */
  Status = gBS->LocateHandleBuffer (ByProtocol,
                  &gEfiI2cIoProtocolGuid,
                  NULL,
                  &HandleCount,
                  &HandleBuffer);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Unable to locate handles\n", __FUNCTION__));
    return Status;
  }

  /* Iterate over all protocol producers and pick one upon DeviceIndex match */
  for (Index = 0; Index < HandleCount; Index++) {
    Status = gBS->OpenProtocol (HandleBuffer[Index],
                    &gEfiI2cIoProtocolGuid,
                    (VOID **)I2cIo,
                    gImageHandle,
                    NULL,
                    EFI_OPEN_PROTOCOL_GET_PROTOCOL);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "%a: Unable to open protocol\n", __FUNCTION__));
      gBS->FreePool (HandleBuffer);
      return Status;
    }
    if ((*I2cIo)->DeviceIndex == I2C_DEVICE_INDEX (I2cBus, I2cAddress)) {
      gBS->FreePool (HandleBuffer);
      return EFI_SUCCESS;
    }
  }

  gBS->FreePool (HandleBuffer);

  return EFI_NOT_FOUND;
}

EFI_STATUS
EFIAPI
MvPca95xxI2cTransfer (
  IN EFI_I2C_IO_PROTOCOL *I2cIo,
  IN UINT8                Address,
  IN UINT8               *Buffer,
  IN PCA95XX_OPERATION    Operation
  )
{
  EFI_I2C_REQUEST_PACKET *RequestPacket;
  UINTN RequestPacketSize;
  UINT8 AddressBuffer;
  EFI_STATUS Status;

  RequestPacketSize = sizeof (UINTN) +
                      sizeof (EFI_I2C_OPERATION) * PCA95XX_OPERATION_COUNT;
  RequestPacket = AllocateZeroPool (RequestPacketSize);
  if (RequestPacket == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  /* Operations contain address and payload, consecutively. */
  RequestPacket->OperationCount = PCA95XX_OPERATION_COUNT;
  RequestPacket->Operation[0].LengthInBytes = PCA95XX_OPERATION_LENGTH;
  RequestPacket->Operation[0].Buffer = &AddressBuffer;
  RequestPacket->Operation[0].Buffer[0] = Address & MAX_UINT8;
  RequestPacket->Operation[1].LengthInBytes = PCA95XX_OPERATION_LENGTH;
  RequestPacket->Operation[1].Buffer = Buffer;
  RequestPacket->Operation[1].Flags = (Operation == PCA95XX_READ ?
                                       I2C_FLAG_READ : I2C_FLAG_NORESTART);

  Status = I2cIo->QueueRequest (I2cIo, 0, NULL, RequestPacket, NULL);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: transmission error: 0x%d\n",
      __FUNCTION__,
      Status));
  }

  gBS->FreePool(RequestPacket);

  return Status;
}

STATIC
EFI_STATUS
MvPca95xxReadRegs (
  IN  EFI_I2C_IO_PROTOCOL *I2cIo,
  IN  UINT8                Reg,
  OUT UINT8               *RegVal
  )
{
  return MvPca95xxI2cTransfer (I2cIo, Reg, RegVal, PCA95XX_READ);
}

STATIC
EFI_STATUS
MvPca95xxWriteRegs (
  IN  EFI_I2C_IO_PROTOCOL *I2cIo,
  IN  UINTN                Reg,
  IN  UINT8                RegVal
  )
{
  return MvPca95xxI2cTransfer (I2cIo, Reg, &RegVal, PCA95XX_WRITE);
}

STATIC
EFI_STATUS
MvPca95xxSetOutputValue (
  IN UINTN   ControllerIndex,
  IN UINTN   GpioPin,
  IN BOOLEAN Value
  )
{
  EFI_I2C_IO_PROTOCOL *I2cIo;
  EFI_STATUS Status;
  UINT8 RegVal;
  UINTN Bank;

  Status = MvPca95xxGetI2c (ControllerIndex, &I2cIo);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: fail to get I2C protocol\n", __FUNCTION__));
    return EFI_DEVICE_ERROR;
  }

  Bank = GpioPin / PCA95XX_BANK_SIZE;

  Status = MvPca95xxReadRegs (I2cIo, PCA95XX_OUTPUT_REG + Bank, &RegVal);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: fail to read device register\n", __FUNCTION__));
    return EFI_DEVICE_ERROR;
  }

  if (Value) {
    RegVal |= BIT (GpioPin % PCA95XX_BANK_SIZE);
  } else {
    RegVal &= ~BIT (GpioPin % PCA95XX_BANK_SIZE);
  }

  Status = MvPca95xxWriteRegs (I2cIo, PCA95XX_OUTPUT_REG + Bank, RegVal);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: fail to write device register\n", __FUNCTION__));
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvPca95xxSetDirection (
  IN UINTN             ControllerIndex,
  IN UINTN             GpioPin,
  IN MARVELL_GPIO_MODE Direction
  )
{
  EFI_I2C_IO_PROTOCOL *I2cIo;
  EFI_STATUS Status;
  UINT8 RegVal;
  UINTN Bank;

  Status = MvPca95xxGetI2c (ControllerIndex, &I2cIo);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: fail to get I2C protocol\n", __FUNCTION__));
    return Status;
  }

  Bank = GpioPin / PCA95XX_BANK_SIZE;

  Status = MvPca95xxReadRegs (I2cIo, PCA95XX_DIRECTION_REG + Bank, &RegVal);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: fail to read device register\n", __FUNCTION__));
    return Status;
  }

  if (Direction == GPIO_MODE_INPUT) {
    RegVal |= BIT (GpioPin % PCA95XX_BANK_SIZE);
  } else {
    RegVal &= ~BIT (GpioPin % PCA95XX_BANK_SIZE);
  }

  Status = MvPca95xxWriteRegs (I2cIo, PCA95XX_DIRECTION_REG + Bank, RegVal);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: fail to write device register\n", __FUNCTION__));
    return Status;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvPca95xxIsOutput (
  IN  UINTN              ControllerIndex,
  IN  UINTN              GpioPin,
  OUT MARVELL_GPIO_MODE *Mode
  )
{
  EFI_I2C_IO_PROTOCOL *I2cIo;
  EFI_STATUS Status;
  UINT8 RegVal;
  UINTN Bank;

  Status = MvPca95xxValidate (ControllerIndex, GpioPin);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: invalid pin/controller data\n",
      __FUNCTION__,
      GpioPin));
    return Status;
  }

  Status = MvPca95xxGetI2c (ControllerIndex, &I2cIo);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: fail to get I2C protocol\n", __FUNCTION__));
    return Status;
  }

  Bank = GpioPin / PCA95XX_BANK_SIZE;

  Status = MvPca95xxReadRegs (I2cIo, PCA95XX_DIRECTION_REG + Bank, &RegVal);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: fail to read device register\n", __FUNCTION__));
    return Status;
  }

  *Mode = ((RegVal & BIT (GpioPin % PCA95XX_BANK_SIZE)) ?
           GPIO_MODE_INPUT : GPIO_MODE_OUTPUT);

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvPca95xxDirectionOutput (
  IN MARVELL_GPIO_PROTOCOL *This,
  IN UINTN                  ControllerIndex,
  IN UINTN                  GpioPin,
  IN BOOLEAN                Value
  )
{
  EFI_STATUS Status;

  Status = MvPca95xxValidate (ControllerIndex, GpioPin);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: invalid pin/controller data\n",
      __FUNCTION__,
      GpioPin));
    return Status;
  }

  /* Configure output value. */
  Status = MvPca95xxSetOutputValue (ControllerIndex, GpioPin, Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: fail to set ouput value\n", __FUNCTION__));
    return EFI_DEVICE_ERROR;
  }

  /* Configure direction as output. */
  Status = MvPca95xxSetDirection (ControllerIndex, GpioPin, GPIO_MODE_OUTPUT);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: fail to set direction\n", __FUNCTION__));
  }

  return Status;
}

STATIC
EFI_STATUS
MvPca95xxDirectionInput (
  IN MARVELL_GPIO_PROTOCOL *This,
  IN UINTN                  ControllerIndex,
  IN UINTN                  GpioPin
  )
{
  EFI_STATUS Status;

  Status = MvPca95xxValidate (ControllerIndex, GpioPin);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: invalid pin/controller data\n",
      __FUNCTION__,
      GpioPin));
    return Status;
  }

  /* Configure direction as input. */
  Status = MvPca95xxSetDirection (ControllerIndex, GpioPin, GPIO_MODE_INPUT);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: fail to set direction\n", __FUNCTION__));
  }

  return Status;
}

STATIC
EFI_STATUS
MvPca95xxGetFunction (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINTN                  ControllerIndex,
  IN  UINTN                  GpioPin,
  OUT MARVELL_GPIO_MODE     *Mode
  )
{
  EFI_STATUS Status;

  Status = MvPca95xxValidate (ControllerIndex, GpioPin);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: invalid pin/controller data\n",
      __FUNCTION__,
      GpioPin));
    return Status;
  }

  Status = MvPca95xxIsOutput (ControllerIndex, GpioPin, Mode);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: fail to get pin %d of controller#%d mode\n",
      __FUNCTION__,
      GpioPin,
      ControllerIndex));
  }

  return Status;
}

STATIC
EFI_STATUS
MvPca95xxGetValue (
  IN     MARVELL_GPIO_PROTOCOL *This,
  IN     UINTN                  ControllerIndex,
  IN     UINTN                  GpioPin,
  IN OUT BOOLEAN               *Value
  )
{
  EFI_I2C_IO_PROTOCOL *I2cIo;
  EFI_STATUS Status;
  UINT8 RegVal;
  UINTN Bank;

  Status = MvPca95xxValidate (ControllerIndex, GpioPin);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: invalid pin/controller data\n",
      __FUNCTION__,
      GpioPin));
    return Status;
  }

  Status = MvPca95xxGetI2c (ControllerIndex, &I2cIo);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: fail to get I2C protocol\n", __FUNCTION__));
    return Status;
  }

  Bank = GpioPin / PCA95XX_BANK_SIZE;

  Status = MvPca95xxReadRegs (I2cIo, PCA95XX_INPUT_REG + Bank, &RegVal);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: fail to read device register\n", __FUNCTION__));
    return Status;
  }

  *Value = !!(RegVal & BIT (GpioPin % PCA95XX_BANK_SIZE));

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvPca95xxSetValue (
  IN MARVELL_GPIO_PROTOCOL *This,
  IN UINTN                  ControllerIndex,
  IN UINTN                  GpioPin,
  IN BOOLEAN                Value
  )
{
  EFI_STATUS Status;

  Status = MvPca95xxValidate (ControllerIndex, GpioPin);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: invalid pin/controller data\n",
      __FUNCTION__,
      GpioPin));
    return Status;
  }

  Status = MvPca95xxSetOutputValue (ControllerIndex, GpioPin, Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: fail to set ouput value\n", __FUNCTION__));
  }

  return Status;
}


STATIC
VOID
MvPca95xxInitProtocol (
  IN MARVELL_GPIO_PROTOCOL *GpioProtocol
  )
{
  GpioProtocol->DirectionInput  = MvPca95xxDirectionInput;
  GpioProtocol->DirectionOutput = MvPca95xxDirectionOutput;
  GpioProtocol->GetFunction     = MvPca95xxGetFunction;
  GpioProtocol->GetValue        = MvPca95xxGetValue;
  GpioProtocol->SetValue        = MvPca95xxSetValue;
}

EFI_STATUS
EFIAPI
MvPca95xxEntryPoint (
  IN EFI_HANDLE       ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  MARVELL_BOARD_DESC_PROTOCOL *BoardDescProtocol;
  MV_GPIO_DEVICE_PATH *Pca95xxDevicePath;
  MV_BOARD_GPIO_DESC *GpioDesc;
  EFI_STATUS Status;

  /* Obtain list of available controllers */
  Status = gBS->LocateProtocol (&gMarvellBoardDescProtocolGuid,
                  NULL,
                  (VOID **)&BoardDescProtocol);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: Cannot locate BoardDesc protocol\n",
      __FUNCTION__));
    return Status;
  }

  Status = BoardDescProtocol->BoardDescGpioGet (BoardDescProtocol, &GpioDesc);
  if (EFI_ERROR (Status) || !GpioDesc->I2cIoExpanderDesc) {
    DEBUG ((DEBUG_ERROR,
      "%a: Cannot get GPIO board desc from BoardDesc protocol\n",
      __FUNCTION__));
    return Status;
  } else if (!GpioDesc->I2cIoExpanderDesc) {
    /* Silently exit, if the board does not support the controllers */
    return EFI_SUCCESS;
  }

  Pca95xxDevicePath = AllocateCopyPool (sizeof (MV_GPIO_DEVICE_PATH),
    &mPca95xxDevicePathTemplate);
  if (Pca95xxDevicePath == NULL) {
    DEBUG ((DEBUG_ERROR,
      "%a: Fail to allocate Pca95xxDevicePath\n",
      __FUNCTION__));
    return EFI_OUT_OF_RESOURCES;
  }

  mPca95xxInstance = AllocateZeroPool (sizeof (PCA95XX));
  if (mPca95xxInstance == NULL) {
    DEBUG ((DEBUG_ERROR,
      "%a: Fail to allocate mPca95xxInstance\n",
      __FUNCTION__));
    goto ErrPca95xxInstanceAlloc;
  }

  MvPca95xxInitProtocol (&mPca95xxInstance->GpioProtocol);

  mPca95xxInstance->Signature = PCA95XX_GPIO_SIGNATURE;
  mPca95xxInstance->ControllerDesc = GpioDesc->I2cIoExpanderDesc;
  mPca95xxInstance->ControllerCount = GpioDesc->I2cIoExpanderCount;

  Status = gBS->InstallMultipleProtocolInterfaces (
                  &(mPca95xxInstance->ControllerHandle),
                  &gMarvellGpioProtocolGuid,
                  &(mPca95xxInstance->GpioProtocol),
                  &gEfiDevicePathProtocolGuid,
                  (EFI_DEVICE_PATH_PROTOCOL *)Pca95xxDevicePath,
                  NULL);
  if (EFI_ERROR (Status)) {
    goto ErrLocateBoardDesc;
  }

  return EFI_SUCCESS;

ErrLocateBoardDesc:
  gBS->FreePool (mPca95xxInstance);

ErrPca95xxInstanceAlloc:
  gBS->FreePool (Pca95xxDevicePath);

  return Status;
}
