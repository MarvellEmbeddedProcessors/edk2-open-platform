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
#include <Protocol/I2cIo.h>

#include <Pi/PiI2c.h>

#include "MvPca95xxDxe.h"

STATIC MV_GPIO_DEVICE_PATH mIOExpanderDevicePathTemplate = {
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
  GPIO_DRIVER_TYPE_IO_EXPANDER,
  {
    END_DEVICE_PATH_TYPE,
    END_ENTIRE_DEVICE_PATH_SUBTYPE,
    {
      sizeof(EFI_DEVICE_PATH_PROTOCOL),
      0
    }
  }
};

MV_IOEXPANDER *mIOExpanderInstance;

#define INFO(NRGPIO, __INT) (NRGPIO | __INT)
static IOEXPANDER_INFO IOExpanderIds[] = {
        {NXP_PCA9505,  INFO(40, PCA_INT)},
        {NXP_PCA9534,  INFO(8, PCA_INT)},
        {NXP_PCA9535,  INFO(16, PCA_INT)},
        {NXP_PCA9536,  INFO(4, 0)},
        {NXP_PCA9537,  INFO(4, PCA_INT)},
        {NXP_PCA9555,  INFO(16,PCA_INT)},
        {}
};


EFI_STATUS
EFIAPI
MvPca95xxGetI2c (
  IN UINT32 DeviceIndex,
  OUT EFI_I2C_IO_PROTOCOL **I2cIo
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  EFI_I2C_IO_PROTOCOL *TempI2cIo;
  EFI_HANDLE *ProtHandle = NULL;
  UINTN HandleSize = 0;
  UINTN Count;

  Status = gBS->LocateHandle (
                    ByProtocol,
                    &gEfiI2cIoProtocolGuid,
                    NULL,
                    &HandleSize,
                    ProtHandle
                    );
  if (Status == EFI_BUFFER_TOO_SMALL) {
    ProtHandle = AllocateZeroPool (HandleSize);
    if (ProtHandle == NULL) {
      return EFI_OUT_OF_RESOURCES;
    }
    Status = gBS->LocateHandle (
                      ByProtocol,
                      &gEfiI2cIoProtocolGuid,
                      NULL,
                      &HandleSize,
                      ProtHandle
                      );
  }

  for (Count = 0; Count < HandleSize/sizeof (EFI_HANDLE); Count++) {
    Status = gBS->OpenProtocol (
                       ProtHandle[Count],
                       &gEfiI2cIoProtocolGuid,
                       (VOID **) &TempI2cIo,
                       gImageHandle,
                       NULL,
                       EFI_OPEN_PROTOCOL_GET_PROTOCOL
                       );

    if (TempI2cIo->DeviceIndex == DeviceIndex) {
      *I2cIo = TempI2cIo;
      break;
    }
  }

  if (Count == HandleSize/sizeof (EFI_HANDLE)) {
    return EFI_NOT_FOUND;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
MvPca95xxI2cTransfer (
  IN EFI_I2C_IO_PROTOCOL *I2cIo,
  IN UINT16 Address,
  IN UINT32 Length,
  IN UINT8 *Buffer,
  IN UINT8 Operation
  )
{
  EFI_I2C_REQUEST_PACKET *RequestPacket;
  UINTN RequestPacketSize;
  EFI_STATUS Status = EFI_SUCCESS;
  UINT32 Transmitted = 0;
  UINT32 CurrentAddress = Address;

  RequestPacketSize = sizeof(UINTN) + sizeof (EFI_I2C_OPERATION) * 2;
  RequestPacket = AllocateZeroPool (RequestPacketSize);
  if (RequestPacket == NULL)
    return EFI_OUT_OF_RESOURCES;
  /* First operation contains address, the second is buffer */
  RequestPacket->OperationCount = 2;
  RequestPacket->Operation[0].LengthInBytes = 1;
  RequestPacket->Operation[0].Buffer = AllocateZeroPool ( RequestPacket->Operation[0].LengthInBytes );
  if (RequestPacket->Operation[0].Buffer == NULL) {
    FreePool(RequestPacket);
    return EFI_OUT_OF_RESOURCES;
  }
  RequestPacket->Operation[1].Flags = (Operation == READ ? I2C_FLAG_READ : I2C_FLAG_NORESTART);

  while (Length > 0) {
    CurrentAddress = Address + Transmitted;
    RequestPacket->Operation[0].Buffer[0] = CurrentAddress & 0xff;
    RequestPacket->Operation[1].LengthInBytes = 1;
    RequestPacket->Operation[1].Buffer = Buffer + Transmitted;
    Status = I2cIo->QueueRequest(I2cIo, 0, NULL, RequestPacket, NULL);
    if (EFI_ERROR(Status)) {
      DEBUG((DEBUG_ERROR, "io-expander: error %d during transmission\n", Status));
      break;
    }
    Length--;
    Transmitted++;
  }

  FreePool(RequestPacket->Operation[0].Buffer);
  FreePool(RequestPacket);
  return Status;
}

STATIC
EFI_STATUS
MvPca95xxReadRegs (
  IN EFI_I2C_IO_PROTOCOL *I2cIo,
  IN UINTN Reg,
  OUT UINT8 *Val
  )
{
  EFI_STATUS Status = EFI_SUCCESS;

  Status = MvPca95xxI2cTransfer(I2cIo, Reg, 1, Val, READ);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Pca95xx Failed to do I2C transfer READ, Status: 0x%x\n", Status));
    return Status;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvPca95xxWriteRegs (
  IN EFI_I2C_IO_PROTOCOL *I2cIo,
  IN UINTN Reg,
  IN UINT8 Val
  )
{
  EFI_STATUS Status = EFI_SUCCESS;

  Status = MvPca95xxI2cTransfer(I2cIo, Reg, 1, &Val, WRITE);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Pca95xx Failed to do I2C transfer WRITE, Status: 0x%x\n", Status));
    return Status;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvPca95xxSetOutputValue (
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin,
  IN  BOOLEAN Value
  )
{
  UINTN Bank = GpioPin / BANK_SZ;
  UINTN Off = GpioPin % BANK_SZ;
  UINT8 Val;
  EFI_STATUS Status;
  UINT32 DeviceIndex;
  EFI_I2C_IO_PROTOCOL *I2cIo = NULL;

  /* Try to Get I2C Protocol */
  DeviceIndex = I2C_DEVICE_INDEX(
          mIOExpanderInstance->IOExpanderControllerDesc[ControllerIndex].IOExpanderI2CBuses,
          mIOExpanderInstance->IOExpanderControllerDesc[ControllerIndex].IOExpanderI2CAddress
          );
  Status = MvPca95xxGetI2c(DeviceIndex, &I2cIo);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to Get I2C protocol, Status: 0x%x\n", Status));
    return EFI_DEVICE_ERROR;
  }

  Status = MvPca95xxReadRegs(I2cIo, (PCA95XX_OUTPUT << 1) + Bank, &Val);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Pca95xx Failed to Read Regs, Status: 0x%x\n", Status));
    return EFI_DEVICE_ERROR;
  }

  if (Value)
    Val |= (1 << Off);
  else
    Val &= (~(1 << Off));

  Status = MvPca95xxWriteRegs(I2cIo, (PCA95XX_OUTPUT << 1) + Bank, Val);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Pca95xx Failed to Write Regs, Status: 0x%x\n", Status));
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvPca95xxSetDirection (
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin,
  IN  BOOLEAN Value
  )
{
  UINTN Bank = GpioPin / BANK_SZ;
  UINTN Off = GpioPin % BANK_SZ;
  UINT8 Val;
  EFI_STATUS Status;
  UINT32 DeviceIndex;
  EFI_I2C_IO_PROTOCOL *I2cIo = NULL;

  DeviceIndex = I2C_DEVICE_INDEX(
          mIOExpanderInstance->IOExpanderControllerDesc[ControllerIndex].IOExpanderI2CBuses,
          mIOExpanderInstance->IOExpanderControllerDesc[ControllerIndex].IOExpanderI2CAddress
          );
  Status = MvPca95xxGetI2c(DeviceIndex, &I2cIo);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to Get I2C protocol, Status: 0x%x\n", Status));
    return Status;
  }

  Status = MvPca95xxReadRegs(I2cIo, (PCA95XX_DIRECTION << 1) + Bank, &Val);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Pca95xx Failed to Read Regs, Status: 0x%x\n", Status));
    return Status;
  }

  if (Value == PCA95XX_DIRECTION_IN)
    Val |= (1 << Off);
  else
    Val &= (~(1 << Off));

  Status = MvPca95xxWriteRegs(I2cIo, (PCA95XX_DIRECTION << 1) + Bank, Val);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Pca95xx Failed to Write Regs, Status: 0x%x\n", Status));
    return Status;
  }

 return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvPca95xxIsOutput (
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin,
  OUT UINT8 *Value
  )
{
  EFI_STATUS Status;
  UINTN Bank = GpioPin / BANK_SZ;
  UINTN Off = GpioPin % BANK_SZ;
  UINT8 Val;
  UINT32 DeviceIndex;
  EFI_I2C_IO_PROTOCOL *I2cIo = NULL;

  DeviceIndex = I2C_DEVICE_INDEX(
          mIOExpanderInstance->IOExpanderControllerDesc[ControllerIndex].IOExpanderI2CBuses,
          mIOExpanderInstance->IOExpanderControllerDesc[ControllerIndex].IOExpanderI2CAddress
          );
  Status = MvPca95xxGetI2c(DeviceIndex, &I2cIo);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to Get I2C protocol, Status: 0x%x\n", Status));
    return Status;
  }

  Status = MvPca95xxReadRegs(I2cIo, (PCA95XX_DIRECTION << 1) + Bank, &Val);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Pca95xx Failed to Read Regs, Status: 0x%x\n", Status));
    return Status;
  }

  *Value = (Val >> Off) & PCA95XX_MASK;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvPca95xxDirectionOutput (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin,
  IN  BOOLEAN Value
  )
{
  EFI_STATUS Status;

  if (ControllerIndex >= PcdGetSize (PcdIoExpanderId)) {
    DEBUG ((DEBUG_ERROR, "Wrong Control ID\n"));
    return EFI_INVALID_PARAMETER;
  }

  if (GpioPin > mIOExpanderInstance->IOExpanderControllerDesc[ControllerIndex].PinCount) {
    DEBUG ((DEBUG_ERROR, "Wrong GPIO PIN number\n"));
    return EFI_INVALID_PARAMETER;
  }

  /* Configure output value. */
  Status = MvPca95xxSetOutputValue(ControllerIndex, GpioPin, Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to Pca95xx Set Output Value, Status: 0x%x\n", Status));
    return EFI_DEVICE_ERROR;
  }

  /* Configure direction as output. */
  Status = MvPca95xxSetDirection(ControllerIndex, GpioPin, PCA95XX_DIRECTION_OUT);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to Pca95xx Set Direction, Status: 0x%x\n", Status));
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvPca95xxDirectionInput (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin
  )
{
  EFI_STATUS Status;

  if (ControllerIndex >= PcdGetSize (PcdIoExpanderId)) {
    DEBUG ((DEBUG_ERROR, "Wrong Control ID\n"));
    return EFI_INVALID_PARAMETER;
  }

  if (GpioPin > mIOExpanderInstance->IOExpanderControllerDesc[ControllerIndex].PinCount) {
    DEBUG ((DEBUG_ERROR, "Wrong GPIO PIN number\n"));
    return EFI_INVALID_PARAMETER;
  }

  /* Configure direction as input. */
  Status = MvPca95xxSetDirection(ControllerIndex, GpioPin, PCA95XX_DIRECTION_IN);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "failed to MvPca95xxSetDirection, Status: 0x%x\n", Status));
    return Status;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvPca95xxGetFunction (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin,
  OUT MARVELL_GPIO_MODE *Mode
  )
{
  EFI_STATUS Status;
  UINT8 Val = 0;

  if (ControllerIndex >= PcdGetSize (PcdIoExpanderId)) {
    DEBUG ((DEBUG_ERROR, "Wrong Control ID\n"));
    return EFI_INVALID_PARAMETER;
  }

  if (GpioPin > mIOExpanderInstance->IOExpanderControllerDesc[ControllerIndex].PinCount) {
    DEBUG ((DEBUG_ERROR, "Wrong GPIO PIN number\n"));
    return EFI_INVALID_PARAMETER;
  }

  Status = MvPca95xxIsOutput(ControllerIndex, GpioPin, &Val);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Pca95xx Failed to get is output or not, Status: 0x%x\n", Status));
    return Status;
  }

  if (Val)
    return GPIO_MODE_INPUT;
  else
    return GPIO_MODE_OUTPUT;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvPca95xxGetValue (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin,
  OUT BOOLEAN *Value
  )
{
  EFI_STATUS Status;
  UINTN Bank = GpioPin / BANK_SZ;
  UINTN Off = GpioPin % BANK_SZ;
  UINT8 Val;
  UINT32 DeviceIndex;
  EFI_I2C_IO_PROTOCOL *I2cIo = NULL;

  if (ControllerIndex >= PcdGetSize (PcdIoExpanderId)) {
    DEBUG ((DEBUG_ERROR, "Wrong Control ID\n"));
    return EFI_INVALID_PARAMETER;
  }

  if (GpioPin > mIOExpanderInstance->IOExpanderControllerDesc[ControllerIndex].PinCount) {
    DEBUG ((DEBUG_ERROR, "Wrong GPIO PIN number\n"));
    return EFI_INVALID_PARAMETER;
  }

  DeviceIndex = I2C_DEVICE_INDEX(
          mIOExpanderInstance->IOExpanderControllerDesc[ControllerIndex].IOExpanderI2CBuses,
          mIOExpanderInstance->IOExpanderControllerDesc[ControllerIndex].IOExpanderI2CAddress
          );
  Status = MvPca95xxGetI2c(DeviceIndex, &I2cIo);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to Get I2C protocol, Status: 0x%x\n", Status));
    return Status;
  }

  Status = MvPca95xxReadRegs(I2cIo, (PCA95XX_INPUT << 1) + Bank, &Val);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Pca95xx failed to Read Regs, Status: 0x%x\n", Status));
    return Status;
  }

  *Value = (Val >> Off) & PCA95XX_MASK;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvPca95xxSetValue (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin,
  IN  BOOLEAN Value
  )
{
  EFI_STATUS Status;

  if (ControllerIndex >= PcdGetSize (PcdIoExpanderId)) {
    DEBUG ((DEBUG_ERROR, "Wrong Control ID\n"));
    return EFI_INVALID_PARAMETER;
  }

  if (GpioPin > mIOExpanderInstance->IOExpanderControllerDesc[ControllerIndex].PinCount) {
    DEBUG ((DEBUG_ERROR, "Wrong GPIO PIN number\n"));
    return EFI_INVALID_PARAMETER;
  }

  Status = MvPca95xxSetOutputValue(ControllerIndex, GpioPin, Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Pca95xx failed to Set Output Value, Status: 0x%x\n", Status));
  }

  return EFI_SUCCESS;
}


STATIC
EFI_STATUS
MvPca95xxInitProtocol (
  IN MARVELL_GPIO_PROTOCOL *IOExpanderProtocol
  )
{
  IOExpanderProtocol->DirectionInput  = MvPca95xxDirectionInput;
  IOExpanderProtocol->DirectionOutput = MvPca95xxDirectionOutput;
  IOExpanderProtocol->GetFunction     = MvPca95xxGetFunction;
  IOExpanderProtocol->GetValue        = MvPca95xxGetValue;
  IOExpanderProtocol->SetValue        = MvPca95xxSetValue;

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
MvPca95xxEntryPoint (
  IN EFI_HANDLE       ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  UINT8 *IOExpanderAddresses;
  UINT8 *IOExpanderBuses;
  UINT8 *ChipId;
  UINT8 Index, Count, Id;
  EFI_STATUS Status;
  MV_GPIO_DEVICE_PATH *IOExpanderDevicePath;
  IOEXPANDER_INFO *Info;


  IOExpanderDevicePath = AllocateCopyPool (sizeof (MV_GPIO_DEVICE_PATH), &mIOExpanderDevicePathTemplate);
  if (IOExpanderDevicePath == NULL) {
    DEBUG ((DEBUG_ERROR, "Allocate IOExpanderDevicePath Fail\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  /* Obtain resource from DSC*/
  ChipId = PcdGetPtr (PcdIoExpanderId);
  if (ChipId == NULL) {
    DEBUG ((DEBUG_ERROR, "Missing PcdI2cIoExpanderId\n"));
    return EFI_INVALID_PARAMETER;
  }

  IOExpanderAddresses = PcdGetPtr (PcdIoExpanderI2cAddress);
  if (IOExpanderAddresses == NULL) {
    DEBUG ((DEBUG_ERROR, "Missing PcdI2cIoExpanderAddress\n"));
    return EFI_INVALID_PARAMETER;
  }

  IOExpanderBuses = PcdGetPtr (PcdIOExpanderI2cBuses);
  if (IOExpanderBuses == NULL) {
    DEBUG ((DEBUG_ERROR, "Missing IOExpanderI2cBuses\n"));
    return EFI_INVALID_PARAMETER;
  }

  mIOExpanderInstance = AllocateZeroPool (sizeof (MV_IOEXPANDER));
  if (mIOExpanderInstance == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  Count = sizeof (IOExpanderIds) / sizeof (IOEXPANDER_INFO);

  EfiInitializeLock (&mIOExpanderInstance->Lock, TPL_NOTIFY);

  MvPca95xxInitProtocol (&mIOExpanderInstance->IOExpanderProtocol);

  mIOExpanderInstance->Signature = GPIO_SIGNATURE;

  /* Initialize IOExpander */
  for (Index = 0; Index < PcdGetSize (PcdIoExpanderId); Index++) {
    Info = IOExpanderIds;
    mIOExpanderInstance->IOExpanderControllerDesc[Index].ChipId = ChipId[Index];
    mIOExpanderInstance->IOExpanderControllerDesc[Index].IOExpanderI2CAddress = IOExpanderAddresses[Index];
    mIOExpanderInstance->IOExpanderControllerDesc[Index].IOExpanderI2CBuses = IOExpanderBuses[Index];
    for (Id = 0; Id < Count; Id++) {
      if (Info->ID == mIOExpanderInstance->IOExpanderControllerDesc[Index].ChipId) {
        mIOExpanderInstance->IOExpanderControllerDesc[Index].PinCount = Info->data & PCA_GPIO_MASK;
        mIOExpanderInstance->IOExpanderControllerDesc[Index].BankCount = (mIOExpanderInstance->IOExpanderControllerDesc[Index].PinCount + (BANK_SZ - 1)) / BANK_SZ;
        break;
      }
      Info++;
    }
    if (Id == Count) {
      DEBUG ((DEBUG_ERROR, "IOExpander ID NOT Support\n"));
      return EFI_INVALID_PARAMETER;
    }
  }

  Status = gBS->InstallMultipleProtocolInterfaces (
                     &(mIOExpanderInstance->ControllerHandle),
                     &gMarvellGpioProtocolGuid,
                     &(mIOExpanderInstance->IOExpanderProtocol),
                     &gEfiDevicePathProtocolGuid,
                     (EFI_DEVICE_PATH_PROTOCOL *) IOExpanderDevicePath,
                     NULL
                     );
  if (EFI_ERROR (Status)) {
    FreePool (mIOExpanderInstance);
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}

