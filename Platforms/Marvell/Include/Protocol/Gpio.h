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
#ifndef __MARVELL_GPIO_PROTOCOL_H__
#define __MARVELL_GPIO_PROTOCOL_H__

extern EFI_GUID gMarvellGpioProtocolGuid;

typedef struct _MARVELL_GPIO_PROTOCOL MARVELL_GPIO_PROTOCOL;

typedef enum {
  GPIO_MODE_INPUT                 = 0x00,
  GPIO_MODE_OUTPUT                = 0x01
} MARVELL_GPIO_MODE;

typedef enum {
  GPIO_DRIVER_TYPE_SOC_CONTROLLER = 0x00,
  GPIO_DRIVER_TYPE_IO_EXPANDER    = 0x01
} MARVELL_GPIO_DRIVER_TYPE;

typedef enum {
  MV_GPIO_AP_CONTROLLER           = 0x00,
  MV_GPIO_CP0_CONTROLLER0         = 0x01,
  MV_GPIO_CP0_CONTROLLER1         = 0x02,
  MV_GPIO_CP1_CONTROLLER0         = 0x03,
  MV_GPIO_CP1_CONTROLLER1         = 0x04
} MARVELL_GPIO_CONTROLLER;

typedef enum {
  MV_GPIO_IOEXPANDER0             = 0x00,
  MV_GPIO_IOEXPANDER1             = 0x01
} MARVELL_IOEXPANDER;

typedef
EFI_STATUS
(EFIAPI *MV_GPIO_DIRECTION_OUTPUT) (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin,
  IN  BOOLEAN Value
  );

typedef
EFI_STATUS
(EFIAPI *MV_GPIO_DIRECTION_INPUT) (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin
  );

typedef
EFI_STATUS
(EFIAPI *MV_GPIO_GET_FUNCTION) (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin,
  OUT MARVELL_GPIO_MODE *Mode
  );

typedef
EFI_STATUS
(EFIAPI *MV_GPIO_GET_VALUE) (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin,
  OUT BOOLEAN *Value
  );

typedef
EFI_STATUS
(EFIAPI *MV_GPIO_SET_VALUE) (
  IN  MARVELL_GPIO_PROTOCOL *This,
  IN  UINT8 ControllerIndex,
  IN  UINT8 GpioPin,
  IN  BOOLEAN Value
  );

struct _MARVELL_GPIO_PROTOCOL {
  MV_GPIO_DIRECTION_INPUT    DirectionInput;
  MV_GPIO_DIRECTION_OUTPUT   DirectionOutput;
  MV_GPIO_GET_FUNCTION       GetFunction;
  MV_GPIO_GET_VALUE          GetValue;
  MV_GPIO_SET_VALUE          SetValue;
};

typedef struct {
  VENDOR_DEVICE_PATH            Header;
  MARVELL_GPIO_DRIVER_TYPE      GpioDriverType;
  EFI_DEVICE_PATH_PROTOCOL      End;
} MV_GPIO_DEVICE_PATH;

typedef struct {
  MARVELL_GPIO_CONTROLLER       ControllerId;
  UINT8                         PinNumber;
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
MarvellGpioGetHandle (
  IN  MARVELL_GPIO_DRIVER_TYPE      GpioDriverType,
  OUT EFI_HANDLE                    **GpioHandle
  )
{
  EFI_STATUS     Status = EFI_SUCCESS;
  UINTN          Count = 0, HandleSize = 0;
  EFI_HANDLE     *ProtHandle = NULL;
  EFI_DEVICE_PATH_PROTOCOL *DevicePath;

  /* Locate Handles of all MARVELL_GPIO_PROTOCOL producers */
  Status = gBS->LocateHandle (
                  ByProtocol,
                  &gMarvellGpioProtocolGuid,
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
                     &gMarvellGpioProtocolGuid,
                     NULL,
                     &HandleSize,
                     ProtHandle);
  }
  if (EFI_ERROR (Status))  {
    DEBUG ((DEBUG_ERROR, "Failed to locate GPIO protocol, Status: 0x%x\n", Status));
  }

  /* Iterate over all protocol producers */
  for (Count = 0; Count < HandleSize/sizeof (EFI_HANDLE); Count++) {
    Status = gBS->OpenProtocol (
                    ProtHandle[Count],
                    &gMarvellGpioProtocolGuid,
                    NULL,
                    gImageHandle,
                    NULL,
                    EFI_OPEN_PROTOCOL_TEST_PROTOCOL
                    );
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "Failed to open GPIO protocol, Status: 0x%x\n", Status));
      continue;
    }

    /* Open device path protocol */
    Status = gBS->OpenProtocol (
                    ProtHandle[Count],
                    &gEfiDevicePathProtocolGuid,
                    (VOID **)&DevicePath,
                    gImageHandle,
                    NULL,
                    EFI_OPEN_PROTOCOL_GET_PROTOCOL
                    );
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "failed to open GPIO DevicePath protocol, Status: 0x%x\n", Status));
      continue;
    }

    while (!IsDevicePathEndType (DevicePath)) {
      MV_GPIO_DEVICE_PATH *GpioPath = (MV_GPIO_DEVICE_PATH *)DevicePath;

      /* Check if GpioDriverType matches one found in the device path */
      if (GpioPath->GpioDriverType == GpioDriverType) {
        *GpioHandle = ProtHandle[Count];
        gBS->CloseProtocol (
               ProtHandle[Count],
               &gEfiDevicePathProtocolGuid,
               gImageHandle,
               ProtHandle[Count]
               );
        return EFI_SUCCESS;
      }

      DevicePath = NextDevicePathNode (DevicePath);
    }
  }

  DEBUG((DEBUG_ERROR, "failed to match GPIO protocol type: 0x%x\n", GpioDriverType));

  return EFI_UNSUPPORTED;
}

#endif // __MARVELL_GPIO_PROTOCOL_H__
