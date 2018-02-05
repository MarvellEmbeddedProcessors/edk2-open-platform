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
#ifndef __MV_GPIO_H__
#define __MV_GPIO_H__


#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>

#include <Protocol/BoardDesc.h>
#include <Protocol/Gpio.h>

#include <Uefi/UefiBaseType.h>

#define GPIO_SIGNATURE                   SIGNATURE_32 ('G', 'P', 'I', 'O')

#ifndef BIT
#define BIT(nr)                          (1 << (nr))
#endif

// Marvell GPIO Controller Registers
#define GPIO_DATA_OUT_REG                (0x0)
#define GPIO_OUT_EN_REG                  (0x4)
#define GPIO_BLINK_EN_REG                (0x8)
#define GPIO_DATA_IN_POL_REG             (0xc)
#define GPIO_DATA_IN_REG                 (0x10)

typedef struct {
  UINTN                   BaseAddress;
  UINTN                   PinCount;
} MV_GPIO_CONTROLLER_DESC;

typedef struct {
  MARVELL_GPIO_PROTOCOL   GpioProtocol;
  MV_GPIO_CONTROLLER_DESC GpioControllerDesc[MVHW_MAX_GPIO_DEVS];
  UINT8                   MaxControllerIndex;
  UINTN                   Signature;
  EFI_HANDLE              Handle;
  EFI_LOCK                Lock;
} MV_GPIO;

#endif // __MV_GPIO_H__
