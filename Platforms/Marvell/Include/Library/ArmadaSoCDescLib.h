/**
*
*  Copyright (C) 2018, Marvell International Ltd. and its affiliates
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
#ifndef __ARMADA_SOC_DESC_LIB_H__
#define __ARMADA_SOC_DESC_LIB_H__

//
// GPIO devices description template definition
//
#define MVHW_MAX_GPIO_DEVS       20
typedef struct {
  UINT8 GpioDevCount;
  UINTN GpioBaseAddresses[MVHW_MAX_GPIO_DEVS];
  UINTN GpioPinCount[MVHW_MAX_GPIO_DEVS];
} MVHW_GPIO_DESC;

EFI_STATUS
EFIAPI
ArmadaSoCDescGpioGet (
  IN OUT MVHW_GPIO_DESC **GpioDesc
  );

#endif /* __ARMADA_SOC_DESC_LIB_H__ */
