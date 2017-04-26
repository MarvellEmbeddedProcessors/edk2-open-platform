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

#ifndef __PCI_EMULATION_INIT_LIB_H__
#define __PCI_EMULATION_INIT_LIB_H__

#include <Library/NonDiscoverableDeviceRegistrationLib.h>

NON_DISCOVERABLE_DEVICE_INIT
EFIAPI
GetInitializerForType (
  IN  NON_DISCOVERABLE_DEVICE_TYPE  Type,
  IN  UINTN                         Index
  );

#endif
