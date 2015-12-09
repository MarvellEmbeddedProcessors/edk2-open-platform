/** @file
*
*  Copyright (c) 2015, Hisilicon Limited. All rights reserved.
*  Copyright (c) 2015, Linaro Limited. All rights reserved.
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#ifndef _UPDATE_SMBIOS_TYPE9_H_
#define _UPDATE_SMBIOS_TYPE9_H_

#include <Uefi.h>
#include <PiDxe.h>
#include <Protocol/PciIo.h>
#include <Protocol/PciRootBridgeIo.h>
#include <Protocol/Smbios.h>
#include <IndustryStandard/SmBios.h>
#include <IndustryStandard/Pci.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiBootServicesTableLib.h>

#define MAX_SMBIOS_STRING_NUMBER 0xFF

typedef struct {
  SMBIOS_TABLE_TYPE9 SmbiosType9;
  CHAR8              String[MAX_SMBIOS_STRING_NUMBER];
}SMBIOS_TABLE_TYPE9_WITH_STRING;

extern EFI_GUID gEfiPciIoProtocolGuid;

#endif
