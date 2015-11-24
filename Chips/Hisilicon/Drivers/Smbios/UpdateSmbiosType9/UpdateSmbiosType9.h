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
#include <Library/HwSafeOperationLib.h>

#define MAX_SMBIOS_STRING_NUMBER 0xFF

typedef struct {
  SMBIOS_TABLE_TYPE9 SmbiosType9;
  CHAR8              String[MAX_SMBIOS_STRING_NUMBER];
}SMBIOS_TABLE_TYPE9_WITH_STRING;

extern EFI_GUID gEfiPciIoProtocolGuid;

#endif
