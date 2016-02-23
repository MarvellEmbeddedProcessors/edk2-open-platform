/********************************************************************************
Copyright (C) 2016 Marvell International Ltd.

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

#include <Uefi.h>

#include <ShellBase.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/ShellCommandLib.h>
#include <Library/ShellLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/PrintLib.h>
#include <Library/HiiLib.h>

#include <Library/UefiLib.h>
#include <Library/ShellCEntryLib.h>
#include <Library/UefiBootServicesTableLib.h>

#include <Protocol/Eeprom.h>

CONST CHAR16 gShellEepromFileName[] = L"ShellCommand";
EFI_HANDLE gShellEepromHiiHandle = NULL;

STATIC CONST SHELL_PARAM_ITEM ParamList[] = {
  {L"-a", TypeValue},
  {L"-l", TypeValue},
  {L"-d", TypeValue},
  {L"-s", TypeValue},
  {L"-c", TypeValue},
  {L"-m", TypeFlag},
  {L"read", TypeFlag},
  {L"write", TypeFlag},
  {L"list", TypeFlag},
  {L"help", TypeFlag},
  {NULL , TypeMax}
  };

/**
  Return the file name of the help text file if not using HII.

  @return The string pointer to the file name.
**/
CONST CHAR16*
EFIAPI
ShellCommandGetManFileNameEeprom (
  VOID
  )
{
  return gShellEepromFileName;
}

VOID
Usage (
  VOID
  )
{
  Print (L"Basic EEPROM commands:\n"
         "eeprom [read] [write] [list] [-m] [-c <Chip>] [-a <Address>] [-l <Length>] [-d <Data>] [-s <Source>]\n"
         "All modes except 'list' require Address, Length and Chip set.\n\n"
         "read    - read from EEPROM\n"
         "write   - write Data to EEPROM, requires -d\n"
         "list    - list available EEPROM devices\n\n"
         "-m      - transfer from/to RAM memory, requires -s\n\n"
         "Chip    - EEPROM unique identifier\n"
         "Address - address in EEPROM to read/write\n"
         "Data    - data to be written\n"
         "Length  - length of data to read/write/copy\n"
         "Source  - address of data in RAM to be copied\n"
  );
}

EFI_STATUS
EepromList (
    )
{
  EFI_HANDLE *HandleBuffer;
  EFI_STATUS Status;
  UINTN ProtocolCount;
  EFI_EEPROM_PROTOCOL *EepromProtocol;
  UINTN i;
  Status = gBS->LocateHandleBuffer ( ByProtocol,
                                     &gEfiEepromProtocolGuid,
                                     NULL,
                                     &ProtocolCount,
                                     &HandleBuffer 
                                   );
  if (ProtocolCount == 0) {
    Print (L"0 devices found.\n");
  } else {
    Print (L"%u devices found: ", ProtocolCount);
  }

  for (i = 0; i < ProtocolCount; i++) {
    Status = gBS->OpenProtocol (
                    HandleBuffer[i],
                    &gEfiEepromProtocolGuid,
                    (VOID **) &EepromProtocol,
                    gImageHandle,
                    NULL,
                    EFI_OPEN_PROTOCOL_BY_HANDLE_PROTOCOL );
    Print (L" 0x%x ", EepromProtocol->Identifier);
    Status = gBS->CloseProtocol ( HandleBuffer[i],
                                  &gEfiEepromProtocolGuid,
                                  gImageHandle,
                                  NULL );
  }
  Print (L"\n");
  return Status;
}

EFI_STATUS
EepromLocateProtocol (
 IN  UINT32 Identifier,
 OUT EFI_HANDLE *FoundHandle,
 OUT EFI_EEPROM_PROTOCOL **FoundEepromProtocol
 )
{
  EFI_HANDLE *HandleBuffer;
  EFI_STATUS Status;
  UINTN ProtocolCount;
  EFI_EEPROM_PROTOCOL *EepromProtocol;
  UINTN i;
  Status = gBS->LocateHandleBuffer ( ByProtocol,
                                     &gEfiEepromProtocolGuid,
                                     NULL,
                                     &ProtocolCount,
                                     &HandleBuffer 
                                   );
  if (EFI_ERROR(Status))
    return Status;

  for (i = 0; i < ProtocolCount; i++) {
    Status = gBS->OpenProtocol (
                    HandleBuffer[i],
                    &gEfiEepromProtocolGuid,
                    (VOID **) &EepromProtocol,
                    gImageHandle,
                    NULL,
                    EFI_OPEN_PROTOCOL_BY_HANDLE_PROTOCOL );
    if (EepromProtocol->Identifier == Identifier) {
      *FoundEepromProtocol = EepromProtocol;
      *FoundHandle = HandleBuffer[i];
      return EFI_SUCCESS;
    }
    Status = gBS->CloseProtocol ( HandleBuffer[i],
                                  &gEfiEepromProtocolGuid,
                                  gImageHandle,
                                  NULL );
  }
  *FoundEepromProtocol = NULL;
  return EFI_UNSUPPORTED;
}

SHELL_STATUS
EFIAPI
ShellCommandRunEeprom (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS              Status;
  LIST_ENTRY              *CheckPackage;
  CHAR16                  *ProblemParam;
  CONST CHAR16            *ValueStr;
  UINTN                   Data, Address, Length, Chip, Source;
  UINT8                   *Buffer;
  BOOLEAN                 ReadMode, MemMode;
  EFI_HANDLE              Handle;
  EFI_EEPROM_PROTOCOL     *EepromProtocol;
  UINTN                   Count;

  Status = ShellInitialize ();
  if (EFI_ERROR (Status)) {
    ASSERT_EFI_ERROR (Status);
    return SHELL_ABORTED;
  }

  Status = ShellCommandLineParse (ParamList, &CheckPackage, &ProblemParam, TRUE);

  if (ShellCommandLineGetFlag (CheckPackage, L"list")) {
    EepromList();
    return SHELL_SUCCESS;
  }

  if (ShellCommandLineGetFlag (CheckPackage, L"help")) {
    Usage();
    return SHELL_SUCCESS;
  }

  ReadMode = ShellCommandLineGetFlag (CheckPackage, L"read");
  if (ReadMode == ShellCommandLineGetFlag (CheckPackage, L"write")) {
    Print (L"Error - type read, write, list or help.\n");
    return SHELL_INVALID_PARAMETER;
  }

  MemMode = ShellCommandLineGetFlag (CheckPackage, L"-m");
  Data = 0;
  if (!ReadMode && !MemMode) {
    ValueStr = ShellCommandLineGetValue (CheckPackage, L"-d");
    if (ValueStr == NULL) {
      Print (L"Error - no data to write.\n");
      return SHELL_INVALID_PARAMETER;
    }
    Data = ShellHexStrToUintn (ValueStr);
  }

  ValueStr = ShellCommandLineGetValue (CheckPackage, L"-a");
  if (ValueStr == NULL) {
    Print (L"Error - no address given.\n");
    return SHELL_INVALID_PARAMETER;
  }
  Address = ShellHexStrToUintn (ValueStr);

  ValueStr = ShellCommandLineGetValue (CheckPackage, L"-l");
  if (ValueStr == NULL) {
    Print (L"Error - no length given.\n");
    return SHELL_INVALID_PARAMETER;
  }
  Length = ShellHexStrToUintn (ValueStr);

  ValueStr = ShellCommandLineGetValue (CheckPackage, L"-c");
  if (ValueStr == NULL) {
    Print (L"Error - no chip ID given.\n");
    return SHELL_INVALID_PARAMETER;
  }
  Chip = ShellHexStrToUintn (ValueStr);

  if (MemMode) {
    ValueStr = ShellCommandLineGetValue (CheckPackage, L"-s");
    if (ValueStr == NULL) {
      Print (L"Error - no memory address given.\n");
      return SHELL_INVALID_PARAMETER;
    }
    Source = ShellHexStrToUintn (ValueStr);
  }

  EepromLocateProtocol (Chip, &Handle, &EepromProtocol);
  if (EepromProtocol == NULL) {
    Print (L"Failed to locate EEPROM protocol.\n");
    return SHELL_INVALID_PARAMETER;
  }

  if (MemMode) {
    Buffer = (VOID *) Source;
  } else {
    Buffer = AllocateZeroPool (Length);
    if (Buffer == NULL) {
      Status = SHELL_OUT_OF_RESOURCES;
      Print (L"Error - out of resources.\n");
      goto out_close;
    }
    if (!ReadMode) {
      for (Count = 0; Count < Length; Count++)
        Buffer[Count] = Data;
    }
  }
  EepromProtocol->Transfer(EepromProtocol, Address, Length, Buffer,
      ReadMode ? EEPROM_READ : EEPROM_WRITE);

  if (MemMode) {
  Print (L"Transfered succesfully.\n");
  } else {
    Print (L"Transfered:\n");
    for (Count = 0; Count < Length; Count++) {
      Print(L"0x%x ", Buffer[Count]);
      if (Count % 8 == 7)
        Print(L"\n");
    }
    Print (L"\n");
  }

  Status = SHELL_SUCCESS;

  if (!MemMode)
    FreePool(Buffer);
out_close:
  gBS->CloseProtocol ( Handle,
                       &gEfiEepromProtocolGuid,
                       gImageHandle,
                       NULL );

  return Status;
}

EFI_STATUS
EFIAPI
EepromCmdConstructor (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{

  gShellEepromHiiHandle = NULL;

  gShellEepromHiiHandle = HiiAddPackages (
			  &gShellEepromHiiGuid, gImageHandle,
			  EepromToolShellStrings, NULL
			  );
  if (gShellEepromHiiHandle == NULL) {
    return EFI_DEVICE_ERROR;
  }
  ShellCommandRegisterCommandName (
     L"eeprom", ShellCommandRunEeprom, ShellCommandGetManFileNameEeprom, 0,
     L"eeprom", TRUE , gShellEepromHiiHandle, STRING_TOKEN (STR_GET_HELP_EEPROM)
     );

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
EepromCmdDestructor (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{

  if (gShellEepromHiiHandle != NULL) {
    HiiRemovePackages (gShellEepromHiiHandle);
  }
  return EFI_SUCCESS;
}
