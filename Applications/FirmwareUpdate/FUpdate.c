/*******************************************************************************
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
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/ShellCommandLib.h>
#include <Library/ShellLib.h>
#include <Library/UefiLib.h>
#include <Library/PrintLib.h>
#include <Library/UefiLib.h>
#include <Library/ShellCEntryLib.h>
#include <Library/HiiLib.h>
#include <Guid/ShellLibHiiGuid.h>
#include <Library/UefiBootServicesTableLib.h>

#define TFTP_CMD_STRING		L"tftp "
#define SPACE_STRING		L" "
#define SF_PROBE_CMD_STRING	L"sf probe"
#define SF_WRITE_CMD_STRING	L"sf update -o 0x0 -f "

CONST CHAR16 gShellFUpdateFileName[] = L"ShellCommand";
EFI_HANDLE gShellFUpdateHiiHandle = NULL;
EFI_HANDLE gShellFUpdateHiiHandle;

STATIC CONST SHELL_PARAM_ITEM ParamList[] = {
  {L"help", TypeFlag},
  {L"-t", TypeFlag},
  {L"-f", TypeValue},
  {NULL , TypeMax}
  };

/**
  Return the file name of the help text file if not using HII.

  @return The string pointer to the file name.
**/
CONST CHAR16*
EFIAPI
ShellCommandGetManFileNameFUpdate (
  VOID
  )
{

  return gShellFUpdateFileName;
}

VOID
FUpdateUsage (
  VOID
  )
{
  Print (L"\nFirmware update command\n"
	 "fupdate [-f <LocalFilePath>] [-t <Host> <RemoteFilePath>]\n\n"
	 "LocalFilePath  - path to local firmware image file\n"
	 "Host           - IP number of TFTP server\n"
	 "RemoteFilePath - path to firmware image file on TFTP server\n"
  );
}

SHELL_STATUS
EFIAPI
ShellCommandRunFUpdate (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS    Status;
  LIST_ENTRY    *CheckPackage;
  CHAR16        *ProblemParam, *TftpCmd = NULL, *SfCmd = NULL;
  CONST CHAR16  *ValueStr, *RemoteFilePath, *Host, *FileToWrite, *Walker;
  UINT8         CmdLen;
  BOOLEAN       FileFlag, TftpFlag;

  Status = ShellInitialize ();
  if (EFI_ERROR (Status)) {
    ASSERT_EFI_ERROR (Status);
    return SHELL_ABORTED;
  }

  Status = ShellCommandLineParse (ParamList, &CheckPackage, &ProblemParam, TRUE);
  if (EFI_ERROR (Status)) {
    Print (L"Parse error!\n");
  }

  if (ShellCommandLineGetFlag (CheckPackage, L"help")) {
    FUpdateUsage();
    return EFI_SUCCESS;
  }

  FileFlag = ShellCommandLineGetFlag (CheckPackage, L"-f");
  TftpFlag = ShellCommandLineGetFlag (CheckPackage, L"-t");

  if (!FileFlag && !TftpFlag) {
    Print (L"Please specify -f or -t flag\n");
    return SHELL_ABORTED;
  } else if (FileFlag && !TftpFlag) {
    ValueStr = ShellCommandLineGetValue (CheckPackage, L"-f");
    if (ValueStr == NULL) {
      Print (L"No LocalFilePath parameter!\n");
      return SHELL_ABORTED;
    } else {
      FileToWrite = ValueStr;
      Status = ShellIsFile (FileToWrite);
      if (EFI_ERROR(Status)) {
        Print (L"Wrong LocalFilePath parameter!\n");
        return SHELL_ABORTED;
      }
    }
  } else if (TftpFlag && !FileFlag) {
    Host = ShellCommandLineGetRawValue (CheckPackage, 1);
    if (Host == NULL) {
      Print (L"No Host parameter!\n");
      return SHELL_ABORTED;
    }

    RemoteFilePath = ShellCommandLineGetRawValue (CheckPackage, 2);
    if (RemoteFilePath == NULL) {
      Print (L"No remote_file_path parameter!\n");
      return SHELL_ABORTED;
    }

    Walker = RemoteFilePath + StrLen (RemoteFilePath);
    while ((--Walker) >= RemoteFilePath) {
      if ((*Walker == L'\\') || (*Walker == L'/')) {
        break;
      }
    }
    FileToWrite = Walker + 1;

    // Allocate buffer for tftp command string
    CmdLen = StrSize (TFTP_CMD_STRING) + StrSize (SPACE_STRING) + StrSize (Host) +
      StrSize (RemoteFilePath);
    TftpCmd = (CHAR16 *) AllocateZeroPool (CmdLen + sizeof(CHAR16));
    if (TftpCmd == NULL) {
      Print (L"Cannot allocate memory\n");
      return SHELL_ABORTED;
    }

    // Concatenate parameters and form command string
    StrCatS (TftpCmd, CmdLen / sizeof(CHAR16), TFTP_CMD_STRING);
    StrCatS (TftpCmd, CmdLen / sizeof(CHAR16), (CHAR16 *)Host);
    // Insert space
    StrCatS (TftpCmd, CmdLen / sizeof(CHAR16), SPACE_STRING);
    StrCatS (TftpCmd, CmdLen / sizeof(CHAR16), (CHAR16 *)RemoteFilePath);

    ShellExecute (ImageHandle, TftpCmd, FALSE, NULL, &Status);
    FreePool (TftpCmd);
    if (EFI_ERROR(Status)) {
      Print (L"Error while performing tftp command\n");
      return SHELL_ABORTED;
    }

  } else {
    Print (L"Both -f and -t flag specified, please choose one\n");
    return SHELL_ABORTED;
  }

  // Probe spi bus
  ShellExecute (ImageHandle, SF_PROBE_CMD_STRING, FALSE, NULL, &Status);
  if (EFI_ERROR(Status)) {
    Print (L"Error while performing sf probe\n");
    return SHELL_ABORTED;
  }

  // Allocate buffer for sf command string
  CmdLen = StrSize (SF_WRITE_CMD_STRING) + StrSize (FileToWrite);
  SfCmd = (CHAR16 *) AllocateZeroPool (CmdLen + sizeof(CHAR16));
  if (SfCmd == NULL) {
    Print (L"Cannot allocate memory\n");
    return SHELL_ABORTED;
  }

  // Concatenate parameters and form command string
  StrCatS (SfCmd, CmdLen / sizeof(CHAR16), SF_WRITE_CMD_STRING);
  StrCatS (SfCmd, CmdLen / sizeof(CHAR16), (CHAR16 *)FileToWrite);

  // Update firmware image in flash
  ShellExecute (ImageHandle, SfCmd, TRUE, NULL, &Status);
  FreePool (SfCmd);
  if (EFI_ERROR(Status)) {
    Print (L"Error while performing sf update\n");
    return SHELL_ABORTED;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
FUpdateConstructor (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS Status;

  gShellFUpdateHiiHandle = NULL;

  gShellFUpdateHiiHandle = HiiAddPackages (
                          &gShellFUpdateHiiGuid, gImageHandle,
                          FUpdateShellStrings, NULL
                          );
  if (gShellFUpdateHiiHandle == NULL) {
    return EFI_DEVICE_ERROR;
  }

  Status = ShellCommandRegisterCommandName (
     L"fupdate", ShellCommandRunFUpdate, ShellCommandGetManFileNameFUpdate, 0,
     L"fupdate", TRUE , gShellFUpdateHiiHandle, STRING_TOKEN (STR_GET_HELP_FUPDATE)
     );
  if (EFI_ERROR(Status)) {
    Print (L"Register error!\n");
  }

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
FUpdateDestructor (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{

  if (gShellFUpdateHiiHandle != NULL) {
    HiiRemovePackages (gShellFUpdateHiiHandle);
  }
  return EFI_SUCCESS;
}
