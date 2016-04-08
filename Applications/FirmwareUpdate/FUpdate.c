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
#include <Library/FileHandleLib.h>
#include "../../../ShellPkg/Application/Shell/Shell.h"

#define TFTP_CMD_STRING       L"tftp "
#define SPACE_STRING          L" "
#define SF_PROBE_CMD_STRING   L"sf probe"
#define SF_WRITE_CMD_STRING   L"sf updatefile "
#define SF_LOAD_ADDR_STRING   L"0x0"

#define HEADER_SIZE           49152
#define MAIN_HDR_MAGIC        0xB105B002

typedef struct {
  UINT32  Magic;              //  0-3
  UINT32  PrologSize;         //  4-7
  UINT32  PrologChecksum;     //  8-11
  UINT32  BootImageSize;      // 12-15
  UINT32  BootImageChecksum;  // 16-19
  UINT32  Reserved0;          // 20-23
  UINT32  LoadAddr;           // 24-27
  UINT32  ExecAddr;           // 28-31
  UINT8   UartConfig;         //  32
  UINT8   Baudrate;           //  33
  UINT8   ExtCount;           //  34
  UINT8   AuxFlags;           //  35
  UINT32  IoArg0;             // 36-39
  UINT32  IoArg1;             // 40-43
  UINT32  IoArg2;             // 43-47
  UINT32  IoArg3;             // 48-51
  UINT32  Reserved1;          // 52-55
  UINT32  Reserved2;          // 56-59
  UINT32  Reserved3;          // 60-63
} MV_IMAGE_HEADER;

STATIC
UINT32
CountChecksum (
  UINT32 *Start,
  UINT32 Length
  )
{
  UINT32 Sum = 0;
  UINT32 *Startp = Start;

  do {
    Sum += *Startp;
    Startp++;
    Length -= 4;
  } while (Length > 0);

  return Sum;
}

STATIC
EFI_STATUS
CheckImageHeader (
  IN VOID *ImageHeader
  )
{
  MV_IMAGE_HEADER *Header;
  UINT32 HeaderLength, Checksum, ChecksumBackup;

  Header = (MV_IMAGE_HEADER *) ImageHeader;
  HeaderLength = Header->PrologSize;
  ChecksumBackup = Header->PrologChecksum;

  // Compare magic number
  if (Header->Magic != MAIN_HDR_MAGIC) {
    Print (L"fupdate: Bad Image magic 0x%08x != 0x%08x", Header->Magic,
      MAIN_HDR_MAGIC);
    return EFI_DEVICE_ERROR;
  }

  // The checksum field is discarded from calculation
  Header->PrologChecksum = 0;

  Checksum = CountChecksum((UINT32 *)Header, HeaderLength);
  if (Checksum != ChecksumBackup) {
    Print (L"fupdate: Bad Image checksum. 0x%x != 0x%x", Checksum,
      ChecksumBackup);
    return EFI_DEVICE_ERROR;
  }

  // Restore checksum backup
  Header->PrologChecksum = ChecksumBackup;

  return 0;
}

STATIC
EFI_STATUS
CheckFirmwareImage (
  CONST CHAR16* FirmwareImage
  )
{
  EFI_STATUS Status;
  VOID *FileBuffer;
  UINT64 OpenMode;
  UINTN HeaderSize;
  SHELL_FILE_HANDLE FileHandle = NULL;

  OpenMode = EFI_FILE_MODE_READ;
  HeaderSize = HEADER_SIZE;

  FileBuffer = AllocateZeroPool (HEADER_SIZE);

  Status = ShellOpenFileByName (FirmwareImage, &FileHandle, OpenMode, 0);
    if (EFI_ERROR (Status)) {
      Print (L"fupdate: Cannot open Image file\n");
      return EFI_DEVICE_ERROR;
    }

  // Read Image header into buffer
  Status = FileHandleRead (FileHandle, &HeaderSize, FileBuffer);
    if (EFI_ERROR (Status)) {
      Print (L"fupdate: Cannot read Image file header\n");
      ShellCloseFile (&FileHandle);
      FreePool (FileBuffer);
      return EFI_DEVICE_ERROR;
    }

  Status = CheckImageHeader (FileBuffer);
  if (EFI_ERROR(Status)) {
    return EFI_DEVICE_ERROR;
  }

  FreePool (FileBuffer);

  return EFI_SUCCESS;
}

STATIC
CONST CHAR16 *
FileNameFromFilePath (
  CONST CHAR16 *RemoteFilePath
  )
{
  CONST CHAR16 *Walker;

  // Gather FileName from FilePath
  Walker = RemoteFilePath + StrLen (RemoteFilePath);
  while ((--Walker) >= RemoteFilePath) {
    if ((*Walker == L'\\') || (*Walker == L'/')) {
      break;
    }
  }

  return (Walker + 1);
}

STATIC
CONST CHAR16*
PrepareFile (
  LIST_ENTRY *CheckPackage
  )
{
  EFI_STATUS Status;
  CONST CHAR16  *ValueStr;

  ValueStr = ShellCommandLineGetValue (CheckPackage, L"-f");
  if (ValueStr == NULL) {
    Print (L"fupdate: No LocalFilePath parameter!\n");
    return NULL;
  } else {
    Status = ShellIsFile (ValueStr);
    if (EFI_ERROR(Status)) {
      Print (L"fupdate: Wrong LocalFilePath parameter!\n");
      return NULL;
    }
  }
  return ValueStr;
}

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
         "Examples:\n"
         "Update firmware from file fs2:Uefi.img\n"
         "  fupdate -f fs2:Uefi.img\n"
         "Update firmware from file path/Uefi.img TFTP server with IP address 10.0.0.200\n"
         "  fupdate -t 10.0.0.200 path/Uefi.img\n"
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
  CONST CHAR16  *RemoteFilePath, *Host, *FileToWrite;
  UINT8         CmdLen;
  BOOLEAN       FileFlag, TftpFlag;

  Status = ShellInitialize ();
  if (EFI_ERROR (Status)) {
    Print (L"fupdate: Error while initializinf Shell\n");
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
    Print (L"fupdate: Please specify -f or -t flag\n");
    return SHELL_ABORTED;
  } else if (FileFlag && !TftpFlag) {
    // Prepare local file to be burned into flash
    FileToWrite = PrepareFile (CheckPackage);
    if (FileToWrite == NULL) {
      Print (L"fupdate: Error while preparing file for burn\n");
      return SHELL_ABORTED;
    }
  } else if (TftpFlag && !FileFlag) {
    Host = ShellCommandLineGetRawValue (CheckPackage, 1);
    if (Host == NULL) {
      Print (L"fupdate: No Host parameter!\n");
      return SHELL_ABORTED;
    }

    RemoteFilePath = ShellCommandLineGetRawValue (CheckPackage, 2);
    if (RemoteFilePath == NULL) {
      Print (L"fupdate: No remote_file_path parameter!\n");
      return SHELL_ABORTED;
    }

    // Gather firmware image name from remote filepath
    FileToWrite = FileNameFromFilePath (RemoteFilePath);

    // Allocate buffer for tftp command string
    CmdLen = StrSize (TFTP_CMD_STRING) + StrSize (SPACE_STRING) + StrSize (Host) +
      StrSize (RemoteFilePath);
    TftpCmd = (CHAR16 *) AllocateZeroPool (CmdLen + sizeof(CHAR16));
    if (TftpCmd == NULL) {
      Print (L"fupdate: Cannot allocate memory\n");
      return SHELL_ABORTED;
    }

    // Concatenate parameters and form tftp command string
    StrCatS (TftpCmd, CmdLen / sizeof(CHAR16), TFTP_CMD_STRING);
    StrCatS (TftpCmd, CmdLen / sizeof(CHAR16), (CHAR16 *)Host);
    // Insert space
    StrCatS (TftpCmd, CmdLen / sizeof(CHAR16), SPACE_STRING);
    StrCatS (TftpCmd, CmdLen / sizeof(CHAR16), (CHAR16 *)RemoteFilePath);

    RunShellCommand (TftpCmd, &Status);
    FreePool (TftpCmd);
    if (EFI_ERROR(Status)) {
      Print (L"fupdate: Error while performing tftp command\n");
      return SHELL_ABORTED;
    }

  } else {
    Print (L"fupdate: Both -f and -t flag specified, please choose one\n");
    return SHELL_ABORTED;
  }

  // Check image checksum and magic
  Status = CheckFirmwareImage (FileToWrite);
  if (EFI_ERROR(Status)) {
    Print (L"fupdate: Wrong firmware Image\n");
    return SHELL_ABORTED;
  }

  // Probe spi bus
  RunShellCommand (SF_PROBE_CMD_STRING, &Status);
  if (EFI_ERROR(Status)) {
    Print (L"fupdate: Error while performing sf probe\n");
    return SHELL_ABORTED;
  }

  // Allocate buffer for sf command string
  CmdLen = StrSize (SF_WRITE_CMD_STRING) + StrSize (FileToWrite) +
    StrSize (SPACE_STRING) + StrSize (SF_LOAD_ADDR_STRING);
  SfCmd = (CHAR16 *) AllocateZeroPool (CmdLen + sizeof(CHAR16));
  if (SfCmd == NULL) {
    Print (L"fupdate: Cannot allocate memory\n");
    return SHELL_ABORTED;
  }

  // Concatenate parameters and form command string
  StrCatS (SfCmd, CmdLen / sizeof(CHAR16), SF_WRITE_CMD_STRING);
  StrCatS (SfCmd, CmdLen / sizeof(CHAR16), (CHAR16 *)FileToWrite);
  StrCatS (SfCmd, CmdLen / sizeof(CHAR16), SPACE_STRING);
  StrCatS (SfCmd, CmdLen / sizeof(CHAR16), SF_LOAD_ADDR_STRING);

  // Update firmware image in flash
  RunShellCommand (SfCmd, &Status);
  FreePool (SfCmd);
  if (EFI_ERROR(Status)) {
    Print (L"fupdate: Error while performing sf update\n");
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
    Print (L"fupdate: Cannot add Hii package\n");
    return EFI_DEVICE_ERROR;
  }

  Status = ShellCommandRegisterCommandName (
     L"fupdate", ShellCommandRunFUpdate, ShellCommandGetManFileNameFUpdate, 0,
     L"fupdate", TRUE , gShellFUpdateHiiHandle, STRING_TOKEN (STR_GET_HELP_FUPDATE)
     );
  if (EFI_ERROR(Status)) {
    Print (L"fupdate: Error while registering command\n");
    return SHELL_ABORTED;
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
