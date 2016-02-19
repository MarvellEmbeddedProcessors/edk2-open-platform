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
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/ShellCommandLib.h>
#include <Library/ShellLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/PrintLib.h>
#include <Library/ShellCEntryLib.h>
#include <Library/HiiLib.h>
#include <Library/FileHandleLib.h>

#include "../../Drivers/Spi/Devices/A8kSpiFlash.h"

EFI_SPI_FLASH_PROTOCOL *SpiFlashProtocol;
EFI_SPI_MASTER_PROTOCOL *SpiMasterProtocol;

CONST CHAR16 gShellSpiFlashFileName[] = L"ShellCommand";
EFI_HANDLE gShellSfHiiHandle = NULL;

BOOLEAN InitFlag = 1;

STATIC CONST SHELL_PARAM_ITEM ParamList[] = {
  {L"-a", TypeValue},
  {L"-l", TypeValue},
  {L"-o", TypeValue},
  {L"-f", TypeValue},
  {L"-d", TypeFlag},
  {L"read", TypeFlag},
  {L"write", TypeFlag},
  {L"erase", TypeFlag},
  {L"update", TypeFlag},
  {L"probe", TypeFlag},
  {L"help", TypeFlag},
  {NULL , TypeMax}
  };

/**
  Return the file name of the help text file if not using HII.

  @return The string pointer to the file name.
**/
CONST CHAR16*
EFIAPI
ShellCommandGetManFileNameSpiFlash (
  VOID
  )
{

  return gShellSpiFlashFileName;
}

VOID
SfUsage (
  VOID
  )
{
  Print (L"\nBasic SPI command\n"
	 "sf [probe] [read] [write] [erase] [update] -a <Address>"
	 "-l <Length> -o <Offset> -f <FileName>\n\n"
	 "Length   - Number of bytes to send\n"
	 "Address  - Address in RAM to store/load data\n"
	 "Offset   - Offset from beginning of SPI flash to store/load data\n"
	 "FileName - Name of file to read/write/update data into/from flash\n"
  );
}

STATIC
EFI_STATUS
FlashProbe (
  IN SPI_DEVICE	      *Slave
  )
{
  UINT8  IdBuffer[4];
  UINT32 Id, RefId;

  Id = PcdGet32 (PcdSpiFlashId);

  IdBuffer[0] = CMD_READ_ID;

  SpiFlashProtocol->Execute (
    Slave,
    SPI_FLASH_READ_ID,
    0,
    4,
    IdBuffer
    );

  RefId = (IdBuffer[0] << 16) + (IdBuffer[1] << 8) + IdBuffer[2];

  if (RefId == Id) {
    Print (L"Detected supported SPI flash with ID=%3x\n", RefId);
    SpiFlashProtocol->Init (SpiFlashProtocol, Slave);
    InitFlag = 0;
    return EFI_SUCCESS;
  } else if (RefId != 0) {
    Print (L"Unsupported SPI flash detected with ID=%2x\n", RefId);
    return SHELL_ABORTED;
  }

  Print (L"No SPI flash detected");
  return SHELL_ABORTED;
}

SHELL_STATUS
EFIAPI
ShellCommandRunSpiFlash (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS		Status;
  SPI_DEVICE		*Slave;
  LIST_ENTRY		*CheckPackage;
  EFI_PHYSICAL_ADDRESS	Address, Offset;
  SHELL_FILE_HANDLE	FileHandle = NULL;
  SPI_COMMAND		Cmd;
  SPI_MODE		Mode;
  UINTN			Iterator, ByteCount, FileSize;
  UINT64		OpenMode;
  UINT32		Cs;
  UINT8			*Buffer, *FileBuffer;
  CHAR16		*ProblemParam, *FileName;
  CONST CHAR16		*ValueStr;
  BOOLEAN		ReadFlag, WriteFlag, EraseFlag, UpdateFlag, ProbeFlag;
  BOOLEAN		DumpFlag, FileMode;

  Mode = PcdGet8 (PcdSpiFlashMode);
  Cs   = PcdGet32 (PcdSpiFlashCs);

  Status = gBS->LocateProtocol (
    &gEfiSpiFlashProtocolGuid,
    NULL,
    (VOID **)&SpiFlashProtocol
  );
  if (EFI_ERROR(Status)) {
    Print (L"SPI flash protocol error!\n");
    return SHELL_ABORTED;
  }

  Status = gBS->LocateProtocol (
    &gEfiSpiMasterProtocolGuid,
    NULL,
    (VOID **)&SpiMasterProtocol
  );
  if (EFI_ERROR(Status)) {
    Print (L"SPI Master protocol error!\n");
    return SHELL_ABORTED;
  }

  Status = ShellInitialize ();
  if (EFI_ERROR (Status)) {
    ASSERT_EFI_ERROR (Status);
    return SHELL_ABORTED;
  }

  Status = ShellCommandLineParse (ParamList, &CheckPackage, &ProblemParam, TRUE);
  if (EFI_ERROR (Status)) {
    Print (L"\nParse error!\n");
  }

  if (ShellCommandLineGetFlag (CheckPackage, L"help")) {
    SfUsage();
    return EFI_SUCCESS;
  }

  ProbeFlag = ShellCommandLineGetFlag (CheckPackage, L"probe");
  ReadFlag  = ShellCommandLineGetFlag (CheckPackage, L"read");
  WriteFlag = ShellCommandLineGetFlag (CheckPackage, L"write");
  EraseFlag = ShellCommandLineGetFlag (CheckPackage, L"erase");
  UpdateFlag = ShellCommandLineGetFlag (CheckPackage, L"update");
  DumpFlag  = ShellCommandLineGetFlag (CheckPackage, L"-d");
  FileMode = ShellCommandLineGetFlag (CheckPackage, L"-f");

  if (InitFlag && !ProbeFlag) {
    Print (L"Please run sf probe\n");
    return EFI_SUCCESS;
  }

  Slave = SpiMasterProtocol->SetupDevice (SpiMasterProtocol, Cs, Mode);
    if (Slave == NULL) {
      Print(L"Cannot allocate SPI device!\n");
    }

  if (ProbeFlag) {
    Status = FlashProbe (Slave);
    if (EFI_ERROR(Status)) {
      return SHELL_ABORTED;
    } else {
      return Status;
    }
  }

  if (!((WriteFlag || UpdateFlag) && FileMode)) {
    ValueStr = ShellCommandLineGetValue (CheckPackage, L"-l");
    if (ValueStr == NULL) {
      Print (L"No lenght parameter!\n");
      return SHELL_ABORTED;
    } else {
      ByteCount = ShellStrToUintn (ValueStr);
      if (ByteCount < 0) {
        Print (L"\nWrong parameter Length = %s!\n", ValueStr);
        return SHELL_ABORTED;
      }
    }
  }
  
  if ((ReadFlag || WriteFlag || UpdateFlag) && !FileMode) {
    ValueStr = ShellCommandLineGetValue (CheckPackage, L"-a");
    if (ValueStr == NULL) {
      Print (L"No address parameter!\n");
      return SHELL_ABORTED;
    }
    Address = ShellHexStrToUintn (ValueStr);
  }

  ValueStr = ShellCommandLineGetValue (CheckPackage, L"-o");
  if (ValueStr == NULL) {
    Print (L"No offset Parameter!\n");
    return SHELL_ABORTED;
  } else {
    Offset = ShellHexStrToUintn (ValueStr);
    if (Offset < 0) {
      Print (L"Wrong offset parameter: %s", ValueStr);
      return SHELL_ABORTED;
    }
  }

  if (FileMode) {
    ValueStr = ShellCommandLineGetValue (CheckPackage, L"-f");
    if (ValueStr == NULL) {
      Print (L"No FileName parameter!\n");
      return SHELL_ABORTED;
    } else {
      FileName = (CHAR16 *) ValueStr;
      Status = ShellIsFile (FileName);
      if (EFI_ERROR(Status) && !ReadFlag) {
        Print (L"Wrong FileName parameter!\n");
        return SHELL_ABORTED;
      }
    }

    if (WriteFlag || UpdateFlag) {
      OpenMode = EFI_FILE_MODE_READ;
    } else if (ReadFlag) {
      OpenMode = EFI_FILE_MODE_READ | EFI_FILE_MODE_WRITE | EFI_FILE_MODE_CREATE;
    }

    Status = ShellOpenFileByName (FileName, &FileHandle, OpenMode, 0);
    if (EFI_ERROR (Status)) {
      Print (L"Cannot open file\n");
    }

    Status = FileHandleSetPosition(FileHandle, 0);
    if (EFI_ERROR(Status)) {
      Print (L"Cannot set file position to first byte\n");
      return Status;
    }

    if (WriteFlag || UpdateFlag) {
      Status = FileHandleGetSize (FileHandle, &FileSize);
      if (EFI_ERROR (Status)) {
        Print (L"Cannot get file size\n");
      }
      ByteCount = (UINTN) FileSize;
    }

    FileBuffer = AllocateZeroPool ((UINTN) ByteCount);
    if (FileBuffer == NULL) {
      Print (L"Cannot allocate memory\n");
      goto Error_Close_File;
    }

    if (WriteFlag || UpdateFlag) {
      Status = FileHandleRead (FileHandle, &ByteCount, FileBuffer);
      if (EFI_ERROR (Status)) {
        Print (L"Read from file error\n");
        goto Error_Free_Buffer;
      } else if (ByteCount != (UINTN) FileSize) {
        Print (L"Not whole file read. Abort\n");
        goto Error_Free_Buffer;
      }
    }
  }

  if (EraseFlag) {
    Cmd = SPI_FLASH_ERASE;
  }

  if (WriteFlag) {
    Cmd = SPI_FLASH_WRITE;
  }

  if (ReadFlag) {
    Cmd = SPI_FLASH_READ;
  }

  if (UpdateFlag) {
    Cmd = SPI_FLASH_UPDATE;
  }

  Buffer = (UINT8 *) Address;
  if (FileMode) {
    Buffer = FileBuffer;
  }

  Status = SpiFlashProtocol->Execute (
    Slave,
    Cmd,
    Offset,
    ByteCount,
    Buffer
    );

  SpiMasterProtocol->FreeDevice(Slave);

  if (EFI_ERROR (Status)) {
    return SHELL_ABORTED;
  }

  if (EraseFlag) {
    Print (L"\nSF: %d bytes succesfully erased at offset 0x%x\n", ByteCount, Offset);
  }

  if (WriteFlag) {
    Print (L"\nSF: Write %d bytes at offset 0x%x\n", ByteCount, Offset);
  }

  if (UpdateFlag) {
    Print (L"\nSF: Update %d bytes at offset 0x%x\n", ByteCount, Offset);
  }

  if (ReadFlag) {
    Print (L"\nSF: Read %d bytes from offset 0x%x\n", ByteCount, Offset);
    if (DumpFlag) {
      Print (L"Data out:\n");
      Print (L"Bytes order:\n3--2--1--0\n");
      for (Iterator = 0; Iterator < ByteCount; Iterator++) {
        Print (L"%02x ", Buffer[Iterator]);
        if (Iterator % 4 == 3) {
          Print (L"\n");
        }
      }
    Print (L"\n");
    }
  }

  if (FileMode) {
    if (ReadFlag) {
      Status = FileHandleWrite (FileHandle, &ByteCount, FileBuffer);
      if (EFI_ERROR(Status)) {
        Print (L"Error while writing into file\n");
        goto Error_Free_Buffer;
      }
    }

    FreePool (FileBuffer);

    if (FileHandle != NULL) {
      ShellCloseFile (&FileHandle);
    }
  }

  return EFI_SUCCESS;

Error_Free_Buffer:
  FreePool (FileBuffer);
Error_Close_File:
  ShellCloseFile (&FileHandle);
  return SHELL_ABORTED;
}

EFI_STATUS
EFIAPI
SpiFlashConstructor (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  gShellSfHiiHandle = NULL;

  gShellSfHiiHandle = HiiAddPackages (
			  &gShellSfHiiGuid, gImageHandle,
			  SpiFlashShellStrings, NULL
			  );
  if (gShellSfHiiHandle == NULL) {
    return EFI_DEVICE_ERROR;
  }

  ShellCommandRegisterCommandName (
     L"sf", ShellCommandRunSpiFlash, ShellCommandGetManFileNameSpiFlash, 0,
     L"sf", TRUE , gShellSfHiiHandle, STRING_TOKEN (STR_GET_HELP_SF)
     );

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
SpiFlashDestructor (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{

  if (gShellSfHiiHandle != NULL) {
    HiiRemovePackages (gShellSfHiiHandle);
  }
  return EFI_SUCCESS;
}
