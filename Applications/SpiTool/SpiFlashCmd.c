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
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/PrintLib.h>
#include <Library/UefiLib.h>
#include <Library/ShellCEntryLib.h>
#include <Guid/ShellLibHiiGuid.h>
#include <Library/HiiLib.h>

#include "../../Drivers/Spi/Devices/A8kSpiFlash.h"

EFI_SPI_FLASH_PROTOCOL *SpiFlashProtocol;
EFI_SPI_MASTER_PROTOCOL *SpiMasterProtocol;
CONST CHAR16 gShellSpiFlashFileName[] = L"ShellCommand";

BOOLEAN InitFlag = 1;

STATIC CONST SHELL_PARAM_ITEM ParamList[] = {
  {L"-a", TypeValue},
  {L"-l", TypeValue},
  {L"-o", TypeValue},
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
	 "-l <Length> -o <Offset>\n\n"
	 "Length  - Number of bytes to send\n"
	 "Address - Address in RAM to store/load data\n"
	 "Offset  - Offset from beggining of SPI flash to store/load data\n"
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
  SPI_COMMAND		Cmd;
  SPI_MODE		Mode;
  UINTN			Iterator;
  UINT32		ByteCount, Cs;
  UINT8			*Buffer;
  CHAR16		*ProblemParam;
  CONST CHAR16		*ValueStr;
  BOOLEAN		ReadFlag, WriteFlag, EraseFlag, UpdateFlag, ProbeFlag;
  BOOLEAN		DumpFlag;

  Mode = PcdGet8 (PcdSpiFlashMode);
  Cs   = PcdGet32 (PcdSpiFlashCs);

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

  if (InitFlag && !ProbeFlag) {
    Print (L"Please run sf probe\n");
    return EFI_SUCCESS;
  }

  ValueStr = ShellCommandLineGetValue (CheckPackage, L"-l");
  if (ValueStr == NULL && !ProbeFlag) {
    Print (L"No lenght parameter!\n");
    return SHELL_ABORTED;
  } else {
    ByteCount = ShellStrToUintn (ValueStr);
    if (ByteCount < 0) {
      Print (L"\nWrong parameter Length = %s!\n", ValueStr);
      return SHELL_ABORTED;
    }
  }
  
  if (ReadFlag || WriteFlag || UpdateFlag) {
    ValueStr = ShellCommandLineGetValue (CheckPackage, L"-a");
    if (ValueStr == NULL) {
      Print (L"No address parameter!\n");
      return SHELL_ABORTED;
    }
    Address = ShellHexStrToUintn (ValueStr);
  }

  ValueStr = ShellCommandLineGetValue (CheckPackage, L"-o");
  if (ValueStr == NULL && !ProbeFlag) {
    Print (L"No offset Parameter!\n");
    return SHELL_ABORTED;
  } else {
    Offset = ShellHexStrToUintn (ValueStr);
    if (Offset < 0) {
      Print (L"Wrong offset parameter: %s", ValueStr);
      return SHELL_ABORTED;
    }
  }

  Status = gBS->LocateProtocol (
    &gEfiSpiFlashProtocolGuid,
    NULL,
    (VOID **)&SpiFlashProtocol
  );
  if (Status != EFI_SUCCESS) {
    Print (L"FLASH PROT error!\n");
    return SHELL_ABORTED;
  }

  Status = gBS->LocateProtocol (
    &gEfiSpiMasterProtocolGuid,
    NULL,
    (VOID **)&SpiMasterProtocol
  );
  if (Status != EFI_SUCCESS) {
    Print (L"Master protocol error!\n");
    return SHELL_ABORTED;
  }

  Slave = SpiMasterProtocol->SetupDevice (SpiMasterProtocol, Cs, Mode);
    if (Slave == NULL) {
      Print(L"Cannot allocate SPI device!\n");
    }

  if (ProbeFlag) {
    Status = FlashProbe (Slave);
    if (Status != EFI_SUCCESS) {
      return SHELL_ABORTED;
    } else {
      return Status;
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

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
SpiFlashConstructor (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{

  ShellCommandRegisterCommandName (
     L"sf", ShellCommandRunSpiFlash, ShellCommandGetManFileNameSpiFlash, 0,
     L"sf", TRUE , NULL, STRING_TOKEN (0)
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

  return EFI_SUCCESS;
}
