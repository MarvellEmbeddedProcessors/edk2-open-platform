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
#include "A8kSpiFlash.h"

EFI_SPI_MASTER_PROTOCOL *SpiMasterProtocol;
SPI_FLASH_INSTANCE  *mSpiFlashInstance;

EFI_STATUS
SpiFlashErase (
  IN SPI_DEVICE *Slave,
  IN UINTN Offset,
  IN UINTN Length
  )
{
  EFI_STATUS Status;
  UINT32 AddrSize, EraseAddr;
  UINTN EraseSize;
  UINT8 Cmd[5];

  AddrSize  = PcdGet32 (PcdSpiFlashAddressCycles);
  EraseSize = PcdGet64 (PcdSpiFlashEraseSize);

  if (Offset % EraseSize || Length % EraseSize) {
    Print (L"SPI flash: Neither erase offset nor length is multiple of erase size");
    return EFI_DEVICE_ERROR;
  }

  Cmd[0] = CMD_ERASE_64K;
  while (Length) {
    EraseAddr = Offset;

    SpiFlashBank (Slave, EraseAddr);

    SpiFlashFormatAddress (EraseAddr, AddrSize, Cmd);

    Status = SpiFlashWriteCommon (Slave, Cmd, AddrSize + 1, NULL, 0);
      if (EFI_ERROR (Status)) {
	return Status;
      }

    Offset += EraseSize;
    Length -= EraseSize;
  }
  return EFI_SUCCESS;
}

VOID
SpiFlashFormatAddress (
  IN	  UINT32  Address,
  IN	  UINT8   AddrSize,
  IN OUT  UINT8   *Cmd
  )
{
  if (AddrSize == 4) {
      Cmd[1] = Address >> 24;
      Cmd[2] = Address >> 16;
      Cmd[3] = Address >> 8;
      Cmd[4] = Address;
  } else {
      Cmd[1] = Address >> 16;
      Cmd[2] = Address >> 8;
      Cmd[3] = Address;
  }
}

EFI_STATUS
SpiFlashRead (
  IN SPI_DEVICE	  *Slave,
  IN UINT32	  Offset,
  IN UINTN	  Length,
  IN VOID	  *Buf
  )
{
  EFI_STATUS Status;
  UINT8 Cmd[6];
  UINT32 AddrSize, ReadAddr, ReadLength, RemainLength;
  UINTN BankSel = 0;

  Cmd[0] = CMD_READ_ARRAY_FAST;
  AddrSize = PcdGet32 (PcdSpiFlashAddressCycles);

  // Sign end of address with 0 byte
  Cmd[5] = 0;

  while (Length) {
    ReadAddr = Offset;

    BankSel = SpiFlashBank (Slave, ReadAddr);

    RemainLength = (SPI_FLASH_16MB_BOUN * (BankSel + 1)) - Offset;
    if (Length < RemainLength) {
      ReadLength = Length;
    } else {
      ReadLength = RemainLength;
    }
    SpiFlashFormatAddress (ReadAddr, AddrSize, Cmd);
    Status = SpiFlashReadCmd (Slave, Cmd, AddrSize + 2, Buf, Length);

    Offset += ReadLength;
    Length -= ReadLength;
    Buf += ReadLength;
  }

  return Status;
}

EFI_STATUS
SpiFlashWriteCommon (
  IN SPI_DEVICE *Slave,
  IN UINT8 *Cmd,
  IN UINT32 Length,
  IN UINT8* Buffer,
  IN UINT32 BufferLength
  )
{
  UINT8 CmdStatus = CMD_FLAG_STATUS;
  UINT8 State;
  UINT32 Counter = 0xFFFFF;

  //Send command
  SpiFlashWriteEnableCmd (Slave);

  SpiMasterProtocol->ReadWrite (SpiMasterProtocol, Slave, Cmd, Length,
    Buffer, NULL, BufferLength);

  //Poll status register
  SpiMasterProtocol->Transfer (SpiMasterProtocol, Slave, 1, &CmdStatus,
    NULL, SPI_TRANSFER_BEGIN);
  do {
    SpiMasterProtocol->Transfer (SpiMasterProtocol, Slave, 1, NULL, &State,
      0);
    Counter--;
    if (((State & STATUS_REG_POLL_BIT) == STATUS_REG_POLL_BIT))
      break;
  } while (Counter > 0);
  if (Counter == 0) {
    return EFI_DEVICE_ERROR;
  }

  SpiMasterProtocol->Transfer (SpiMasterProtocol, Slave, 0, NULL, NULL, SPI_TRANSFER_END);

  return EFI_SUCCESS;
}

VOID
SpiFlashCmdBankaddrWrite (
  IN SPI_DEVICE *Slave,
  IN UINT8 BankSel
  )
{
  UINT8 Cmd = CMD_BANK_WRITE;

  SpiFlashWriteCommon (Slave, &Cmd, 1, &BankSel, 1);
}

UINT8
SpiFlashBank (
  IN SPI_DEVICE *Slave,
  IN UINT32 Offset
  )
{
  UINT8 BankSel;

  BankSel = Offset / SPI_FLASH_16MB_BOUN;

  SpiFlashCmdBankaddrWrite (Slave, BankSel);

  return BankSel;
}

EFI_STATUS
SpiFlashWrite (
  IN SPI_DEVICE	*Slave,
  IN UINT32	Offset,
  IN UINTN	Length,
  IN VOID	*Buf
  )
{
  EFI_STATUS Status;
  UINTN ByteAddr, ChunkLength, ActualIndex, PageSize;
  UINT32 WriteAddr;
  UINT8 Cmd[5], AddrSize;

  AddrSize = PcdGet32 (PcdSpiFlashAddressCycles);
  PageSize = PcdGet32 (PcdSpiFlashPageSize);

  Cmd[0] = CMD_PAGE_PROGRAM;

  for (ActualIndex = 0; ActualIndex < Length; ActualIndex += ChunkLength) {
    WriteAddr = Offset;

    SpiFlashBank (Slave, WriteAddr);

    ByteAddr = Offset % PageSize;

    ChunkLength = MIN(Length - ActualIndex, (UINT64) (PageSize - ByteAddr));

    SpiFlashFormatAddress (WriteAddr, AddrSize, Cmd);

    Status = SpiFlashWriteCommon (Slave, Cmd, AddrSize + 1, Buf + ActualIndex, ChunkLength);
    if (EFI_ERROR (Status)) {
      return Status;
    }

    Offset += ChunkLength;
  }
  return EFI_SUCCESS;
}

EFI_STATUS
SpiFlashUpdateBlock (
  IN SPI_DEVICE	*Slave,
  IN UINT32 Offset,
  IN UINTN ToUpdate,
  IN UINT8 *Buf,
  IN UINT8 *TmpBuf,
  IN UINTN EraseSize
  )
{
  EFI_STATUS Status;

  // Read backup
  Status = SpiFlashRead (Slave, Offset, EraseSize, TmpBuf);
    if (EFI_ERROR (Status)) {
      Print (L"SF Update: Error while reading\n");
      return Status;
    }

  // Erase entire sector
  Status = SpiFlashErase (Slave, Offset, EraseSize);
  if (EFI_ERROR (Status)) {
      Print (L"SF Update: Error while erasing\n");
      return Status;
    }

  // Write new data
  SpiFlashWrite (Slave, Offset, ToUpdate, Buf);
  if (EFI_ERROR (Status)) {
      Print (L"SF Update: Error while writing\n");
      return Status;
    }

  // Write backup
  if (ToUpdate != EraseSize) {
    Status = SpiFlashWrite (Slave, Offset + ToUpdate, EraseSize - ToUpdate,
      &TmpBuf[ToUpdate]);
    if (EFI_ERROR (Status)) {
      Print (L"SF Update: Error while writing\n");
      return Status;
    }
  }

  return EFI_SUCCESS;
}

EFI_STATUS
SpiFlashUpdate (
  IN SPI_DEVICE	*Slave,
  IN UINT32 Offset,
  IN UINTN ByteCount,
  IN UINT8 *Buf
  )
{
  EFI_STATUS Status;
  UINT64 EraseSize, ToUpdate;
  UINT8 *TmpBuf, *End;

  EraseSize = PcdGet64 (PcdSpiFlashEraseSize);

  End = Buf + ByteCount;

  TmpBuf = (UINT8 *)AllocateZeroPool (EraseSize);
  if (TmpBuf == NULL) {
    Print (L"SF update error - out of resources\n");
    return EFI_OUT_OF_RESOURCES;
  }

  for (; Buf < End; Buf += ToUpdate, Offset += ToUpdate) {
    ToUpdate = MIN((UINT64)(End - Buf), EraseSize);
    Status = SpiFlashUpdateBlock (Slave, Offset, ToUpdate, Buf, TmpBuf, EraseSize);
    if (EFI_ERROR (Status)) {
      return Status;
    }
  }

  FreePool (TmpBuf);

  return EFI_SUCCESS;
}

EFI_STATUS
SpiFlashWriteEnableCmd (
  IN  SPI_DEVICE   *Slave
  )
{
  EFI_STATUS Status;
  UINT8 CmdEn = CMD_WRITE_ENABLE;

  Status = SpiMasterProtocol->Transfer (SpiMasterProtocol, Slave, 1,
    &CmdEn, NULL, SPI_TRANSFER_BEGIN | SPI_TRANSFER_END);

  return Status;
}

EFI_STATUS
SpiFlashReadCmd (
  IN  SPI_DEVICE *Slave,
  IN  UINT8 *Cmd,
  IN  UINTN CmdSize,
  OUT UINT8 *DataIn,
  IN  UINTN DataSize
  )
{
  EFI_STATUS Status;

  Status = SpiMasterProtocol->ReadWrite (SpiMasterProtocol, Slave, Cmd,
    CmdSize, NULL, DataIn, DataSize);

  return Status;
}

EFI_STATUS
EFIAPI
EfiSpiFlashExecute (
  IN     SPI_DEVICE         *SpiDev,
  IN     UINT8              Command,
  IN     UINTN              Address,
  IN     UINT32             DataByteCount,
  IN OUT UINT8              *Buffer
  )
{
  EFI_STATUS Status;
  UINT8 *DataOut;

  switch (Command) {
    case SPI_FLASH_READ_ID:
      DataOut = (UINT8 *) AllocateZeroPool (DataByteCount);
      if (DataOut == NULL) {
        return EFI_OUT_OF_RESOURCES;
      }
      SpiMasterProtocol->Transfer (SpiMasterProtocol, SpiDev, DataByteCount,
        Buffer, DataOut, SPI_TRANSFER_BEGIN | SPI_TRANSFER_END);
      // Bytes 1,2 and 3 contain SPI flash ID
      Buffer[0] = DataOut[1];
      Buffer[1] = DataOut[2];
      Buffer[2] = DataOut[3];
      FreePool (DataOut);
      break;
    case SPI_FLASH_READ:
      Status = SpiFlashRead (SpiDev, (UINT32)Address, DataByteCount, Buffer);
      break;
    case SPI_FLASH_WRITE:
      Status = SpiFlashWrite (SpiDev, (UINT32)Address, DataByteCount, Buffer);
      break;
    case SPI_FLASH_ERASE:
      Status = SpiFlashErase (SpiDev, Address, DataByteCount);
      break;
    case SPI_FLASH_UPDATE:
      Status = SpiFlashUpdate (SpiDev, (UINT32)Address, DataByteCount, Buffer);
      break;
    default:
      return EFI_INVALID_PARAMETER;
  }

  return Status;
}

EFI_STATUS
EFIAPI
EfiSpiFlashInit (
  IN EFI_SPI_FLASH_PROTOCOL * This,
  IN SPI_DEVICE	*Slave
  )
{
  EFI_STATUS Status;
  UINT8 Cmd, StatusRegister;
  UINT32 AddrSize;

  AddrSize = PcdGet32 (PcdSpiFlashAddressCycles);
  if (AddrSize == 4) {
  // Set 4 byte address mode
  Status = SpiFlashWriteEnableCmd (Slave);
  if (EFI_ERROR (Status))
    return Status;

  Cmd = CMD_4B_ADDR_ENABLE;
  Status = SpiMasterProtocol->Transfer (SpiMasterProtocol, Slave, 1, &Cmd, NULL,
    SPI_TRANSFER_BEGIN | SPI_TRANSFER_END);
  if (EFI_ERROR (Status))
    return Status;
  }

  // Write flash status register
  Status = SpiFlashWriteEnableCmd (Slave);
  if (EFI_ERROR (Status))
    return Status;

  Cmd = CMD_WRITE_STATUS_REG;
  StatusRegister = 0x0;
  Status = SpiMasterProtocol->ReadWrite (SpiMasterProtocol, Slave, &Cmd, 1,
    &StatusRegister, NULL, 1);
  if (EFI_ERROR (Status))
    return Status;

  return EFI_SUCCESS;
}

EFI_STATUS
SpiFlashInitProtocol (
  IN EFI_SPI_FLASH_PROTOCOL *SpiFlashProtocol
  )
{

  SpiFlashProtocol->Init = EfiSpiFlashInit;
  SpiFlashProtocol->Execute = EfiSpiFlashExecute;

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
SpiFlashEntryPoint (
  IN EFI_HANDLE	      ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  EFI_STATUS  Status;

  Status = gBS->LocateProtocol (
    &gEfiSpiMasterProtocolGuid,
    NULL,
    (VOID **)&SpiMasterProtocol
  );
  if (EFI_ERROR (Status)) {
    Print (L"Cannot locate SPI Master protocol\n");
    return EFI_DEVICE_ERROR;
  }

  mSpiFlashInstance = AllocateZeroPool (sizeof (SPI_FLASH_INSTANCE));

  if (mSpiFlashInstance == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  SpiFlashInitProtocol (&mSpiFlashInstance->SpiFlashProtocol);

  mSpiFlashInstance->Signature = SPI_FLASH_SIGNATURE;

  Status = gBS->InstallMultipleProtocolInterfaces (
                  &(mSpiFlashInstance->Handle),
                  &gEfiSpiFlashProtocolGuid,
                  &(mSpiFlashInstance->SpiFlashProtocol),
                  NULL
                  );
  if (EFI_ERROR (Status)) {
    FreePool (mSpiFlashInstance);
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}
