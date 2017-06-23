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
#include "MvSpiFlash.h"

MARVELL_SPI_MASTER_PROTOCOL *SpiMasterProtocol;
SPI_FLASH_INSTANCE  *mSpiFlashInstance;

#define INFO(JedecId, ExtId, SecSize, NSectors, FlashFlags)  \
  .Id = {                       \
    ((JedecId) >> 16) & 0xff,   \
    ((JedecId) >> 8) & 0xff,    \
    (JedecId) & 0xff,           \
    ((ExtId) >> 8) & 0xff,      \
    (ExtId) & 0xff,             \
    },                          \
  .IdLen = (!(JedecId) ? 0 : (3 + ((ExtId) ? 2 : 0))),  \
  .SectorSize = (SecSize),      \
  .SectorCount = (NSectors),    \
  .PageSize = 256,              \
  .Flags = (FlashFlags),

static SPI_FLASH_INFO SpiFlashIds[] = {
  /* ATMEL */
  {L"at45db011d",        INFO(0x1f2200, 0x0, 64 * 1024,     4, SECT_4K) },
  {L"at45db021d",        INFO(0x1f2300, 0x0, 64 * 1024,     8, SECT_4K) },
  {L"at45db041d",        INFO(0x1f2400, 0x0, 64 * 1024,     8, SECT_4K) },
  {L"at45db081d",        INFO(0x1f2500, 0x0, 64 * 1024,    16, SECT_4K) },
  {L"at45db161d",        INFO(0x1f2600, 0x0, 64 * 1024,    32, SECT_4K) },
  {L"at45db321d",        INFO(0x1f2700, 0x0, 64 * 1024,    64, SECT_4K) },
  {L"at45db641d",        INFO(0x1f2800, 0x0, 64 * 1024,   128, SECT_4K) },
  {L"at25df321a",        INFO(0x1f4701, 0x0, 64 * 1024,    64, SECT_4K) },
  {L"at25df321",         INFO(0x1f4700, 0x0, 64 * 1024,    64, SECT_4K) },
  {L"at26df081a",        INFO(0x1f4501, 0x0, 64 * 1024,    16, SECT_4K) },
  /* EON */
  {L"en25q32b",          INFO(0x1c3016, 0x0, 64 * 1024,    64, 0) },
  {L"en25q64",           INFO(0x1c3017, 0x0, 64 * 1024,   128, SECT_4K) },
  {L"en25q128b",         INFO(0x1c3018, 0x0, 64 * 1024,   256, 0) },
  {L"en25s64",           INFO(0x1c3817, 0x0, 64 * 1024,   128, 0) },
  /* GIGADEVICE */
  {L"gd25q64b",          INFO(0xc84017, 0x0, 64 * 1024,   128, SECT_4K) },
  {L"gd25lq32",          INFO(0xc86016, 0x0, 64 * 1024,    64, SECT_4K) },
  /* ISSI */
  {L"is25lp032",         INFO(0x9d6016, 0x0, 64 * 1024,    64, 0) },
  {L"is25lp064",         INFO(0x9d6017, 0x0, 64 * 1024,   128, 0) },
  {L"is25lp128",         INFO(0x9d6018, 0x0, 64 * 1024,   256, 0) },
  /* MACRONIX */
  {L"mx25l2006e",        INFO(0xc22012, 0x0, 64 * 1024,     4, 0) },
  {L"mx25l4005",         INFO(0xc22013, 0x0, 64 * 1024,     8, 0) },
  {L"mx25l8005",         INFO(0xc22014, 0x0, 64 * 1024,    16, 0) },
  {L"mx25l1605d",        INFO(0xc22015, 0x0, 64 * 1024,    32, 0) },
  {L"mx25l3205d",        INFO(0xc22016, 0x0, 64 * 1024,    64, 0) },
  {L"mx25l6405d",        INFO(0xc22017, 0x0, 64 * 1024,   128, 0) },
  {L"mx25l12805",        INFO(0xc22018, 0x0, 64 * 1024,   256, RD_FULL | WR_QPP) },
  {L"mx25l25635f",       INFO(0xc22019, 0x0, 64 * 1024,   512, RD_FULL | WR_QPP | ADDR_CYC_4) },
  {L"mx25l51235f",       INFO(0xc2201a, 0x0, 64 * 1024,  1024, RD_FULL | WR_QPP) },
  {L"mx25l12855e",       INFO(0xc22618, 0x0, 64 * 1024,   256, RD_FULL | WR_QPP) },
  {L"mx66u51235f",       INFO(0xc2253a, 0x0, 64 * 1024,  1024, RD_FULL | WR_QPP) },
  {L"mx66l1g45g",        INFO(0xc2201b, 0x0, 64 * 1024,  2048, RD_FULL | WR_QPP) },
  /* SPANSION */
  {L"s25fl008a",         INFO(0x010213, 0x0, 64 * 1024,    16, 0) },
  {L"s25fl016a",         INFO(0x010214, 0x0, 64 * 1024,    32, 0) },
  {L"s25fl032a",         INFO(0x010215, 0x0, 64 * 1024,    64, 0) },
  {L"s25fl064a",         INFO(0x010216, 0x0, 64 * 1024,   128, 0) },
  {L"s25fl116k",         INFO(0x014015, 0x0, 64 * 1024,   128, 0) },
  {L"s25fl164k",         INFO(0x014017, 0x0140,  64 * 1024,   128, 0) },
  {L"s25fl128p_256k",    INFO(0x012018, 0x0300, 256 * 1024,    64, RD_FULL | WR_QPP) },
  {L"s25fl128p_64k",     INFO(0x012018, 0x0301,  64 * 1024,   256, RD_FULL | WR_QPP) },
  {L"s25fl032p",         INFO(0x010215, 0x4d00,  64 * 1024,    64, RD_FULL | WR_QPP) },
  {L"s25fl064p",         INFO(0x010216, 0x4d00,  64 * 1024,   128, RD_FULL | WR_QPP) },
  {L"s25fl128s_256k",    INFO(0x012018, 0x4d00, 256 * 1024,    64, RD_FULL | WR_QPP) },
  {L"s25fl128s_64k",     INFO(0x012018, 0x4d01,  64 * 1024,   256, RD_FULL | WR_QPP) },
  {L"s25fl256s_256k",    INFO(0x010219, 0x4d00, 256 * 1024,   128, RD_FULL | WR_QPP) },
  {L"s25fl256s_64k",     INFO(0x010219, 0x4d01,  64 * 1024,   512, RD_FULL | WR_QPP) },
  {L"s25fl512s_256k",    INFO(0x010220, 0x4d00, 256 * 1024,   256, RD_FULL | WR_QPP) },
  {L"s25fl512s_64k",     INFO(0x010220, 0x4d01,  64 * 1024,  1024, RD_FULL | WR_QPP) },
  {L"s25fl512s_512k",    INFO(0x010220, 0x4f00, 256 * 1024,   256, RD_FULL | WR_QPP) },
  /* STMICRO */
  {L"m25p10",            INFO(0x202011, 0x0, 32 * 1024,     4, 0) },
  {L"m25p20",            INFO(0x202012, 0x0, 64 * 1024,     4, 0) },
  {L"m25p40",            INFO(0x202013, 0x0, 64 * 1024,     8, 0) },
  {L"m25p80",            INFO(0x202014, 0x0, 64 * 1024,    16, 0) },
  {L"m25p16",            INFO(0x202015, 0x0, 64 * 1024,    32, 0) },
  {L"m25pE16",           INFO(0x208015, 0x1000, 64 * 1024, 32, 0) },
  {L"m25pX16",           INFO(0x207115, 0x1000, 64 * 1024, 32, RD_QUAD | RD_DUAL) },
  {L"m25p32",            INFO(0x202016, 0x0,  64 * 1024,    64, 0) },
  {L"m25p64",            INFO(0x202017, 0x0,  64 * 1024,   128, 0) },
  {L"m25p128",           INFO(0x202018, 0x0, 256 * 1024,    64, 0) },
  {L"m25pX64",           INFO(0x207117, 0x0,  64 * 1024,   128, SECT_4K) },
  {L"n25q016a",          INFO(0x20bb15, 0x0,  64 * 1024,    32, SECT_4K) },
  {L"n25q32",            INFO(0x20ba16, 0x0,  64 * 1024,    64, RD_FULL | WR_QPP | SECT_4K) },
  {L"n25q32a",           INFO(0x20bb16, 0x0,  64 * 1024,    64, RD_FULL | WR_QPP | SECT_4K) },
  {L"n25q64",            INFO(0x20ba17, 0x0,  64 * 1024,   128, RD_FULL | WR_QPP | SECT_4K) },
  {L"n25q64a",           INFO(0x20bb17, 0x0,  64 * 1024,   128, RD_FULL | WR_QPP | SECT_4K) },
  {L"n25q128",           INFO(0x20ba18, 0x0,  64 * 1024,   256, RD_FULL | WR_QPP) },
  {L"n25q128a",          INFO(0x20bb18, 0x0,  64 * 1024,   256, RD_FULL | WR_QPP) },
  {L"n25q256",           INFO(0x20ba19, 0x0,  64 * 1024,   512, RD_FULL | WR_QPP | SECT_4K) },
  {L"n25q256a",          INFO(0x20bb19, 0x0,  64 * 1024,   512, RD_FULL | WR_QPP | SECT_4K) },
  {L"n25q512",           INFO(0x20ba20, 0x0,  64 * 1024,  1024, RD_FULL | WR_QPP | E_FSR | SECT_4K) },
  {L"n25q512a",          INFO(0x20bb20, 0x0,  64 * 1024,  1024, RD_FULL | WR_QPP | E_FSR | SECT_4K) },
  {L"n25q1024",          INFO(0x20ba21, 0x0,  64 * 1024,  2048, RD_FULL | WR_QPP | E_FSR | SECT_4K | ADDR_CYC_4) },
  {L"n25q1024a",         INFO(0x20bb21, 0x0,  64 * 1024,  2048, RD_FULL | WR_QPP | E_FSR | SECT_4K) },
  {L"mt25qu02g",         INFO(0x20bb22, 0x0,  64 * 1024,  4096, RD_FULL | WR_QPP | E_FSR | SECT_4K) },
  {L"mt25ql02g",         INFO(0x20ba22, 0x0,  64 * 1024,  4096, RD_FULL | WR_QPP | E_FSR | SECT_4K) },
  /* SST */
  {L"sst25vf040b",       INFO(0xbf258d, 0x0,  64 * 1024,     8, SECT_4K | SST_WR) },
  {L"sst25vf080b",       INFO(0xbf258e, 0x0,  64 * 1024,    16, SECT_4K | SST_WR) },
  {L"sst25vf016b",       INFO(0xbf2541, 0x0,  64 * 1024,    32, SECT_4K | SST_WR) },
  {L"sst25vf032b",       INFO(0xbf254a, 0x0,  64 * 1024,    64, SECT_4K | SST_WR) },
  {L"sst25vf064c",       INFO(0xbf254b, 0x0,  64 * 1024,   128, SECT_4K) },
  {L"sst25wf512",        INFO(0xbf2501, 0x0,  64 * 1024,     1, SECT_4K | SST_WR) },
  {L"sst25wf010",        INFO(0xbf2502, 0x0,  64 * 1024,     2, SECT_4K | SST_WR) },
  {L"sst25wf020",        INFO(0xbf2503, 0x0,  64 * 1024,     4, SECT_4K | SST_WR) },
  {L"sst25wf040",        INFO(0xbf2504, 0x0,  64 * 1024,     8, SECT_4K | SST_WR) },
  {L"sst25wf040b",       INFO(0x621613, 0x0,  64 * 1024,     8, SECT_4K) },
  {L"sst25wf080",        INFO(0xbf2505, 0x0,  64 * 1024,    16, SECT_4K | SST_WR) },
  /* WINBOND */
  {L"w25p80",            INFO(0xef2014, 0x0,  64 * 1024,    16, 0) },
  {L"w25p16",            INFO(0xef2015, 0x0,  64 * 1024,    32, 0) },
  {L"w25p32",            INFO(0xef2016, 0x0,  64 * 1024,    64, 0) },
  {L"w25x40",            INFO(0xef3013, 0x0,  64 * 1024,     8, SECT_4K) },
  {L"w25x16",            INFO(0xef3015, 0x0,  64 * 1024,    32, SECT_4K) },
  {L"w25x32",            INFO(0xef3016, 0x0,  64 * 1024,    64, SECT_4K) },
  {L"w25x64",            INFO(0xef3017, 0x0,  64 * 1024,   128, SECT_4K) },
  {L"w25q80bl",          INFO(0xef4014, 0x0,  64 * 1024,    16, RD_FULL | WR_QPP | SECT_4K) },
  {L"w25q16cl",          INFO(0xef4015, 0x0,  64 * 1024,    32, RD_FULL | WR_QPP | SECT_4K) },
  {L"w25q32bv",          INFO(0xef4016, 0x0,  64 * 1024,    64, RD_FULL | WR_QPP | SECT_4K) },
  {L"w25q64cv",          INFO(0xef4017, 0x0,  64 * 1024,   128, RD_FULL | WR_QPP | SECT_4K) },
  {L"w25q128bv",         INFO(0xef4018, 0x0,  64 * 1024,   256, RD_FULL | WR_QPP | SECT_4K) },
  {L"w25q256",           INFO(0xef4019, 0x0,  64 * 1024,   512, RD_FULL | WR_QPP | SECT_4K) },
  {L"w25q80bw",          INFO(0xef5014, 0x0,  64 * 1024,    16, RD_FULL | WR_QPP | SECT_4K) },
  {L"w25q16dw",          INFO(0xef6015, 0x0,  64 * 1024,    32, RD_FULL | WR_QPP | SECT_4K) },
  {L"w25q32dw",          INFO(0xef6016, 0x0,  64 * 1024,    64, RD_FULL | WR_QPP | SECT_4K) },
  {L"w25q64dw",          INFO(0xef6017, 0x0,  64 * 1024,   128, RD_FULL | WR_QPP | SECT_4K) },
  {L"w25q128fw",         INFO(0xef6018, 0x0,  64 * 1024,   256, RD_FULL | WR_QPP | SECT_4K) },
  {},  /* Empty entry to terminate the list */
};

STATIC
VOID
SpiFlashFormatAddress (
  IN      UINT32  Address,
  IN      UINT8   AddrSize,
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

STATIC
EFI_STATUS
MvSpiFlashReadCmd (
  IN  SPI_DEVICE *Slave,
  IN  UINT8 *Cmd,
  IN  UINTN CmdSize,
  OUT UINT8 *DataIn,
  IN  UINTN DataSize
  )
{
  EFI_STATUS Status;

  // Send command and gather response
  Status = SpiMasterProtocol->ReadWrite (SpiMasterProtocol, Slave, Cmd,
    CmdSize, NULL, DataIn, DataSize);

  return Status;
}

STATIC
EFI_STATUS
MvSpiFlashWriteEnableCmd (
  IN  SPI_DEVICE   *Slave
  )
{
  EFI_STATUS Status;
  UINT8 CmdEn = CMD_WRITE_ENABLE;

  // Send write_enable command
  Status = SpiMasterProtocol->Transfer (SpiMasterProtocol, Slave, 1,
    &CmdEn, NULL, SPI_TRANSFER_BEGIN | SPI_TRANSFER_END);

  return Status;
}

STATIC
EFI_STATUS
MvSpiFlashWriteCommon (
  IN SPI_DEVICE *Slave,
  IN UINT8 *Cmd,
  IN UINT32 Length,
  IN UINT8* Buffer,
  IN UINT32 BufferLength
  )
{
  UINT8 CmdStatus = CMD_READ_STATUS;
  UINT8 State;
  UINT32 Counter = 0xFFFFF;
  UINT8 PollBit = STATUS_REG_POLL_WIP;
  UINT8 CheckStatus = 0x0;

  if (Slave->Info->Flags & E_FSR) {
    CmdStatus = CMD_FLAG_STATUS;
    PollBit = STATUS_REG_POLL_PEC;
    CheckStatus = STATUS_REG_POLL_PEC;
  }

  // Send command
  MvSpiFlashWriteEnableCmd (Slave);

  // Write data
  SpiMasterProtocol->ReadWrite (SpiMasterProtocol, Slave, Cmd, Length,
    Buffer, NULL, BufferLength);

  // Poll status register
  SpiMasterProtocol->Transfer (SpiMasterProtocol, Slave, 1, &CmdStatus,
    NULL, SPI_TRANSFER_BEGIN);
  do {
    SpiMasterProtocol->Transfer (SpiMasterProtocol, Slave, 1, NULL, &State,
      0);
    Counter--;
    if ((State & PollBit) == CheckStatus)
      break;
  } while (Counter > 0);
  if (Counter == 0) {
    DEBUG((DEBUG_ERROR, "SpiFlash: Timeout while writing to spi flash\n"));
    return EFI_DEVICE_ERROR;
  }

  // Deactivate CS
  SpiMasterProtocol->Transfer (SpiMasterProtocol, Slave, 0, NULL, NULL, SPI_TRANSFER_END);

  return EFI_SUCCESS;
}

STATIC
VOID
SpiFlashCmdBankaddrWrite (
  IN SPI_DEVICE *Slave,
  IN UINT8 BankSel
  )
{
  UINT8 Cmd = CMD_BANK_WRITE;

  MvSpiFlashWriteCommon (Slave, &Cmd, 1, &BankSel, 1);
}

STATIC
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
MvSpiFlashErase (
  IN SPI_DEVICE *Slave,
  IN UINTN Offset,
  IN UINTN Length
  )
{
  EFI_STATUS Status;
  UINT32 AddrSize, EraseAddr;
  UINTN EraseSize;
  UINT8 Cmd[5];

  if (Slave->Info->Flags & ADDR_CYC_4) {
    AddrSize = 4;
  } else {
    AddrSize = 3;
  }

  if (Slave->Info->Flags & SECT_4K) {
    Cmd[0] = CMD_ERASE_4K;
    EraseSize = SPI_ERASE_SIZE_4K;
  } else {
    Cmd[0] = CMD_ERASE_64K;
    EraseSize = Slave->Info->SectorSize;
  }

  // Check input parameters
  if (Offset % EraseSize || Length % EraseSize) {
    DEBUG((DEBUG_ERROR, "SpiFlash: Either erase offset or length "
      "is not multiple of erase size\n"));
    return EFI_DEVICE_ERROR;
  }

  while (Length) {
    EraseAddr = Offset;

    SpiFlashBank (Slave, EraseAddr);

    SpiFlashFormatAddress (EraseAddr, AddrSize, Cmd);

    // Programm proper erase address
    Status = MvSpiFlashWriteCommon (Slave, Cmd, AddrSize + 1, NULL, 0);
      if (EFI_ERROR (Status)) {
        DEBUG((DEBUG_ERROR, "SpiFlash: Error while programming target address\n"));
        return Status;
      }

    Offset += EraseSize;
    Length -= EraseSize;
  }
  return EFI_SUCCESS;
}

EFI_STATUS
MvSpiFlashRead (
  IN SPI_DEVICE   *Slave,
  IN UINT32       Offset,
  IN UINTN        Length,
  IN VOID         *Buf
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINT8 Cmd[6];
  UINT32 AddrSize, ReadAddr, ReadLength, RemainLength;
  UINTN BankSel = 0;

  if (Slave->Info->Flags & ADDR_CYC_4) {
    AddrSize = 4;
  } else {
    AddrSize = 3;
  }

  Cmd[0] = CMD_READ_ARRAY_FAST;

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
    // Program proper read address and read data
    Status = MvSpiFlashReadCmd (Slave, Cmd, AddrSize + 2, Buf, Length);

    Offset += ReadLength;
    Length -= ReadLength;
    Buf += ReadLength;
  }

  return Status;
}

EFI_STATUS
MvSpiFlashWrite (
  IN SPI_DEVICE *Slave,
  IN UINT32     Offset,
  IN UINTN      Length,
  IN VOID       *Buf
  )
{
  EFI_STATUS Status;
  UINTN ByteAddr, ChunkLength, ActualIndex, PageSize;
  UINT32 WriteAddr;
  UINT8 Cmd[5], AddrSize;

  if (Slave->Info->Flags & ADDR_CYC_4) {
    AddrSize = 4;
  } else {
    AddrSize = 3;
  }

  PageSize = Slave->Info->PageSize;

  Cmd[0] = CMD_PAGE_PROGRAM;

  for (ActualIndex = 0; ActualIndex < Length; ActualIndex += ChunkLength) {
    WriteAddr = Offset;

    SpiFlashBank (Slave, WriteAddr);

    ByteAddr = Offset % PageSize;

    ChunkLength = MIN(Length - ActualIndex, (UINT64) (PageSize - ByteAddr));

    SpiFlashFormatAddress (WriteAddr, AddrSize, Cmd);

    // Program proper write address and write data
    Status = MvSpiFlashWriteCommon (Slave, Cmd, AddrSize + 1, Buf + ActualIndex,
      ChunkLength);
    if (EFI_ERROR (Status)) {
      DEBUG((DEBUG_ERROR, "SpiFlash: Error while programming write address\n"));
      return Status;
    }

    Offset += ChunkLength;
  }
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvSpiFlashUpdateBlock (
  IN SPI_DEVICE *Slave,
  IN UINT32 Offset,
  IN UINTN ToUpdate,
  IN UINT8 *Buf,
  IN UINT8 *TmpBuf,
  IN UINTN EraseSize
  )
{
  EFI_STATUS Status;

  // Read backup
  Status = MvSpiFlashRead (Slave, Offset, EraseSize, TmpBuf);
    if (EFI_ERROR (Status)) {
      DEBUG((DEBUG_ERROR, "SpiFlash: Update: Error while reading old data\n"));
      return Status;
    }

  // Erase entire sector
  Status = MvSpiFlashErase (Slave, Offset, EraseSize);
  if (EFI_ERROR (Status)) {
      DEBUG((DEBUG_ERROR, "SpiFlash: Update: Error while erasing block\n"));
      return Status;
    }

  // Write new data
  MvSpiFlashWrite (Slave, Offset, ToUpdate, Buf);
  if (EFI_ERROR (Status)) {
      DEBUG((DEBUG_ERROR, "SpiFlash: Update: Error while writing new data\n"));
      return Status;
    }

  // Write backup
  if (ToUpdate != EraseSize) {
    Status = MvSpiFlashWrite (Slave, Offset + ToUpdate, EraseSize - ToUpdate,
      &TmpBuf[ToUpdate]);
    if (EFI_ERROR (Status)) {
      DEBUG((DEBUG_ERROR, "SpiFlash: Update: Error while writing backup\n"));
      return Status;
    }
  }

  return EFI_SUCCESS;
}

EFI_STATUS
MvSpiFlashUpdate (
  IN SPI_DEVICE *Slave,
  IN UINT32 Offset,
  IN UINTN ByteCount,
  IN UINT8 *Buf
  )
{
  EFI_STATUS Status;
  UINT64 SectorSize, ToUpdate, Scale = 1;
  UINT8 *TmpBuf, *End;

  SectorSize = Slave->Info->SectorSize;

  End = Buf + ByteCount;

  TmpBuf = (UINT8 *)AllocateZeroPool (SectorSize);
  if (TmpBuf == NULL) {
    DEBUG((DEBUG_ERROR, "SpiFlash: Cannot allocate memory\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  if (End - Buf >= 200)
    Scale = (End - Buf) / 100;

  for (; Buf < End; Buf += ToUpdate, Offset += ToUpdate) {
    ToUpdate = MIN((UINT64)(End - Buf), SectorSize);
    Print (L"   \rUpdating, %d%%", 100 - (End - Buf) / Scale);
    Status = MvSpiFlashUpdateBlock (Slave, Offset, ToUpdate, Buf, TmpBuf, SectorSize);

    if (EFI_ERROR (Status)) {
      DEBUG((DEBUG_ERROR, "SpiFlash: Error while updating\n"));
      return Status;
    }
  }

  Print(L"\n");
  FreePool (TmpBuf);

  return EFI_SUCCESS;
}

STATIC
VOID
MvPrintFlashInfo (
  IN     SPI_FLASH_INFO *Info
  )
{
  UINTN EraseSize;

  if (Info->Flags & SECT_4K) {
    EraseSize = SPI_ERASE_SIZE_4K;
  } else {
    EraseSize = Info->SectorSize;
  }

  DEBUG ((
    DEBUG_ERROR,
    "Detected %s SPI flash with page size %d B, erase size %d KB, total %d MB\n",
    Info->Name,
    Info->PageSize,
    EraseSize / 1024,
    (Info->SectorSize * Info->SectorCount) / 1024 / 1024
    ));
}

EFI_STATUS
EFIAPI
MvSpiFlashReadId (
  IN     SPI_DEVICE *SpiDev
  )
{
  SPI_FLASH_INFO *Info;
  EFI_STATUS Status;
  UINT8 Id[SPI_FLASH_MAX_ID_LEN];
  UINT8 Cmd;

  Cmd = CMD_READ_ID;
  Status = SpiMasterProtocol->ReadWrite (
                    SpiMasterProtocol,
                    SpiDev,
                    &Cmd,
                    SPI_CMD_LEN,
                    NULL,
                    Id,
                    SPI_FLASH_MAX_ID_LEN
                    );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "ReadId: Spi transfer error\n"));
    return Status;
  }

  Info = SpiFlashIds;
  for (; Info->Name != NULL; Info++) {
    if (Info->IdLen != 0) {
      if (CompareMem (Info->Id, Id, Info->IdLen) == 0) {
        SpiDev->Info = Info;
        MvPrintFlashInfo (Info);
        return EFI_SUCCESS;
      }
    }
  }

  DEBUG ((DEBUG_ERROR, "ReadId: Unrecognized JEDEC Id bytes: 0x%02x%02x%02x\n", Id[0], Id[1], Id[2]));

  return EFI_NOT_FOUND;
}

EFI_STATUS
EFIAPI
MvSpiFlashInit (
  IN MARVELL_SPI_FLASH_PROTOCOL *This,
  IN SPI_DEVICE *Slave
  )
{
  EFI_STATUS Status;
  UINT8 Cmd, StatusRegister;
  UINT32 AddrSize;

  if (Slave->Info->Flags & ADDR_CYC_4) {
    AddrSize = 4;
  } else {
    AddrSize = 3;
  }

  if (AddrSize == 4) {
    // Set 4 byte address mode
    Status = MvSpiFlashWriteEnableCmd (Slave);
    if (EFI_ERROR (Status)) {
      DEBUG((DEBUG_ERROR, "SpiFlash: Error while setting write_enable\n"));
      return Status;
    }

    Cmd = CMD_4B_ADDR_ENABLE;
    Status = SpiMasterProtocol->Transfer (SpiMasterProtocol, Slave, 1, &Cmd, NULL,
      SPI_TRANSFER_BEGIN | SPI_TRANSFER_END);
    if (EFI_ERROR (Status)) {
      DEBUG((DEBUG_ERROR, "SpiFlash: Error while setting 4B address\n"));
      return Status;
    }
  }

  // Write flash status register
  Status = MvSpiFlashWriteEnableCmd (Slave);
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "SpiFlash: Error while setting write_enable\n"));
    return Status;
  }

  Cmd = CMD_WRITE_STATUS_REG;
  StatusRegister = 0x0;
  Status = SpiMasterProtocol->ReadWrite (SpiMasterProtocol, Slave, &Cmd, 1,
    &StatusRegister, NULL, 1);
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "SpiFlash: Error with spi transfer\n"));
    return Status;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
MvSpiFlashInitProtocol (
  IN MARVELL_SPI_FLASH_PROTOCOL *SpiFlashProtocol
  )
{

  SpiFlashProtocol->Init = MvSpiFlashInit;
  SpiFlashProtocol->ReadId = MvSpiFlashReadId;
  SpiFlashProtocol->Read = MvSpiFlashRead;
  SpiFlashProtocol->Write = MvSpiFlashWrite;
  SpiFlashProtocol->Erase = MvSpiFlashErase;
  SpiFlashProtocol->Update = MvSpiFlashUpdate;

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
MvSpiFlashEntryPoint (
  IN EFI_HANDLE       ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  EFI_STATUS  Status;

  Status = gBS->LocateProtocol (
    &gMarvellSpiMasterProtocolGuid,
    NULL,
    (VOID **)&SpiMasterProtocol
  );
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "SpiFlash: Cannot locate SPI Master protocol\n"));
    return EFI_DEVICE_ERROR;
  }

  mSpiFlashInstance = AllocateZeroPool (sizeof (SPI_FLASH_INSTANCE));

  if (mSpiFlashInstance == NULL) {
    DEBUG((DEBUG_ERROR, "SpiFlash: Cannot allocate memory\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  MvSpiFlashInitProtocol (&mSpiFlashInstance->SpiFlashProtocol);

  mSpiFlashInstance->Signature = SPI_FLASH_SIGNATURE;

  Status = gBS->InstallMultipleProtocolInterfaces (
                  &(mSpiFlashInstance->Handle),
                  &gMarvellSpiFlashProtocolGuid,
                  &(mSpiFlashInstance->SpiFlashProtocol),
                  NULL
                  );
  if (EFI_ERROR (Status)) {
    FreePool (mSpiFlashInstance);
    DEBUG((DEBUG_ERROR, "SpiFlash: Cannot install SPI flash protocol\n"));
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}
