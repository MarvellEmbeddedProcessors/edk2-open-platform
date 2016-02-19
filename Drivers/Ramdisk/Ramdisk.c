/*
 * Copyright (c) 1999, 2000
 * Intel Corporation.
 * Copyright (C) 2016 Marvell International Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. All advertising materials mentioning features or use of this software must
 *    display the following acknowledgement:
 *
 *    This product includes software developed by Intel Corporation and its
 *    contributors.
 *
 * 4. Neither the name of Intel Corporation or its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY INTEL CORPORATION AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL INTEL CORPORATION OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <Uefi.h>
#include <Uefi/UefiBaseType.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/UefiBootServicesTableLib.h>

#include <Protocol/BlockIo.h>
#include <Protocol/DevicePath.h>

#include "Ramdisk.h"

// EFI device path definition
static RAM_DISK_DEVICE_PATH RamDiskDevicePath =
{
  {
    MESSAGING_DEVICE_PATH,
    MSG_VENDOR_DP,
    {
      (sizeof(RAM_DISK_DEVICE_PATH) - END_DEVICE_PATH_LENGTH),
      0
    }
  },
  {
    // {06ED4DD0-FF78-11d3-BDC4-00A0C94053D1}
    0x06ed4dd0,
    0xff78,
    0x11d3,
    {
      0xbd, 0xc4, 0x0, 0xa0, 0xc9, 0x40, 0x53, 0xd1
    }
  },
  {
    0,0,0,0,0,0,0,0, // ID assigned below
  },
  {
    END_DEVICE_PATH_TYPE,
    END_ENTIRE_DEVICE_PATH_SUBTYPE,
    {
      END_DEVICE_PATH_LENGTH,
      0
    }
  }
};
//
// Lookup table of total sectors vs. cluster size.
// Ramdisk sizes between 0x20D0 (4.1MB) and 0x100000 (512MB) sectors are valid
// FAT16 drive sizes.
//
static FAT16TABLE Fat16Tbl[] =
{
  // {0x000020D0, 0}
  {0x00000800, 1},	// 800 sectors * 1 sec/cluster * 512 bytes = 1 M
  {0x00001000, 1},	// 1000 sectors * 1 sec/cluster * 512 bytes = 2 M
  {0x00001800, 1},	// 1800 sectors * 1 sec/cluster * 512 bytes = 3 M
  {0x00007FA8, 2},
  {0x00040000, 4},
  {0x00080000, 8},
  {0x00100000,16},
  {0xFFFFFFFF, 0}
};

//
// RAM pseudo-boot sector.  No code.
// Needs BS_Sig, BPB_SecPerClus, BPB_TotSec32, BootSec.BPB_TotSec16,
// and BPB_FATSz16 filled out properly by FormatRamdisk().
//
static BOOTSEC BootSec =
{
  /* BS_jmpBoot     */ {0xeb, 0x0, 0x90},
  /* BS_OEMName     */ {'E', 'F', 'I', 'R', 'D', 'I', 'S', 'K'},
  /* BPB_BytsPerSec */ 512,
  /* BPB_SecPerClus */ 0,
  /* BPB_RsvdSecCnt */ 1,
  /* BPB_NumFATs    */ 2,
  /* BPB_RootEntCnt */ 512,
  /* BPB_TotSec16   */ 0,
  /* BPB_Media      */ 0xF8,
  /* BPB_FATSz16    */ 0,
  /* BPB_SecPerTrk  */ 0,
  /* BPB_NumHeads   */ 0,
  /* BPB_HiddSec    */ 0,
  /* BPB_TotSec32   */ 0,
  /* BS_DrvNum      */ 0,
  /* BS_Reserved1   */ 0,
  /* BS_BootSig     */ 0x29,
  /* BS_VolID       */ 0,
  /* BS_VolLab      */ {'N', 'O', ' ', 'N', 'A', 'M', 'E', ' ', ' ', ' '},
  /* BS_FilSysType  */ {'F', 'A', 'T', '1', '6', ' ', ' ', ' '}
};

//
// Helper function to compute cluster size
// vs. total sectors on drive.
//
STATIC
UINT8
Size2spc (
  UINT32 Ts
)
{
  UINTN Iterator = 0;

  while(Fat16Tbl[Iterator].Size != 0xFFFFFFFF)
  {
    if (Ts <= Fat16Tbl[Iterator].Size) {
      return Fat16Tbl[Iterator].Spc;
    }
    ++Iterator;
  }

  return 0;
}

EFI_STATUS
InitializeRamDiskDriver (
  IN EFI_HANDLE       ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  EFI_STATUS   Status;
  RAM_DISK_DEV *RamDiskDev;
  UINT32       RamDiskSize, NumPages, BlockSize;
  UINT64       DiskId;

  //  Set the disk size in MB
  RamDiskSize = PcdGet32 (PcdRamDiskSize) * 1024 * 1024;
  BlockSize   = 512;

  // Allocate storage for ramdisk device info on the heap.
  RamDiskDev = AllocateZeroPool (sizeof(RAM_DISK_DEV));
  if (RamDiskDev == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  //
  // Compute the number of 4KB pages needed by the ramdisk and allocate the
  // memory.
  //
  NumPages = RamDiskSize / EFI_PAGE_SIZE;
  if (NumPages % RamDiskSize) {
    NumPages++;
  }

  Status = gBS->AllocatePages (AllocateAnyPages, EfiBootServicesData, NumPages,
    &RamDiskDev->Start);
  if (EFI_ERROR(Status)) {
    FreePool(RamDiskDev);
    return Status;
  }

  // Initialize the ramdisk's device info.
  (void) gBS->GetNextMonotonicCount (&DiskId);
  CopyMem (&RamDiskDevicePath.DiskId, &DiskId, sizeof(DiskId));

  RamDiskDev->Signature            = PBLOCK_DEVICE_SIGNATURE;
  RamDiskDev->BlkIo.Revision       = EFI_BLOCK_IO_INTERFACE_REVISION;
  RamDiskDev->BlkIo.Media          = &RamDiskDev->Media;
  RamDiskDev->Media.RemovableMedia = FALSE;
  RamDiskDev->Media.MediaPresent   = TRUE;

  RamDiskDev->Media.LastBlock        = RamDiskSize/BlockSize - 1;
  RamDiskDev->Media.BlockSize        = BlockSize;
  RamDiskDev->Media.LogicalPartition = TRUE;
  RamDiskDev->Media.ReadOnly         = FALSE;
  RamDiskDev->Media.WriteCaching     = TRUE;

  RamDiskDev->BlkIo.ReadBlocks  = RamDiskReadBlocks;
  RamDiskDev->BlkIo.WriteBlocks = RamDiskWriteBlocks;
  RamDiskDev->BlkIo.FlushBlocks = RamDiskFlushBlocks;

  RamDiskDev->DevicePath = (EFI_DEVICE_PATH_PROTOCOL *)&RamDiskDevicePath;


  // Build a FAT16 file system on the ramdisk.
  FormatRamdisk ((VOID*)RamDiskDev->Start,RamDiskSize);

  // Install the device.
  Status = gBS->InstallMultipleProtocolInterfaces (
    &ImageHandle,
    &gEfiBlockIoProtocolGuid,
    &RamDiskDev->BlkIo,
    &gEfiDevicePathProtocolGuid,
    RamDiskDev->DevicePath,
    NULL);

  return Status;
}

//
// Given a block of memory representing a ramdisk, build a pseudo-boot sector
// and initialize the drive.
//
// Assumes the global boot sector structure BootSec has been filled out with the
// static information the boot sector requires.  Also assumes the ramdisk size
// is between 4.1MB and 512MB as appropriate for FAT16 file system.
//
STATIC
VOID
FormatRamdisk (
  IN VOID*  Start,
  IN UINT32 Size
  )
{
  UINT32 TotalSectors, RootDirSectors, FatSz, Tmp1, Tmp2;
  UINT8 *Fat1, *Fat2;

  // The boot signature needs to be filled out
  BootSec.BS_Sig = 0xAA55;

  // Compute the total sectors and appropriate cluster size
  TotalSectors = Size / BootSec.BPB_BytsPerSec;
  BootSec.BPB_SecPerClus = Size2spc (TotalSectors);
  ASSERT (BootSec.BPB_SecPerClus != 0);

  // Compute how many root directory sectors are needed
  RootDirSectors = (BootSec.BPB_RootEntCnt * 32 + BootSec.BPB_BytsPerSec - 1) /
    BootSec.BPB_BytsPerSec;

  /* Compute how many sectors are required per FAT */
  Tmp1 = TotalSectors - (BootSec.BPB_RsvdSecCnt + RootDirSectors);
  Tmp2 = 256 * BootSec.BPB_SecPerClus + BootSec.BPB_NumFATs;
  FatSz = (Tmp1 + Tmp2 - 1) / Tmp2;
  ASSERT (FatSz <= 0xFFFF);

  // Store the total sectors and fat size values
  if (TotalSectors > 0xFFFF) {
    BootSec.BPB_TotSec32 = TotalSectors;
  } else {
    BootSec.BPB_TotSec16 = (UINT16) TotalSectors;
  }

  BootSec.BPB_FATSz16 = (UINT16) FatSz;

  //
  // The FAT table and root directory need to be all zeroes.
  // We'll zero the whole drive.
  //
  ZeroMem (Start, Size);

  // Write the completed boot sector to the ramdisk
  CopyMem (Start, &BootSec, 512);

  // Compute the starting offsets of the two FATs
  Fat1 = (UINT8*) Start + BootSec.BPB_RsvdSecCnt * 512;
  Fat2 = (UINT8*) Start + (BootSec.BPB_RsvdSecCnt + FatSz) * 512;

  // Initialize FAT1
  Fat1[0] = BootSec.BPB_Media;
  Fat1[1] = 0xFF;
  Fat1[2] = 0xFF;
  Fat1[3] = 0xFF;

  // Initialize FAT2
  Fat2[0] = BootSec.BPB_Media;
  Fat2[1] = 0xFF;
  Fat2[2] = 0xFF;
  Fat2[3] = 0xFF;
}

// Implementation of block I/O read
STATIC
EFI_STATUS
RamDiskReadBlocks (
  IN EFI_BLOCK_IO *This,
  IN UINT32       MediaId,
  IN EFI_LBA      LBA,
  IN UINTN        BufferSize,
  OUT VOID        *Buffer
)
{
  EFI_BLOCK_IO_MEDIA   *Media;
  RAM_DISK_DEV         *RamDiskDev;
  EFI_PHYSICAL_ADDRESS RamDiskLBA;

  Media = This->Media;

  if (BufferSize % Media->BlockSize != 0) {
    return EFI_BAD_BUFFER_SIZE;
  }

  if (LBA > Media->LastBlock) {
    return EFI_DEVICE_ERROR;
  }

  if (LBA + BufferSize / Media->BlockSize - 1 > Media->LastBlock) {
    return EFI_DEVICE_ERROR;
  }

  RamDiskDev = RAM_DISK_FROM_THIS(This);
  RamDiskLBA = RamDiskDev->Start + MultU64x32 (LBA, Media->BlockSize);
  CopyMem (Buffer, (VOID*) RamDiskLBA, BufferSize);

  return EFI_SUCCESS;
}

// Implementation of block I/O write
STATIC
EFI_STATUS
RamDiskWriteBlocks (
  IN EFI_BLOCK_IO *This,
  IN UINT32       MediaId,
  IN EFI_LBA      LBA,
  IN UINTN        BufferSize,
  IN VOID         *Buffer
)
{
  EFI_BLOCK_IO_MEDIA   *Media;
  RAM_DISK_DEV         *RamDiskDev;
  EFI_PHYSICAL_ADDRESS RamDiskLBA;

  Media = This->Media;
  if (Media->ReadOnly) {
    return EFI_WRITE_PROTECTED;
  }

  if (BufferSize % Media->BlockSize != 0) {
    return EFI_BAD_BUFFER_SIZE;
  }

  if (LBA > Media->LastBlock) {
    return EFI_DEVICE_ERROR;
  }

  if (LBA + BufferSize / Media->BlockSize - 1 > Media->LastBlock) {
    return EFI_DEVICE_ERROR;
  }

  RamDiskDev = RAM_DISK_FROM_THIS(This);
  RamDiskLBA = RamDiskDev->Start + MultU64x32 (LBA, Media->BlockSize);
  CopyMem ((VOID*) RamDiskLBA, Buffer, BufferSize);

  return EFI_SUCCESS;
}

// Implementation of block I/O flush
STATIC
EFI_STATUS
RamDiskFlushBlocks (
  IN EFI_BLOCK_IO *This
)
{

  return EFI_SUCCESS;
}
