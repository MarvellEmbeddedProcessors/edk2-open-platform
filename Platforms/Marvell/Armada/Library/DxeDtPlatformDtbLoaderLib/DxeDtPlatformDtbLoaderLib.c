/** @file
*
*  Copyright (c) 2017, Linaro, Ltd. All rights reserved.
*  Copyright (C) 2017, Marvell International Ltd. and its affiliates
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

#include <PiDxe.h>

#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/DxeServicesLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/MvBoardDescLib.h>

/**
  Return a pool allocated copy of the DTB image that is appropriate for
  booting the current platform via DT.

  @param[out]   Dtb                   Pointer to the DTB copy
  @param[out]   DtbSize               Size of the DTB copy

  @retval       EFI_SUCCESS           Operation completed successfully
  @retval       EFI_NOT_FOUND         No suitable DTB image could be located
  @retval       EFI_OUT_OF_RESOURCES  No pool memory available

**/
EFI_STATUS
EFIAPI
DtPlatformLoadDtb (
  OUT   VOID        **Dtb,
  OUT   UINTN       *DtbSize
  )
{
  EFI_STATUS      Status;
  VOID            *OrigDtb;
  VOID            *CopyDtb;
  UINTN           OrigDtbSize;
  UINTN           DtbInstance;
  UINT8           BoardId;

  MVBOARD_ID_GET (BoardId);

  switch (BoardId) {
  case MVBOARD_ID_ARMADA7040_DB:
  case MVBOARD_ID_ARMADA8040_DB:
  case MVBOARD_ID_ARMADA8040_MCBIN:
    DtbInstance = BoardId;
    break;
  default:
    DEBUG ((DEBUG_ERROR, "DtPlatformLoadDtb: Invalid Board Id 0x%x\n", BoardId));
    return EFI_INVALID_PARAMETER;
  }

  Status = GetSectionFromAnyFv (
                   &gDtPlatformDefaultDtbFileGuid,
                   EFI_SECTION_RAW,
                   DtbInstance,
                   &OrigDtb,
                   &OrigDtbSize
                   );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "DtPlatformLoadDtb: No DTB for Board Id 0x%x\n", BoardId));
    return EFI_NOT_FOUND;
  }

  CopyDtb = AllocateCopyPool (OrigDtbSize, OrigDtb);
  if (CopyDtb == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  *Dtb = CopyDtb;
  *DtbSize = OrigDtbSize;

  return EFI_SUCCESS;
}
