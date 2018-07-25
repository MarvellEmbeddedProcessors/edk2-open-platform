/** @file
*
*  Copyright (c) 2017, Linaro, Ltd. All rights reserved.
*  Copyright (c) 2018, Marvell International Ltd. and its affiliates
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

#include <IndustryStandard/MvSmc.h>

#include <Library/ArmSmcLib.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/DxeServicesLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>

STATIC EFI_EVENT mArmada7k8kExitBootServicesEvent;

#define DT_ACPI_SELECT_DT       0x0
#define DT_ACPI_SELECT_ACPI     0x1

typedef struct {
  UINT8         Pref;
  UINT8         Reserved[3];
} DT_ACPI_VARSTORE_DATA;

/**
  Disable extra EL3 handling of the PMU interrupts for DT world.

  @param[in]   Event                  Event structure
  @param[in]   Context                Notification context

**/
STATIC
VOID
Armada7k8kExitBootEvent (
  IN EFI_EVENT  Event,
  IN VOID      *Context
  )
{
  ARM_SMC_ARGS SmcRegs = {0};

  SmcRegs.Arg0 = MV_SMC_ID_PMU_IRQ_DISABLE;
  ArmCallSmc (&SmcRegs);

  return;
}

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
  DT_ACPI_VARSTORE_DATA  DtAcpiPref;
  ARM_SMC_ARGS           SmcRegs = {0};
  EFI_STATUS             Status;
  VOID                   *OrigDtb;
  VOID                   *CopyDtb;
  UINTN                  OrigDtbSize;
  UINTN                  BufferSize;

  Status = GetSectionFromAnyFv (&gDtPlatformDefaultDtbFileGuid,
             EFI_SECTION_RAW,
             0,
             &OrigDtb,
             &OrigDtbSize);
  if (EFI_ERROR (Status)) {
    return EFI_NOT_FOUND;
  }

  CopyDtb = AllocateCopyPool (OrigDtbSize, OrigDtb);
  if (CopyDtb == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  *Dtb = CopyDtb;
  *DtbSize = OrigDtbSize;

  /* Enable EL3 handler for regardless HW description */
  SmcRegs.Arg0 = MV_SMC_ID_PMU_IRQ_ENABLE;
  ArmCallSmc (&SmcRegs);

  /*
   * Get the current DT/ACPI preference from the DtAcpiPref variable.
   * Register ExitBootServices event only in case the DT will be used.
   */
  BufferSize = sizeof (DtAcpiPref);
  Status = gRT->GetVariable (L"DtAcpiPref",
                  &gDtPlatformFormSetGuid,
                  NULL,
                  &BufferSize,
                  &DtAcpiPref);
  if (EFI_ERROR (Status) || DtAcpiPref.Pref == DT_ACPI_SELECT_DT) {
    Status = gBS->CreateEvent (EVT_SIGNAL_EXIT_BOOT_SERVICES,
                    TPL_NOTIFY,
                    Armada7k8kExitBootEvent,
                    NULL,
                    &mArmada7k8kExitBootServicesEvent);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "%a: Fail to register EBS event\n", __FUNCTION__));
    }
  }

  return EFI_SUCCESS;
}
