/**
*
*  Copyright (C) 2018, Marvell International Ltd. and its affiliates.
*
*  This program and the accompanying materials are licensed and made available
*  under the terms and conditions of the BSD License which accompanies this
*  distribution. The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
*  Glossary - abbreviations used in Marvell SampleAtReset library implementation:
*  AP - Application Processor hardware block (Armada 7k8k incorporates AP806)
*  CP - South Bridge hardware blocks (Armada 7k8k incorporates CP110)
**/

#include <Uefi.h>

#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>

#include <Protocol/BoardDesc.h>

//
// Common macros
//
#define MV_SOC_CP_BASE(Cp)               (0xF2000000 + (Cp) * 0x2000000)

//
// Platform description of UTMI PHY's
//
#define MV_SOC_UTMI_PER_CP_COUNT         2
#define MV_SOC_UTMI_ID(Utmi)             (Utmi)
#define MV_SOC_UTMI_BASE(Utmi)           (0x580000 + (Utmi) * 0x1000)
#define MV_SOC_UTMI_CFG_BASE             0x440440
#define MV_SOC_UTMI_USB_CFG_BASE         0x440420

EFI_STATUS
EFIAPI
ArmadaSoCDescUtmiGet (
  IN OUT MV_SOC_UTMI_DESC  **UtmiDesc,
  IN OUT UINT8              *DescCount
  )
{
  MV_SOC_UTMI_DESC *Desc;
  UINT8 CpCount = FixedPcdGet8 (PcdMaxCpCount);
  UINT8 Index, CpIndex, UtmiIndex = 0;

  Desc = AllocateZeroPool (CpCount * MV_SOC_UTMI_PER_CP_COUNT *
                           sizeof (MV_SOC_UTMI_DESC));
  if (Desc == NULL) {
    DEBUG ((DEBUG_ERROR, "%a: Cannot allocate memory\n", __FUNCTION__));
    return EFI_OUT_OF_RESOURCES;
  }

  for (CpIndex = 0; CpIndex < CpCount; CpIndex++) {
    for (Index = 0; Index < MV_SOC_UTMI_PER_CP_COUNT; Index++) {
      Desc[UtmiIndex].UtmiPhyId = MV_SOC_UTMI_ID (UtmiIndex);
      Desc[UtmiIndex].UtmiBaseAddress =
                             MV_SOC_CP_BASE (CpIndex) + MV_SOC_UTMI_BASE (Index);
      Desc[UtmiIndex].UtmiConfigAddress =
                                 MV_SOC_CP_BASE (CpIndex) + MV_SOC_UTMI_CFG_BASE;
      Desc[UtmiIndex].UsbConfigAddress =
                             MV_SOC_CP_BASE (CpIndex) + MV_SOC_UTMI_USB_CFG_BASE;
      UtmiIndex++;
    }
  }

  *UtmiDesc = Desc;
  *DescCount = UtmiIndex;

  return EFI_SUCCESS;
}
