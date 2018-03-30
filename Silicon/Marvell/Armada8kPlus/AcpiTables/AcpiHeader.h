/** @file

  Multiple APIC Description Table (MADT)

  Copyright (c) 2017, Linaro Ltd. All rights reserved.<BR>

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <IndustryStandard/Acpi.h>

#define ACPI_OEM_ID_ARRAY        {'M','V','E','B','U',' '}
#define ACPI_OEM_REVISION        0
#define ACPI_CREATOR_ID          SIGNATURE_32('L','N','R','O')
#define ACPI_CREATOR_REVISION    0

#define ACPI_OEM_TABLE_ID SIGNATURE_64('A','R','M','A','D','A','8','K')

/**
 * A macro to initialize the common header part of EFI ACPI tables
 * as defined by EFI_ACPI_DESCRIPTION_HEADER structure.
 **/
#define __ACPI_HEADER(sign, type, rev) {                \
  sign,                   /* UINT32  Signature */       \
  sizeof (type),          /* UINT32  Length */          \
  rev,                    /* UINT8   Revision */        \
  0,                      /* UINT8   Checksum */        \
  ACPI_OEM_ID_ARRAY,      /* UINT8   OemId[6] */        \
  ACPI_OEM_TABLE_ID,      /* UINT64  OemTableId */      \
  ACPI_OEM_REVISION,      /* UINT32  OemRevision */     \
  ACPI_CREATOR_ID,        /* UINT32  CreatorId */       \
  ACPI_CREATOR_REVISION   /* UINT32  CreatorRevision */ \
  }

#define EFI_ACPI_6_0_GIC_REDISTRIBUTOR_INIT(RedisRegionAddr, RedisDiscLength) \
  {                                                                           \
    EFI_ACPI_6_0_GICR,                                                        \
    sizeof (EFI_ACPI_6_0_GICR_STRUCTURE),                                     \
    0,                                                                        \
    RedisRegionAddr,                                                          \
    RedisDiscLength                                                           \
  }

#define EFI_ACPI_6_0_GIC_ITS_FRAME_INIT(Id, PhysAddress)                      \
  {                                                                           \
    EFI_ACPI_6_0_GIC_ITS,                                                     \
    sizeof (EFI_ACPI_6_0_GIC_ITS_STRUCTURE),                                  \
    0,                                                                        \
    Id,                                                                       \
    PhysAddress,                                                              \
    0                                                                         \
  }
