/** @file
*
*  Copyright (c) 2011-2015, ARM Limited. All rights reserved.
*  Copyright (c) 2015, Hisilicon Limited. All rights reserved.
*  Copyright (c) 2015, Linaro Limited. All rights reserved.
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



#ifndef __ACPI_NEXT_LIB_H__
#define __ACPI_NEXT_LIB_H__

#define EFI_ACPI_6_0_GIC_ITS_INIT(GicITSHwId, GicITSBase) \
  { \
    EFI_ACPI_6_0_GIC_ITS, sizeof (EFI_ACPI_6_0_GIC_ITS_STRUCTURE), EFI_ACPI_RESERVED_WORD, \
    GicITSHwId, GicITSBase, EFI_ACPI_RESERVED_DWORD\
  }

#define EFI_ACPI_5_1_GICR_STRUCTURE_INIT(    \
    GicRBase, GicRlength)                                   \
  {                                                                                  \
    EFI_ACPI_5_1_GICR, sizeof (EFI_ACPI_5_1_GICR_STRUCTURE), EFI_ACPI_RESERVED_WORD,   \
     GicRBase, GicRlength \
  }

#define EFI_ACPI_5_1_GICC_AFFINITY_STRUCTURE_INIT(                                              \
    ProximityDomain, ACPIProcessorUID, Flags, ClockDomain)                                      \
  {                                                                                             \
    3, sizeof (EFI_ACPI_5_1_GICC_AFFINITY_STRUCTURE),ProximityDomain ,                          \
     ACPIProcessorUID,  Flags,  ClockDomain                                                     \
  }

#define EFI_ACPI_5_0_MEMORY_AFFINITY_STRUCTURE_INIT(                                              \
    ProximityDomain, AddressBaseLow, AddressBaseHigh, LengthLow, LengthHigh, Flags)               \
  {                                                                                               \
    1, sizeof (EFI_ACPI_5_0_MEMORY_AFFINITY_STRUCTURE),ProximityDomain , EFI_ACPI_RESERVED_WORD,  \
    AddressBaseLow, AddressBaseHigh, LengthLow, LengthHigh, EFI_ACPI_RESERVED_DWORD, Flags,       \
    EFI_ACPI_RESERVED_QWORD                                                                       \
  }


#endif

