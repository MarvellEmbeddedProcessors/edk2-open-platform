/** @file
*
*  Copyright (c) 2016, Hisilicon Limited. All rights reserved.
*  Copyright (c) 2016, Linaro Limited. All rights reserved.
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

#ifndef _PLATFORM_PCI_LIB_H_
#define _PLATFORM_PCI_LIB_H_

#define PCIE_MAX_HOSTBRIDGE      2
#define PCIE_MAX_ROOTBRIDGE      4

#define PCI_HB0RB0_PCI_BASE        FixedPcdGet64(PciHb0Rb0Base)
#define PCI_HB0RB1_PCI_BASE        FixedPcdGet64(PciHb0Rb1Base)
#define PCI_HB0RB2_PCI_BASE        FixedPcdGet64(PciHb0Rb2Base)
#define PCI_HB0RB3_PCI_BASE        FixedPcdGet64(PciHb0Rb3Base)

#define PCI_HB1RB0_PCI_BASE        0xb0090000
#define PCI_HB1RB1_PCI_BASE        0xb0200000
#define PCI_HB1RB2_PCI_BASE        0xb00a0000
#define PCI_HB1RB3_PCI_BASE        0xb00b0000

#define PCI_HB0RB0_ECAM_BASE      FixedPcdGet64 (PcdHb0Rb0PciConfigurationSpaceBaseAddress)
#define PCI_HB0RB0_ECAM_SIZE      FixedPcdGet64 (PcdHb0Rb0PciConfigurationSpaceSize)
#define PCI_HB0RB1_ECAM_BASE      FixedPcdGet64 (PcdHb0Rb1PciConfigurationSpaceBaseAddress)
#define PCI_HB0RB1_ECAM_SIZE      FixedPcdGet64 (PcdHb0Rb1PciConfigurationSpaceSize)
#define PCI_HB0RB2_ECAM_BASE      FixedPcdGet64 (PcdHb0Rb2PciConfigurationSpaceBaseAddress)
#define PCI_HB0RB2_ECAM_SIZE      FixedPcdGet64 (PcdHb0Rb2PciConfigurationSpaceSize)
#define PCI_HB0RB3_ECAM_BASE      FixedPcdGet64 (PcdHb0Rb3PciConfigurationSpaceBaseAddress)
#define PCI_HB0RB3_ECAM_SIZE      FixedPcdGet64 (PcdHb0Rb3PciConfigurationSpaceSize)

#define PCI_HB1RB0_ECAM_BASE      (FixedPcdGet64 (PcdHb1BaseAddress) + PCI_HB0RB0_ECAM_BASE)
#define PCI_HB1RB0_ECAM_SIZE       PCI_HB0RB0_ECAM_SIZE
#define PCI_HB1RB1_ECAM_BASE      (FixedPcdGet64 (PcdHb1BaseAddress) + PCI_HB0RB1_ECAM_BASE)
#define PCI_HB1RB1_ECAM_SIZE       PCI_HB0RB1_ECAM_SIZE
#define PCI_HB1RB2_ECAM_BASE      0xb8000000
#define PCI_HB1RB2_ECAM_SIZE      0x4000000
#define PCI_HB1RB3_ECAM_BASE      0xbc000000
#define PCI_HB1RB3_ECAM_SIZE      0x4000000
#define PCI_HB0RB0_PCIREGION_BASE (FixedPcdGet64 (PcdHb0Rb0PciRegionBaseAddress))
#define PCI_HB0RB0_PCIREGION_SIZE (FixedPcdGet64 (PcdHb0Rb0PciRegionSize))
#define PCI_HB0RB1_PCIREGION_BASE (FixedPcdGet64 (PcdHb0Rb1PciRegionBaseAddress))
#define PCI_HB0RB1_PCIREGION_SIZE (FixedPcdGet64 (PcdHb0Rb1PciRegionSize))
#define PCI_HB0RB2_PCIREGION_BASE (FixedPcdGet64 (PcdHb0Rb2PciRegionBaseAddress))
#define PCI_HB0RB2_PCIREGION_SIZE (FixedPcdGet64 (PcdHb0Rb2PciRegionSize))
#define PCI_HB0RB3_PCIREGION_BASE (FixedPcdGet64 (PcdHb0Rb3PciRegionBaseAddress))
#define PCI_HB0RB3_PCIREGION_SIZE (FixedPcdGet64 (PcdHb0Rb3PciRegionSize))

#define PCI_HB0RB0_CPUMEMREGIONBASE (FixedPcdGet64 (PcdHb0Rb0CpuMemRegionBase))
#define PCI_HB0RB1_CPUMEMREGIONBASE (FixedPcdGet64 (PcdHb0Rb1CpuMemRegionBase))
#define PCI_HB0RB2_CPUMEMREGIONBASE (FixedPcdGet64 (PcdHb0Rb2CpuMemRegionBase))
#define PCI_HB0RB3_CPUMEMREGIONBASE (FixedPcdGet64 (PcdHb0Rb3CpuMemRegionBase))

#define PCI_HB0RB0_CPUIOREGIONBASE (FixedPcdGet64 (PcdHb0Rb0CpuIoRegionBase))
#define PCI_HB0RB1_CPUIOREGIONBASE (FixedPcdGet64 (PcdHb0Rb1CpuIoRegionBase))
#define PCI_HB0RB2_CPUIOREGIONBASE (FixedPcdGet64 (PcdHb0Rb2CpuIoRegionBase))
#define PCI_HB0RB3_CPUIOREGIONBASE (FixedPcdGet64 (PcdHb0Rb3CpuIoRegionBase))

#define PCI_HB0RB0_IO_BASE (FixedPcdGet64 (PcdHb0Rb0IoBase))
#define PCI_HB0RB1_IO_BASE (FixedPcdGet64 (PcdHb0Rb1IoBase))
#define PCI_HB0RB2_IO_BASE (FixedPcdGet64 (PcdHb0Rb2IoBase))
#define PCI_HB0RB3_IO_BASE (FixedPcdGet64 (PcdHb0Rb3IoBase))

#define PCI_HB0RB0_IO_SIZE (FixedPcdGet64 (PcdHb0Rb0IoSize))
#define PCI_HB0RB1_IO_SIZE (FixedPcdGet64 (PcdHb0Rb1IoSize))
#define PCI_HB0RB2_IO_SIZE (FixedPcdGet64 (PcdHb0Rb2IoSize))
#define PCI_HB0RB3_IO_SIZE (FixedPcdGet64 (PcdHb0Rb3IoSize))

typedef struct {
  UINT64          Ecam;
  UINT64          BusBase;
  UINT64          BusLimit;
  UINT64          MemBase;
  UINT64          MemLimit;
  UINT64          IoBase;
  UINT64          IoLimit;
  UINT64          CpuMemRegionBase;
  UINT64          CpuIoRegionBase;
  UINT64          RbPciBar;
} PCI_ROOT_BRIDGE_RESOURCE_APPETURE;

extern PCI_ROOT_BRIDGE_RESOURCE_APPETURE  mResAppeture[PCIE_MAX_HOSTBRIDGE][PCIE_MAX_ROOTBRIDGE];
#endif

