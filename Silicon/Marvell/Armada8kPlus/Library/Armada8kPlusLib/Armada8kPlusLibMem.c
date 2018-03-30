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

#include <Base.h>
#include <Library/ArmPlatformLib.h>
#include <Library/DebugLib.h>
#include <Library/HobLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>

// The total number of descriptors, including the final "end-of-table" descriptor.
#define MAX_VIRTUAL_MEMORY_MAP_DESCRIPTORS 16

// DDR attributes
#define DDR_ATTRIBUTES_CACHED           ARM_MEMORY_REGION_ATTRIBUTE_NONSECURE_WRITE_BACK
#define DDR_ATTRIBUTES_UNCACHED         ARM_MEMORY_REGION_ATTRIBUTE_UNCACHED_UNBUFFERED

STATIC ARM_MEMORY_REGION_DESCRIPTOR VirtualMemoryTable[MAX_VIRTUAL_MEMORY_MAP_DESCRIPTORS];

// Obtain DRAM size basing on register values filled by early firmware.
STATIC
UINT64
GetDramSize (
  UINT64 *MemSize
  )
{
  *MemSize = FixedPcdGet64 (PcdSystemMemorySize);

  return EFI_SUCCESS;
}

/**
  Return the Virtual Memory Map of your platform

  This Virtual Memory Map is used by MemoryInitPei Module to initialize the MMU on your platform.

  @param[out]   VirtualMemoryMap    Array of ARM_MEMORY_REGION_DESCRIPTOR describing a Physical-to-
                                    Virtual Memory mapping. This array must be ended by a zero-filled
                                    entry

**/
VOID
ArmPlatformGetVirtualMemoryMap (
  IN ARM_MEMORY_REGION_DESCRIPTOR** VirtualMemoryMap
  )
{
  UINTN                         Index = 0;
  UINT64                        MemSize;
  UINT64                        MemLowSize;
  UINT64                        MemHighStart;
  UINT64                        MemHighSize;
  EFI_RESOURCE_ATTRIBUTE_TYPE   ResourceAttributes;
  EFI_STATUS                    Status;

  ASSERT (VirtualMemoryMap != NULL);

  // Obtain total memory size from the hardware.
  Status = GetDramSize (&MemSize);
  if (EFI_ERROR (Status)) {
    MemSize = FixedPcdGet64 (PcdSystemMemorySize);
    DEBUG ((DEBUG_ERROR, "Limit total memory size to %d MB\n", MemSize / 1024 / 1024));
  }

  MemLowSize = MIN (FixedPcdGet64 (PcdDramRemapTarget), MemSize);
  MemHighStart = (UINT64)FixedPcdGet64 (PcdDramRemapTarget) +
                 FixedPcdGet32 (PcdDramRemapSize);
  MemHighSize = MemSize - MemLowSize;

  ResourceAttributes = (
      EFI_RESOURCE_ATTRIBUTE_PRESENT |
      EFI_RESOURCE_ATTRIBUTE_INITIALIZED |
      EFI_RESOURCE_ATTRIBUTE_WRITE_COMBINEABLE |
      EFI_RESOURCE_ATTRIBUTE_WRITE_THROUGH_CACHEABLE |
      EFI_RESOURCE_ATTRIBUTE_WRITE_BACK_CACHEABLE |
      EFI_RESOURCE_ATTRIBUTE_TESTED
  );

  BuildResourceDescriptorHob (
    EFI_RESOURCE_SYSTEM_MEMORY,
    ResourceAttributes,
    FixedPcdGet64 (PcdSystemMemoryBase),
    MemLowSize
    );

  // DDR
  VirtualMemoryTable[Index].PhysicalBase    = FixedPcdGet64 (PcdSystemMemoryBase);
  VirtualMemoryTable[Index].VirtualBase     = FixedPcdGet64 (PcdSystemMemoryBase);
  VirtualMemoryTable[Index].Length          = MemLowSize;
  VirtualMemoryTable[Index].Attributes      = DDR_ATTRIBUTES_CACHED;

  // Configuration space 0xDC00_0000 - 0xEFFF_FFFF
  VirtualMemoryTable[++Index].PhysicalBase  = 0xDC000000;
  VirtualMemoryTable[Index].VirtualBase     = 0xDC000000;
  VirtualMemoryTable[Index].Length          = 0x14000000;
  VirtualMemoryTable[Index].Attributes      = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;

  // SPI direct address space 0x9d_0000_0000 - 0x9d0_1000_0000 */
  VirtualMemoryTable[++Index].PhysicalBase  = PcdGet64 (PcdSpiMemoryBase);
  VirtualMemoryTable[Index].VirtualBase     = PcdGet64 (PcdSpiMemoryBase);
  VirtualMemoryTable[Index].Length          = 0x010000000;
  VirtualMemoryTable[Index].Attributes      = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;

  // Configuration space 0x81_0000_0000 - 0x81_01FF_FFFF
  VirtualMemoryTable[++Index].PhysicalBase  = 0x8100000000;
  VirtualMemoryTable[Index].VirtualBase     = 0x8100000000;
  VirtualMemoryTable[Index].Length          = 0x2000000;
  VirtualMemoryTable[Index].Attributes      = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;

  // Configuration space 0x88_0000_0000 - 0x88_01FF_FFFF
  VirtualMemoryTable[++Index].PhysicalBase  = 0x8800000000;
  VirtualMemoryTable[Index].VirtualBase     = 0x8800000000;
  VirtualMemoryTable[Index].Length          = 0x2000000;
  VirtualMemoryTable[Index].Attributes      = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;

  if (MemSize > MemLowSize) {
    //
    // If we have more than MemLowSize worth of DRAM, the remainder will be
    // mapped at the top of the remapped window.
    //
    VirtualMemoryTable[++Index].PhysicalBase  = MemHighStart;
    VirtualMemoryTable[Index].VirtualBase     = MemHighStart;
    VirtualMemoryTable[Index].Length          = MemHighSize;
    VirtualMemoryTable[Index].Attributes      = DDR_ATTRIBUTES_CACHED;

    BuildResourceDescriptorHob (
      EFI_RESOURCE_SYSTEM_MEMORY,
      ResourceAttributes,
      MemHighStart,
      MemHighSize
      );
  }

  // End of Table
  VirtualMemoryTable[++Index].PhysicalBase  = 0;
  VirtualMemoryTable[Index].VirtualBase     = 0;
  VirtualMemoryTable[Index].Length          = 0;
  VirtualMemoryTable[Index].Attributes      = 0;

  *VirtualMemoryMap = VirtualMemoryTable;
}
