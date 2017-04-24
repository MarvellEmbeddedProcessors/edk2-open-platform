//Based on ArmPlatformPkg/Library/ArmPlatformLibNull/AArch64/ArmPlatformHelper.S
//
//  Copyright (c) 2012-2013, ARM Limited. All rights reserved.
//  Copyright (c) 2016, Marvell. All rights reserved.
//  Copyright (c) 2017, Linaro Limited. All rights reserved.
//
//  This program and the accompanying materials are licensed and made available
//  under the terms and conditions of the BSD License which accompanies this
//  distribution. The full text of the license may be found at
//  http://opensource.org/licenses/bsd-license.php
//
//  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
//  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED
//

#include <AsmMacroIoLibV8.h>
#include <Library/ArmLib.h>

#define CCU_MC_BASE                     0xF0001700
#define CCU_MC_RCR_OFFSET               0x0
#define CCU_MC_RCR_REMAP_EN             BIT0
#define CCU_MC_RCR_REMAP_SIZE(Size)     (((Size) - 1) ^ (SIZE_1MB - 1))

#define CCU_MC_RSBR_OFFSET              0x4
#define CCU_MC_RSBR_SOURCE_BASE(Base)   (((Base) >> 20) << 10)
#define CCU_MC_RTBR_OFFSET              0x8
#define CCU_MC_RTBR_TARGET_BASE(Base)   (((Base) >> 20) << 10)

ASM_FUNC(ArmPlatformPeiBootAction)
  mov   x29, xzr

  .if   FixedPcdGet64 (PcdSystemMemoryBase) != 0
  .err  PcdSystemMemoryBase should be 0x0 on this platform!
  .endif

  .if   FixedPcdGet64 (PcdSystemMemorySize) > FixedPcdGet32 (PcdDramRemapTarget)
    //
    // Remap the DRAM region [DramRemapTarget, DramRemapTarget + DramRemapSize)
    // to the top of the DRAM space. This is mainly intended to free up 32-bit
    // addressable memory for MMIO and PCI windows.
    //
    MOV32 (x0, CCU_MC_BASE)
    MOV32 (w1, CCU_MC_RSBR_SOURCE_BASE (FixedPcdGet64 (PcdSystemMemorySize)))
    MOV32 (w2, CCU_MC_RTBR_TARGET_BASE (FixedPcdGet32 (PcdDramRemapTarget)))
    MOV32 (w3, CCU_MC_RCR_REMAP_SIZE (FixedPcdGet32 (PcdDramRemapSize)) | CCU_MC_RCR_REMAP_EN)
    str   w1, [x0, #CCU_MC_RSBR_OFFSET]
    str   w2, [x0, #CCU_MC_RTBR_OFFSET]
    str   w3, [x0, #CCU_MC_RCR_OFFSET]

    //
    // Use the low range for UEFI itself. The remaining memory will be mapped
    // and added to the GCD map later.
    //
    adr   x0, mSystemMemoryEnd
    MOV64 (x1, FixedPcdGet32 (PcdDramRemapTarget) - 1)
    str   x1, [x0]
  .endif

  ret

//UINTN
//ArmPlatformGetCorePosition (
//  IN UINTN MpId
//  );
// With this function: CorePos = (ClusterId * 4) + CoreId
ASM_FUNC(ArmPlatformGetCorePosition)
  and   x1, x0, #ARM_CORE_MASK
  and   x0, x0, #ARM_CLUSTER_MASK
  add   x0, x1, x0, LSR #6
  ret

//UINTN
//ArmPlatformGetPrimaryCoreMpId (
//  VOID
//  );
ASM_FUNC(ArmPlatformGetPrimaryCoreMpId)
  MOV32 (w0, FixedPcdGet32(PcdArmPrimaryCore))
  ret

//UINTN
//ArmPlatformIsPrimaryCore (
//  IN UINTN MpId
//  );
ASM_FUNC(ArmPlatformIsPrimaryCore)
  MOV32 (w1, FixedPcdGet32(PcdArmPrimaryCoreMask))
  and   x0, x0, x1
  MOV32 (w1, FixedPcdGet32(PcdArmPrimaryCore))
  cmp   w0, w1
  cset  x0, eq
  ret
