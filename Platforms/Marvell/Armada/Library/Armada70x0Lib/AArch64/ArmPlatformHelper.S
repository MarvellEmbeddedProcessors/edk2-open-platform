//Based on ArmPlatformPkg/Library/ArmPlatformLibNull/AArch64/ArmPlatformHelper.S
//
//  Copyright (c) 2012-2013, ARM Limited. All rights reserved.
//  Copyright (c) 2016, Marvell. All rights reserved.
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

ASM_FUNC(ArmPlatformPeiBootAction)
  mov   x29, xzr

  .if   FixedPcdGet64 (PcdSystemMemoryBase) != 0
  .err  PcdSystemMemoryBase should be 0x0 on this platform!
  .endif

  .if   FixedPcdGet64 (PcdSystemMemorySize) > FixedPcdGet32 (PcdDramRemapTarget)
    //
    // Use the low range for UEFI itself. The remaining memory will be mapped
    // and added to the GCD map later.
    //
    adr   x0, mSystemMemoryEnd
    MOV64 (x1, FixedPcdGet32 (PcdDramRemapTarget) - 1)
    str   x1, [x0]
  .endif

  MOV32 (x0, FixedPcdGet64 (PcdFvBaseAddress))
  MOV32 (x3, FixedPcdGet32 (PcdFvSize))
  add   x3, x3, x0

  mrs   x1, ctr_el0
  and   x1, x1, #0xf      // Dminline
  mov   x2, #4
  lsl   x1, x2, x1        // by-VA stride for D-cache maintenance

0:dc    civac, x0
  add   x0, x0, x1
  cmp   x0, x3
  b.lt  0b

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
