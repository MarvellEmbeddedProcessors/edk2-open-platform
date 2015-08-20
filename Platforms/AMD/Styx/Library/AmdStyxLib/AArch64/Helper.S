#/**
#
#  Copyright (c) 2011-2013, ARM Limited. All rights reserved.<BR>
#  Copyright (c) 2014 - 2016, AMD Inc. All rights reserved.<BR>
#
#  This program and the accompanying materials
#  are licensed and made available under the terms and conditions of the BSD License
#  which accompanies this distribution.  The full text of the license may be found at
#  http://opensource.org/licenses/bsd-license.php
#
#  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
#  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
#
#**/
#/**
#  Derived from:
#   ArmPlatformPkg/ArmVExpressPkg/Library/ArmVExpressLibRTSM/RTSMHelper.S
#
#**/

#include <AsmMacroIoLibV8.h>
#include <Base.h>
#include <Library/ArmLib.h>
#include <Library/PcdLib.h>
#include <AutoGen.h>

.text
.align 2

GCC_ASM_EXPORT(ArmPlatformPeiBootAction)
GCC_ASM_EXPORT(ArmPlatformIsPrimaryCore)
GCC_ASM_EXPORT(ArmPlatformGetPrimaryCoreMpId)
GCC_ASM_EXPORT(ArmPlatformGetCorePosition)
GCC_ASM_EXPORT(ArmGetCpuCountPerCluster)

GCC_ASM_IMPORT(_gPcd_FixedAtBuild_PcdArmPrimaryCore)
GCC_ASM_IMPORT(_gPcd_FixedAtBuild_PcdArmPrimaryCoreMask)
GCC_ASM_IMPORT(_gPcd_FixedAtBuild_PcdCoreCount)

PrimaryCoreMpid:  .word    0x0
PrimaryCoreBoot:  .word    0x0

//VOID
//ArmPlatformPeiBootAction (
//  VOID
//  );
ASM_PFX(ArmPlatformPeiBootAction):
  // Save the primary CPU MPID
  ldr  x1, =PrimaryCoreBoot
  ldrh w0, [x1]
  cmp  wzr, w0
  b.ne 1f
  mrs  x0, mpidr_el1
  ldr  x1, =PrimaryCoreMpid
  str  w0, [x1]
  mov  w0, 1
  ldr  x1, =PrimaryCoreBoot
  str  w0, [x1]
1:
  ret

//UINTN
//ArmPlatformGetPrimaryCoreMpId (
//  VOID
//  );
ASM_PFX(ArmPlatformGetPrimaryCoreMpId):
  ldr   x0, =PrimaryCoreMpid
  ldrh  w0, [x0]
  ret

# IN None
# OUT x0 = number of cores present in the system
ASM_PFX(ArmGetCpuCountPerCluster):
  LoadConstantToReg (_gPcd_FixedAtBuild_PcdCoreCount, x0)
  ldrh  w0, [x0]
  ret

//UINTN
//ArmPlatformIsPrimaryCore (
//  IN UINTN MpId
//  );
ASM_PFX(ArmPlatformIsPrimaryCore):
  LoadConstantToReg (_gPcd_FixedAtBuild_PcdArmPrimaryCoreMask, x1)
  ldrh  w1, [x1]
  and   x0, x0, x1

  ldr   x1, =PrimaryCoreMpid
  ldrh  w1, [x1]

  cmp   w0, w1
  mov   x0, #1
  mov   x1, #0
  csel  x0, x0, x1, eq
  ret

//UINTN
//ArmPlatformGetCorePosition (
//  IN UINTN MpId
//  );
// With this function: CorePos = (ClusterId * 2) + CoreId
ASM_PFX(ArmPlatformGetCorePosition):
  and   x1, x0, #ARM_CORE_MASK
  and   x0, x0, #ARM_CLUSTER_MASK
  add   x0, x1, x0, LSR #7
  ret

ASM_FUNCTION_REMOVE_IF_UNREFERENCED
