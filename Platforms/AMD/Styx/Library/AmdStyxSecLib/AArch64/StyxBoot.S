//
//  Copyright (c) 2011 - 2014, ARM Limited. All rights reserved.<BR>
//  Copyright (c) 2014 - 2016, AMD Inc. All rights reserved.<BR>
//
//  This program and the accompanying materials
//  are licensed and made available under the terms and conditions of the BSD License
//  which accompanies this distribution.  The full text of the license may be found at
//  http://opensource.org/licenses/bsd-license.php
//
//  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
//  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
//
//**
//  Derived from:
//   ArmPlatformPkg/ArmVExpressPkg/Library/ArmVExpressSecLibRTSM/AArch64/RTSMBoot.S
//
//**

#include <AsmMacroIoLibV8.h>
#include <Base.h>
#include <Library/ArmPlatformLib.h>
#include <AutoGen.h>

.text
.align 3

GCC_ASM_EXPORT(ArmPlatformSecBootAction)
GCC_ASM_EXPORT(ArmPlatformSecBootMemoryInit)
GCC_ASM_EXPORT(ArmSecMpCoreSecondariesWrite)
GCC_ASM_EXPORT(ArmSecMpCoreSecondariesRead)

/**
  Call at the beginning of the platform boot up

  This function allows the firmware platform to do extra actions at the early
  stage of the platform power up.

  Note: This function must be implemented in assembler as there is no stack set up yet

**/
ASM_PFX(ArmPlatformSecBootAction):
  /*
   * Cortex-A57 Errata
   */
  // Enable CPUECTLR.SMPEN for SMP
  mrs x13, s3_1_c15_c2_1        // Read CPUECTLR
  bfi x13, x1, #6, #1           // Set bit 6; SMPEN: Allow PE to receive IC and TLB operations broadcast from others
  msr s3_1_c15_c2_1, x13
  isb

  /*
   * NIC400 initialization.
   *
   * Setting all registers to unsecure.
   */
  movz    x1, #0xFFFF, LSL #16
  movk    x1, #0xFFFF

  // NIC400 Bot VDD
  movz    x0, #0xE1B0, LSL #16
  movk    x0, #0x0008
  str     w1, [x0]
  movk    x0, #0x000C
  str     w1, [x0]
  movk    x0, #0x0010
  str     w1, [x0]
  movk    x0, #0x0014
  str     w1, [x0]
  movk    x0, #0x0018
  str     w1, [x0]
  movk    x0, #0x0020
  str     w1, [x0]
  movk    x0, #0x0024
  str     w1, [x0]
  movk    x0, #0x0028
  str     w1, [x0]
  movk    x0, #0x002C
  str     w1, [x0]
  movk    x0, #0x0030
  str     w1, [x0]
  movk    x0, #0x0038
  str     w1, [x0]
  movk    x0, #0x001C
  str     w1, [x0]

  // NIC400 Top VDD
  movz    x0, #0xE1C0, LSL #16
  movk    x0, #0x0010
  str     w1, [x0]
  movk    x0, #0x0014
  str     w1, [x0]
  movk    x0, #0x0018
  str     w1, [x0]

  // NIC400 TOP VDDN
  movz    x0, #0xE1D0, LSL #16
  movk    x0, #0x0008
  str     w1, [x0]
  movk    x0, #0x000C
  str     w1, [x0]
  movk    x0, #0x0010
  str     w1, [x0]
  movk    x0, #0x0014
  str     w1, [x0]
  movk    x0, #0x0018
  str     w1, [x0]
  movk    x0, #0x0020
  str     w1, [x0]
  movk    x0, #0x0024
  str     w1, [x0]
  movk    x0, #0x002C
  str     w1, [x0]
  movk    x0, #0x0030
  str     w1, [x0]

# 0000_1111_0111_1111_0000_0001_0000_0001 (0x0F7F0101):
# |||| |||| |||| |||| |||| |||| |||| ||||
# |||| |||| |||| |||| |||| |||| |||| |||+- 0:     CLIENTPD      (1)
# |||| |||| |||| |||| |||| |||| |||| ||+-- 1:     GFRE          (0)
# |||| |||| |||| |||| |||| |||| |||| |+--- 2:     GFIE          (0)
# |||| |||| |||| |||| |||| |||| |||| +---- 3:     Reserved      (0)
# |||| |||| |||| |||| |||| |||| ||||
# |||| |||| |||| |||| |||| |||| |||+ ----- 4:     GCFGFRE       (0)
# |||| |||| |||| |||| |||| |||| ||+- ----- 5:     GCFGFIE       (0)
# |||| |||| |||| |||| |||| |||| ++-- ----- 6-7:   Reserved      (0)
# |||| |||| |||| |||| |||| ||||
# |||| |||| |||| |||| |||| |||| ---- ----- 8:     STALLD        (1)
# |||| |||| |||| |||| |||| ||+- ---- ----- 9:     GSE           (0)
# |||| |||| |||| |||| |||| |+-- ---- ----- 10:    USFCFG        (0)
# |||| |||| |||| |||| |||| +--- ---- ----- 11:    VMIDPNE       (0)
# |||| |||| |||| |||| ||||
# |||| |||| |||| |||| |||+ ---- ---- ----- 12:    PTM           (0)
# |||| |||| |||| |||| ||+- ---- ---- ----- 13:    FB            (0)
# |||| |||| |||| |||| ++-- ---- ---- ----- 14-15: BSU           (0)
# |||| |||| |||| ||||
# |||| |||| |||| ++++ ---- ---- ---- ----- 16-19: MEMATTR       (F)
# |||| |||| ||||
# |||| |||| |||+ ---- ---- ---- ---- ----- 20:    MTCFG         (1)
# |||| |||| ||+- ---- ---- ---- ---- ----- 21:    SMCFCFG       (1)
# |||| |||| ++-- ---- ---- ---- ---- ----- 22-23: SHCFG         (1)
# |||| ||||
# |||| ||++ ---- ---- ---- ---- ---- ----- 24-25: RACFG         (3)
# |||| ++-- ---- ---- ---- ---- ---- ----- 26-27: WACFG         (3)
# ||||
# ||++ ---- ---- ---- ---- ---- ---- ----- 28-29: NSCFG         (0)
# ++-- ---- ---- ---- ---- ---- ---- ----- 30-31: Reserved      (0)

  // Workaround for A0, force the memory attributes
  ldr     x0, =0xE0200000               // SMMU_SCR0
  ldr     w1, =0x0F7F0101
  str     w1, [x0, #0]

  ldr     x0, =0xE0200400               // SMMU_NSCR0
  ldr     w1, =0x0F7F0101
  str     w1, [x0, #0]
  ret

/**
  Initialize the memory where the initial stacks will reside

  This memory can contain the initial stacks (Secure and Secure Monitor stacks).
  In some platform, this region is already initialized and the implementation of this function can
  do nothing. This memory can also represent the Secure RAM.
  This function is called before the satck has been set up. Its implementation must ensure the stack
  pointer is not used (probably required to use assembly language)

**/
ASM_PFX(ArmPlatformSecBootMemoryInit):
  // The SMC does not need to be initialized for RTSM
  ret

/* Write the flag register used to start Secondary cores */
ASM_PFX(ArmSecMpCoreSecondariesWrite):
  // Write to the CPU Mailbox
  ret

/* Read the flag register used to start Secondary cores */
ASM_PFX(ArmSecMpCoreSecondariesRead):
  // Return the value from the CPU Mailbox
  mov x0, #0
  ret

