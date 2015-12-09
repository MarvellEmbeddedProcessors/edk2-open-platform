/** @file
*
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


#define ICC_EOIR1_EL1   S3_0_C12_C12_1
#define ICC_EOIR0_EL1   S3_0_C12_C8_1
#define ICC_SRE_EL3     S3_6_C12_C12_5

.text
.align 2


GCC_ASM_EXPORT(WriteIccEoirNs)

ASM_PFX(WriteIccEoirNs):
  msr ICC_EOIR1_EL1, x0
  isb
  ret


