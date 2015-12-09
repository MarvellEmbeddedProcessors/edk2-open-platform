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


#ifndef _UNI_L1_L2_LIB_H_
#define _UNI_L1_L2_LIB_H_

int BTRM_L1L2_Valid (VOID);

BOOLEAN UniIsL2BiosFail (VOID);
BOOLEAN UniIsCurrentBiosL2 (VOID);
EFI_STATUS UniResetL2Counter (VOID);
EFI_STATUS UniResetL2FailFlag (VOID);

#endif
