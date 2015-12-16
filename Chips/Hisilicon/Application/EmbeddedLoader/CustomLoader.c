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

#include "CustomLoader.h"

EFI_STATUS
PreparePlatformHardware (
  VOID
  )
{
    //Note: Interrupts will be disabled by the GIC driver when ExitBootServices() will be called.

    // Clean before Disable else the Stack gets corrupted with old data.
    ArmCleanDataCache ();

    LlcCleanInvalidate ();

    ArmDisableDataCache ();
    
    // Invalidate all the entries that might have snuck in.
    ArmInvalidateDataCache ();

    // Disable and invalidate the instruction cache
    ArmDisableInstructionCache ();
    ArmInvalidateInstructionCache ();

    // Turn off MMU
    ArmDisableMmu();

    return EFI_SUCCESS;
}

