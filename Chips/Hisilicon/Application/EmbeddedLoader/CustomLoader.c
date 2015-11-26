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

