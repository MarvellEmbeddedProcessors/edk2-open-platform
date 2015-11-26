#include <Uefi.h>
#include <Library/IoLib.h>
#include <Library/UefiLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/ArmLib.h>
#include <Guid/Fdt.h>
#include <Chipset/ArmCortexA5x.h>

#include <PlatformArch.h>
#include <Library/PlatformSysCtrlLib.h>
#include <Library/TimerLib.h>
#include <Library/OemAddressMapLib.h>
#include <Library/FdtUpdateLib.h>
#include <Library/LzmaCustomDecompressLib/LzmaDecompressLibInternal.h>

typedef VOID (*ESL_LINUX)(UINTN ParametersBase, UINTN Reserved0,
                          UINTN Reserved1, UINTN Reserved2);

EFI_STATUS
ShutdownUefiBootServices (
  VOID
  );

EFI_STATUS
PreparePlatformHardware (
  VOID
  );

VOID
ESL_Start_OS (
  );

VOID
Flash_Start_OS (
  );

