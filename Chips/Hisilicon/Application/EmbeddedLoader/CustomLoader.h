#include <Uefi.h>
#include <Library/IoLib.h>
#include <Library/UefiLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/ArmLib.h>
#include <Guid/Fdt.h>

#include <PlatformArch.h>
#include <Library/PlatformSysCtrlLib.h>
#include <Library/TimerLib.h>
#include <Library/OemAddressMapLib.h>
#include <Library/FdtUpdateLib.h>
// We move asm_read_reg from ArmLib to HwArmLib
#include <Library/HwArmLib.h>

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

