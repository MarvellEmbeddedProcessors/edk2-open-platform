#
#  Copyright (c) 2011-2012, ARM Limited. All rights reserved.
#
#  This program and the accompanying materials
#  are licensed and made available under the terms and conditions of the BSD License
#  which accompanies this distribution.  The full text of the license may be found at
#  http://opensource.org/licenses/bsd-license.php
#
#  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
#  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
#
#

################################################################################
#
# Defines Section - statements that will be processed to create a Makefile.
#
################################################################################
[Defines]
  PLATFORM_NAME                  = Pv660D02
  PLATFORM_GUID                  = E1AB8AC3-3EF1-4c6f-8D9F-ABE3EC67188E
  PLATFORM_VERSION               = 0.1
  DSC_SPECIFICATION              = 0x00010005
  OUTPUT_DIRECTORY               = Build/$(PLATFORM_NAME)
  SUPPORTED_ARCHITECTURES        = AARCH64
  BUILD_TARGETS                  = DEBUG|RELEASE
  SKUID_IDENTIFIER               = DEFAULT
  FLASH_DEFINITION               = OpenPlatformPkg/Platforms/Hisilicon/D02/$(PLATFORM_NAME).fdf
  DEFINE EDK2_SKIP_PEICORE=0

!include OpenPlatformPkg/Chips/Hisilicon/P660/Pv660.dsc.inc

[LibraryClasses.common]
  ArmLib|ArmPkg/Library/ArmLib/AArch64/AArch64Lib.inf
  ArmCpuLib|ArmPkg/Drivers/ArmCpuLib/ArmCortexAEMv8Lib/ArmCortexAEMv8Lib.inf
  ArmPlatformLib|OpenPlatformPkg/Chips/Hisilicon/Library/ArmPlatformLibPv660/ArmPlatformLib.inf

  ArmPlatformSysConfigLib|ArmPlatformPkg/ArmVExpressPkg/Library/ArmVExpressSysConfigLib/ArmVExpressSysConfigLib.inf
  NorFlashPlatformLib|ArmPlatformPkg/ArmVExpressPkg/Library/NorFlashArmVExpressLib/NorFlashArmVExpressLib.inf
  LcdPlatformLib|ArmPlatformPkg/ArmVExpressPkg/Library/PL111LcdArmVExpressLib/PL111LcdArmVExpressLib.inf

  #DebugAgentTimerLib|ArmPlatformPkg/ArmVExpressPkg/Library/DebugAgentTimerLib/DebugAgentTimerLib.inf

 
  I2CLib|OpenPlatformPkg/Chips/Hisilicon/Library/I2CLib/I2CLib.inf
  #UpdateCpldLib|OpenPlatformPkg/Platforms/Hisilicon/D02/UpdateCpldLib/UpdateCpldLib.inf
  TimerLib|ArmPkg/Library/ArmArchTimerLib/ArmArchTimerLib.inf


  NetLib|MdeModulePkg/Library/DxeNetLib/DxeNetLib.inf
  DpcLib|MdeModulePkg/Library/DxeDpcLib/DxeDpcLib.inf
  HiiLib|MdeModulePkg/Library/UefiHiiLib/UefiHiiLib.inf
  UefiHiiServicesLib|MdeModulePkg/Library/UefiHiiServicesLib/UefiHiiServicesLib.inf
  UdpIoLib|MdeModulePkg/Library/DxeUdpIoLib/DxeUdpIoLib.inf
  IpIoLib|MdeModulePkg/Library/DxeIpIoLib/DxeIpIoLib.inf



  #FDTUpdateLib
  FdtUpdateLib|OpenPlatformPkg/Platforms/Hisilicon/D02/FdtUpdateLibD02/FdtUpdateLib.inf

  #PcieInitLib|OpenPlatformPkg/Chips/Hisilicon/PcieInitLib/PcieInitLib.inf

  CpldDriverLib|OpenPlatformPkg/Chips/Hisilicon/Library/CpldDriverLib/CpldDriverLib.inf

  SerdesLib|OpenPlatformPkg/Chips/Hisilicon/Library/Serdes/Pv660Serdes/Pv660SerdesLib.inf

  # ARM PL031 RTC Driver
  #RealTimeClockLib|ArmPlatformPkg/Library/PL031RealTimeClockLib/PL031RealTimeClockLib.inf
  RealTimeClockLib|OpenPlatformPkg/Chips/Hisilicon/Library/DS3231RealTimeClockLib/DS3231RealTimeClockLib.inf

  OemMiscLib|OpenPlatformPkg/Platforms/Hisilicon/D02/Library/OemMiscLibD02/OemMiscLibD02.inf
  #OemMiscLib|HwProductsPkg/Pv660Evb/Library/OemMiscLibEvb/OemMiscLibEvbb.inf
  BootLineLib|OpenPlatformPkg/Platforms/Hisilicon/D02/Library/BootLineLibD02/BootLineLibD02.inf
  
  OemAddressMapLib|OpenPlatformPkg/Platforms/Hisilicon/D02/Library/AddressMapPv660D02/OemAddressMapPv660D02.inf
  PlatformSysCtrlLib|OpenPlatformPkg/Chips/Hisilicon/Library/PlatformSysCtrlLib/PlatformSysCtrlLibPv660/PlatformSysCtrlLibPv660.inf
  #IpmiCmdLib|OpenPlatformPkg/Chips/Hisilicon/Library/IpmiCmdLibNull/IpmiCmdLibNull.inf

[LibraryClasses.common.SEC]
  ArmLib|ArmPkg/Library/ArmLib/AArch64/AArch64LibSec.inf
  #ArmPlatformSecLib|OpenPlatformPkg/Platforms/Hisilicon/D02/Library/Pv660D02SecLib/ArmVExpressSecLib.inf
  ArmPlatformLib|OpenPlatformPkg/Chips/Hisilicon/Library/ArmPlatformLibPv660/ArmPlatformLibSec.inf
  EfiResetSystemLib|OpenPlatformPkg/Platforms/Hisilicon/D02/Library/ResetSystemLibCpld/ResetSystemLibSec.inf

[LibraryClasses.common.DXE_RUNTIME_DRIVER]
  #EfiResetSystemLib|OpenPlatformPkg/Platforms/Hisilicon/D02/Library/ResetSystemLibCpld/ResetSystemLib.inf

[BuildOptions]
  GCC:*_*_AARCH64_ARCHCC_FLAGS  = -DARM_CPU_AARCH64 -mstrict-align
  GCC:*_*_AARCH64_PP_FLAGS  = -DARM_CPU_AARCH64
  GCC:*_*_AARCH64_PLATFORM_FLAGS == -I$(WORKSPACE)/ArmPlatformPkg/ArmVExpressPkg/Include -I$(WORKSPACE)/ArmPlatformPkg/ArmVExpressPkg/Include/Platform/RTSM -I$(WORKSPACE)/OpenPlatformPkg/Chips/Hisilicon/Include/Platform/PV660


################################################################################
#
# Pcd Section - list of all EDK II PCD Entries defined by this Platform
#
################################################################################

[PcdsFeatureFlag.common]

!if $(EDK2_SKIP_PEICORE) == 1
  gArmPlatformTokenSpaceGuid.PcdSystemMemoryInitializeInSec|TRUE
  gArmPlatformTokenSpaceGuid.PcdSendSgiToBringUpSecondaryCores|TRUE
!endif

  ## If TRUE, Graphics Output Protocol will be installed on virtual handle created by ConsplitterDxe.
  #  It could be set FALSE to save size.
  gEfiMdeModulePkgTokenSpaceGuid.PcdConOutGopSupport|FALSE

[PcdsFixedAtBuild.common]
  gArmPlatformTokenSpaceGuid.PcdFirmwareVendor|"ARM Versatile Express"
  gEmbeddedTokenSpaceGuid.PcdEmbeddedPrompt|"D02"

  gArmPlatformTokenSpaceGuid.PcdCoreCount|8

  #
  # NV Storage PCDs. Use base of 0x0C000000 for NOR1
  #
  gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageVariableBase|0xa49D0000
  gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageVariableSize|0x0000E000
  gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageFtwWorkingBase|0xa49DE000
  gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageFtwWorkingSize|0x00002000
  gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageFtwSpareBase|0xa49E0000
  gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageFtwSpareSize|0x00010000

  #gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageVariableBase|0x310C0000
  #gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageVariableSize|0x00010000
  #gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageFtwWorkingBase|0x310D0000
  #gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageFtwWorkingSize|0x00010000
  #gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageFtwSpareBase|0x310E0000
  #gEfiMdeModulePkgTokenSpaceGuid.PcdFlashNvStorageFtwSpareSize|0x00010000

  #gEfiSecurityPkgTokenSpaceGuid.PcdOptionRomImageVerificationPolicy|0x02
  #gEfiSecurityPkgTokenSpaceGuid.PcdRemovableMediaImageVerificationPolicy|0x02
  #gEfiSecurityPkgTokenSpaceGuid.PcdFixedMediaImageVerificationPolicy|0x02
  
  gEfiMdeModulePkgTokenSpaceGuid.PcdMaxVariableSize|0x2000

  gArmTokenSpaceGuid.PcdVFPEnabled|0

  # Stacks for MPCores in Secure World
  #gArmPlatformTokenSpaceGuid.PcdCPUCoresSecStackBase|0x21000000
  #gArmPlatformTokenSpaceGuid.PcdCPUCoreSecPrimaryStackSize|0x10000
  gArmPlatformTokenSpaceGuid.PcdCPUCoresSecStackBase|0xE1000000
  gArmPlatformTokenSpaceGuid.PcdCPUCoreSecPrimaryStackSize|0x10000

  # Stacks for MPCores in Monitor Mode
  #gArmPlatformTokenSpaceGuid.PcdCPUCoresSecMonStackBase|0x2E008000
  #gArmPlatformTokenSpaceGuid.PcdCPUCoreSecMonStackSize|0x100
  gArmPlatformTokenSpaceGuid.PcdCPUCoresSecMonStackBase|0xE100FF00
  gArmPlatformTokenSpaceGuid.PcdCPUCoreSecMonStackSize|0x100

  # Stacks for MPCores in Normal World
  #gArmPlatformTokenSpaceGuid.PcdCPUCoresStackBase|0x2E000000
  #gArmPlatformTokenSpaceGuid.PcdCPUCorePrimaryStackSize|0x4000
  gArmPlatformTokenSpaceGuid.PcdCPUCoresStackBase|0xE1000000
  gArmPlatformTokenSpaceGuid.PcdCPUCorePrimaryStackSize|0xFF00
  #UEFI BIOS_t00216239_end   2014-10-29 11:00:07

  gArmTokenSpaceGuid.PcdSystemMemoryBase|0x00000000
  gArmTokenSpaceGuid.PcdSystemMemorySize|0x3c000000
  #gArmTokenSpaceGuid.PcdSystemMemorySize|0x20000000

  # Size of the region used by UEFI in permanent memory (Reserved 64MB)
  gArmPlatformTokenSpaceGuid.PcdSystemMemoryUefiRegionSize|0x10000000

  #
  # ARM Pcds
  #
  gArmTokenSpaceGuid.PcdArmUncachedMemoryMask|0x0000000040000000

  #
  # ARM PrimeCell
  #


  ## SP805 Watchdog - Motherboard Watchdog
  gArmPlatformTokenSpaceGuid.PcdSP805WatchdogBase|0x801e0000

  ## Serial Terminal
  gEfiMdeModulePkgTokenSpaceGuid.PcdSerialRegisterBase|0x80300000
  gEfiMdePkgTokenSpaceGuid.PcdUartDefaultBaudRate|115200

  gHwTokenSpaceGuid.PcdUartClkInHz|200000000

  gHwTokenSpaceGuid.PcdSerialPortSendDelay|10000000

  gEfiMdePkgTokenSpaceGuid.PcdUartDefaultDataBits|8
  gEfiMdePkgTokenSpaceGuid.PcdUartDefaultParity|1
  gEfiMdePkgTokenSpaceGuid.PcdUartDefaultStopBits|1

  gHwTokenSpaceGuid.PcdM3SmmuBaseAddress|0xa0040000
  gHwTokenSpaceGuid.PcdPcieSmmuBaseAddress|0xb0040000
  gHwTokenSpaceGuid.PcdDsaSmmuBaseAddress|0xc0040000
  gHwTokenSpaceGuid.PcdAlgSmmuBaseAddress|0xd0040000
  gEfiMdeModulePkgTokenSpaceGuid.PcdFirmwareVersionString|L"D02 BIOS LINARO B905"

  gHwTokenSpaceGuid.PcdSystemProductName|L"D02"
  gHwTokenSpaceGuid.PcdSystemVersion|L"LINARO"
  gHwTokenSpaceGuid.PcdBaseBoardProductName|L"CH02TEVBC"
  gHwTokenSpaceGuid.PcdBaseBoardVersion|L"LINARO"
  
  gHwTokenSpaceGuid.PcdSlotPerChannelNum|0x2
  
  #
  # ARM PL390 General Interrupt Controller
  #

  
  gArmTokenSpaceGuid.PcdGicDistributorBase|0x8D000000
  gArmTokenSpaceGuid.PcdGicInterruptInterfaceBase|0xFE000000

  ## DTB address at spi flash
  gHwTokenSpaceGuid.FdtFileAddress|0xA4B00000

  ## NORFlash add by c00213799
  gHwTokenSpaceGuid.PcdNORFlashBase|0x90000000
  gHwTokenSpaceGuid.PcdNORFlashCachableSize|0x8000000

  #
  # ARM OS Loader
  #
  # Versatile Express machine type (ARM VERSATILE EXPRESS = 2272) required for ARM Linux:
  gArmPlatformTokenSpaceGuid.PcdDefaultBootDescription|L"Linux from SATA"
  gArmPlatformTokenSpaceGuid.PcdDefaultBootDevicePath|L"EFI\GRUB2\grubaa64.efi"
  gArmPlatformTokenSpaceGuid.PcdDefaultBootArgument|""

  # Use the serial console (ConIn & ConOut) and the Graphic driver (ConOut)
  gArmPlatformTokenSpaceGuid.PcdDefaultConOutPaths|L"VenHw(D3987D4B-971A-435F-8CAF-4967EB627241)/Uart(115200,8,N,1)/VenPcAnsi();VenHw(407B4008-BF5B-11DF-9547-CF16E0D72085)"
  gArmPlatformTokenSpaceGuid.PcdDefaultConInPaths|L"VenHw(D3987D4B-971A-435F-8CAF-4967EB627241)/Uart(115200,8,N,1)/VenPcAnsi()"

  #
  # ARM L2x0 PCDs
  #
  gArmTokenSpaceGuid.PcdL2x0ControllerBase|0x1E00A000

  #
  # ARM Architectual Timer Frequency
  #
  # Set model tick to 120Mhz. This depends a lot on workstation performance.
  #gArmTokenSpaceGuid.PcdArmArchTimerFreqInHz|120000000

  
  gArmTokenSpaceGuid.PcdArmArchTimerFreqInHz|50000000


  # add by t00200952
  gHwTokenSpaceGuid.PcdSysControlBaseAddress|0x80010000

  gHwTokenSpaceGuid.PcdMailBoxAddress|0x0000FFF8

  gHwTokenSpaceGuid.PcdCpldBaseAddress|0x98000000

 
  gHwTokenSpaceGuid.PcdTimerBaseAddress|0x80060000

 

  gHwTokenSpaceGuid.PcdSFCCFGBaseAddress|0xA6000000     
  gHwTokenSpaceGuid.PcdSFCMEM0BaseAddress|0xA4000000      

 
  gHwTokenSpaceGuid.PcdRamDiskMaxSize|128

  
  gHwTokenSpaceGuid.PcdPeriSubctrlAddress|0x80000000
  
  gHwTokenSpaceGuid.PcdMdioSubctrlAddress|0x80000000

  ## NORFlash add by c00213799
  gHwTokenSpaceGuid.PcdNORFlashBase|0x90000000
  gHwTokenSpaceGuid.PcdNORFlashCachableSize|0x8000000

  ## 1+1
  gHwTokenSpaceGuid.PcdPlatformDefaultPackageType|0x0

  gHwTokenSpaceGuid.PcdArmPrimaryCoreTemp|0x80020000

  gHwTokenSpaceGuid.PcdTopOfLowMemory|0x80000000
  gHwTokenSpaceGuid.PcdBottomOfHighMemory|0x1000000000
  
  gHwTokenSpaceGuid.PcdTrustedFirmwareEnable|0x1
  gHwTokenSpaceGuid.PcdTrustedFirmwareBL1Base|0xA4A00000

################################################################################
#
# Components Section - list of all EDK II Modules needed by this Platform
#
################################################################################
[Components.common]

  #
  # SEC
  #
  #OpenPlatformPkg/Chips/Hisilicon/Override/ArmPlatformPkg/Sec/Sec.inf {
  #  <LibraryClasses>
  #    # Use the implementation which set the Secure bits
  #    ArmGicLib|ArmPkg/Drivers/ArmGic/ArmGicSecLib.inf
  #}

  #
  # PEI Phase modules
  #
!if $(EDK2_SKIP_PEICORE) == 1
  ArmPlatformPkg/PrePi/PeiMPCore.inf {
    <LibraryClasses>
      ArmLib|ArmPkg/Library/ArmLib/AArch64/AArch64Lib.inf
      ArmPlatformLib|OpenPlatformPkg/Chips/Hisilicon/Library/ArmPlatformLibPv660/ArmPlatformLib.inf
      ArmPlatformGlobalVariableLib|ArmPlatformPkg/Library/ArmPlatformGlobalVariableLib/PrePi/PrePiArmPlatformGlobalVariableLib.inf
  }
  ArmPlatformPkg/PrePi/PeiUniCore.inf {
    <LibraryClasses>
      ArmLib|ArmPkg/Library/ArmLib/AArch64/AArch64Lib.inf
      ArmPlatformLib|OpenPlatformPkg/Chips/Hisilicon/Library/ArmPlatformLibPv660/ArmPlatformLib.inf
      ArmPlatformGlobalVariableLib|ArmPlatformPkg/Library/ArmPlatformGlobalVariableLib/PrePi/PrePiArmPlatformGlobalVariableLib.inf
  }
!else
  ArmPlatformPkg/PrePeiCore/PrePeiCoreMPCore.inf {
    <LibraryClasses>
      ArmPlatformGlobalVariableLib|ArmPlatformPkg/Library/ArmPlatformGlobalVariableLib/Pei/PeiArmPlatformGlobalVariableLib.inf
  }
  MdeModulePkg/Core/Pei/PeiMain.inf
  MdeModulePkg/Universal/PCD/Pei/Pcd.inf  {
    <LibraryClasses>
      PcdLib|MdePkg/Library/BasePcdLibNull/BasePcdLibNull.inf
  }
  
  ArmPlatformPkg/PlatformPei/PlatformPeim.inf
  #ArmPlatformPkg/MemoryInitPei/MemoryInitPeim.inf
  #OpenPlatformPkg/Chips/Hisilicon/Override/ArmPlatformPkg/MemoryInitPei/MemoryInitPeim.inf
  OpenPlatformPkg/Platforms/Hisilicon/D02/MemoryInitPei/MemoryInitPeim.inf
  ArmPkg/Drivers/CpuPei/CpuPei.inf
  IntelFrameworkModulePkg/Universal/StatusCode/Pei/StatusCodePei.inf
  OpenPlatformPkg/Chips/Hisilicon/Drivers/BootModePei/BootModePei.inf
  MdeModulePkg/Universal/FaultTolerantWritePei/FaultTolerantWritePei.inf
  MdeModulePkg/Universal/Variable/Pei/VariablePei.inf
  OpenPlatformPkg/Platforms/Hisilicon/D02/BootLinuxConfig/BootLinuxConfigPeim.inf

  MdeModulePkg/Core/DxeIplPeim/DxeIpl.inf {
    <LibraryClasses>
      NULL|IntelFrameworkModulePkg/Library/LzmaCustomDecompressLib/LzmaCustomDecompressLib.inf
  }
!endif

  #
  # DXE
  #
  MdeModulePkg/Core/Dxe/DxeMain.inf {
    <LibraryClasses>
      PcdLib|MdePkg/Library/BasePcdLibNull/BasePcdLibNull.inf
      NULL|MdeModulePkg/Library/DxeCrc32GuidedSectionExtractLib/DxeCrc32GuidedSectionExtractLib.inf
  }

  OpenPlatformPkg/Chips/Hisilicon/Drivers/IoInitDxe/IoInitDxe.inf

  #
  # Architectural Protocols
  #
  ArmPkg/Drivers/CpuDxe/CpuDxe.inf
  MdeModulePkg/Core/RuntimeDxe/RuntimeDxe.inf
 
  OpenPlatformPkg/Chips/Hisilicon/Drivers/NorFlashDxe/NorFlashDxe.inf

 
  OpenPlatformPkg/Platforms/Hisilicon/D02/OemNicConfigD02/OemNicConfigD02.inf

  #OpenPlatformPkg/Chips/Hisilicon/Drivers/SFC/SfcDxeDriver.inf
  OpenPlatformPkg/Platforms/Hisilicon/D02/Drivers/SFC/SfcDxeDriver.inf

  MdeModulePkg/Universal/SecurityStubDxe/SecurityStubDxe.inf
  MdeModulePkg/Universal/Variable/EmuRuntimeDxe/EmuVariableRuntimeDxe.inf
  #OpenPlatformPkg/Chips/Hisilicon/Drivers/NorFlashFvbDxe/NorFlashFvbDxe.inf
  OpenPlatformPkg/Chips/Hisilicon/Drivers/FlashFvbDxe/FlashFvbDxe.inf
  MdeModulePkg/Universal/Variable/RuntimeDxe/VariableRuntimeDxe.inf {
    <LibraryClasses>
      NULL|MdeModulePkg/Library/VarCheckUefiLib/VarCheckUefiLib.inf
  }
  MdeModulePkg/Universal/CapsuleRuntimeDxe/CapsuleRuntimeDxe.inf
  MdeModulePkg/Universal/FaultTolerantWriteDxe/FaultTolerantWriteDxe.inf
  EmbeddedPkg/EmbeddedMonotonicCounter/EmbeddedMonotonicCounter.inf

  MdeModulePkg/Universal/MonotonicCounterRuntimeDxe/MonotonicCounterRuntimeDxe.inf
  EmbeddedPkg/ResetRuntimeDxe/ResetRuntimeDxe.inf
  EmbeddedPkg/RealTimeClockRuntimeDxe/RealTimeClockRuntimeDxe.inf
  EmbeddedPkg/MetronomeDxe/MetronomeDxe.inf

  MdeModulePkg/Universal/Console/ConPlatformDxe/ConPlatformDxe.inf
  MdeModulePkg/Universal/Console/ConSplitterDxe/ConSplitterDxe.inf
  MdeModulePkg/Universal/Console/GraphicsConsoleDxe/GraphicsConsoleDxe.inf
  MdeModulePkg/Universal/Console/TerminalDxe/TerminalDxe.inf
  EmbeddedPkg/SerialDxe/SerialDxe.inf

  # Simple TextIn/TextOut for UEFI Terminal
  EmbeddedPkg/SimpleTextInOutSerial/SimpleTextInOutSerial.inf

  MdeModulePkg/Universal/HiiDatabaseDxe/HiiDatabaseDxe.inf

  #ArmPkg/Drivers/PL390Gic/PL390GicDxe.inf
  OpenPlatformPkg/Chips/Hisilicon/Drivers/PL390Gic/PL390GicDxe.inf

  #ArmPkg/Drivers/TimerDxe/TimerDxe
  OpenPlatformPkg/Chips/Hisilicon/Drivers/TimerDxe_SOC/TimerDxe_SOC.inf
  ArmPlatformPkg/Drivers/LcdGraphicsOutputDxe/PL111LcdGraphicsOutputDxe.inf

  OpenPlatformPkg/Chips/Hisilicon/Override/ArmPlatformPkg/Drivers/SP805WatchdogDxe/SP805WatchdogDxe.inf

  #OpenPlatformPkg/Platforms/Hisilicon/D02/CpuSelfTestDxe/CpuSelfTestDxe.inf

  IntelFrameworkModulePkg/Universal/StatusCode/RuntimeDxe/StatusCodeRuntimeDxe.inf
  #
  #ACPI
  #
  #OpenPlatformPkg/Chips/Hisilicon/AcpiTables/AcpiTables.inf
  MdeModulePkg/Universal/Acpi/AcpiPlatformDxe/AcpiPlatformDxe.inf
  MdeModulePkg/Universal/Acpi/AcpiTableDxe/AcpiTableDxe.inf

  OpenPlatformPkg/Chips/Hisilicon/P660/Pv660AcpiTables/AcpiTables.inf

  #Pci Express
  #OpenPlatformPkg/Chips/Hisilicon/Override/UefiCpuPkg/CpuIo2Dxe/CpuIo2Dxe.inf
  #OpenPlatformPkg/Chips/Hisilicon/Drivers/PcieInitDxe/PcieInitDxe.inf
  #OpenPlatformPkg/Chips/Hisilicon/Drivers/PciHostBridge/PciHostBridge.inf
  OpenPlatformPkg/Platforms/Hisilicon/D02/Drivers/CpuIo2Dxe/CpuIo2Dxe.inf
  OpenPlatformPkg/Platforms/Hisilicon/D02/Drivers/PcieInitDxe/PcieInitDxe.inf
  OpenPlatformPkg/Platforms/Hisilicon/D02/Drivers/PciHostBridge/PciHostBridge.inf
  MdeModulePkg/Bus/Pci/PciBusDxe/PciBusDxe.inf 

  #
  #network
  #
 
  #OpenPlatformPkg/Chips/Hisilicon/Drivers/Network/SnpPV600DxeDebug/SnpPV600Dxe.inf
  #OpenPlatformPkg/Chips/Hisilicon/Drivers/Network/SnpPV600DxeDebug_GE6/SnpPV600Dxe.inf
  #OpenPlatformPkg/Chips/Hisilicon/Drivers/Network/SnpPV600DxeD02Service_GE5/SnpPV600Dxe.inf
  #OpenPlatformPkg/Chips/Hisilicon/Drivers/Network/SnpPV600Dxe_PLAT/SnpPV600DxeMac4.inf
  #OpenPlatformPkg/Chips/Hisilicon/Drivers/Network/SnpPV600Dxe_PLAT/SnpPV600DxeMac5.inf
  OpenPlatformPkg/Platforms/Hisilicon/D02/Drivers/SnpPV600Dxe_PLAT/SnpPV600DxeMac5.inf
  
  MdeModulePkg/Universal/Network/ArpDxe/ArpDxe.inf
  MdeModulePkg/Universal/Network/Dhcp4Dxe/Dhcp4Dxe.inf
  MdeModulePkg/Universal/Network/DpcDxe/DpcDxe.inf
  MdeModulePkg/Universal/Network/Ip4Dxe/Ip4Dxe.inf
  MdeModulePkg/Universal/Network/MnpDxe/MnpDxe.inf
  MdeModulePkg/Universal/Network/Mtftp4Dxe/Mtftp4Dxe.inf
  MdeModulePkg/Universal/Network/Tcp4Dxe/Tcp4Dxe.inf
  MdeModulePkg/Universal/Network/Udp4Dxe/Udp4Dxe.inf
  MdeModulePkg/Universal/Network/UefiPxeBcDxe/UefiPxeBcDxe.inf
  MdeModulePkg/Universal/Network/VlanConfigDxe/VlanConfigDxe.inf

  #
  # Semi-hosting filesystem
  #
  ArmPkg/Filesystem/SemihostFs/SemihostFs.inf

  #
  # Multimedia Card Interface
  #
  #EmbeddedPkg/Universal/MmcDxe/MmcDxe.inf
  #ArmPlatformPkg/Drivers/PL180MciDxe/PL180MciDxe.inf

  #
  # FAT filesystem + GPT/MBR partitioning
  #
 
  OpenPlatformPkg/Chips/Hisilicon/Drivers/ramdisk/ramdisk.inf
  MdeModulePkg/Universal/Disk/DiskIoDxe/DiskIoDxe.inf
  MdeModulePkg/Universal/Disk/PartitionDxe/PartitionDxe.inf
  MdeModulePkg/Universal/Disk/UnicodeCollation/EnglishDxe/EnglishDxe.inf

  #OpenPlatformPkg/Chips/Hisilicon/Override/EmbeddedPkg/Ebl/Ebl.inf
  OpenPlatformPkg/Platforms/Hisilicon/D02/Ebl/Ebl.inf
  #c00213799 secure boot test
  MdeModulePkg/Application/HelloWorld/HelloWorld.inf
  #
  # Bds
  #
  MdeModulePkg/Universal/DevicePathDxe/DevicePathDxe.inf

  #OpenPlatformPkg/Chips/Hisilicon/Network/OemFtp/OemFtp.inf


  #OpenPlatformPkg/Chips/Hisilicon/Drivers/AtaAtapiPassThru/AtaAtapiPassThru.inf
  OpenPlatformPkg/Platforms/Hisilicon/D02/Drivers/AtaAtapiPassThru/AtaAtapiPassThru.inf
  MdeModulePkg/Bus/Ata/AtaBusDxe/AtaBusDxe.inf


  #
  # USB Support
  #
  OpenPlatformPkg/Chips/Hisilicon/Drivers/EhciDxe/EhciDxe.inf
  MdeModulePkg/Bus/Usb/UsbBusDxe/UsbBusDxe.inf
  MdeModulePkg/Bus/Usb/UsbMassStorageDxe/UsbMassStorageDxe.inf

 
  MdeModulePkg/Universal/SmbiosDxe/SmbiosDxe.inf
  OpenPlatformPkg/Chips/Hisilicon/Drivers/Smbios/SmbiosMiscDxe/SmbiosMiscDxe.inf

  OpenPlatformPkg/Chips/Hisilicon/Drivers/UpdateFdtDxe/UpdateFdtDxe.inf
  
 
  OpenPlatformPkg/Chips/Hisilicon/Drivers/Smbios/MemorySubClassDxe/MemorySubClassDxe.inf
  
  OpenPlatformPkg/Chips/Hisilicon/Drivers/Smbios/ProcessorSubClassDxe/ProcessorSubClassDxe.inf

  OpenPlatformPkg/Chips/Hisilicon/Bds/Bds.inf

  #
  # UEFI application (Shell Embedded Boot Loader)
  #
  ShellPkg/Application/Shell/Shell.inf {
    <LibraryClasses>
      ShellCommandLib|ShellPkg/Library/UefiShellCommandLib/UefiShellCommandLib.inf
      NULL|ShellPkg/Library/UefiShellLevel2CommandsLib/UefiShellLevel2CommandsLib.inf
      NULL|ShellPkg/Library/UefiShellLevel1CommandsLib/UefiShellLevel1CommandsLib.inf
      NULL|ShellPkg/Library/UefiShellLevel3CommandsLib/UefiShellLevel3CommandsLib.inf
      NULL|ShellPkg/Library/UefiShellDriver1CommandsLib/UefiShellDriver1CommandsLib.inf
      NULL|ShellPkg/Library/UefiShellDebug1CommandsLib/UefiShellDebug1CommandsLib.inf
      NULL|ShellPkg/Library/UefiShellInstall1CommandsLib/UefiShellInstall1CommandsLib.inf
      NULL|ShellPkg/Library/UefiShellNetwork1CommandsLib/UefiShellNetwork1CommandsLib.inf
      HandleParsingLib|ShellPkg/Library/UefiHandleParsingLib/UefiHandleParsingLib.inf
      PrintLib|MdePkg/Library/BasePrintLib/BasePrintLib.inf
      BcfgCommandLib|ShellPkg/Library/UefiShellBcfgCommandLib/UefiShellBcfgCommandLib.inf
!ifdef $(INCLUDE_DP)
      NULL|ShellPkg/Library/UefiDpLib/UefiDpLib.inf
!endif #$(INCLUDE_DP)
!ifdef $(INCLUDE_TFTP_COMMAND)
      NULL|ShellPkg/Library/UefiShellTftpCommandLib/UefiShellTftpCommandLib.inf
!endif #$(INCLUDE_TFTP_COMMAND)

    <PcdsFixedAtBuild>
      gEfiMdePkgTokenSpaceGuid.PcdDebugPropertyMask|0xFF
      gEfiShellPkgTokenSpaceGuid.PcdShellLibAutoInitialize|FALSE
      gEfiMdePkgTokenSpaceGuid.PcdUefiLibMaxPrintBufferSize|8000
  }
