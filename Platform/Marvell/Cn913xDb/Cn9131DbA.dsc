## @file
#  Component description file for the CN9131 Development Board (variant A)
#
#  Copyright (c) 2019 Marvell International Ltd.<BR>
#
#  SPDX-License-Identifier: BSD-2-Clause-Patent
#
##

################################################################################
#
# Defines Section - statements that will be processed to create a Makefile.
#
################################################################################
[Defines]
  PLATFORM_NAME                  = Cn9131DbA
  PLATFORM_GUID                  = 9fcb32d0-ea4e-4e9c-863d-06d90b160855
  PLATFORM_VERSION               = 0.1
  DSC_SPECIFICATION              = 0x0001000B
  OUTPUT_DIRECTORY               = Build/$(PLATFORM_NAME)-$(ARCH)
  SUPPORTED_ARCHITECTURES        = AARCH64|ARM
  BUILD_TARGETS                  = DEBUG|RELEASE|NOOPT
  SKUID_IDENTIFIER               = DEFAULT
  FLASH_DEFINITION               = Silicon/Marvell/Armada7k8k/Armada7k8k.fdf
  BOARD_DXE_FV_COMPONENTS        = Platform/Marvell/Cn913xDb/Cn9131DbA.fdf.inc

  #
  # Network definition
  #
  DEFINE NETWORK_IP6_ENABLE             = FALSE
  DEFINE NETWORK_TLS_ENABLE             = FALSE
  DEFINE NETWORK_HTTP_BOOT_ENABLE       = FALSE
  DEFINE NETWORK_ISCSI_ENABLE           = FALSE

!include Silicon/Marvell/Armada7k8k/Armada7k8k.dsc.inc
!include Platform/Marvell/Cn913xDb/Cn9130DbA.dsc.inc
!include Platform/Marvell/Cn913xDb/Cn9131DbA.dsc.inc

[Components.common]
  Silicon/Marvell/OcteonTx/DeviceTree/T91/Cn9131DbA.inf

[Components.AARCH64]
  Silicon/Marvell/OcteonTx/AcpiTables/T91/Cn9131DbA.inf

[LibraryClasses.common]
  ArmadaBoardDescLib|Platform/Marvell/Cn913xDb/BoardDescriptionLib/Cn9130DbABoardDescLib.inf
  NonDiscoverableInitLib|Platform/Marvell/Cn913xDb/NonDiscoverableInitLib/NonDiscoverableInitLib.inf
