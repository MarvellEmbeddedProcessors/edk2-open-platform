## @file
#  Component description file for the CN9132 Development Board (variant A)
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
  PLATFORM_NAME                  = Cn9132DbA
  PLATFORM_GUID                  = b9d2c816-296f-460f-a190-fe9afdd208c7
  PLATFORM_VERSION               = 0.1
  DSC_SPECIFICATION              = 0x0001000B
  OUTPUT_DIRECTORY               = Build/$(PLATFORM_NAME)-$(ARCH)
  SUPPORTED_ARCHITECTURES        = AARCH64|ARM
  BUILD_TARGETS                  = DEBUG|RELEASE|NOOPT
  SKUID_IDENTIFIER               = DEFAULT
  FLASH_DEFINITION               = Silicon/Marvell/Armada7k8k/Armada7k8k.fdf
  BOARD_DXE_FV_COMPONENTS        = Platform/Marvell/Cn913xDb/Cn9132DbA.fdf.inc

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
!include Platform/Marvell/Cn913xDb/Cn9132DbA.dsc.inc

[Components.common]
  Silicon/Marvell/OcteonTx/DeviceTree/T91/Cn9132DbA.inf

[LibraryClasses.common]
  ArmadaBoardDescLib|Platform/Marvell/Cn913xDb/BoardDescriptionLib/Cn9132DbABoardDescLib.inf
  NonDiscoverableInitLib|Platform/Marvell/Cn913xDb/NonDiscoverableInitLib/NonDiscoverableInitLib.inf
