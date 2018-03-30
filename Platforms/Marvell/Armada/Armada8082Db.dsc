#Copyright (C) 2018 Marvell International Ltd.
#
#Marvell BSD License Option
#
#If you received this File from Marvell, you may opt to use, redistribute and/or
#modify this File under the following licensing terms.
#Redistribution and use in source and binary forms, with or without modification,
#are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# * Neither the name of Marvell nor the names of its contributors may be
# used to endorse or promote products derived from this software without
# specific prior written permission.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
#ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
################################################################################
#
# Defines Section - statements that will be processed to create a Makefile.
#
################################################################################
[Defines]
  PLATFORM_NAME                  = Armada8082Db
  PLATFORM_GUID                  = 0b17c771-1a19-4b13-b32d-b1dd357e9f40
  PLATFORM_VERSION               = 0.1
  DSC_SPECIFICATION              = 0x00010019
  OUTPUT_DIRECTORY               = Build/$(PLATFORM_NAME)-$(ARCH)
  SUPPORTED_ARCHITECTURES        = AARCH64|ARM
  BUILD_TARGETS                  = DEBUG|RELEASE
  SKUID_IDENTIFIER               = DEFAULT
  FLASH_DEFINITION               = OpenPlatformPkg/Platforms/Marvell/Armada/Armada8kPlus.fdf

!include Armada8kPlus.dsc.inc

[LibraryClasses.common]
  ArmadaBoardDescLib|OpenPlatformPkg/Platforms/Marvell/Armada/Library/Armada8082DbBoardDescLib/Armada8082DbBoardDescLib.inf

################################################################################
#
# Pcd Section - list of all EDK II PCD Entries defined by this Platform
#
################################################################################
[PcdsFixedAtBuild.common]
  #BoardId: Armada-8082-Db
  gMarvellTokenSpaceGuid.PcdBoardId|0x3

  #CPU count
  gArmPlatformTokenSpaceGuid.PcdCoreCount|8

  #Timer
  gEmbeddedTokenSpaceGuid.PcdTimerPeriod|10000

  #MPP
  gMarvellTokenSpaceGuid.PcdMppChipCount|3

  # AP810 MPP SET
  gMarvellTokenSpaceGuid.PcdChip0MppReverseFlag|FALSE
  gMarvellTokenSpaceGuid.PcdChip0MppBaseAddress|0xE86F4000
  gMarvellTokenSpaceGuid.PcdChip0MppPinCount|21
  gMarvellTokenSpaceGuid.PcdChip0MppSel0|{ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0x2, 0x2, 0x2 }
  gMarvellTokenSpaceGuid.PcdChip0MppSel1|{ 0x2, 0x3, 0x2, 0x3, 0x3, 0x0, 0x0, 0x0, 0x3, 0x3 }
  gMarvellTokenSpaceGuid.PcdChip0MppSel2|{ 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }

  # CP110 MPP SET - master
  gMarvellTokenSpaceGuid.PcdChip1MppReverseFlag|FALSE
  gMarvellTokenSpaceGuid.PcdChip1MppBaseAddress|0x8100440000
  gMarvellTokenSpaceGuid.PcdChip1MppPinCount|64
  gMarvellTokenSpaceGuid.PcdChip1MppSel0|{ 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3 }
  gMarvellTokenSpaceGuid.PcdChip1MppSel1|{ 0x3, 0x3, 0x0, 0x3, 0x3, 0x3, 0x3, 0x0, 0x0, 0x0 }
  gMarvellTokenSpaceGuid.PcdChip1MppSel2|{ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x9, 0xA }
  gMarvellTokenSpaceGuid.PcdChip1MppSel3|{ 0xA, 0x0, 0x7, 0x0, 0x7, 0x5, 0x5, 0x2, 0x2, 0x0 }
  gMarvellTokenSpaceGuid.PcdChip1MppSel4|{ 0x7, 0x7, 0x0, 0x0, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1 }
  gMarvellTokenSpaceGuid.PcdChip1MppSel5|{ 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0xE, 0xE, 0xE, 0xE }
  gMarvellTokenSpaceGuid.PcdChip1MppSel6|{ 0xE, 0xE, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }

  # CP110 MPP SET - slave
  gMarvellTokenSpaceGuid.PcdChip2MppReverseFlag|FALSE
  gMarvellTokenSpaceGuid.PcdChip2MppBaseAddress|0x8800440000
  gMarvellTokenSpaceGuid.PcdChip2MppPinCount|64
  gMarvellTokenSpaceGuid.PcdChip2MppSel0|{ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }
  gMarvellTokenSpaceGuid.PcdChip2MppSel1|{ 0x0, 0x0, 0x0, 0x3, 0x3, 0x3, 0x3, 0x0, 0x0, 0x0 }
  gMarvellTokenSpaceGuid.PcdChip2MppSel2|{ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }
  gMarvellTokenSpaceGuid.PcdChip2MppSel3|{ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x2, 0x2, 0x0 }
  gMarvellTokenSpaceGuid.PcdChip2MppSel4|{ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }
  gMarvellTokenSpaceGuid.PcdChip2MppSel5|{ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }
  gMarvellTokenSpaceGuid.PcdChip2MppSel6|{ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }

  #SPI
  gMarvellTokenSpaceGuid.PcdSpiRegBase|0xE8510600
  gMarvellTokenSpaceGuid.PcdSpiMaxFrequency|10000000
  gMarvellTokenSpaceGuid.PcdSpiClockFrequency|200000000

  gMarvellTokenSpaceGuid.PcdSpiFlashMode|3
  gMarvellTokenSpaceGuid.PcdSpiFlashCs|0

  #ComPhy
  gMarvellTokenSpaceGuid.PcdComPhyDevices|{ 0x1, 0x1 }
  # ComPhy0
  # 0: PCIE0         5 Gbps
  # 1: PCIE0         5 Gbps
  # 2: PCIE0         5 Gbps
  # 3: PCIE0         5 Gbps
  # 4: SFI           10.31 Gbps
  # 5: SATA1         5 Gbps
  gMarvellTokenSpaceGuid.PcdChip0ComPhyTypes|{ 0x1, 0x1, 0x1, 0x1, 0x17, 0x6 }
  gMarvellTokenSpaceGuid.PcdChip0ComPhySpeeds|{ $(CP_5G), $(CP_5G), $(CP_5G), $(CP_5G), $(CP_10_3125G), $(CP_5G) }
  # ComPhy1
  # 0: PCIE0         5 Gbps
  # 1: PCIE0         5 Gbps
  # 2: PCIE0         5 Gbps
  # 3: PCIE0         5 Gbps
  # 4: SFI           10.31 Gbps
  # 5: SATA1         5 Gbps
  gMarvellTokenSpaceGuid.PcdChip1ComPhyTypes|{ 0x1, 0x1, 0x1, 0x1, 0x17, 0x8 }
  gMarvellTokenSpaceGuid.PcdChip1ComPhySpeeds|{ $(CP_5G), $(CP_5G), $(CP_5G), $(CP_5G), $(CP_10_3125G), $(CP_5G) }

  #UtmiPhy
  gMarvellTokenSpaceGuid.PcdUtmiControllers|{ 0x1, 0x1, 0x0, 0x0 }
  gMarvellTokenSpaceGuid.PcdUtmiPortType|{ 0x0, 0x1, 0x0, 0x0 }

  #MDIO
  gMarvellTokenSpaceGuid.PcdMdioControllers|{ 0x1, 0x0 }

  #PHY
  gMarvellTokenSpaceGuid.PcdPhy2MdioController|{ 0x0, 0x0 }
  gMarvellTokenSpaceGuid.PcdPhyDeviceIds|{ 0x0, 0x0 }
  gMarvellTokenSpaceGuid.PcdPhySmiAddresses|{ 0x0, 0x1 }
  gMarvellTokenSpaceGuid.PcdPhyStartupAutoneg|FALSE

  #NET
  gMarvellTokenSpaceGuid.PcdPp2GopIndexes|{ 0x0, 0x2, 0x3, 0x0 }
  gMarvellTokenSpaceGuid.PcdPp2InterfaceAlwaysUp|{ 0x0, 0x0, 0x0, 0x0 }
  gMarvellTokenSpaceGuid.PcdPp2InterfaceSpeed|{ 0x5, 0x3, 0x3, 0x5 }
  gMarvellTokenSpaceGuid.PcdPp2PhyConnectionTypes|{ 0x8, 0x0, 0x0, 0x8 }
  gMarvellTokenSpaceGuid.PcdPp2PhyIndexes|{ 0xFF, 0x0, 0x1, 0xFF }
  gMarvellTokenSpaceGuid.PcdPp2Port2Controller|{ 0x0, 0x0, 0x0, 0x1 }
  gMarvellTokenSpaceGuid.PcdPp2PortIds|{ 0x0, 0x1, 0x2, 0x0 }
  gMarvellTokenSpaceGuid.PcdPp2Controllers|{ 0x1, 0x1 }

  #Pcie
  gMarvellTokenSpaceGuid.PcdPcieControllersEnabled|{ 0x1 }

  #NonDiscoverableDevices
  gMarvellTokenSpaceGuid.PcdPciEXhci|{ 0x1, 0x1, 0x0, 0x0 }
  gMarvellTokenSpaceGuid.PcdPciEAhci|{ 0x1, 0x1 }
  gMarvellTokenSpaceGuid.PcdPciESdhci|{ 0x0, 0x0 }

  #RTC
  gMarvellTokenSpaceGuid.PcdRtcBaseAddress|0x8800284000

  #SdMmc
  gMarvellTokenSpaceGuid.PcdXenon1v8Enable|{ 0x0, 0x0 }
  gMarvellTokenSpaceGuid.PcdXenon8BitBusEnable|{ 0x0, 0x0 }
  gMarvellTokenSpaceGuid.PcdXenonSlowModeEnable|{ 0x0, 0x0 }
