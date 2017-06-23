#Copyright (C) 2017 Marvell International Ltd.
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
  PLATFORM_NAME                  = Armada80x0McBin
  PLATFORM_GUID                  = 256e46dc-bff2-4e83-8ab3-6d2a3bec3f62
  PLATFORM_VERSION               = 0.1
  DSC_SPECIFICATION              = 0x00010019
  OUTPUT_DIRECTORY               = Build/$(PLATFORM_NAME)-$(ARCH)
  SUPPORTED_ARCHITECTURES        = AARCH64|ARM
  BUILD_TARGETS                  = DEBUG|RELEASE
  SKUID_IDENTIFIER               = DEFAULT
  FLASH_DEFINITION               = OpenPlatformPkg/Platforms/Marvell/Armada/Armada.fdf

!include Armada.dsc.inc

################################################################################
#
# Pcd Section - list of all EDK II PCD Entries defined by this Platform
#
################################################################################
[PcdsFixedAtBuild.common]
  #BoardId: Armada-8040-MachiatoBin
  gMarvellTokenSpaceGuid.PcdBoardId|0x2

  #Timer
  gEmbeddedTokenSpaceGuid.PcdTimerPeriod|10000

  #MPP
  gMarvellTokenSpaceGuid.PcdMppChipCount|3

  # APN806-A0 MPP SET
  gMarvellTokenSpaceGuid.PcdChip0MppReverseFlag|FALSE
  gMarvellTokenSpaceGuid.PcdChip0MppBaseAddress|0xF06F4000
  gMarvellTokenSpaceGuid.PcdChip0MppPinCount|20
  gMarvellTokenSpaceGuid.PcdChip0MppSel0|{ 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1 }
  gMarvellTokenSpaceGuid.PcdChip0MppSel1|{ 0x1, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x3 }

  # CP110 MPP SET - master
  gMarvellTokenSpaceGuid.PcdChip1MppReverseFlag|FALSE
  gMarvellTokenSpaceGuid.PcdChip1MppBaseAddress|0xF2440000
  gMarvellTokenSpaceGuid.PcdChip1MppPinCount|64
  gMarvellTokenSpaceGuid.PcdChip1MppSel0|{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
  gMarvellTokenSpaceGuid.PcdChip1MppSel1|{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
  gMarvellTokenSpaceGuid.PcdChip1MppSel2|{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
  gMarvellTokenSpaceGuid.PcdChip1MppSel3|{ 0xFF, 0x0, 0x7, 0xA, 0x7, 0x2, 0x2, 0x2, 0x2, 0xA }
  gMarvellTokenSpaceGuid.PcdChip1MppSel4|{ 0x7, 0x7, 0x8, 0x8, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }
  gMarvellTokenSpaceGuid.PcdChip1MppSel5|{ 0x0, 0x0, 0x9, 0x0, 0x0, 0x0, 0xE, 0xE, 0xE, 0xE }
  gMarvellTokenSpaceGuid.PcdChip1MppSel6|{ 0xE, 0xE, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }

  # CP110 MPP SET - slave
  gMarvellTokenSpaceGuid.PcdChip2MppReverseFlag|FALSE
  gMarvellTokenSpaceGuid.PcdChip2MppBaseAddress|0xF4440000
  gMarvellTokenSpaceGuid.PcdChip2MppPinCount|64
  gMarvellTokenSpaceGuid.PcdChip2MppSel0|{ 0x4, 0x4, 0x4, 0x4, 0x4, 0x4, 0x8, 0x8, 0x0, 0x0 }
  gMarvellTokenSpaceGuid.PcdChip2MppSel1|{ 0x0, 0x0, 0x3, 0x3, 0x3, 0x3, 0x3, 0xFF, 0xFF, 0xFF }
  gMarvellTokenSpaceGuid.PcdChip2MppSel2|{ 0xFF, 0xFF, 0xFF, 0xFF, 0x0, 0xFF, 0x0, 0x0, 0x0, 0x0 }
  gMarvellTokenSpaceGuid.PcdChip2MppSel3|{ 0x0, 0x0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
  gMarvellTokenSpaceGuid.PcdChip2MppSel4|{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
  gMarvellTokenSpaceGuid.PcdChip2MppSel5|{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }
  gMarvellTokenSpaceGuid.PcdChip2MppSel6|{ 0xFF, 0xFF, 0xFF, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }

  #SPI
  gMarvellTokenSpaceGuid.PcdSpiRegBase|0xF4700680
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
  gMarvellTokenSpaceGuid.PcdChip0ComPhySpeeds|{ 0x6, 0x6, 0x6, 0x6, 0xA, 0x6 }
  # ComPhy1
  # 0: SGMII1        1.25 Gbps
  # 1: SATA0         5 Gbps
  # 2: USB3_HOST0    5 Gbps
  # 3: SATA1         5 Gbps
  # 4: SFI           10.31 Gbps
  # 5: SGMII2        3.125 Gbps
  gMarvellTokenSpaceGuid.PcdChip1ComPhyTypes|{ 0xA, 0x7, 0xE, 0x8, 0x17, 0xB }
  gMarvellTokenSpaceGuid.PcdChip1ComPhySpeeds|{ 0x1, 0x6, 0x6, 0x6, 0xA, 0x5 }

  #UtmiPhy
  gMarvellTokenSpaceGuid.PcdUtmiControllers|{ 0x1, 0x1, 0x1, 0x0 }
  gMarvellTokenSpaceGuid.PcdUtmiPortType|{ 0x0, 0x1, 0x0, 0x0 }

  #MDIO
  gMarvellTokenSpaceGuid.PcdMdioControllers|{ 0x1, 0x0 }

  #PHY
  gMarvellTokenSpaceGuid.PcdPhy2MdioController|{ 0x0 }
  gMarvellTokenSpaceGuid.PcdPhyDeviceIds|{ 0x0 }
  gMarvellTokenSpaceGuid.PcdPhySmiAddresses|{ 0x0 }
  gMarvellTokenSpaceGuid.PcdPhyStartupAutoneg|FALSE

  #NET
  gMarvellTokenSpaceGuid.PcdPp2GopIndexes|{ 0x0, 0x0, 0x2, 0x3 }
  gMarvellTokenSpaceGuid.PcdPp2InterfaceAlwaysUp|{ 0x0, 0x0, 0x0, 0x0 }
  gMarvellTokenSpaceGuid.PcdPp2InterfaceSpeed|{ 0x5, 0x5, 0x3, 0x4 }
  gMarvellTokenSpaceGuid.PcdPp2PhyConnectionTypes|{ 0x8, 0x8, 0x4, 0x4 }
  gMarvellTokenSpaceGuid.PcdPp2PhyIndexes|{ 0xFF, 0xFF, 0x0, 0xFF }
  gMarvellTokenSpaceGuid.PcdPp2Port2Controller|{ 0x0, 0x1, 0x1, 0x1 }
  gMarvellTokenSpaceGuid.PcdPp2PortIds|{ 0x0, 0x0, 0x1, 0x2 }
  gMarvellTokenSpaceGuid.PcdPp2Controllers|{ 0x1, 0x1 }

  #PciEmulation
  gMarvellTokenSpaceGuid.PcdPciEXhci|{ 0x1, 0x1, 0x1, 0x0 }
  gMarvellTokenSpaceGuid.PcdPciEAhci|{ 0x1, 0x1 }
  gMarvellTokenSpaceGuid.PcdPciESdhci|{ 0x0, 0x0 }

  #RTC
  gMarvellTokenSpaceGuid.PcdRtcEnabled|{ 0x0, 0x1 }

  #SdMmc
  gMarvellTokenSpaceGuid.PcdXenon1v8Enable|{ 0x0, 0x0 }
  gMarvellTokenSpaceGuid.PcdXenon8BitBusEnable|{ 0x1, 0x0 }
  gMarvellTokenSpaceGuid.PcdXenonSlowModeEnable|{ 0x1, 0x0 }

[Components.AARCH64]
  OpenPlatformPkg/Platforms/Marvell/Armada/AcpiTables/Armada80x0/AcpiTables.inf
