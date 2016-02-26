#Copyright (C) 2016 Marvell International Ltd.
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
  PLATFORM_NAME                  = Armada7040_rz
  PLATFORM_GUID                  = 617dfbfc-b506-4365-9690-b212b6538b71
  PLATFORM_VERSION               = 0.1
  DSC_SPECIFICATION              = 0x00010005
  OUTPUT_DIRECTORY               = Build/$(PLATFORM_NAME)
  SUPPORTED_ARCHITECTURES        = AARCH64
  BUILD_TARGETS                  = DEBUG|RELEASE
  SKUID_IDENTIFIER               = DEFAULT
  FLASH_DEFINITION               = OpenPlatformPkg/Platforms/Marvell/Armada7040/Armada7040.fdf

!include Armada7040.dsc.inc

################################################################################
#
# Pcd Section - list of all EDK II PCD Entries defined by this Platform
#
################################################################################
  # Marvell Pcds
[PcdsFixedAtBuild.common]
  gMarvellTokenSpaceGuid.PcdI2cSlaveAddresses|{ 0x50, 0x57, 0x60  }
  gMarvellTokenSpaceGuid.PcdEepromI2cAddresses|{ 0x50, 0x57 }
  gMarvellTokenSpaceGuid.PcdI2cBaseAddress|0xF0511000
  gMarvellTokenSpaceGuid.PcdTclkFrequency|200000000

  #SPI
  gMarvellTokenSpaceGuid.PcdSpiRegBase|0xF2700680
  gMarvellTokenSpaceGuid.PcdSpiTimerClock|250000000
  gMarvellTokenSpaceGuid.PcdSpiMaxFrequency|10000000
  gMarvellTokenSpaceGuid.PcdSpiTimeout|100000

  gMarvellTokenSpaceGuid.PcdSpiFlashAddressCycles|3
  gMarvellTokenSpaceGuid.PcdSpiFlashEraseSize|65536
  gMarvellTokenSpaceGuid.PcdSpiFlashPageSize|256
  gMarvellTokenSpaceGuid.PcdSpiFlashMode|0
  gMarvellTokenSpaceGuid.PcdSpiFlashCs|0
  gMarvellTokenSpaceGuid.PcdSpiFlashId|0x20BA18

#MPP
  gMarvellTokenSpaceGuid.PcdApPinMuxChipCount|1

  # APN806-Z1 MPP SET
  gMarvellTokenSpaceGuid.PcdAp0PinMuxReverseFlag|TRUE
  gMarvellTokenSpaceGuid.PcdAp0PinMuxBaseAddress|0xF06F008C
  gMarvellTokenSpaceGuid.PcdAp0MppRegCount|2
  gMarvellTokenSpaceGuid.PcdAp0MppSel0|{ 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1 }
  gMarvellTokenSpaceGuid.PcdAp0MppSel1|{ 0x1, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0 }

  # APN806-A0 MPP SET
  #gMarvellTokenSpaceGuid.PcdAp0PinMuxReverseFlag|FALSE
  #gMarvellTokenSpaceGuid.PcdAp0PinMuxBaseAddress|0xF06F4000
  #gMarvellTokenSpaceGuid.PcdAp0MppRegCount|3
  #gMarvellTokenSpaceGuid.PcdAp0MppSel0|{ 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1 }
  #gMarvellTokenSpaceGuid.PcdAp0MppSel1|{ 0x1, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0, 0x0 }
  #gMarvellTokenSpaceGuid.PcdAp0MppSel2|{ 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }
