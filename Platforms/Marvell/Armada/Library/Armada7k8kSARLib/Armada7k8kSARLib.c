/********************************************************************************
Copyright (C) 2018 Marvell International Ltd.

Marvell BSD License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File under the following licensing terms.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must Retain the above copyright notice,
  this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

* Neither the name of Marvell nor the names of its contributors may be
  used to endorse or promote products derived from this software without
  specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include <Uefi.h>

#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MvSARLib.h>

#define SAR_CLOCK_FREQ_MODE_OFFSET      0
#define SAR_CLOCK_FREQ_MODE_MASK        (0x1f << SAR_CLOCK_FREQ_MODE_OFFSET)
#define SAR_MAX_OPTIONS                 16

#define CPU_CLOCK_ID                    0
#define DDR_CLOCK_ID                    1
#define RING_CLOCK_ID                   2

#define MV_AP_SAR_BASE                  0xf06f8200
#define MV_CP_SAR_BASE(_CpIndex)        (0xf2000000 + (0x2000000 * _CpIndex) + 0x400200)

#define MHz                             1000000

#define CP0_PCIE0_CLK_MASK      0x4
#define CP0_PCIE1_CLK_MASK      0x8
#define CP1_PCIE0_CLK_MASK      0x1
#define CP1_PCIE1_CLK_MASK      0x2
#define CP0_PCIE0_CLK_OFFSET      2
#define CP0_PCIE1_CLK_OFFSET      3
#define CP1_PCIE0_CLK_OFFSET      0
#define CP1_PCIE1_CLK_OFFSET      1

typedef enum {
  CPU_2000_DDR_1200_RCLK_1200 = 0x0,
  CPU_2000_DDR_1050_RCLK_1050 = 0x1,
  CPU_1600_DDR_800_RCLK_800   = 0x4,
  CPU_1800_DDR_1200_RCLK_1200 = 0x6,
  CPU_1800_DDR_1050_RCLK_1050 = 0x7,
  CPU_1600_DDR_1050_RCLK_1050 = 0x0d,
  CPU_1000_DDR_650_RCLK_650   = 0x13,
  CPU_1300_DDR_800_RCLK_800   = 0x14,
  CPU_1300_DDR_650_RCLK_650   = 0x17,
  CPU_1200_DDR_800_RCLK_800   = 0x19,
  CPU_1400_DDR_800_RCLK_800   = 0x1a,
  CPU_600_DDR_800_RCLK_800    = 0x1b,
  CPU_800_DDR_800_RCLK_800    = 0x1c,
  CPU_1000_DDR_800_RCLK_800   = 0x1d,
} ClockingOptions;

static const UINT32 PllFreqTbl[SAR_MAX_OPTIONS][4] = {
  /* CPU */   /* DDR */   /* Ring */
  {2000 * MHz, 1200 * MHz, 1200 * MHz, CPU_2000_DDR_1200_RCLK_1200},
  {2000 * MHz, 1050 * MHz, 1050 * MHz, CPU_2000_DDR_1050_RCLK_1050},
  {1800 * MHz, 1200 * MHz, 1200 * MHz, CPU_1800_DDR_1200_RCLK_1200},
  {1800 * MHz, 1050 * MHz, 1050 * MHz, CPU_1800_DDR_1050_RCLK_1050},
  {1600 * MHz, 1050 * MHz, 1050 * MHz, CPU_1600_DDR_1050_RCLK_1050},
  {1300 * MHz, 800  * MHz, 800  * MHz, CPU_1300_DDR_800_RCLK_800},
  {1300 * MHz, 650  * MHz, 650  * MHz, CPU_1300_DDR_650_RCLK_650},
  {1600 * MHz, 800  * MHz, 800  * MHz, CPU_1600_DDR_800_RCLK_800},
  {1000 * MHz, 650  * MHz, 650  * MHz, CPU_1000_DDR_650_RCLK_650},
  {1200 * MHz, 800  * MHz, 800  * MHz, CPU_1200_DDR_800_RCLK_800},
  {1400 * MHz, 800  * MHz, 800  * MHz, CPU_1400_DDR_800_RCLK_800},
  {600  * MHz, 800  * MHz, 800  * MHz, CPU_600_DDR_800_RCLK_800},
  {800  * MHz, 800  * MHz, 800  * MHz, CPU_800_DDR_800_RCLK_800},
  {1000 * MHz, 800  * MHz, 800  * MHz, CPU_1000_DDR_800_RCLK_800}
};

static const UINT32 PcieClkMask[2][2] = {
  {CP0_PCIE0_CLK_MASK, CP0_PCIE1_CLK_MASK},
  {CP1_PCIE0_CLK_MASK, CP1_PCIE1_CLK_MASK}
};

static const UINT32 PcieClkOffset[2][2] = {
  {CP0_PCIE0_CLK_OFFSET, CP0_PCIE1_CLK_OFFSET},
  {CP1_PCIE0_CLK_OFFSET, CP1_PCIE1_CLK_OFFSET}
};

UINT32
EFIAPI
MvSARGetCpuFreq (
  VOID
  )
{
  UINT32 ClkVal;
  UINT32 Index;

  ClkVal = MmioAnd32 (MV_AP_SAR_BASE,
                      SAR_CLOCK_FREQ_MODE_MASK >> SAR_CLOCK_FREQ_MODE_OFFSET);

  for (Index = 0; Index < SAR_MAX_OPTIONS; Index++) {
    if (PllFreqTbl[Index][3] == ClkVal) {
      break;
    }
  }

  return PllFreqTbl[Index][CPU_CLOCK_ID] / MHz;
}

UINT32
EFIAPI
MvSARGetDramFreq (
  VOID
  )
{
  UINT32 ClkVal;
  UINT32 Index;

  ClkVal = MmioAnd32 (MV_AP_SAR_BASE,
                      SAR_CLOCK_FREQ_MODE_MASK >> SAR_CLOCK_FREQ_MODE_OFFSET);

  for (Index = 0; Index < SAR_MAX_OPTIONS; Index++) {
    if (PllFreqTbl[Index][3] == ClkVal) {
      break;
    }
  }

  return PllFreqTbl[Index][DDR_CLOCK_ID] / MHz;
}

UINT32
EFIAPI
MvSARGetPcieClkDirection (
  IN UINT32      CpIndex,
  IN UINT32      PcieIndex
  )
{
  UINT32  ClkDir;

  ClkDir = MmioAnd32 (MV_CP_SAR_BASE(CpIndex),
                      PcieClkMask[CpIndex][PcieIndex] >>
                      PcieClkOffset[CpIndex][PcieIndex]);

  return ClkDir;
}
