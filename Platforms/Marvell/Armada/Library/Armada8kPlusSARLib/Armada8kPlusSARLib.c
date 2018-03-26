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

/* SAR AP810 register */
#define MV_AP_SAR_BASE                  0xEC6F8200
#define SAR_CLOCK_FREQ_MODE_OFFSET      0
#define SAR_CLOCK_FREQ_MODE_MASK        (0x7 << SAR_CLOCK_FREQ_MODE_OFFSET)

/* SAR CP110 register */
#define MV_CP0_SAR_BASE                 0x8100400200
#define MV_CP1_SAR_BASE                 0x8800400200
#define CP0_PCIE0_CLK_MASK              0x4
#define CP0_PCIE1_CLK_MASK              0x8
#define CP1_PCIE0_CLK_MASK              0x1
#define CP1_PCIE1_CLK_MASK              0x2
#define CP0_PCIE0_CLK_OFFSET            2
#define CP0_PCIE1_CLK_OFFSET            3
#define CP1_PCIE0_CLK_OFFSET            0
#define CP1_PCIE1_CLK_OFFSET            1

/* efuse register */
#define EFUSE_FREQ_REG                  0xEC6F4410
#define EFUSE_FREQ_OFFSET               24
#define EFUSE_FREQ_MASK                 (0x1 << EFUSE_FREQ_OFFSET)

/* Temporay substitute for sampled-at-reset
 * register due to bug in AP810 A0
 * the frequency option is sampled to the 3 LSB in the
 * scratch-pad register.
 */
#define SCRATCH_PAD_FREQ_REG            0xEC6F43E4

/* AP810 revision ID */
#define MVEBU_CSS_GWD_CTRL_IIDR2_REG    0xEC6F0240
#define GWD_IIDR2_REV_ID_OFFSET         16
#define GWD_IIDR2_REV_ID_MASK           0xF

#define CPU_CLOCK_ID                    0
#define DDR_CLOCK_ID                    1
#define RING_CLOCK_ID                   2

#define SAR_SUPPORTED_FREQ_NUM          8
#define SAR_FREQ_VAL                    4
#define SAR_SUPPORTED_TABLES            2

#define MHz                             1000000
#define GHz                             1000000000

typedef enum {
 SAR_AP810_CPU_FREQ = 0,
 SAR_AP810_DDR_FREQ,
 SAR_AP810_AP_FABRIC_FREQ,
} SarOptions;

typedef enum {
 HP_CPU_1600_DDR_1600_RCLK_1200_IO_800_PIDI_1000 = 0x0,
 HP_CPU_2000_DDR_2400_RCLK_1200_IO_800_PIDI_1000 = 0x1,
 HP_CPU_2000_DDR_2400_RCLK_1400_IO_1000_PIDI_1000 = 0x2,
 HP_CPU_2200_DDR_2400_RCLK_1400_IO_1000_PIDI_1000 = 0x3,
 HP_CPU_2200_DDR_2667_RCLK_1400_IO_1000_PIDI_1000 = 0x4,
 HP_CPU_2500_DDR_2400_RCLK_1400_IO_1000_PIDI_1000 = 0x5,
 HP_CPU_2500_DDR_2933_RCLK_1400_IO_1000_PIDI_1000 = 0x6,
 HP_CPU_2700_DDR_3200_RCLK_1400_IO_1000_PIDI_1000 = 0x7,
} ClockingOptionsHP;

typedef enum {
 LP_CPU_1200_DDR_1600_RCLK_800_IO_800_PIDI_1000 = 0x0,
 LP_CPU_1800_DDR_2400_RCLK_1000_IO_800_PIDI_1000 = 0x1,
 LP_CPU_1800_DDR_2400_RCLK_1100_IO_800_PIDI_1000 = 0x2,
 LP_CPU_1800_DDR_2400_RCLK_1200_IO_800_PIDI_1000 = 0x3,
 LP_CPU_1800_DDR_2400_RCLK_1400_IO_800_PIDI_1000 = 0x4,
 LP_CPU_2000_DDR_2400_RCLK_1100_IO_800_PIDI_1000 = 0x5,
 LP_CPU_2000_DDR_2400_RCLK_1200_IO_800_PIDI_1000 = 0x6,
 LP_CPU_2000_DDR_2400_RCLK_1300_IO_800_PIDI_1000 = 0x7,
} ClockingOptionsLP;

static const UINT32 PllFreqTbl[SAR_SUPPORTED_TABLES]
                              [SAR_SUPPORTED_FREQ_NUM]
                              [SAR_FREQ_VAL] = {
 {
  /* CPU */   /* DDR */   /* Ring */
  {1.6 * GHz, 1.6  * GHz, 1.2  * GHz,
   HP_CPU_1600_DDR_1600_RCLK_1200_IO_800_PIDI_1000},
  {2.0 * GHz, 2.4 * GHz, 1.2 * GHz,
   HP_CPU_2000_DDR_2400_RCLK_1200_IO_800_PIDI_1000},
  {2.0 * GHz, 2.4  * GHz, 1.4  * GHz,
   HP_CPU_2000_DDR_2400_RCLK_1400_IO_1000_PIDI_1000},
  {2.2 * GHz, 2.4 * GHz, 1.4 * GHz,
   HP_CPU_2200_DDR_2400_RCLK_1400_IO_1000_PIDI_1000},
  {2.2 * GHz, 2.667 * GHz, 1.4 * GHz,
   HP_CPU_2200_DDR_2667_RCLK_1400_IO_1000_PIDI_1000},
  {2.5 * GHz, 2.4  * GHz, 1.4  * GHz,
   HP_CPU_2500_DDR_2400_RCLK_1400_IO_1000_PIDI_1000},
  {2.5 * GHz, 2.93  * GHz, 1.4  * GHz,
   HP_CPU_2500_DDR_2933_RCLK_1400_IO_1000_PIDI_1000},
  {2.7 * GHz, 3.2  * GHz, 1.4  * GHz,
   HP_CPU_2700_DDR_3200_RCLK_1400_IO_1000_PIDI_1000},
 },
 {
  {1.2 * GHz, 1.6  * GHz, 0.8 * GHz,
   LP_CPU_1200_DDR_1600_RCLK_800_IO_800_PIDI_1000},
  {1.8 * GHz, 2.4 * GHz, 1.0 * GHz,
   LP_CPU_1800_DDR_2400_RCLK_1000_IO_800_PIDI_1000},
  {1.8 * GHz, 2.4  * GHz, 1.1  * GHz,
   LP_CPU_1800_DDR_2400_RCLK_1100_IO_800_PIDI_1000},
  {1.8 * GHz, 2.4 * GHz, 1.2 * GHz,
   LP_CPU_1800_DDR_2400_RCLK_1200_IO_800_PIDI_1000},
  {1.8 * GHz, 2.4 * GHz, 1.4 * GHz,
   LP_CPU_1800_DDR_2400_RCLK_1400_IO_800_PIDI_1000},
  {2.0 * GHz, 2.4  * GHz, 1.1  * GHz,
   LP_CPU_2000_DDR_2400_RCLK_1100_IO_800_PIDI_1000},
  {2.0 * GHz, 2.4  * GHz, 1.2  * GHz,
   LP_CPU_2000_DDR_2400_RCLK_1200_IO_800_PIDI_1000},
  {2.0 * GHz, 2.4  * GHz, 1.3  * GHz,
   LP_CPU_2000_DDR_2400_RCLK_1300_IO_800_PIDI_1000},
 },
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
Ap810SarValueGet (
 IN SarOptions SarOpt
 )
{
  UINT32 ClockType;
  UINT32 ClockFreqMode;
  UINT32 EfuseReg;
  UINT32 ChipRev;

  switch (SarOpt) {
  case(SAR_AP810_CPU_FREQ):
    ClockType = CPU_CLOCK_ID;
    break;
  case(SAR_AP810_DDR_FREQ):
    ClockType = DDR_CLOCK_ID;
    break;
  case(SAR_AP810_AP_FABRIC_FREQ):
    ClockType = RING_CLOCK_ID;
    break;
  default:
    DEBUG((DEBUG_ERROR, "AP810-SAR: Unsupported SAR option %d.\n", SarOpt));
    return 0;
  }

  /* fetch eFuse value and device whether it's H/L */
  EfuseReg = MmioRead32(EFUSE_FREQ_REG);
  EfuseReg &= EFUSE_FREQ_MASK;
  EfuseReg = EfuseReg >> EFUSE_FREQ_OFFSET;

  ChipRev = (MmioRead32(MVEBU_CSS_GWD_CTRL_IIDR2_REG) >>
             GWD_IIDR2_REV_ID_OFFSET) &
             GWD_IIDR2_REV_ID_MASK;

  /* Read from scratch-pad instead of sampled-at-reset in A0 */
  if (ChipRev) {
    ClockFreqMode = (MmioRead32(MV_AP_SAR_BASE) &
                     SAR_CLOCK_FREQ_MODE_MASK) >>
                     SAR_CLOCK_FREQ_MODE_OFFSET;
  } else {
    ClockFreqMode = MmioRead32(SCRATCH_PAD_FREQ_REG);
  }

  if (ClockFreqMode < 0 ||
      (ClockFreqMode > (SAR_SUPPORTED_FREQ_NUM - 1))) {
      DEBUG((DEBUG_ERROR, "sar regs: unsupported clock freq mode %d\n",
            ClockFreqMode));
      return 0;
}

  return PllFreqTbl[EfuseReg][ClockFreqMode][ClockType] / MHz;
}

UINT32
EFIAPI
MvSARGetCpuFreq (
  VOID
  )
{
  UINT32 ClkVal;

  ClkVal = Ap810SarValueGet (SAR_AP810_CPU_FREQ);

  return ClkVal;
}

UINT32
EFIAPI
MvSARGetDramFreq (
  VOID
  )
{
  UINT32 ClkVal;

  ClkVal = Ap810SarValueGet (SAR_AP810_DDR_FREQ);

  return ClkVal;
}

UINT32
EFIAPI
MvSARGetPcieClkDirection (
  IN UINT32      CpIndex,
  IN UINT32      PcieIndex
  )
{
  UINT32  ClkDir;
  UINTN   CPSarBase;

  if (CpIndex) {
    CPSarBase = MV_CP1_SAR_BASE;
  } else {
    CPSarBase = MV_CP0_SAR_BASE;
  }
  ClkDir = MmioAnd32 (CPSarBase,
                      PcieClkMask[CpIndex][PcieIndex] >>
                      PcieClkOffset[CpIndex][PcieIndex]);

  return ClkDir;
}
