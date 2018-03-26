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

Glossary - abbreviations used in Marvell SampleAtReset library implementation:
AP - Application Processor hardware block (Armada 8kPlus incorporates AP810)
CP - South Bridge hardware blocks (Armada 8kPlus incorporates CP110)
SAR - Sample At Reset

*******************************************************************************/

/* SAR AP810 register */
#define AP810_SAR_BASE                  0xEC6F8200
#define SAR_CLOCK_FREQ_MODE_OFFSET      0
#define SAR_CLOCK_FREQ_MODE_MASK        (0x7 << SAR_CLOCK_FREQ_MODE_OFFSET)

/* eFuse register */
#define EFUSE_FREQ_REG                  0xEC6F4410
#define EFUSE_FREQ_OFFSET               24
#define EFUSE_FREQ_MASK                 (0x1 << EFUSE_FREQ_OFFSET)
#define EFUSE_PLL_FREQUENCY_LOW         0x0

#define AP810_A0_REVISION_ID            0x0

/*
 * Temporary substitute for sampled-at-reset
 * register due to bug in AP810 A0
 * the frequency option is sampled to the 3 LSB in the
 * scratch-pad register.
 */
#define SCRATCH_PAD_FREQ_REG            0xEC6F43E4

/* AP810 revision ID */
#define MVEBU_CSS_GWD_CTRL_IIDR2_REG    0xEC6F0240
#define GWD_IIDR2_REV_ID_OFFSET         16
#define GWD_IIDR2_REV_ID_MASK           0xF

#define SAR_MAX_OPTIONS                 8

typedef enum {
  SAR_AP810_CPU_FREQ,
  SAR_AP810_DDR_FREQ,
  SAR_AP810_FABRIC_FREQ,
} FREQUENCY_OPTION;

typedef enum {
  HP_CPU_1600_DDR_1600_RCLK_1200_IO_800_PIDI_1000 = 0x0,
  HP_CPU_2000_DDR_2400_RCLK_1200_IO_800_PIDI_1000 = 0x1,
  HP_CPU_2000_DDR_2400_RCLK_1400_IO_1000_PIDI_1000 = 0x2,
  HP_CPU_2200_DDR_2400_RCLK_1400_IO_1000_PIDI_1000 = 0x3,
  HP_CPU_2200_DDR_2667_RCLK_1400_IO_1000_PIDI_1000 = 0x4,
  HP_CPU_2500_DDR_2400_RCLK_1400_IO_1000_PIDI_1000 = 0x5,
  HP_CPU_2500_DDR_2933_RCLK_1400_IO_1000_PIDI_1000 = 0x6,
  HP_CPU_2700_DDR_3200_RCLK_1400_IO_1000_PIDI_1000 = 0x7,
  LP_CPU_1200_DDR_1600_RCLK_800_IO_800_PIDI_1000 = 0x0,
  LP_CPU_1800_DDR_2400_RCLK_1000_IO_800_PIDI_1000 = 0x1,
  LP_CPU_1800_DDR_2400_RCLK_1100_IO_800_PIDI_1000 = 0x2,
  LP_CPU_1800_DDR_2400_RCLK_1200_IO_800_PIDI_1000 = 0x3,
  LP_CPU_1800_DDR_2400_RCLK_1400_IO_800_PIDI_1000 = 0x4,
  LP_CPU_2000_DDR_2400_RCLK_1100_IO_800_PIDI_1000 = 0x5,
  LP_CPU_2000_DDR_2400_RCLK_1200_IO_800_PIDI_1000 = 0x6,
  LP_CPU_2000_DDR_2400_RCLK_1300_IO_800_PIDI_1000 = 0x7,
} CLOCKING_OPTIONS;

typedef struct {
  UINT32 CpuFrequency;
  UINT32 DdrFrequency;
  UINT32 RingFrequency;
  CLOCKING_OPTIONS ClockingOption;
} PLL_FREQUENCY_DESCRIPTION;

STATIC CONST PLL_FREQUENCY_DESCRIPTION PllFrequencyTableHigh[SAR_MAX_OPTIONS] = {
  /* CPU   DDR   Ring  [MHz] */
  {1600, 1600, 1200, HP_CPU_1600_DDR_1600_RCLK_1200_IO_800_PIDI_1000},
  {2000, 2400, 1200, HP_CPU_2000_DDR_2400_RCLK_1200_IO_800_PIDI_1000},
  {2000, 2400, 1400, HP_CPU_2000_DDR_2400_RCLK_1400_IO_1000_PIDI_1000},
  {2200, 2400, 1400, HP_CPU_2200_DDR_2400_RCLK_1400_IO_1000_PIDI_1000},
  {2200, 2667, 1400, HP_CPU_2200_DDR_2667_RCLK_1400_IO_1000_PIDI_1000},
  {2500, 2400, 1400, HP_CPU_2500_DDR_2400_RCLK_1400_IO_1000_PIDI_1000},
  {2500, 2930, 1400, HP_CPU_2500_DDR_2933_RCLK_1400_IO_1000_PIDI_1000},
  {2700, 3200, 1400, HP_CPU_2700_DDR_3200_RCLK_1400_IO_1000_PIDI_1000}
};

STATIC CONST PLL_FREQUENCY_DESCRIPTION PllFrequencyTableLow[SAR_MAX_OPTIONS] = {
  /* CPU   DDR   Ring  [MHz] */
  {1200, 1600, 800,  LP_CPU_1200_DDR_1600_RCLK_800_IO_800_PIDI_1000},
  {1800, 2400, 1000, LP_CPU_1800_DDR_2400_RCLK_1000_IO_800_PIDI_1000},
  {1800, 2400, 1100, LP_CPU_1800_DDR_2400_RCLK_1100_IO_800_PIDI_1000},
  {1800, 2400, 1200, LP_CPU_1800_DDR_2400_RCLK_1200_IO_800_PIDI_1000},
  {1800, 2400, 1400, LP_CPU_1800_DDR_2400_RCLK_1400_IO_800_PIDI_1000},
  {2000, 2400, 1100, LP_CPU_2000_DDR_2400_RCLK_1100_IO_800_PIDI_1000},
  {2000, 2400, 1200, LP_CPU_2000_DDR_2400_RCLK_1200_IO_800_PIDI_1000},
  {2000, 2400, 1300, LP_CPU_2000_DDR_2400_RCLK_1300_IO_800_PIDI_1000}
};
