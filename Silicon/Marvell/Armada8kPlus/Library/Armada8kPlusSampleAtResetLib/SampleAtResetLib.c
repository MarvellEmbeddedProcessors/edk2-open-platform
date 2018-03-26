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

#include <Uefi.h>

#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/SampleAtResetLib.h>

#include "SampleAtResetLib.h"

UINT32
EFIAPI
Ap810SampleAtResetValueGet (
 IN FREQUENCY_OPTION FrequencyOption
 )
{
  CONST PLL_FREQUENCY_DESCRIPTION *PllFrequencies;
  UINT32 ClockFrequencyMode;
  UINT32 EfuseRegValue;
  UINT32 ChipRevision;

  /* Fetch eFuse value and device whether it's H/L */
  EfuseRegValue = (MmioRead32 (EFUSE_FREQ_REG) & EFUSE_FREQ_MASK) >> EFUSE_FREQ_OFFSET;
  if (EfuseRegValue == EFUSE_PLL_FREQUENCY_LOW) {
    PllFrequencies = PllFrequencyTableLow;
  } else {
    PllFrequencies = PllFrequencyTableHigh;
  }

  ChipRevision = (MmioRead32 (MVEBU_CSS_GWD_CTRL_IIDR2_REG) >>
                  GWD_IIDR2_REV_ID_OFFSET) &
                  GWD_IIDR2_REV_ID_MASK;

  /* Read from scratch-pad instead of sampled-at-reset in A0 revision */
  if (ChipRevision == AP810_A0_REVISION_ID) {
    ClockFrequencyMode = MmioRead32 (SCRATCH_PAD_FREQ_REG);
  } else {
    ClockFrequencyMode = (MmioRead32 (AP810_SAR_BASE) &
                          SAR_CLOCK_FREQ_MODE_MASK) >>
                          SAR_CLOCK_FREQ_MODE_OFFSET;
  }

  switch (FrequencyOption) {
  case (SAR_AP810_CPU_FREQ):
    return PllFrequencies[ClockFrequencyMode].CpuFrequency;
  case (SAR_AP810_DDR_FREQ):
    return PllFrequencies[ClockFrequencyMode].DdrFrequency;
  case (SAR_AP810_FABRIC_FREQ):
    return PllFrequencies[ClockFrequencyMode].RingFrequency;
  default:
    DEBUG ((DEBUG_ERROR,
      "%a: Unsupported SAR option %d\n",
      __FUNCTION__,
      FrequencyOption));
    return 0;
  }
}

UINT32
EFIAPI
SampleAtResetGetCpuFrequency (
  VOID
  )
{
  UINT32 ClockValue;

  ClockValue = Ap810SampleAtResetValueGet (SAR_AP810_CPU_FREQ);

  return ClockValue;
}

UINT32
EFIAPI
SampleAtResetGetDramFrequency (
  VOID
  )
{
  UINT32 ClockValue;

  ClockValue = Ap810SampleAtResetValueGet (SAR_AP810_DDR_FREQ);

  return ClockValue;
}
