/*******************************************************************************
Copyright (C) 2016 Marvell International Ltd.

Marvell BSD License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File under the following licensing terms.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
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

#include "XenonSdhci.h"

VOID
XenonSetMpp (
  VOID
  )
{
  UINT32 Reg;

  // Set eMMC/SD PHY output instead of MPPs
  Reg = MmioRead32 (MVEBU_IP_CONFIG_REG);
  Reg &= ~(1 << 0);
  MmioWrite32 (MVEBU_IP_CONFIG_REG, Reg);
}

VOID
XenonReadVersion (
  IN  EFI_PCI_IO_PROTOCOL   *PciIo,
  OUT UINT32 *ControllerVersion
  )
{

  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX,SD_MMC_HC_CTRL_VER, TRUE, 2, ControllerVersion);
}

VOID
XenonSetFifo (
  IN     EFI_PCI_IO_PROTOCOL   *PciIo
  )
{
  UINTN Data = 0x315;

  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SDHC_SLOT_FIFO_CTRL, FALSE, 4, &Data);
}

// Auto Clock Gating
VOID
XenonSetAcg (
  IN     EFI_PCI_IO_PROTOCOL   *PciIo,
  IN BOOLEAN Enable
  )
{
  UINT32 Var;

  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SDHC_SYS_OP_CTRL, TRUE, 4, &Var);

  if (Enable) {
    Var &= ~AUTO_CLKGATE_DISABLE_MASK;
  } else {
    Var |= AUTO_CLKGATE_DISABLE_MASK;
  }

  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SDHC_SYS_OP_CTRL, FALSE, 4, &Var);
}

VOID
XenonSetSlot (
  IN EFI_PCI_IO_PROTOCOL *PciIo,
  IN UINT8 Slot,
  IN BOOLEAN Enable
  )
{
  UINT32 Var;

  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SDHC_SYS_OP_CTRL, TRUE, 4, &Var);
  if (Enable)
    Var |= ((0x1 << Slot) << SLOT_ENABLE_SHIFT);
  else
    Var &= ~((0x1 << Slot) << SLOT_ENABLE_SHIFT);

  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SDHC_SYS_OP_CTRL, FALSE, 4, &Var);
}

STATIC
VOID
XenonSetSdio (
  IN EFI_PCI_IO_PROTOCOL *PciIo,
  IN UINTN Voltage
  )
{

  return;
}

VOID
XenonSetPower (
  IN EFI_PCI_IO_PROTOCOL *PciIo,
  IN UINT32 Vcc,
  IN UINT32 Vccq,
  IN UINT8 Mode
  )
{
  UINT8 Pwr = 0;
  UINT32 Ctrl = 0;

  // Set VCC
  switch (Vcc) {
  case MMC_VDD_165_195:
    Pwr = SDHCI_POWER_180;
    if (Mode == XENON_MMC_MODE_SD_SDIO)
      XenonSetSdio (PciIo, 0);
    break;
  case MMC_VDD_29_30:
  case MMC_VDD_30_31:
    Pwr = SDHCI_POWER_300;
    if (Mode == XENON_MMC_MODE_SD_SDIO)
      XenonSetSdio (PciIo, 1);
    break;
  case MMC_VDD_32_33:
  case MMC_VDD_33_34:
    Pwr = SDHCI_POWER_330;
    if (Mode == XENON_MMC_MODE_SD_SDIO)
      XenonSetSdio (PciIo, 1);
    break;
  default:
    DEBUG((DEBUG_ERROR, "Does not support power mode(0x%X)\n", Vcc));
    break;
  }

  if (Pwr == 0) {
    SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SD_MMC_HC_POWER_CTRL, FALSE, 1, &Pwr);
    return;
  }

  Pwr |= SDHCI_POWER_ON;

  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX,SD_MMC_HC_POWER_CTRL, FALSE, 1, &Pwr);

  // Set VCCQ
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SDHC_SLOT_eMMC_CTRL, TRUE, 4, &Ctrl);
  Ctrl |= Vccq;
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SDHC_SLOT_eMMC_CTRL, FALSE, 4, &Ctrl);
}

UINTN
XenonSetClk (
  IN EFI_PCI_IO_PROTOCOL   *PciIo,
  IN SD_MMC_HC_PRIVATE_DATA *Private,
  IN UINT32 Clock
  )
{
  UINT32 Div;
  UINT32 Clk;
  UINT32 Timeout;
  UINT16 Value = 0;

  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SD_MMC_HC_CLOCK_CTRL, FALSE, 2, &Value);

  if (Clock == 0)
    return 0;

  if (Private->ControllerVersion >= SDHCI_SPEC_300) {
    // Version 3.00 Divisors must be a multiple of 2
    if (XENON_MMC_MAX_CLK <= Clock) {
      Div = 1;
    } else {
      for (Div = 2; Div < SDHCI_MAX_DIV_SPEC_300; Div += 2) {
        if ((XENON_MMC_MAX_CLK / Div) <= Clock)
          break;
      }
    }
  } else {
    // Version 2.00 Divisors must be a power of 2
    for (Div = 1; Div < SDHCI_MAX_DIV_SPEC_200; Div *= 2) {
      if ((XENON_MMC_MAX_CLK / Div) <= Clock)
        break;
    }
  }
  Div >>= 1;

  Clk = (Div & SDHCI_DIV_MASK) << SDHCI_DIVIDER_SHIFT;
  Clk |= ((Div & SDHCI_DIV_HI_MASK) >> SDHCI_DIV_MASK_LEN)
    << SDHCI_DIVIDER_HI_SHIFT;
  Clk |= SDHCI_CLOCK_INT_EN;

  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SD_MMC_HC_CLOCK_CTRL, FALSE, 2, &Clk);

  // Wait max 20 ms
  Timeout = 200;

  do {
    SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SD_MMC_HC_CLOCK_CTRL, TRUE, 2, &Clk);
    if (Timeout == 0) {
      DEBUG((DEBUG_ERROR, "SD/MMC: Internal Clock never stabilised\n"));
      return -1;
    }
    Timeout--;
    gBS->Stall (1);
  } while (!(Clk & SDHCI_CLOCK_INT_STABLE));

  Clk |= SDHCI_CLOCK_CARD_EN;
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SD_MMC_HC_CLOCK_CTRL, FALSE, 2, &Clk);

  return 0;
}

VOID
XenonPhyInit (
  IN EFI_PCI_IO_PROTOCOL   *PciIo
  )
{
  UINT32 Var, Wait, Time;
  UINT32 Clock = XENON_MMC_MAX_CLK;
  UINT16 ClkCtrl;

  // Need to disable the clock to set EMMC_PHY_TIMING_ADJUST register
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SD_MMC_HC_CLOCK_CTRL, TRUE, 2, &ClkCtrl);
  ClkCtrl &= ~(SDHCI_CLOCK_CARD_EN | SDHCI_CLOCK_INT_EN);
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SD_MMC_HC_CLOCK_CTRL, FALSE, 2, &ClkCtrl);

  // Enable QSP PHASE SELECT
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, EMMC_PHY_TIMING_ADJUST, TRUE, 4, &Var);
  Var |= SAMPL_INV_QSP_PHASE_SELECT;
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, EMMC_PHY_TIMING_ADJUST, FALSE, 4, &Var);

  // Enable internal clock
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SD_MMC_HC_CLOCK_CTRL, TRUE, 2, &ClkCtrl);
  ClkCtrl |= SDHCI_CLOCK_INT_EN;
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SD_MMC_HC_CLOCK_CTRL, FALSE, 2, &ClkCtrl);

  //
  // Poll for host MMC PHY clock init to be stable
  // Wait up to 10ms
  //
  Time = 100;
  while (Time--) {
    SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, EMMC_PHY_TIMING_ADJUST, TRUE, 4, &Var);
    if (Var & SDHCI_CLOCK_INT_STABLE)
      break;

    gBS->Stall (1);
  }
  if (Time <= 0) {
    DEBUG((DEBUG_ERROR, "Failed to enable MMC internal clock in Time\n"));
    return;
  }

  // Enable bus clock
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SD_MMC_HC_CLOCK_CTRL, TRUE, 2, &ClkCtrl);
  ClkCtrl |= SDHCI_CLOCK_CARD_EN;
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SD_MMC_HC_CLOCK_CTRL, FALSE, 2, &ClkCtrl);

  // Delay 200us to Wait for the completion of bus clock
  gBS->Stall (200);

  // Init PHY
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, EMMC_PHY_TIMING_ADJUST, TRUE, 4, &Var);
  Var |= PHY_INITIALIZAION;
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, EMMC_PHY_TIMING_ADJUST, FALSE, 4, &Var);

  // Add duration of FC_SYNC_RST
  Wait = ((Var >> FC_SYNC_RST_DURATION_SHIFT) & FC_SYNC_RST_DURATION_MASK);
  // Add interval between FC_SYNC_EN and FC_SYNC_RST
  Wait += ((Var >> FC_SYNC_RST_EN_DURATION_SHIFT) & FC_SYNC_RST_EN_DURATION_MASK);
  // Add duration of asserting FC_SYNC_EN
  Wait += ((Var >> FC_SYNC_EN_DURATION_SHIFT) & FC_SYNC_EN_DURATION_MASK);
  // Add duration of Waiting for PHY
  Wait += ((Var >> WAIT_CYCLE_BEFORE_USING_SHIFT) & WAIT_CYCLE_BEFORE_USING_MASK);
  //
  // According to Moyang, 4 addtional bus clock and 4 AXI bus clock are required
  // left shift 20 bits
  Wait += 8;
  Wait <<= 20;

  if (Clock == 0)
    // Use the possibly slowest bus frequency value
    Clock = 100000;
  // Get the Wait Time in unit of ms
  Wait = Wait / Clock;
  Wait++;

  //
  // Poll for host eMMC PHY init to complete
  // Wait up to 10ms
  //
  Time = 100;
  while (Time--) {
    Var = SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, EMMC_PHY_TIMING_ADJUST, TRUE, 4, &Var);
    Var &= PHY_INITIALIZAION;
    if (!Var)
      break;

    // Wait for host eMMC PHY init to complete
    gBS->Stall (1);
  }
  if (Time <= 0) {
    DEBUG((DEBUG_ERROR, "Sd/Mmc: Failed to init MMC PHY in Time\n"));
    return;
  }

  return;
}

VOID
XenonSetPhy (
  IN EFI_PCI_IO_PROTOCOL   *PciIo,
  UINT8 Timing
  )
{
  UINT32 Var = 0;

  // Setup pad, set bit[30], bit[28] and bits[26:24]
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, EMMC_PHY_PAD_CONTROL, TRUE, 4, &Var);
  Var |= (AUTO_RECEN_CTRL | OEN_QSN | FC_QSP_RECEN | FC_CMD_RECEN | FC_DQ_RECEN);
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, EMMC_PHY_PAD_CONTROL, FALSE, 4, &Var);

  //
  // If Timing belongs to high speed, set bit[17] of
  // EMMC_PHY_TIMING_ADJUST register
  //
  if ((Timing == MMC_TIMING_MMC_HS400) ||
      (Timing == MMC_TIMING_MMC_HS200) ||
      (Timing == MMC_TIMING_UHS_SDR50) ||
      (Timing == MMC_TIMING_UHS_SDR104) ||
      (Timing == MMC_TIMING_UHS_DDR50) ||
      (Timing == MMC_TIMING_UHS_SDR25)) {
    SdMmcHcRwMmio(PciIo, SD_BAR_INDEX, EMMC_PHY_TIMING_ADJUST, TRUE, 4, &Var);

    Var |= OUTPUT_QSN_PHASE_SELECT | (1 << 19);
    SdMmcHcRwMmio(PciIo, SD_BAR_INDEX, EMMC_PHY_TIMING_ADJUST, FALSE, 4, &Var);
  }

  SdMmcHcRwMmio(PciIo, SD_BAR_INDEX, EMMC_PHY_FUNC_CONTROL, TRUE, 4, &Var);
  Var |= (DQ_DDR_MODE_MASK << DQ_DDR_MODE_SHIFT) | CMD_DDR_MODE;
  SdMmcHcRwMmio(PciIo, SD_BAR_INDEX, EMMC_PHY_FUNC_CONTROL, FALSE, 4, &Var);

  if (Timing == MMC_TIMING_MMC_HS400) {
    SdMmcHcRwMmio(PciIo, SD_BAR_INDEX, EMMC_PHY_FUNC_CONTROL, TRUE, 4, &Var);
    Var &= ~DQ_ASYNC_MODE;
    SdMmcHcRwMmio(PciIo, SD_BAR_INDEX, EMMC_PHY_FUNC_CONTROL, FALSE, 4, &Var);

    Var = LOGIC_TIMING_VALUE;
    SdMmcHcRwMmio(PciIo, SD_BAR_INDEX, EMMC_LOGIC_TIMING_ADJUST, FALSE, 4, &Var);
  }

  XenonPhyInit (PciIo);
}

VOID
XenonConfigureInterrupts (
  IN EFI_PCI_IO_PROTOCOL *PciIo
  )
{
  UINT32 Var;

  // Clear interrupt status
  Var = 0xFFFFFFFF;
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SD_MMC_HC_NOR_INT_STS, FALSE, 4, &Var);
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SD_MMC_HC_NOR_INT_STS, FALSE, 4, &Var);

  // Enable only interrupts served by the SD controller
  Var = 0xFFFFFEBF;
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SD_MMC_HC_NOR_INT_STS_EN, FALSE, 4, &Var);

  // Mask all sdhci interrupt sources
  Var = 0xFFFFFEFF;
  SdMmcHcRwMmio (PciIo, SD_BAR_INDEX, SD_MMC_HC_NOR_INT_SIG_EN, FALSE, 4, &Var);
}

// Enable Parallel Transfer Mode
VOID
XenonSetParallelTransfer (
  IN EFI_PCI_IO_PROTOCOL *PciIo,
  IN UINT8 Slot,
  IN BOOLEAN Enable
  )
{
  UINT32 Var;

  SdMmcHcRwMmio(PciIo, SD_BAR_INDEX, SDHC_SYS_EXT_OP_CTRL, TRUE, 4, &Var);
  if (Enable)
    Var |= (0x1 << Slot);
  else
    Var &= ~(0x1 << Slot);
  SdMmcHcRwMmio(PciIo, SD_BAR_INDEX, SDHC_SYS_EXT_OP_CTRL, FALSE, 4, &Var);
}

VOID
XenonSetTuning (
  IN EFI_PCI_IO_PROTOCOL   *PciIo,
  IN UINT8 Slot,
  IN BOOLEAN Enable
  )
{
  UINT32 Var;

  // Set the Re-Tuning Request functionality
  SdMmcHcRwMmio(PciIo, SD_BAR_INDEX, SDHC_SLOT_RETUNING_REQ_CTRL, TRUE, 4, &Var);
  if (Enable)
    Var |= RETUNING_COMPATIBLE;
  else
    Var &= ~RETUNING_COMPATIBLE;
  SdMmcHcRwMmio(PciIo, SD_BAR_INDEX, SDHC_SLOT_RETUNING_REQ_CTRL, FALSE, 4, &Var);

  // Set the Re-tuning Event Signal Enable
  SdMmcHcRwMmio(PciIo, SD_BAR_INDEX, SDHCI_SIGNAL_ENABLE, TRUE, 4, &Var);
  if (Enable)
    Var |= SDHCI_RETUNE_EVT_INTSIG;
  else
    Var &= ~SDHCI_RETUNE_EVT_INTSIG;
  SdMmcHcRwMmio(PciIo, SD_BAR_INDEX, SDHCI_SIGNAL_ENABLE, FALSE, 4, &Var);
}

VOID
XenonReset (
  IN SD_MMC_HC_PRIVATE_DATA *Private,
  IN UINT8 Slot,
  IN UINT8 Mask
  )
{
  UINT32 Timeout = 1000;
  UINT8 SwReset;

  SwReset = Mask;

  SdMmcHcRwMmio (Private->PciIo, Slot, SD_MMC_HC_SW_RST, FALSE,
    sizeof (SwReset), &SwReset);

  SdMmcHcRwMmio (Private->PciIo, Slot, SD_MMC_HC_SW_RST, TRUE,
    sizeof (SwReset), &SwReset);
  while (SwReset & Mask) {
    if (Timeout == 0) {
      DEBUG((DEBUG_ERROR, "Reset never completed\n"));
      return;
    }
    Timeout--;
    gBS-> Stall (100);
    SdMmcHcRwMmio (Private->PciIo, Slot, SD_MMC_HC_SW_RST, TRUE,
      sizeof (SwReset), &SwReset);
  }
}

VOID
XenonTransferPio (
  IN SD_MMC_HC_PRIVATE_DATA *Private,
  IN UINT8 Slot,
  IN OUT VOID *Buffer,
  IN UINT16 BlockSize,
  IN BOOLEAN Read
  )
{
  UINTN i;
  UINT8 *Offs;

  //
  // SD stack's intrinsic functions cannot perform properly reading/writing from
  // buffer register, that is way MmioRead/MmioWrite are used. It is temporary
  // solution.
  //
  for (i = 0; i < BlockSize; i += 4) {
    Offs = Buffer + i;
    if (Read) {
      *(UINT32 *)Offs = MmioRead32 (0xf06e0020); // SDHCI_BUFFER
    } else {
      MmioWrite32 (0xf06e0020, *(UINT32 *)Offs); // SDHCI_BUFFER
    }
  }
}

EFI_STATUS
XenonTransferData (
  IN SD_MMC_HC_PRIVATE_DATA *Private,
  IN UINT8 Slot,
  IN OUT VOID *Buffer,
  IN UINT32 DataLen,
  IN UINT16 BlockSize,
  IN UINT16 Blocks,
  IN BOOLEAN Read
  )
{
  UINT32 IntStatus, PresentState, Rdy, Mask, Timeout, Block = 0;

  if (Buffer == NULL) {
    return EFI_DEVICE_ERROR;
  }

  Timeout = 100000;
  Rdy = 0x10 | 0x20; // SDHCI_INT_SPACE_AVAIL | SDHCI_INT_DATA_AVAIL
  Mask = 0x800 | 0x400; // SDHCI_DATA_AVAILABLE | SDHCI_SPACE_AVAILABLE

  do {
    SdMmcHcRwMmio (Private->PciIo, Slot, SD_MMC_HC_NOR_INT_STS, TRUE,
      sizeof (IntStatus), &IntStatus);
    if (IntStatus & BIT15) { // SDHCI_INT_ERROR
      DEBUG((DEBUG_INFO, "Error detected in status %0x\n", IntStatus));
      return EFI_DEVICE_ERROR;
    }

    if (IntStatus & Rdy) {
      SdMmcHcRwMmio (Private->PciIo, Slot, SD_MMC_HC_PRESENT_STATE, TRUE,
	sizeof (PresentState), &PresentState);
      if (!(PresentState & Mask))
	continue;
      SdMmcHcRwMmio (Private->PciIo, Slot, SD_MMC_HC_NOR_INT_STS, FALSE,
	sizeof (Rdy), &Rdy);

      XenonTransferPio (Private, Slot, Buffer, BlockSize, Read);

      Buffer += BlockSize;
      if (++Block >= Blocks)
	break;
    }
    if (Timeout-- > 0) {
      gBS->Stall (10);
    } else {
      DEBUG((DEBUG_INFO, "Transfer data timeout\n"));
      return EFI_TIMEOUT;
    }
  } while (!(IntStatus & BIT1)); // SDHCI_INT_DATA_END
  return EFI_SUCCESS;
}
