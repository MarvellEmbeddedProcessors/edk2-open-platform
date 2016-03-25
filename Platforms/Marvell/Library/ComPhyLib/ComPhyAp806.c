/********************************************************************************
Copyright (C) 2016 Marvell International Ltd.

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

#include "ComPhyLib.h"

#define HPIPE_ADDR(base, lane)       (base + 0x800 * lane)
#define COMPHY_RESET_REG             0x120

#define COMPHY_RESET_SW_OFFSET       14
#define COMPHY_RESET_SW_MASK         (1 << COMPHY_RESET_SW_OFFSET)
#define COMPHY_RESET_CORE_OFFSET     13
#define COMPHY_RESET_CORE_MASK       (1 << COMPHY_RESET_CORE_OFFSET)

#define COMPHY_PCI_MAC_CTRL          0x200

#define COMPHY_PCI_EN_OFFSET         0
#define COMPHY_PCI_EN_MASK           (0x1 << COMPHY_PCI_EN_OFFSET)
#define COMPHY_PCI_AXI_CACHE_OFFSET  8
#define COMPHY_PCI_AXI_CACHE_MASK    (0xF << COMPHY_PCI_AXI_CACHE_OFFSET)
#define COMPHY_PCI_COHERENT          0x7
#define COMPHY_PCI_X1_EN_OFFSET      14
#define COMPHY_PCI_X1_EN_MASK        (0x1 << COMPHY_PCI_X1_EN_OFFSET)

STATIC
VOID
ComPhyPcieReleaseSoftReset (
  IN EFI_PHYSICAL_ADDRESS HpipeAddr
  )
{
  /* Set MAX PLL Calibration */
  RegSet (HpipeAddr + HPIPE_KVCO_CALIB_CTRL_REG,
    0x1 << HPIPE_KVCO_CALIB_CTRL_MAX_PLL_OFFSET,
    HPIPE_KVCO_CALIB_CTRL_MAX_PLL_MASK);
  RegSet (HpipeAddr + HPIPE_LANE_CONFIG0_REG,
    0x1 << HPIPE_LANE_CONFIG0_MAX_PLL_OFFSET,
    HPIPE_LANE_CONFIG0_MAX_PLL_MASK);
  RegSet (HpipeAddr + HPIPE_LANE_CONFIG0_REG,
    0x1 << HPIPE_LANE_CONFIG0_GEN2_PLL_OFFSET,
    HPIPE_LANE_CONFIG0_GEN2_PLL_MASK);

  /* DFE reset sequence */
  RegSet (HpipeAddr + HPIPE_PWR_CTR_REG,
    0x1 << HPIPE_PWR_CTR_RST_DFE_OFFSET, HPIPE_PWR_CTR_RST_DFE_MASK);
  MicroSecondDelay (10);
  RegSet (HpipeAddr + HPIPE_PWR_CTR_REG,
    0x0 << HPIPE_PWR_CTR_RST_DFE_OFFSET, HPIPE_PWR_CTR_RST_DFE_MASK);

  /* SW reset for interrupt logic */
  RegSet (HpipeAddr + HPIPE_PWR_CTR_REG,
    0x1 << HPIPE_PWR_CTR_SFT_RST_OFFSET, HPIPE_PWR_CTR_SFT_RST_MASK);
  MicroSecondDelay (10);
  RegSet (HpipeAddr + HPIPE_PWR_CTR_REG,
    0x0 << HPIPE_PWR_CTR_SFT_RST_OFFSET, HPIPE_PWR_CTR_SFT_RST_MASK);
}

STATIC
EFI_STATUS
ComPhyPciePowerUp (
  IN UINT32 Lane,
  IN UINT32 PcieBy4,
  IN EFI_PHYSICAL_ADDRESS HpipeAddr
  )
{
  UINT32 StartVal, BreakVal, MasterVal;

  /* Enable CLK 500 */
  RegSet (HpipeAddr + HPIPE_MISC_REG, 0x1 << HPIPE_MISC_CLK500_EN_OFFSET,
    HPIPE_MISC_CLK500_EN_MASK);
  /* Clear lane align off */
  if (PcieBy4)
    RegSet (HpipeAddr + HPIPE_LANE_ALIGN_REG,
      0x0 << HPIPE_LANE_ALIGN_OFF_OFFSET, HPIPE_LANE_ALIGN_OFF_MASK);
  /* Reference Frequency Select set 0 (for PCIe 0 = 100Mhz) */
  RegSet (HpipeAddr + HPIPE_PWR_PLL_REG, 0x0 << HPIPE_PWR_PLL_REF_FREQ_OFFSET,
    HPIPE_PWR_PLL_REF_FREQ_MASK);
  /* PHY Mode Select (set PCIe = 0x3) */
  RegSet (HpipeAddr + HPIPE_PWR_PLL_REG, 0x3 << HPIPE_PWR_PLL_PHY_MODE_OFFSET,
    HPIPE_PWR_PLL_PHY_MODE_MASK);
  /* Set PIPE RESET - SW reset for the PIPE */
  RegSet (HpipeAddr + HPIPE_RST_CLK_CTRL_REG,
    0x1 << HPIPE_RST_CLK_CTRL_PIPE_RST_OFFSET,
    HPIPE_RST_CLK_CTRL_PIPE_RST_MASK);
  /* Set PCIe fixed mode to 8 bit @ 250 Mhz */
  RegSet (HpipeAddr + HPIPE_RST_CLK_CTRL_REG,
    0x1 << HPIPE_RST_CLK_CTRL_FIXED_PCLK_OFFSET,
    HPIPE_RST_CLK_CTRL_FIXED_PCLK_MASK);
  /* Set 5Gbps for RX and TX */
  RegSet (HpipeAddr + HPIPE_ISOLATE_MODE_REG,
    0x1 << HPIPE_ISOLATE_MODE_GEN_RX_OFFSET, HPIPE_ISOLATE_MODE_GEN_RX_MASK);
  RegSet (HpipeAddr + HPIPE_ISOLATE_MODE_REG,
    0x1 << HPIPE_ISOLATE_MODE_GEN_TX_OFFSET, HPIPE_ISOLATE_MODE_GEN_TX_MASK);
  /* Set Max PHY generation setting - 5GBps */
  RegSet (HpipeAddr + HPIPE_INTERFACE_REG,
    0x1 << HPIPE_INTERFACE_GEN_MAX_OFFSET, HPIPE_INTERFACE_GEN_MAX_MASK);
  /*
   * Set Lane Break/Start/Master:
   * master - Provide RefClock to MAC
   * start - Start of providing RefClock
   * break - Stop passing the RefClock
   */
  if (PcieBy4) {
    /*
     * If By4 Lane 0 - is master and start PHY
     * Lane 1-2 - pass refclock to next phy
     * Lane 3 - stop passing refclock
     */
    if (Lane == 0) {
      StartVal = 0x1;
      BreakVal = 0x0;
      MasterVal = 0x1;
    } else if (Lane == 3) {
      StartVal = 0x0;
      BreakVal = 0x1;
      MasterVal = 0x0;
    } else {
      StartVal = 0x0;
      BreakVal = 0x0;
      MasterVal = 0x0;
    }
  } else {
    StartVal = 0x1;
    BreakVal = 0x1;
    MasterVal = 0x1;
  }
  RegSet (HpipeAddr + HPIPE_CLK_SRC_HI_REG,
    StartVal << HPIPE_CLK_SRC_HI_LANE_STRT_OFFSET,
    HPIPE_CLK_SRC_HI_LANE_STRT_MASK);
  RegSet (HpipeAddr + HPIPE_CLK_SRC_HI_REG,
    BreakVal << HPIPE_CLK_SRC_HI_LANE_BREAK_OFFSET,
    HPIPE_CLK_SRC_HI_LANE_BREAK_MASK);
  RegSet (HpipeAddr + HPIPE_CLK_SRC_HI_REG,
    MasterVal << HPIPE_CLK_SRC_HI_LANE_MASTER_OFFSET,
    HPIPE_CLK_SRC_HI_LANE_MASTER_MASK);

  /* For PCIe by4 need to reset after configure all 4 Lanes */
  if (PcieBy4) {
    return EFI_SUCCESS;
  }

  ComPhyPcieReleaseSoftReset(HpipeAddr);
  /* Release PIPE RESET - release PHY from reset */
  RegSet (HpipeAddr + HPIPE_RST_CLK_CTRL_REG,
    0x0 << HPIPE_RST_CLK_CTRL_PIPE_RST_OFFSET,
    HPIPE_RST_CLK_CTRL_PIPE_RST_MASK);

  MicroSecondDelay (20000);

  /* Return the status of the PLL */
  return (EFI_STATUS) (MmioRead32 (HpipeAddr + HPIPE_LANE_STATUS0_REG) &
    HPIPE_LANE_STATUS0_PCLK_EN_MASK);
}

EFI_STATUS
ComPhyAp806Init (
  IN CHIP_COMPHY_CONFIG *PtrChipCfg
  )
{
  EFI_STATUS Status;
  COMPHY_MAP *PtrComPhyMap, *SerdesMap;
  EFI_PHYSICAL_ADDRESS ComPhyBaseAddr, HpipeBaseAddr;
  UINT32 ComPhyMaxCount, Lane;
  UINT32 PcieBy4 = 1;

  ComPhyBaseAddr = PtrChipCfg->ComPhyBaseAddr;
  ComPhyMaxCount = PtrChipCfg->LanesCount;
  HpipeBaseAddr = PtrChipCfg->Hpipe3BaseAddr;
  SerdesMap = PtrChipCfg->MapData;

  /* Set PHY to Normal mode */
  RegSet (ComPhyBaseAddr + COMPHY_RESET_REG, 1 << COMPHY_RESET_SW_OFFSET,
    COMPHY_RESET_SW_MASK);
  RegSet (ComPhyBaseAddr + COMPHY_RESET_REG, 1 << COMPHY_RESET_CORE_OFFSET,
    COMPHY_RESET_CORE_MASK);

  /* Check if the first 4 Lanes configured as By-4 */
  for (Lane = 0, PtrComPhyMap = SerdesMap; Lane < 4; Lane++, PtrComPhyMap++) {
    if (PtrComPhyMap->Type != PHY_TYPE_PEX0) {
      PcieBy4 = 0;
      break;
    }
  }

  for (Lane = 0, PtrComPhyMap = SerdesMap; Lane < ComPhyMaxCount;
       Lane++, PtrComPhyMap++) {
    DEBUG((DEBUG_INFO, "ComPhy: Initialize serdes number %d\n", Lane));
    DEBUG((DEBUG_INFO, "ComPhy: Serdes Type = 0x%x\n", PtrComPhyMap->Type));
    switch (PtrComPhyMap->Type) {
    case PHY_TYPE_UNCONNECTED:
      continue;
      break;
    case PHY_TYPE_PEX0:
    case PHY_TYPE_PEX1:
    case PHY_TYPE_PEX2:
    case PHY_TYPE_PEX3:
      Status = ComPhyPciePowerUp (Lane, PcieBy4, HPIPE_ADDR(HpipeBaseAddr,
        Lane));
      MicroSecondDelay (20);
      break;
    default:
      DEBUG((DEBUG_INFO, "ComPhy: Unknown SerDes Type, skip initialize "
        "SerDes %d\n", Lane));
      break;
    }
    if (EFI_ERROR(Status))
      DEBUG((DEBUG_ERROR, "ComPhy: PLL is not locked - Failed to initialize "
        "Lane %d\n", Lane));
  }

  /* SW reset for PCIe for all Lanes after power up */
  if (PcieBy4) {
    for (Lane = 0; Lane < 4; Lane++) {
      ComPhyPcieReleaseSoftReset (HPIPE_ADDR(HpipeBaseAddr, Lane));
    }

    /*
     * Release PIPE RESET - release PHY from reset
     * need to release the Lanes without delay between them
     */
    DEBUG((DEBUG_INFO, "ComPhy: Release PIPE reset for PCIe-By4, write to "
      "Reset Clock control register\n"));
    for (Lane = 0; Lane < 4; Lane++) {
      RegSetSilent(HPIPE_ADDR(HpipeBaseAddr, Lane) + HPIPE_RST_CLK_CTRL_REG,
        0x0 << HPIPE_RST_CLK_CTRL_PIPE_RST_OFFSET,
        HPIPE_RST_CLK_CTRL_PIPE_RST_MASK);
    }

    MicroSecondDelay (20000);
    for (Lane = 0; Lane < 4; Lane++) {
      Status = (EFI_STATUS) MmioRead32 (HPIPE_ADDR(HpipeBaseAddr, Lane) +
           HPIPE_LANE_STATUS0_REG) & HPIPE_LANE_STATUS0_PCLK_EN_MASK;
      if (EFI_ERROR(Status))
        DEBUG((DEBUG_ERROR, "ComPhy: PLL is not locked - Failed to initialize "
        "Lane %d\n", Lane));
    }
  }

  /*
   * Set PCIe transactions towards A2 as:
   * - read allocate
   * - write non alocate
   * - outer sharable
   */
  RegSet (ComPhyBaseAddr + COMPHY_PCI_MAC_CTRL, COMPHY_PCI_COHERENT <<
    COMPHY_PCI_AXI_CACHE_OFFSET, COMPHY_PCI_AXI_CACHE_MASK);

  /* Set the Port x1 */
  if (PcieBy4)
    RegSet (ComPhyBaseAddr + COMPHY_PCI_MAC_CTRL, 0 << COMPHY_PCI_X1_EN_OFFSET,
      COMPHY_PCI_X1_EN_MASK);
  else
    RegSet (ComPhyBaseAddr + COMPHY_PCI_MAC_CTRL, 1 << COMPHY_PCI_X1_EN_OFFSET,
      COMPHY_PCI_X1_EN_MASK);

  /* Enable PCIe unit */
  RegSet (ComPhyBaseAddr + COMPHY_PCI_MAC_CTRL, 1 << COMPHY_PCI_EN_OFFSET,
    COMPHY_PCI_EN_MASK);

  return EFI_SUCCESS;
}
