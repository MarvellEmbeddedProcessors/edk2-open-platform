/********************************************************************************
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

#include "Armada7040IcuLib.h"

/*
 * Allocate the MSI address per interrupt group,
 * unsupported groups get NULL address
 */
STATIC ICU_MSI MsiAddr[ICU_GRP_MAX] = {
  {ICU_GRP_NSR,  0xf03f0040, 0xf03f0048}, /* Non secure interrupts*/
  {ICU_GRP_SR,   0,          0x0},        /* Secure interrupts */
  {ICU_GRP_LPI,  0x0,        0x0},        /* LPI interrupts */
  {ICU_GRP_VLPI, 0x0,        0x0},        /* Virtual LPI interrupts */
  {ICU_GRP_SEI,  0xf03f0230, 0x0},        /* System error interrupts */
  {ICU_GRP_REI,  0xf03f0270, 0x0},        /* RAM error interrupts */
};

/* Multi instance sources, multiplied in dual CP mode */
STATIC ICU_IRQ IrqMapNsMulti[NS_MULTI_IRQS] = {
  {22, 0, 0},   /* PCIx4 INT A interrupt */
  {23, 1, 0},   /* PCIx1 INT A interrupt */
  {24, 2, 0},   /* PCIx1 INT A interrupt */

  {33, 3, 0},   /* PPv2 DBG AXI monitor */
  {34, 3, 0},   /* HB1      AXI monitor */
  {35, 3, 0},   /* AP       AXI monitor */
  {36, 3, 0},   /* PPv2     AXI monitor */

  {38,  4, 0},  /* PPv2 Misc */

  {39,  5, 0},  /* PPv2 Irq */
  {40,  6, 0},  /* PPv2 Irq */
  {41,  7, 0},  /* PPv2 Irq */
  {42,  8, 0},  /* PPv2 Irq */
  {43,  9, 0},  /* PPv2 Irq */
  {44, 10, 0},  /* PPv2 Irq */
  {45, 11, 0},  /* PPv2 Irq */
  {46, 12, 0},  /* PPv2 Irq */
  {47, 13, 0},  /* PPv2 Irq */
  {48, 14, 0},  /* PPv2 Irq */
  {49, 15, 0},  /* PPv2 Irq */
  {50, 16, 0},  /* PPv2 Irq */
  {51, 17, 0},  /* PPv2 Irq */
  {52, 18, 0},  /* PPv2 Irq */
  {53, 19, 0},  /* PPv2 Irq */
  {54, 20, 0},  /* PPv2 Irq */

  {78, 21, 0},  /* MG Irq */
  {88, 22, 0},  /* EIP-197 ring-0 */
  {89, 23, 0},  /* EIP-197 ring-1 */
  {90, 24, 0},  /* EIP-197 ring-2 */
  {91, 25, 0},  /* EIP-197 ring-3 */
  {92, 26, 0},  /* EIP-197 UINTN */
  {95, 27, 0},  /* EIP-150 Irq */
  {102, 28, 0}, /* USB3 Device Irq */
  {105, 29, 0}, /* USB3 Host-1 Irq */
  {106, 30, 0}, /* USB3 Host-0 Irq */
  {107, 31, 0}, /* SATA Host-1 Irq */
  {109, 31, 0}, /* SATA Host-0 Irq */
  {126, 33, 0}, /* PTP Irq */
  {127, 34, 0}, /* GOP-3 Irq */
  {128, 35, 0}, /* GOP-2 Irq */
  {129, 36, 0}, /* GOP-0 Irq */
};

/* Single instance sources, not multiplies in dual CP mode */
STATIC ICU_IRQ IrqMapNsSingle[NS_SINGLE_IRQS] = {
  {27, 37, 0},  /* SD/MMC */
  {76, 38, 0},  /* Audio */
  {77, 39, 0},  /* MSS RTC */
  {79, 40, 0},  /* GPIO 56-63 */
  {80, 41, 0},  /* GPIO 48-55 */
  {81, 42, 0},  /* GPIO 40-47 */
  {82, 43, 0},  /* GPIO 32-39 */
  {83, 44, 0},  /* GPIO 24-31 */
  {84, 45, 0},  /* GPIO 16-23 */
  {85, 46, 0},  /* GPIO  8-15 */
  {86, 47, 0},  /* GPIO  0-7  */
  {111, 48, 0}, /* TDM-MC func 1 */
  {112, 49, 0}, /* TDM-MC func 0 */
  {113, 50, 0}, /* TDM-MC Irq */
  {115, 51, 0}, /* NAND Irq */
  {117, 52, 0}, /* SPI-1 Irq */
  {118, 53, 0}, /* SPI-0 Irq */
  {120, 54, 0}, /* I2C 0 Irq */
  {121, 55, 0}, /* I2C 1 Irq */
  {122, 56, 0}, /* UART 0 Irq */
  {123, 57, 0}, /* UART 1 Irq */
  {124, 58, 0}, /* UART 2 Irq */
  {125, 59, 0}, /* UART 3 Irq */
};

/* SEI - System Error Interrupts */
STATIC ICU_IRQ IrqMapSei[SEI_IRQS] = {
  {11, 0, 0},   /* SEI error CP-2-CP */
  {15, 1, 0},   /* PIDI-64 SOC */
  {16, 2, 0},   /* D2D error Irq */
  {17, 3, 0},   /* D2D Irq */
  {18, 4, 0},   /* NAND error */
  {19, 5, 0},   /* PCIx4 error */
  {20, 6, 0},   /* PCIx1_0 error */
  {21, 7, 0},   /* PCIx1_1 error */
  {25, 8, 0},   /* SDIO reg error */
  {75, 9, 0},   /* IOB error */
  {94, 10, 0},  /* EIP150 error */
  {97, 11, 0},  /* XOR-1 system error */
  {99, 12, 0},  /* XOR-0 system error */
  {108, 13, 0}, /* SATA-1 error */
  {110, 14, 0}, /* SATA-0 error */
  {114, 15, 0}, /* TDM-MC error */
  {116, 16, 0}, /* DFX server Irq */
  {117, 17, 0}, /* Device bus error */
  {147, 18, 0}, /* Audio error */
  {171, 19, 0}, /* PIDI Sync error */
};

/* REI - RAM Error Interrupts */
STATIC ICU_IRQ IrqMapRei[REI_IRQS] = {
  {12, 0, 0},  /* REI error CP-2-CP */
  {26, 1, 0},  /* SDIO memory error */
  {87, 2, 0},  /* EIP-197 ECC error */
  {93, 3, 0},  /* EIP-150 RAM error */
  {96, 4, 0},  /* XOR-1 memory Irq */
  {98, 5, 0},  /* XOR-0 memory Irq */
  {100, 6, 0}, /* USB3 device tx parity */
  {101, 7, 0}, /* USB3 device rq parity */
  {103, 8, 0}, /* USB3H-1 RAM error */
  {104, 9, 0}, /* USB3H-0 RAM error */
};

STATIC
VOID
IcuClearIrq (
  IN UINTN IcuBase,
  IN UINTN Nbr
  )
{

  MmioWrite32 (IcuBase + ICU_INT_CFG(Nbr), 0);
}

STATIC
VOID
IcuSetIrq (
  IN UINTN IcuBase,
  IN ICU_IRQ *Irq,
  IN UINT32 SpiBase,
  IN ICU_GROUP Group
  )
{
  UINT32 IcuInt;

  IcuInt  = (Irq->SpiId + SpiBase) | (1 << ICU_INT_ENABLE_OFFSET);
  IcuInt |= Irq->IsEdge << ICU_IS_EDGE_OFFSET;
  IcuInt |= Group << ICU_GROUP_OFFSET;

  MmioWrite32 (IcuBase + ICU_INT_CFG(Irq->IcuId), IcuInt);
}

/*
 *  This function uses 2 spi values to initialize the ICU
 *  SpiBase: used to set the base of the SPI id in the MSI message
 *           generated by the ICU. AP806-Z1 required SpiBase=64 while
 *           AP806-A0 uses SpiBase=0
 *  SpiOffset: used to shift the multi instance interrupts between CP-0
 *             and CP-1
 */
VOID
IcuInit (
  VOID
  )
{
  UINTN i, CpBase, SpiBase, SpiOffset, IcuBase;
  ICU_IRQ *Irq;
  ICU_MSI *Msi;

  CpBase = PcdGet64 (PcdIcuCpBase);

  /* Prevent execution on Apn806 board */
  if (CpBase == 0) {
    return;
  }

  SpiBase = PcdGet64 (PcdIcuSpiBase);
  SpiOffset = PcdGet64 (PcdIcuSpiOffset);

  IcuBase = CpBase + ICU_REG_BASE;

  /* Set the addres for SET_SPI and CLR_SPI registers in AP */
  Msi = MsiAddr;
  for (i = 0; i < ICU_GRP_MAX; i++, Msi++) {
    MmioWrite32 (IcuBase + ICU_SET_SPI_AL(Msi->Group), Msi->SetSpiAddr &
      0xFFFFFFFF);
    MmioWrite32 (IcuBase + ICU_SET_SPI_AH(Msi->Group), Msi->SetSpiAddr >> 32);
    MmioWrite32 (IcuBase + ICU_CLR_SPI_AL(Msi->Group), Msi->ClrSpiAddr &
      0xFFFFFFFF);
    MmioWrite32 (IcuBase + ICU_CLR_SPI_AH(Msi->Group), Msi->ClrSpiAddr >> 32);
  }

  /* Mask all ICU interrupts */
  for (i = 0; i < MAX_ICU_IRQS; i++)
    IcuClearIrq(IcuBase, i);

  /* Configure the ICU interrupt lines */
  /* Multi instance interrupts use different SPI ID for CP-1 */
  Irq = IrqMapNsMulti;
  for (i = 0; i < NS_MULTI_IRQS; i++, Irq++)
    IcuSetIrq (IcuBase, Irq, SpiBase + SpiOffset, ICU_GRP_NSR);

  Irq = IrqMapNsSingle;
  for (i = 0; i < NS_MULTI_IRQS; i++, Irq++)
    IcuSetIrq (IcuBase, Irq, SpiBase, ICU_GRP_NSR);

  Irq = IrqMapSei;
  for (i = 0; i < NS_MULTI_IRQS; i++, Irq++)
    IcuSetIrq (IcuBase, Irq, SpiBase, ICU_GRP_SEI);

  Irq = IrqMapRei;
  for (i = 0; i < NS_MULTI_IRQS; i++, Irq++)
    IcuSetIrq (IcuBase, Irq, SpiBase, ICU_GRP_REI);
}
