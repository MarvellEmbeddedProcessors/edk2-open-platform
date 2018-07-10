/**
*
*  Copyright (C) 2018, Marvell International Ltd. and its affiliates.
*
*  This program and the accompanying materials are licensed and made available
*  under the terms and conditions of the BSD License which accompanies this
*  distribution. The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
*  Glossary - abbreviations used in Marvell SampleAtReset library implementation:
*  ICU - Interrupt Consolidation Unit
*  AP - Application Processor hardware block (Armada 7k8k incorporates AP806)
*  CP - South Bridge hardware blocks (Armada 7k8k incorporates CP110)
*
**/

#include "IcuLib.h"

EFI_EVENT EfiExitBootServicesEvent = (EFI_EVENT)NULL;

STATIC CONST ICU_IRQ IrqMapNonSecure[] = {
  {22,   0, Level}, /* PCIx4 INT A interrupt */
  {23,   1, Level}, /* PCIx1 INT A interrupt */
  {24,   2, Level}, /* PCIx1 INT A interrupt */
  {27,   3, Level}, /* SD/MMC */
  {33,   4, Level}, /* PPv2 DBG AXI monitor */
  {34,   4, Level}, /* HB1      AXI monitor */
  {35,   4, Level}, /* AP       AXI monitor */
  {36,   4, Level}, /* PPv2     AXI monitor */
  {39,   5, Level}, /* PPv2 Irq */
  {40,   6, Level}, /* PPv2 Irq */
  {41,   7, Level}, /* PPv2 Irq */
  {43,   8, Level}, /* PPv2 Irq */
  {44,   9, Level}, /* PPv2 Irq */
  {45,  10, Level}, /* PPv2 Irq */
  {47,  11, Level}, /* PPv2 Irq */
  {48,  12, Level}, /* PPv2 Irq */
  {49,  13, Level}, /* PPv2 Irq */
  {51,  14, Level}, /* PPv2 Irq */
  {52,  15, Level}, /* PPv2 Irq */
  {53,  16, Level}, /* PPv2 Irq */
  {55,  17, Level}, /* PPv2 Irq */
  {56,  18, Level}, /* PPv2 Irq */
  {57,  19, Level}, /* PPv2 Irq */
  {59,  20, Level}, /* PPv2 Irq */
  {60,  21, Level}, /* PPv2 Irq */
  {61,  22, Level}, /* PPv2 Irq */
  {63,  23, Level}, /* PPv2 Irq */
  {64,  24, Level}, /* PPv2 Irq */
  {65,  25, Level}, /* PPv2 Irq */
  {67,  26, Level}, /* PPv2 Irq */
  {68,  27, Level}, /* PPv2 Irq */
  {69,  28, Level}, /* PPv2 Irq */
  {71,  29, Level}, /* PPv2 Irq */
  {72,  30, Level}, /* PPv2 Irq */
  {73,  31, Level}, /* PPv2 Irq */
  {78,  32, Level}, /* MG Irq */
  {79,  33, Level}, /* GPIO 56-63 */
  {80,  34, Level}, /* GPIO 48-55 */
  {81,  35, Level}, /* GPIO 40-47 */
  {82,  36, Level}, /* GPIO 32-39 */
  {83,  37, Level}, /* GPIO 24-31 */
  {84,  38, Level}, /* GPIO 16-23 */
  {85,  39, Level}, /* GPIO  8-15 */
  {86,  40, Level}, /* GPIO  0-7  */
  {88,  41, Level}, /* EIP-197 ring-0 */
  {89,  42, Level}, /* EIP-197 ring-1 */
  {90,  43, Level}, /* EIP-197 ring-2 */
  {91,  44, Level}, /* EIP-197 ring-3 */
  {92,  45, Level}, /* EIP-197 int */
  {95,  46, Level}, /* EIP-150 Irq */
  {102, 47, Level}, /* USB3 Device Irq */
  {105, 48, Level}, /* USB3 Host-1 Irq */
  {106, 49, Level}, /* USB3 Host-0 Irq */
  {107, 50, Level}, /* SATA Host-1 Irq */
  {109, 50, Level}, /* SATA Host-0 Irq */
  {115, 52, Level}, /* NAND Irq */
  {117, 53, Level}, /* SPI-1 Irq */
  {118, 54, Level}, /* SPI-0 Irq */
  {120, 55, Level}, /* I2C 0 Irq */
  {121, 56, Level}, /* I2C 1 Irq */
  {122, 57, Level}, /* UART 0 Irq */
  {123, 58, Level}, /* UART 1 Irq */
  {124, 59, Level}, /* UART 2 Irq */
  {125, 60, Level}, /* UART 3 Irq */
  {127, 61, Level}, /* GOP-3 Irq */
  {128, 62, Level}, /* GOP-2 Irq */
  {129, 63, Level}, /* GOP-0 Irq */
};

/*
 * SEI - System Error Interrupts
 * Note: SPI ID 0-20 are reserved for North-Bridge
 */
STATIC ICU_IRQ IrqMapSei[] = {
  {11,  21, Level}, /* SEI error CP-2-CP */
  {15,  22, Level}, /* PIDI-64 SOC */
  {16,  23, Level}, /* D2D error Irq */
  {17,  24, Level}, /* D2D Irq */
  {18,  25, Level}, /* NAND error */
  {19,  26, Level}, /* PCIx4 error */
  {20,  27, Level}, /* PCIx1_0 error */
  {21,  28, Level}, /* PCIx1_1 error */
  {25,  29, Level}, /* SDIO reg error */
  {75,  30, Level}, /* IOB error */
  {94,  31, Level}, /* EIP150 error */
  {97,  32, Level}, /* XOR-1 system error */
  {99,  33, Level}, /* XOR-0 system error */
  {108, 34, Level}, /* SATA-1 error */
  {110, 35, Level}, /* SATA-0 error */
  {114, 36, Level}, /* TDM-MC error */
  {116, 37, Level}, /* DFX server Irq */
  {117, 38, Level}, /* Device bus error */
  {147, 39, Level}, /* Audio error */
  {171, 40, Level}, /* PIDI Sync error */
};

/* REI - RAM Error Interrupts */
STATIC CONST ICU_IRQ IrqMapRei[] = {
  {12,  0, Level}, /* REI error CP-2-CP */
  {26,  1, Level}, /* SDIO memory error */
  {87,  2, Level}, /* EIP-197 ECC error */
  {93,  3, Edge},  /* EIP-150 RAM error */
  {96,  4, Level}, /* XOR-1 memory Irq */
  {98,  5, Level}, /* XOR-0 memory Irq */
  {100, 6, Edge},  /* USB3 device tx parity */
  {101, 7, Edge},  /* USB3 device rq parity */
  {103, 8, Edge},  /* USB3H-1 RAM error */
  {104, 9, Edge},  /* USB3H-0 RAM error */
};

STATIC CONST ICU_CONFIG IcuConfigDefault = {
  .NonSecure =  { IrqMapNonSecure, ARRAY_SIZE (IrqMapNonSecure) },
  .Sei =        { IrqMapSei, ARRAY_SIZE (IrqMapSei) },
  .Rei =        { IrqMapRei, ARRAY_SIZE (IrqMapRei) },
};

STATIC
VOID
IcuClearIrq (
  IN UINTN IcuBase,
  IN UINTN Nr
)
{
  MmioWrite32 (IcuBase + ICU_INT_CFG (Nr), 0);
}

STATIC
VOID
IcuSetIrq (
  IN UINTN           IcuBase,
  IN CONST ICU_IRQ  *Irq,
  IN UINTN           SpiBase,
  IN ICU_GROUP       Group
  )
{
  UINT32 IcuInt;

  IcuInt  = (Irq->SpiId + SpiBase) | (1 << ICU_INT_ENABLE_OFFSET);
  IcuInt |= Irq->IrqType << ICU_IS_EDGE_OFFSET;
  IcuInt |= Group << ICU_GROUP_OFFSET;

  MmioWrite32 (IcuBase + ICU_INT_CFG (Irq->IcuId), IcuInt);
}

STATIC
VOID
IcuConfigure (
  IN UINTN             CpIndex,
  IN MV_SOC_ICU_DESC  *IcuDesc,
  IN CONST ICU_CONFIG *Config
  )
{
  UINTN IcuBase, Index, SpiOffset, SpiBase;
  CONST ICU_IRQ *Irq;
  ICU_MSI *Msi;

  /* Get ICU registers base address */
  IcuBase = ICU_REG_BASE (CpIndex);
  /* Get the base of the GIC SPI ID in the MSI message */
  SpiBase = IcuDesc->IcuSpiBase;
  /* Get multiple CP110 instances SPI ID shift */
  SpiOffset = CpIndex * ICU_MAX_IRQS_PER_CP;
  /* Get MSI addresses per interrupt group */
  Msi = IcuDesc->IcuMsi;

  /* Set the addres for SET_SPI and CLR_SPI registers in AP */
  for (Index = 0; Index < ICU_GROUP_MAX; Index++, Msi++) {
    MmioWrite32 (IcuBase + ICU_SET_SPI_AL (Msi->Group), Msi->SetSpiAddr & 0xFFFFFFFF);
    MmioWrite32 (IcuBase + ICU_SET_SPI_AH (Msi->Group), Msi->SetSpiAddr >> 32);
    MmioWrite32 (IcuBase + ICU_CLR_SPI_AL (Msi->Group), Msi->ClrSpiAddr & 0xFFFFFFFF);
    MmioWrite32 (IcuBase + ICU_CLR_SPI_AH (Msi->Group), Msi->ClrSpiAddr >> 32);
  }

  /* Mask all ICU interrupts */
  for (Index = 0; Index < MAX_ICU_IRQS; Index++) {
    IcuClearIrq (IcuBase, Index);
  }

  /* Configure the ICU interrupt lines */
  Irq = Config->NonSecure.Map;
  for (Index = 0; Index < Config->NonSecure.Size; Index++, Irq++) {
    IcuSetIrq (IcuBase, Irq, SpiBase + SpiOffset, ICU_GROUP_NSR);
  }

  Irq = Config->Sei.Map;
  for (Index = 0; Index < Config->Sei.Size; Index++, Irq++) {
    IcuSetIrq (IcuBase, Irq, SpiBase, ICU_GROUP_SEI);
  }

  Irq = Config->Rei.Map;
  for (Index = 0; Index < Config->Rei.Size; Index++, Irq++) {
    IcuSetIrq (IcuBase, Irq, SpiBase, ICU_GROUP_REI);
  }
}

STATIC
VOID
IcuClearGicSpi (
  IN UINTN             CpIndex,
  IN MV_SOC_ICU_DESC  *IcuDesc
  )
{
  CONST ICU_CONFIG *Config;
  UINTN Index, SpiOffset, SpiBase;
  CONST ICU_IRQ *Irq;
  ICU_MSI *Msi;

  Config = &IcuConfigDefault;

  /* Get the base of the GIC SPI ID in the MSI message */
  SpiBase = IcuDesc->IcuSpiBase;
  /* Get multiple CP110 instances SPI ID shift */
  SpiOffset = CpIndex * ICU_MAX_IRQS_PER_CP;
  /* Get MSI addresses per interrupt group */
  Msi = IcuDesc->IcuMsi;

  /* Clear ICU-generated GIC SPI interrupts */
  Irq = Config->NonSecure.Map;
  for (Index = 0; Index < Config->NonSecure.Size; Index++, Irq++) {
    MmioWrite32 (Msi->ClrSpiAddr, Irq->SpiId + SpiBase + SpiOffset);
  }
}

VOID
EFIAPI
IcuCleanUp (
  IN EFI_EVENT  Event,
  IN VOID      *Context
  )
{
  MV_SOC_ICU_DESC *IcuDesc;
  UINTN CpCount, CpIndex;

  IcuDesc = Context;

  CpCount = FixedPcdGet8 (PcdMaxCpCount);
  if (CpCount > ICU_MAX_SUPPORTED_UNITS) {
    CpCount = ICU_MAX_SUPPORTED_UNITS;
  }

  for (CpIndex = 0; CpIndex < CpCount; CpIndex++) {
    IcuClearGicSpi (CpIndex, IcuDesc);
  }
}

EFI_STATUS
EFIAPI
ArmadaIcuInitialize (
  )
{
  MV_SOC_ICU_DESC *IcuDesc;
  UINTN CpCount, CpIndex;
  EFI_STATUS Status;

  /*
   * Due to limited amount of interrupt lanes, only 2 units can be
   * wired to the GIC.
   */
  CpCount = FixedPcdGet8 (PcdMaxCpCount);
  if (CpCount > ICU_MAX_SUPPORTED_UNITS) {
    DEBUG ((DEBUG_ERROR,
      "%a: Default ICU to GIC mapping is available for maximum %d CP110 units",
      ICU_MAX_SUPPORTED_UNITS,
      __FUNCTION__));
    CpCount = ICU_MAX_SUPPORTED_UNITS;
  }

  /* Obtain SoC description of the ICU */
  Status = ArmadaSoCDescIcuGet (&IcuDesc);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  /* Configure default ICU to GIC interrupt mapping for each CP110 */
  for (CpIndex = 0; CpIndex < CpCount; CpIndex++) {
    IcuConfigure (CpIndex, IcuDesc, &IcuConfigDefault);
  }

  /*
   * In order to be immune to the OS capability of clearing ICU-generated
   * GIC interrupts, register ExitBootServices event, that will
   * make sure they remain disabled during OS boot.
   */
  Status = gBS->CreateEvent (
                  EVT_SIGNAL_EXIT_BOOT_SERVICES,
                  TPL_NOTIFY,
                  IcuCleanUp,
                  IcuDesc,
                  &EfiExitBootServicesEvent
                  );

  return Status;
}
