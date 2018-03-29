/** @file
  PCI Host Bridge Library instance for Marvell 70x0/80x0

  Copyright (c) 2017, Linaro Ltd. All rights reserved.<BR>

  This program and the accompanying materials are licensed and made available
  under the terms and conditions of the BSD License which accompanies this
  distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php.

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS, WITHOUT
  WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <PiDxe.h>
#include <IndustryStandard/Pci22.h>
#include <Library/ArmLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/UefiBootServicesTableLib.h>

#include <Protocol/BoardDesc.h>
#include <Protocol/Gpio.h>

#define IATU_VIEWPORT_OFF                                   0x900
#define IATU_VIEWPORT_INBOUND                               BIT31
#define IATU_VIEWPORT_OUTBOUND                              0
#define IATU_VIEWPORT_REGION_INDEX(Idx)                     ((Idx) & 7)

#define IATU_REGION_CTRL_1_OFF_OUTBOUND_0                   0x904
#define IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_MEM          0x0
#define IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_IO           0x2
#define IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_CFG0         0x4
#define IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_CFG1         0x5

#define IATU_REGION_CTRL_2_OFF_OUTBOUND_0                   0x908
#define IATU_REGION_CTRL_2_OFF_OUTBOUND_0_REGION_EN         BIT31
#define IATU_REGION_CTRL_2_OFF_OUTBOUND_0_CFG_SHIFT_MODE    BIT28

#define IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0                   0x90C
#define IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_0                 0x910
#define IATU_LIMIT_ADDR_OFF_OUTBOUND_0                      0x914
#define IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0                 0x918
#define IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_0               0x91C

#define PORT_LINK_CTRL_OFF                                  0x710
#define PORT_LINK_CTRL_OFF_LINK_CAPABLE_x1                  (0x01 << 16)
#define PORT_LINK_CTRL_OFF_LINK_CAPABLE_x2                  (0x03 << 16)
#define PORT_LINK_CTRL_OFF_LINK_CAPABLE_x4                  (0x07 << 16)
#define PORT_LINK_CTRL_OFF_LINK_CAPABLE_x8                  (0x0f << 16)
#define PORT_LINK_CTRL_OFF_LINK_CAPABLE_x16                 (0x1f << 16)
#define PORT_LINK_CTRL_OFF_LINK_CAPABLE_MASK                (0x3f << 16)

#define GEN2_CTRL_OFF                                       0x80c
#define GEN2_CTRL_OFF_NUM_OF_LANES(n)                       (((n) & 0x1f) << 8)
#define GEN2_CTRL_OFF_NUM_OF_LANES_MASK                     (0x1f << 8)
#define GEN2_CTRL_OFF_DIRECT_SPEED_CHANGE                   BIT17

#define PCIE_GLOBAL_CTRL_OFFSET                             0x8000
#define PCIE_GLOBAL_APP_LTSSM_EN                            BIT2
#define PCIE_GLOBAL_CTRL_DEVICE_TYPE_RC                     (0x4 << 4)
#define PCIE_GLOBAL_CTRL_DEVICE_TYPE_MASK                   (0xF << 4)

#define PCIE_GLOBAL_STATUS_REG                              0x8008
#define PCIE_GLOBAL_STATUS_RDLH_LINK_UP                     BIT1
#define PCIE_GLOBAL_STATUS_PHY_LINK_UP                      BIT9

#define PCIE_PM_STATUS                                      0x8014
#define PCIE_PM_LTSSM_STAT_MASK                             (0x3f << 3)

#define PCIE_GLOBAL_INT_MASK1_REG                           0x8020
#define PCIE_INT_A_ASSERT_MASK                              BIT9
#define PCIE_INT_B_ASSERT_MASK                              BIT10
#define PCIE_INT_C_ASSERT_MASK                              BIT11
#define PCIE_INT_D_ASSERT_MASK                              BIT12

#define PCIE_ARCACHE_TRC_REG                                0x8050
#define PCIE_AWCACHE_TRC_REG                                0x8054
#define PCIE_ARUSER_REG                                     0x805C
#define PCIE_AWUSER_REG                                     0x8060

#define ARCACHE_DEFAULT_VALUE                               0x3511
#define AWCACHE_DEFAULT_VALUE                               0x5311

#define AX_USER_DOMAIN_INNER_SHAREABLE                      (0x1 << 4)
#define AX_USER_DOMAIN_OUTER_SHAREABLE                      (0x2 << 4)
#define AX_USER_DOMAIN_MASK                                 (0x3 << 4)

#define PCIE_LINK_CAPABILITY                                0x7C
#define PCIE_LINK_CTL_2                                     0xA0
#define TARGET_LINK_SPEED_MASK                              0xF
#define LINK_SPEED_GEN_1                                    0x1
#define LINK_SPEED_GEN_2                                    0x2
#define LINK_SPEED_GEN_3                                    0x3

#define PCIE_GEN3_EQU_CTRL                                  0x8A8
#define GEN3_EQU_EVAL_2MS_DISABLE                           BIT5

#define PCIE_LINK_UP_TIMEOUT_US                             (40000)

STATIC
VOID
ConfigureWindow (
  UINTN     PcieRegBase,
  UINTN     Index,
  UINT64    CpuBase,
  UINT64    PciBase,
  UINT64    Size,
  UINTN     Type,
  UINTN     EnableFlags
  )
{
  ArmDataMemoryBarrier ();

  MmioWrite32 (PcieRegBase + IATU_VIEWPORT_OFF,
               IATU_VIEWPORT_OUTBOUND | IATU_VIEWPORT_REGION_INDEX (Index));

  ArmDataMemoryBarrier ();

  MmioWrite32 (PcieRegBase + IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0,
               (UINT32)(CpuBase & 0xFFFFFFFF));
  MmioWrite32 (PcieRegBase + IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_0,
               (UINT32)(CpuBase >> 32));
  MmioWrite32 (PcieRegBase + IATU_LIMIT_ADDR_OFF_OUTBOUND_0,
               (UINT32)(CpuBase + Size - 1));
  MmioWrite32 (PcieRegBase + IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0,
               (UINT32)(PciBase & 0xFFFFFFFF));
  MmioWrite32 (PcieRegBase + IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_0,
               (UINT32)(PciBase >> 32));
  MmioWrite32 (PcieRegBase + IATU_REGION_CTRL_1_OFF_OUTBOUND_0,
               Type);
  MmioWrite32 (PcieRegBase + IATU_REGION_CTRL_2_OFF_OUTBOUND_0,
               IATU_REGION_CTRL_2_OFF_OUTBOUND_0_REGION_EN | EnableFlags);
}

STATIC
VOID
WaitForLink (
  UINTN PcieRegBase
  )
{
  UINT32 Mask;
  UINT32 Status;
  UINT32 Timeout;

  if (!(MmioRead32 (PcieRegBase + PCIE_PM_STATUS) & PCIE_PM_LTSSM_STAT_MASK)) {
    DEBUG ((DEBUG_INIT, "%a: no PCIe device detected\n", __FUNCTION__));
    return;
  }

  //
  // Wait for the link to establish itself
  //
  DEBUG ((DEBUG_INIT, "%a: waiting for PCIe link\n", __FUNCTION__));

  Mask = PCIE_GLOBAL_STATUS_RDLH_LINK_UP | PCIE_GLOBAL_STATUS_PHY_LINK_UP;
  Timeout = PCIE_LINK_UP_TIMEOUT_US / 10;
  do {
    Status = MmioRead32 (PcieRegBase + PCIE_GLOBAL_STATUS_REG);
    if ((Status & Mask) == Mask) {
      break;
    }
    gBS->Stall (10);
  } while (Timeout--);
}

STATIC
EFI_STATUS
ResetPciSlot (
  IN GPIO_PIN_DESC *PcieResetGpio
  )
{
  MARVELL_GPIO_PROTOCOL *GpioProtocol;
  EFI_HANDLE                *ProtHandle = NULL;
  EFI_STATUS                 Status;

  /* Get GPIO protocol */
  Status = MarvellGpioGetHandle (GPIO_DRIVER_TYPE_SOC_CONTROLLER, &ProtHandle);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to find GPIO for SoC protocol, Status: 0x%x\n", Status));
    return Status;
  }

  Status = gBS->OpenProtocol (
                  ProtHandle,
                  &gMarvellGpioProtocolGuid,
                  (void **)&GpioProtocol,
                  gImageHandle,
                  NULL,
                  EFI_OPEN_PROTOCOL_GET_PROTOCOL
                  );
  if(EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to open GPIO protocol, Status: 0x%x\n", Status));
    return Status;
  }

  //
  // Reset the slot
  //
  Status = GpioProtocol->DirectionOutput(
                  GpioProtocol,
                  PcieResetGpio->ControllerId,
                  PcieResetGpio->PinNumber,
                  PcieResetGpio->ActiveHigh
                  );
  gBS->Stall (10 * 1000);

  Status = GpioProtocol->SetValue(
                  GpioProtocol,
                  PcieResetGpio->ControllerId,
                  PcieResetGpio->PinNumber,
                  0
                  );
  gBS->Stall (20 * 1000);

  Status = gBS->CloseProtocol (
                  ProtHandle,
                  &gMarvellGpioProtocolGuid,
                  gImageHandle,
                  NULL
                  );

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
Armada70x0PciHostBridgeLibConstructor (
  IN EFI_HANDLE       ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  MARVELL_BOARD_DESC_PROTOCOL *BoardDescProtocol;
  MV_BOARD_PCIE_DESC  *PcieDesc;
  EFI_STATUS           Status;
  UINT8                Index;
  MV_BOARD_PCIE_DEV_DESC *PcieDevDesc;
  UINTN                PcieBaseReg;

  /* Obtain list of available controllers */
  Status = gBS->LocateProtocol (&gMarvellBoardDescProtocolGuid,
                NULL,
                (VOID **)&BoardDescProtocol);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: Cannot locate BoardDesc protocol\n",
      __FUNCTION__));
    return EFI_DEVICE_ERROR;
  }

  Status = BoardDescProtocol->BoardDescPcieGet (BoardDescProtocol, &PcieDesc);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR,
      "%a: Cannot get Pcie board desc from BoardDesc protocol\n",
      __FUNCTION__));
    return EFI_DEVICE_ERROR;
  }

  for (Index = 0; Index < PcieDesc->PcieDevCount; Index++) {
    PcieDevDesc = &(PcieDesc->PcieDevDesc[Index]);
    PcieBaseReg = PcieDevDesc->PcieRegBase;

    ASSERT (PcieDevDesc->PcieBusMin == 0);
    ASSERT (PcieDevDesc->PcieBaseAddress % SIZE_256MB == 0);

    /* Reset PCIe slot */
    Status = ResetPciSlot(&PcieDevDesc->PcieResetGpio);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR,
        "%a: Cannot reset Pcie Slot\n",
        __FUNCTION__));
      return EFI_DEVICE_ERROR;
    }

    MmioAndThenOr32 (PcieBaseReg + PORT_LINK_CTRL_OFF,
                     ~PORT_LINK_CTRL_OFF_LINK_CAPABLE_MASK,
                     PORT_LINK_CTRL_OFF_LINK_CAPABLE_x4);

    MmioAndThenOr32 (PcieBaseReg + GEN2_CTRL_OFF,
                     ~GEN2_CTRL_OFF_NUM_OF_LANES_MASK,
                     GEN2_CTRL_OFF_NUM_OF_LANES(4) |
                     GEN2_CTRL_OFF_DIRECT_SPEED_CHANGE);

    MmioAndThenOr32 (PcieBaseReg + PCIE_GLOBAL_CTRL_OFFSET,
                     ~(PCIE_GLOBAL_CTRL_DEVICE_TYPE_MASK |
                       PCIE_GLOBAL_APP_LTSSM_EN),
                     PCIE_GLOBAL_CTRL_DEVICE_TYPE_RC);

    MmioWrite32 (PcieBaseReg + PCIE_ARCACHE_TRC_REG,
                 ARCACHE_DEFAULT_VALUE);

    MmioWrite32 (PcieBaseReg + PCIE_AWCACHE_TRC_REG,
                 AWCACHE_DEFAULT_VALUE);

    MmioAndThenOr32 (PcieBaseReg + PCIE_ARUSER_REG,
                     ~AX_USER_DOMAIN_MASK,
                     AX_USER_DOMAIN_OUTER_SHAREABLE);

    MmioAndThenOr32 (PcieBaseReg + PCIE_AWUSER_REG,
                     ~AX_USER_DOMAIN_MASK,
                     AX_USER_DOMAIN_OUTER_SHAREABLE);

    MmioAndThenOr32 (PcieBaseReg + PCIE_LINK_CTL_2,
                     ~TARGET_LINK_SPEED_MASK,
                     LINK_SPEED_GEN_3);

    MmioAndThenOr32 (PcieBaseReg + PCIE_LINK_CAPABILITY,
                     ~TARGET_LINK_SPEED_MASK,
                     LINK_SPEED_GEN_3);

    MmioOr32 (PcieBaseReg + PCIE_GEN3_EQU_CTRL,
              GEN3_EQU_EVAL_2MS_DISABLE);

    MmioOr32 (PcieBaseReg + PCIE_GLOBAL_CTRL_OFFSET,
              PCIE_GLOBAL_APP_LTSSM_EN);

    // Region 0: MMIO32 range
    ConfigureWindow (PcieBaseReg,
                     0,
                     PcieDevDesc->PcieMmio32WinBase,
                     PcieDevDesc->PcieMmio32WinBase,
                     PcieDevDesc->PcieMmio32WinSize,
                     IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_MEM,
                     0);

    // Region 1: Type 0 config space
    ConfigureWindow (PcieBaseReg,
                     1,
                     PcieDevDesc->PcieBaseAddress,
                     0x0,
                     SIZE_64KB,
                     IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_CFG0,
                     IATU_REGION_CTRL_2_OFF_OUTBOUND_0_CFG_SHIFT_MODE);

    // Region 2: Type 1 config space
    ConfigureWindow (PcieBaseReg,
                     2,
                     PcieDevDesc->PcieBaseAddress + SIZE_64KB,
                     0x0,
                     PcieDevDesc->PcieBusMax * SIZE_1MB,
                     IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_CFG1,
                     IATU_REGION_CTRL_2_OFF_OUTBOUND_0_CFG_SHIFT_MODE);

    // Region 3: port I/O range
    ConfigureWindow (PcieBaseReg,
                     3,
                     PcieDevDesc->PcieIoTranslation,
                     PcieDevDesc->PcieIoWinBase,
                     PcieDevDesc->PcieIoWinSize,
                     IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_IO,
                     0);

    // Region 4: MMIO64 range
    ConfigureWindow (PcieBaseReg,
                     4,
                     PcieDevDesc->PcieMmio64WinBase,
                     PcieDevDesc->PcieMmio64WinBase,
                     PcieDevDesc->PcieMmio64WinSize,
                     IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_MEM,
                     0);

    MmioOr32 (PcieBaseReg + PCIE_GLOBAL_INT_MASK1_REG,
              PCIE_INT_A_ASSERT_MASK |
              PCIE_INT_B_ASSERT_MASK |
              PCIE_INT_C_ASSERT_MASK |
              PCIE_INT_D_ASSERT_MASK);

    WaitForLink (PcieBaseReg);

    //
    // Enable the RC
    //
    MmioOr32 (PcieBaseReg + PCI_COMMAND_OFFSET,
              EFI_PCI_COMMAND_IO_SPACE |
              EFI_PCI_COMMAND_MEMORY_SPACE |
              EFI_PCI_COMMAND_BUS_MASTER);
  }

  return EFI_SUCCESS;
}
