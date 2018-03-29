/**
*
*  Copyright (C) 2018, Marvell International Ltd. and its affiliates
*
*  This program and the accompanying materials are licensed and made available
*  under the terms and conditions of the BSD License which accompanies this
*  distribution. The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/
#ifndef __ARMADA_BOARD_DESC_LIB_H__
#define __ARMADA_BOARD_DESC_LIB_H__

#include <Library/ArmadaSoCDescLib.h>

#include <Protocol/Gpio.h>

//
// COMPHY NIC devices per-board description
//
// TODO - Extend structure with entire
// ports description instead of PCDs.
//
typedef struct {
  MV_SOC_COMPHY_DESC *SoC;
  UINT8               ComPhyDevCount;
} MV_BOARD_COMPHY_DESC;

//
// I2C devices per-board description
//
// TODO - Extend structure with entire
// ports description instead of PCDs.
//
typedef struct {
  MV_SOC_I2C_DESC *SoC;
  UINT8            I2cDevCount;
} MV_BOARD_I2C_DESC;

//
// MDIO devices per-board description
//
typedef struct {
  MV_SOC_MDIO_DESC *SoC;
  UINT8             MdioDevCount;
} MV_BOARD_MDIO_DESC;

//
// NonDiscoverableDevices per-board description
//

//
// AHCI devices per-board description
//
typedef struct {
  MV_SOC_AHCI_DESC *SoC;
  UINT8             AhciDevCount;
} MV_BOARD_AHCI_DESC;

//
// SDMMC devices per-board description
//
// TODO - Extend structure with entire
// ports description instead of PCDs.
//
typedef struct {
  MV_SOC_SDMMC_DESC *SoC;
  UINT8              SdMmcDevCount;
} MV_BOARD_SDMMC_DESC;

//
// XHCI devices per-board description
//
typedef struct {
  MV_SOC_XHCI_DESC *SoC;
  UINT8             XhciDevCount;
} MV_BOARD_XHCI_DESC;

//
// PP2 NIC devices per-board description
//
// TODO - Extend structure with entire
// ports description instead of PCDs.
//
typedef struct {
  MV_SOC_PP2_DESC *SoC;
  UINT8            Pp2DevCount;
} MV_BOARD_PP2_DESC;

//
// UTMI PHY devices per-board description
//
typedef struct {
  MV_SOC_UTMI_DESC *SoC;
  UINT8             UtmiDevCount;
  UINT8             UtmiPortType;
} MV_BOARD_UTMI_DESC;

//
// PCIE devices per-device description
//
typedef struct {
  UINT8 PcieIndex;
  UINTN PcieRegBase;
  UINT8 PcieBusMin;
  UINT8 PcieBusMax;
  UINTN PcieBaseAddress;
  UINTN PcieIoTranslation;
  UINTN PcieIoWinBase;
  UINTN PcieIoWinSize;
  UINT8 PcieMmio32Translation;
  UINTN PcieMmio32WinBase;
  UINTN PcieMmio32WinSize;
  UINT8 PcieMmio64Translation;
  UINT64 PcieMmio64WinBase;
  UINT64 PcieMmio64WinSize;
  GPIO_PIN_DESC PcieResetGpio;
} MV_BOARD_PCIE_DEV_DESC;

typedef struct {
  MV_BOARD_PCIE_DEV_DESC *PcieDevDesc;
  UINT8                   PcieDevCount;
} MV_BOARD_PCIE_DESC;

EFI_STATUS
EFIAPI
ArmadaBoardDescPcieGet (
  IN OUT UINT8 *PcieDevCount,
  IN OUT MV_BOARD_PCIE_DEV_DESC **PcieDesc
  );
#endif /* __ARMADA_SOC_DESC_LIB_H__ */
