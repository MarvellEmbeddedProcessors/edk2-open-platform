/*******************************************************************************
Copyright (C) 2018 Marvell International Ltd.

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
#include "MvBoardDescDxe.h"

MV_BOARD_DESC *mBoardDescInstance;

STATIC
EFI_STATUS
MvBoardDescComPhyGet (
  IN MARVELL_BOARD_DESC_PROTOCOL  *This,
  IN OUT MV_BOARD_COMPHY_DESC    **ComPhyDesc
  )
{
  UINT8 *ComPhyDeviceTable, ComPhyCount;
  UINTN ComPhyDeviceTableSize, ComPhyIndex, Index;
  MV_BOARD_COMPHY_DESC *BoardDesc;
  MV_SOC_COMPHY_DESC *SoCDesc;
  EFI_STATUS Status;

  /* Get SoC data about all available COMPHY controllers */
  Status = ArmadaSoCDescComPhyGet (&SoCDesc, &ComPhyCount);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  /* Obtain table with enabled COMPHY controllers */
  ComPhyDeviceTable = (UINT8 *)PcdGetPtr (PcdComPhyDevices);
  if (ComPhyDeviceTable == NULL) {
    /* No COMPHY controllers declared */
    return EFI_NOT_FOUND;
  }

  ComPhyDeviceTableSize = PcdGetSize (PcdComPhyDevices);

  /* Check if PCD with COMPHY NICs is correctly defined */
  if (ComPhyDeviceTableSize > ComPhyCount) {
    DEBUG ((DEBUG_ERROR, "%a: Wrong PcdComPhyDevices format\n", __FUNCTION__));
    return EFI_INVALID_PARAMETER;
  }

  /* Allocate and fill board description */
  BoardDesc = AllocateZeroPool (ComPhyDeviceTableSize * sizeof (MV_BOARD_COMPHY_DESC));
  if (BoardDesc == NULL) {
    DEBUG ((DEBUG_ERROR, "%a: Cannot allocate memory\n", __FUNCTION__));
    return EFI_OUT_OF_RESOURCES;
  }

  ComPhyIndex = 0;
  for (Index = 0; Index < ComPhyDeviceTableSize; Index++) {
    if (!MVHW_DEV_ENABLED (ComPhy, Index)) {
      DEBUG ((DEBUG_ERROR, "%a: Skip ComPhy controller %d\n", __FUNCTION__, Index));
      continue;
    }

    BoardDesc[ComPhyIndex].SoC = &SoCDesc[Index];
    ComPhyIndex++;
  }

  BoardDesc->ComPhyDevCount = ComPhyIndex;

  *ComPhyDesc = BoardDesc;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvBoardDescGpioGet (
  IN MARVELL_BOARD_DESC_PROTOCOL  *This,
  IN OUT MVHW_GPIO_DESC          **GpioDesc
  )
{
  /*
   * Get SoC data about all available GPIO controllers and
   * and pass to further without any additonal processing.
   */
  return ArmadaSoCDescGpioGet (GpioDesc);
}

STATIC
EFI_STATUS
MvBoardDescI2cGet (
  IN MARVELL_BOARD_DESC_PROTOCOL  *This,
  IN OUT MV_BOARD_I2C_DESC       **I2cDesc
  )
{
  UINT8 *I2cDeviceTable, I2cCount;
  UINTN I2cDeviceTableSize, I2cIndex, Index;
  MV_BOARD_I2C_DESC *BoardDesc;
  MV_SOC_I2C_DESC *SoCDesc;
  EFI_STATUS Status;

  /* Get SoC data about all available I2C controllers */
  Status = ArmadaSoCDescI2cGet (&SoCDesc, &I2cCount);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  /* Obtain table with enabled I2C controllers */
  I2cDeviceTable = (UINT8 *)PcdGetPtr (PcdI2cControllers);
  if (I2cDeviceTable == NULL) {
    /* No I2C on platform */
    return EFI_SUCCESS;
  }

  I2cDeviceTableSize = PcdGetSize (PcdI2cControllers);

  /* Check if PCD with I2C controllers is correctly defined */
  if (I2cDeviceTableSize > I2cCount) {
    DEBUG ((DEBUG_ERROR, "%a: Wrong PcdI2cControllers format\n", __FUNCTION__));
    return EFI_INVALID_PARAMETER;
  }

  /* Allocate and fill board description */
  BoardDesc = AllocateZeroPool (I2cDeviceTableSize * sizeof (MV_BOARD_I2C_DESC));
  if (BoardDesc == NULL) {
    DEBUG ((DEBUG_ERROR, "%a: Cannot allocate memory\n", __FUNCTION__));
    return EFI_OUT_OF_RESOURCES;
  }

  I2cIndex = 0;
  for (Index = 0; Index < I2cDeviceTableSize; Index++) {
    if (!MVHW_DEV_ENABLED (I2c, Index)) {
      DEBUG ((DEBUG_INFO, "%a: Skip I2c controller %d\n", __FUNCTION__, Index));
      continue;
    }

    BoardDesc[I2cIndex].SoC = &SoCDesc[Index];
    I2cIndex++;
  }

  BoardDesc->I2cDevCount = I2cIndex;

  *I2cDesc = BoardDesc;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvBoardDescMdioGet (
  IN MARVELL_BOARD_DESC_PROTOCOL  *This,
  IN OUT MV_BOARD_MDIO_DESC      **MdioDesc
  )
{
  MV_BOARD_MDIO_DESC *BoardDesc;
  MV_SOC_MDIO_DESC *SoCDesc;
  EFI_STATUS Status;
  UINT8 MdioCount;
  UINTN Index;

  /* Get SoC data about all available MDIO controllers */
  Status = ArmadaSoCDescMdioGet (&SoCDesc, &MdioCount);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  /* Allocate and fill board description */
  BoardDesc = AllocateZeroPool (MdioCount * sizeof (MV_BOARD_MDIO_DESC));
  if (BoardDesc == NULL) {
    DEBUG ((DEBUG_ERROR, "%a: Cannot allocate memory\n", __FUNCTION__));
    return EFI_OUT_OF_RESOURCES;
  }

  for (Index = 0; Index < MdioCount; Index++) {
    BoardDesc[Index].SoC = &SoCDesc[Index];
  }

  BoardDesc->MdioDevCount = MdioCount;
  *MdioDesc = BoardDesc;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvBoardDescAhciGet (
  IN MARVELL_BOARD_DESC_PROTOCOL  *This,
  IN OUT MV_BOARD_AHCI_DESC      **AhciDesc
  )
{
  UINT8 *AhciDeviceTable, AhciCount;
  UINTN AhciDeviceTableSize, AhciIndex, Index;
  MV_BOARD_AHCI_DESC *BoardDesc;
  MV_SOC_AHCI_DESC *SoCDesc;
  EFI_STATUS Status;

  /* Get SoC data about all available AHCI controllers */
  Status = ArmadaSoCDescAhciGet (&SoCDesc, &AhciCount);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  /* Obtain table with enabled AHCI controllers */
  AhciDeviceTable = (UINT8 *)PcdGetPtr (PcdPciEAhci);
  if (AhciDeviceTable == NULL) {
    /* No AHCI on platform */
    return EFI_SUCCESS;
  }

  AhciDeviceTableSize = PcdGetSize (PcdPciEAhci);

  /* Check if PCD with AHCI controllers is correctly defined */
  if (AhciDeviceTableSize > AhciCount) {
    DEBUG ((DEBUG_ERROR, "%a: Wrong PcdPciEAhci format\n", __FUNCTION__));
    return EFI_INVALID_PARAMETER;
  }

  /* Allocate and fill board description */
  BoardDesc = AllocateZeroPool (AhciDeviceTableSize * sizeof (MV_BOARD_AHCI_DESC));
  if (BoardDesc == NULL) {
    DEBUG ((DEBUG_ERROR, "%a: Cannot allocate memory\n", __FUNCTION__));
    return EFI_OUT_OF_RESOURCES;
  }

  AhciIndex = 0;
  for (Index = 0; Index < AhciDeviceTableSize; Index++) {
    if (!MVHW_DEV_ENABLED (Ahci, Index)) {
      DEBUG ((DEBUG_INFO, "%a: Skip Ahci controller %d\n", __FUNCTION__, Index));
      continue;
    }

    BoardDesc[AhciIndex].SoC = &SoCDesc[Index];
    AhciIndex++;
  }

  BoardDesc->AhciDevCount = AhciIndex;

  *AhciDesc = BoardDesc;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvBoardDescSdMmcGet (
  IN MARVELL_BOARD_DESC_PROTOCOL  *This,
  IN OUT MV_BOARD_SDMMC_DESC     **SdMmcDesc
  )
{
  UINT8 *SdMmcDeviceTable, SdMmcCount;
  UINTN SdMmcDeviceTableSize, SdMmcIndex, Index;
  MV_BOARD_SDMMC_DESC *BoardDesc;
  MV_SOC_SDMMC_DESC *SoCDesc;
  EFI_STATUS Status;

  /* Get SoC data about all available SDMMC controllers */
  Status = ArmadaSoCDescSdMmcGet (&SoCDesc, &SdMmcCount);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  /* Obtain table with enabled SDMMC controllers */
  SdMmcDeviceTable = (UINT8 *)PcdGetPtr (PcdPciESdhci);
  if (SdMmcDeviceTable == NULL) {
    /* No SDMMC on platform */
    return EFI_SUCCESS;
  }

  SdMmcDeviceTableSize = PcdGetSize (PcdPciESdhci);

  /* Check if PCD with SDMMC controllers is correctly defined */
  if (SdMmcDeviceTableSize > SdMmcCount) {
    DEBUG ((DEBUG_ERROR, "%a: Wrong PcdPciESdhci format\n", __FUNCTION__));
    return EFI_INVALID_PARAMETER;
  }

  /* Allocate and fill board description */
  BoardDesc = AllocateZeroPool (SdMmcDeviceTableSize * sizeof (MV_BOARD_SDMMC_DESC));
  if (BoardDesc == NULL) {
    DEBUG ((DEBUG_ERROR, "%a: Cannot allocate memory\n", __FUNCTION__));
    return EFI_OUT_OF_RESOURCES;
  }

  SdMmcIndex = 0;
  for (Index = 0; Index < SdMmcDeviceTableSize; Index++) {
    if (!MVHW_DEV_ENABLED (SdMmc, Index)) {
      DEBUG ((DEBUG_INFO, "%a: Skip SdMmc controller %d\n", __FUNCTION__, Index));
      continue;
    }

    BoardDesc[SdMmcIndex].SoC = &SoCDesc[Index];
    SdMmcIndex++;
  }

  BoardDesc->SdMmcDevCount = SdMmcIndex;

  *SdMmcDesc = BoardDesc;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvBoardDescXhciGet (
  IN MARVELL_BOARD_DESC_PROTOCOL  *This,
  IN OUT MV_BOARD_XHCI_DESC      **XhciDesc
  )
{
  UINT8 *XhciDeviceTable, XhciCount;
  UINTN XhciDeviceTableSize, XhciIndex, Index;
  MV_BOARD_XHCI_DESC *BoardDesc;
  MV_SOC_XHCI_DESC *SoCDesc;
  EFI_STATUS Status;

  /* Get SoC data about all available XHCI controllers */
  Status = ArmadaSoCDescXhciGet (&SoCDesc, &XhciCount);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  /* Obtain table with enabled XHCI controllers */
  XhciDeviceTable = (UINT8 *)PcdGetPtr (PcdPciEXhci);
  if (XhciDeviceTable == NULL) {
    /* No XHCI on platform */
    return EFI_SUCCESS;
  }

  XhciDeviceTableSize = PcdGetSize (PcdPciEXhci);

  /* Check if PCD with XHCI controllers is correctly defined */
  if (XhciDeviceTableSize > XhciCount) {
    DEBUG ((DEBUG_ERROR, "%a: Wrong PcdPciEXhci format\n", __FUNCTION__));
    return EFI_INVALID_PARAMETER;
  }

  /* Allocate and fill board description */
  BoardDesc = AllocateZeroPool (XhciDeviceTableSize * sizeof (MV_BOARD_XHCI_DESC));
  if (BoardDesc == NULL) {
    DEBUG ((DEBUG_ERROR, "%a: Cannot allocate memory\n", __FUNCTION__));
    return EFI_OUT_OF_RESOURCES;
  }

  XhciIndex = 0;
  for (Index = 0; Index < XhciDeviceTableSize; Index++) {
    if (!MVHW_DEV_ENABLED (Xhci, Index)) {
      DEBUG ((DEBUG_INFO, "%a: Skip Xhci controller %d\n", __FUNCTION__, Index));
      continue;
    }

    BoardDesc[XhciIndex].SoC = &SoCDesc[Index];
    XhciIndex++;
  }

  BoardDesc->XhciDevCount = XhciIndex;

  *XhciDesc = BoardDesc;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvBoardDescPp2Get (
  IN MARVELL_BOARD_DESC_PROTOCOL  *This,
  IN OUT MV_BOARD_PP2_DESC       **Pp2Desc
  )
{
  UINT8 *Pp2DeviceTable, Pp2Count;
  UINTN Pp2DeviceTableSize, Pp2Index, Index;
  MV_BOARD_PP2_DESC *BoardDesc;
  MV_SOC_PP2_DESC *SoCDesc;
  EFI_STATUS Status;

  /* Get SoC data about all available PP2 controllers */
  Status = ArmadaSoCDescPp2Get (&SoCDesc, &Pp2Count);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  /* Obtain table with enabled PP2 NICs */
  Pp2DeviceTable = (UINT8 *)PcdGetPtr (PcdPp2Controllers);
  if (Pp2DeviceTable == NULL) {
    /* No PP2 NIC on platform */
    return EFI_SUCCESS;
  }

  Pp2DeviceTableSize = PcdGetSize (PcdPp2Controllers);

  /* Check if PCD with PP2 NICs is correctly defined */
  if (Pp2DeviceTableSize > Pp2Count) {
    DEBUG ((DEBUG_ERROR, "%a: Wrong PcdPp2Controllers format\n", __FUNCTION__));
    return EFI_INVALID_PARAMETER;
  }

  /* Allocate and fill board description */
  BoardDesc = AllocateZeroPool (Pp2DeviceTableSize * sizeof (MV_BOARD_PP2_DESC));
  if (BoardDesc == NULL) {
    DEBUG ((DEBUG_ERROR, "%a: Cannot allocate memory\n", __FUNCTION__));
    return EFI_OUT_OF_RESOURCES;
  }

  Pp2Index = 0;
  for (Index = 0; Index < Pp2DeviceTableSize; Index++) {
    if (!MVHW_DEV_ENABLED (Pp2, Index)) {
      DEBUG ((DEBUG_ERROR, "%a: Skip Pp2 controller %d\n", __FUNCTION__, Index));
      continue;
    }

    BoardDesc[Pp2Index].SoC = &SoCDesc[Index];
    Pp2Index++;
  }

  BoardDesc->Pp2DevCount = Pp2Index;

  *Pp2Desc = BoardDesc;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvBoardDescUtmiGet (
  IN MARVELL_BOARD_DESC_PROTOCOL  *This,
  IN OUT MV_BOARD_UTMI_DESC      **UtmiDesc
  )
{
  UINT8 *UtmiDeviceTable, *XhciDeviceTable, *UtmiPortType, UtmiCount;
  UINTN UtmiDeviceTableSize, UtmiIndex, Index;
  MV_BOARD_UTMI_DESC *BoardDesc;
  MV_SOC_UTMI_DESC *SoCDesc;
  EFI_STATUS Status;

  /* Get SoC data about all available UTMI controllers */
  Status = ArmadaSoCDescUtmiGet (&SoCDesc, &UtmiCount);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  /* Obtain table with enabled Utmi PHY's */
  UtmiDeviceTable = (UINT8 *)PcdGetPtr (PcdUtmiControllers);
  if (UtmiDeviceTable == NULL) {
    /* No UTMI PHY on platform */
    return EFI_SUCCESS;
  }

  /* Make sure XHCI controllers table is present */
  XhciDeviceTable = (UINT8 *)PcdGetPtr (PcdPciEXhci);
  if (XhciDeviceTable == NULL) {
    DEBUG ((DEBUG_ERROR, "%a: Missing PcdPciEXhci\n", __FUNCTION__));
    return EFI_INVALID_PARAMETER;
  }

  UtmiDeviceTableSize = PcdGetSize (PcdUtmiControllers);

  /* Check if PCD with UTMI PHYs is correctly defined */
  if (UtmiDeviceTableSize > UtmiCount ||
      UtmiDeviceTableSize > PcdGetSize (PcdPciEXhci)) {
    DEBUG ((DEBUG_ERROR, "%a: Wrong PcdUtmiControllers format\n", __FUNCTION__));
    return EFI_INVALID_PARAMETER;
  }

  /* Obtain port type table */
  UtmiPortType = (UINT8 *)PcdGetPtr (PcdUtmiPortType);
  if (UtmiPortType == NULL ||
      PcdGetSize (PcdUtmiPortType) != UtmiDeviceTableSize) {
    DEBUG ((DEBUG_ERROR, "%a: Wrong PcdUtmiPortType format\n", __FUNCTION__));
    return EFI_INVALID_PARAMETER;
  }

  /* Allocate and fill board description */
  BoardDesc = AllocateZeroPool (UtmiDeviceTableSize * sizeof (MV_BOARD_UTMI_DESC));
  if (BoardDesc == NULL) {
    DEBUG ((DEBUG_ERROR, "%a: Cannot allocate memory\n", __FUNCTION__));
    return EFI_OUT_OF_RESOURCES;
  }

  UtmiIndex = 0;
  for (Index = 0; Index < UtmiDeviceTableSize; Index++) {
    if (!MVHW_DEV_ENABLED (Utmi, Index)) {
      continue;
    }

    /* UTMI PHY without enabled XHCI controller is useless */
    if (!MVHW_DEV_ENABLED (Xhci, Index)) {
      DEBUG ((DEBUG_ERROR,
             "%a: Disabled Xhci controller %d\n",
             Index,
             __FUNCTION__));
      return EFI_INVALID_PARAMETER;
    }

    BoardDesc[UtmiIndex].SoC = &SoCDesc[Index];
    BoardDesc[UtmiIndex].UtmiPortType = UtmiPortType[Index];
    UtmiIndex++;
  }

  BoardDesc->UtmiDevCount = UtmiIndex;

  *UtmiDesc = BoardDesc;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvBoardDescPcieGet (
  IN MARVELL_BOARD_DESC_PROTOCOL  *This,
  IN OUT MV_BOARD_PCIE_DESC      **PcieBoardDesc
  )
{
  UINT8 Index;
  UINT8 *PcieDeviceTable;

  MV_BOARD_PCIE_DEV_DESC *PcieDevDesc;
  MV_BOARD_PCIE_DEV_DESC *PcieDevDescTmp;
  UINTN *PcieRegBase;
  UINT8 PcieDevNum = 0;
  UINT8 PcieEnabledDevNum = 0;
  UINT8 PcieDevIndex = 0;

  /*
   * Get SoC data about all available PCIE controllers, and
   * get the enabled ones from PCD, then get other configurations
   * from BoardDescLib.
   */
  ArmadaSoCDescPcieGet (&PcieDevNum, &PcieRegBase);
  ArmadaBoardDescPcieGet (&PcieEnabledDevNum, &PcieDevDesc);

  /* Obtain table with enabled PCIe devices */
  PcieDeviceTable = (UINT8 *)PcdGetPtr (PcdPcieControllersEnabled);
  if (PcieDeviceTable == NULL) {
    DEBUG ((DEBUG_ERROR, "%a: Missing PcdPcieControllersEnabled\n", __FUNCTION__));
    return EFI_INVALID_PARAMETER;
  }

  /* Allocate and fill board description */
  *PcieBoardDesc = AllocateZeroPool (sizeof (MV_BOARD_PCIE_DESC));
  if (*PcieBoardDesc == NULL) {
    DEBUG ((DEBUG_ERROR, "%a: Cannot allocate memory\n", __FUNCTION__));
    return EFI_OUT_OF_RESOURCES;
  }

  PcieDevDescTmp = PcieDevDesc;

  for (Index = 0; Index < PcieEnabledDevNum; Index++) {
    if (!MVHW_DEV_ENABLED (Pcie, Index))
      continue;

    if (PcieDevDescTmp->PcieIndex > PcieDevNum) {
      DEBUG ((DEBUG_ERROR, "%a: Wrong pcie index\n", __FUNCTION__));
      return EFI_INVALID_PARAMETER;
    }

    PcieDevDescTmp->PcieRegBase = PcieRegBase[PcieDevDescTmp->PcieIndex];
    PcieDevIndex++;
    PcieDevDescTmp++;
  }

  (*PcieBoardDesc)->PcieDevCount = PcieDevIndex;
  (*PcieBoardDesc)->PcieDevDesc = PcieDevDesc;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvBoardDescInitProtocol (
  IN MARVELL_BOARD_DESC_PROTOCOL *BoardDescProtocol
  )
{
  BoardDescProtocol->BoardDescComPhyGet = MvBoardDescComPhyGet;
  BoardDescProtocol->BoardDescGpioGet = MvBoardDescGpioGet;
  BoardDescProtocol->BoardDescI2cGet = MvBoardDescI2cGet;
  BoardDescProtocol->BoardDescMdioGet = MvBoardDescMdioGet;
  BoardDescProtocol->BoardDescAhciGet = MvBoardDescAhciGet;
  BoardDescProtocol->BoardDescSdMmcGet = MvBoardDescSdMmcGet;
  BoardDescProtocol->BoardDescXhciGet = MvBoardDescXhciGet;
  BoardDescProtocol->BoardDescPp2Get = MvBoardDescPp2Get;
  BoardDescProtocol->BoardDescUtmiGet = MvBoardDescUtmiGet;
  BoardDescProtocol->BoardDescPcieGet = MvBoardDescPcieGet;

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
MvBoardDescEntryPoint (
  IN EFI_HANDLE       ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  EFI_STATUS Status;

  mBoardDescInstance = AllocateZeroPool (sizeof (MV_BOARD_DESC));
  if (mBoardDescInstance == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  MvBoardDescInitProtocol (&mBoardDescInstance->BoardDescProtocol);

  mBoardDescInstance->Signature = BOARD_DESC_SIGNATURE;

  Status = gBS->InstallMultipleProtocolInterfaces (&(mBoardDescInstance->Handle),
                  &gMarvellBoardDescProtocolGuid,
                  &(mBoardDescInstance->BoardDescProtocol));
  if (EFI_ERROR (Status)) {
    FreePool (mBoardDescInstance);
    return Status;
  }

  return EFI_SUCCESS;
}
