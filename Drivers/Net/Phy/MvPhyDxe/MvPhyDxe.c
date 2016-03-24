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

#include <Protocol/DriverBinding.h>
#include <Protocol/Phy.h>
#include <Protocol/Mdio.h>

#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/IoLib.h>
#include <Library/DebugLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiBootServicesTableLib.h>

#include "MvPhyDxe.h"

STATIC EFI_MDIO_PROTOCOL *Mdio;

STATIC MV_PHY_DEVICE MvPhyDevices[] = {
  { MV_PHY_DEVICE_1512, MvPhyInit1512 },
  { 0, NULL }
};

EFI_STATUS
MvPhyStatus (
  IN CONST EFI_PHY_PROTOCOL *This,
  IN PHY_DEVICE  *PhyDev
  );

EFI_STATUS
MvPhyReset (
  IN UINT32 PhyAddr
  )
{
  UINT32 Reg = 0;
  INTN timeout = 500;

  Reg = Mdio->Read(Mdio, PhyAddr, MII_BMCR);
  Reg |= BMCR_RESET;
  Mdio->Write(Mdio, PhyAddr, MII_BMCR, Reg);

  while ((Reg & BMCR_RESET) && timeout--) {
    Reg = Mdio->Read(Mdio, PhyAddr, MII_BMCR);
    gBS->Stall(1000);
  }

  if (Reg & BMCR_RESET) {
    DEBUG((DEBUG_ERROR, "PHY reset timed out\n"));
    return EFI_TIMEOUT;
  }

  return EFI_SUCCESS;
}

/* Marvell 88E1111S */
EFI_STATUS
MvPhyM88e1111sConfig (
  IN PHY_DEVICE *PhyDev
  )
{
  UINT32 Reg;

  if ((PhyDev->Connection == PHY_INTERFACE_MODE_RGMII) ||
      (PhyDev->Connection == PHY_INTERFACE_MODE_RGMII_ID) ||
      (PhyDev->Connection == PHY_INTERFACE_MODE_RGMII_RXID) ||
      (PhyDev->Connection == PHY_INTERFACE_MODE_RGMII_TXID)) {
    Reg = Mdio->Read(Mdio, PhyDev->Addr,
      MIIM_88E1111_PHY_EXT_CR);
    if ((PhyDev->Connection == PHY_INTERFACE_MODE_RGMII) ||
      (PhyDev->Connection == PHY_INTERFACE_MODE_RGMII_ID)) {
      Reg |= (MIIM_88E1111_RX_DELAY | MIIM_88E1111_TX_DELAY);
    } else if (PhyDev->Connection == PHY_INTERFACE_MODE_RGMII_RXID) {
      Reg &= ~MIIM_88E1111_TX_DELAY;
      Reg |= MIIM_88E1111_RX_DELAY;
    } else if (PhyDev->Connection == PHY_INTERFACE_MODE_RGMII_TXID) {
      Reg &= ~MIIM_88E1111_RX_DELAY;
      Reg |= MIIM_88E1111_TX_DELAY;
    }

    Mdio->Write(Mdio, PhyDev->Addr,
      MIIM_88E1111_PHY_EXT_CR, Reg);

    Reg = Mdio->Read(Mdio, PhyDev->Addr,
      MIIM_88E1111_PHY_EXT_SR);

    Reg &= ~(MIIM_88E1111_HWCFG_MODE_MASK);

    if (Reg & MIIM_88E1111_HWCFG_FIBER_COPPER_RES)
      Reg |= MIIM_88E1111_HWCFG_MODE_FIBER_RGMII;
    else
      Reg |= MIIM_88E1111_HWCFG_MODE_COPPER_RGMII;

    Mdio->Write(Mdio, PhyDev->Addr,
      MIIM_88E1111_PHY_EXT_SR, Reg);
  }

  if (PhyDev->Connection == PHY_INTERFACE_MODE_SGMII) {
    Reg = Mdio->Read(Mdio, PhyDev->Addr,
      MIIM_88E1111_PHY_EXT_SR);

    Reg &= ~(MIIM_88E1111_HWCFG_MODE_MASK);
    Reg |= MIIM_88E1111_HWCFG_MODE_SGMII_NO_CLK;
    Reg |= MIIM_88E1111_HWCFG_FIBER_COPPER_AUTO;

    Mdio->Write(Mdio, PhyDev->Addr,
      MIIM_88E1111_PHY_EXT_SR, Reg);
  }

  if (PhyDev->Connection == PHY_INTERFACE_MODE_RTBI) {
    Reg = Mdio->Read(Mdio, PhyDev->Addr,
      MIIM_88E1111_PHY_EXT_CR);
    Reg |= (MIIM_88E1111_RX_DELAY | MIIM_88E1111_TX_DELAY);
    Mdio->Write(Mdio, PhyDev->Addr,
      MIIM_88E1111_PHY_EXT_CR, Reg);

    Reg = Mdio->Read(Mdio, PhyDev->Addr,
      MIIM_88E1111_PHY_EXT_SR);
    Reg &= ~(MIIM_88E1111_HWCFG_MODE_MASK |
      MIIM_88E1111_HWCFG_FIBER_COPPER_RES);
    Reg |= 0x7 | MIIM_88E1111_HWCFG_FIBER_COPPER_AUTO;
    Mdio->Write(Mdio, PhyDev->Addr,
      MIIM_88E1111_PHY_EXT_SR, Reg);

    /* soft reset */
    MvPhyReset(PhyDev->Addr);

    Reg = Mdio->Read(Mdio, PhyDev->Addr,
      MIIM_88E1111_PHY_EXT_SR);
    Reg &= ~(MIIM_88E1111_HWCFG_MODE_MASK |
      MIIM_88E1111_HWCFG_FIBER_COPPER_RES);
    Reg |= MIIM_88E1111_HWCFG_MODE_COPPER_RTBI |
      MIIM_88E1111_HWCFG_FIBER_COPPER_AUTO;
    Mdio->Write(Mdio, PhyDev->Addr,
      MIIM_88E1111_PHY_EXT_SR, Reg);
  }

  Reg = Mdio->Read(Mdio, PhyDev->Addr, MII_BMCR);
  Reg |= (BMCR_ANENABLE | BMCR_ANRESTART);
  Reg &= ~BMCR_ISOLATE;
  Mdio->Write(Mdio, PhyDev->Addr, MII_BMCR, Reg);

  /* soft reset */
  MvPhyReset(PhyDev->Addr);

  MvPhyReset(PhyDev->Addr);

  return EFI_SUCCESS;
}

EFI_STATUS
MvPhyParseStatus (
  IN PHY_DEVICE *PhyDev
  )
{
  UINT32 Data;
  UINT32 Speed;

  Data = Mdio->Read(Mdio, PhyDev->Addr, MIIM_88E1xxx_PHY_STATUS);

  if ((Data & MIIM_88E1xxx_PHYSTAT_LINK) &&
    !(Data & MIIM_88E1xxx_PHYSTAT_SPDDONE)) {
    INTN i = 0;

    DEBUG((DEBUG_ERROR,"MvPhyDxe: Waiting for PHY realtime link"));
    while (!(Data & MIIM_88E1xxx_PHYSTAT_SPDDONE)) {
      if (i > PHY_AUTONEGOTIATE_TIMEOUT) {
        DEBUG((DEBUG_ERROR," TIMEOUT !\n"));
        PhyDev->LinkUp = FALSE;
        break;
      }

      if ((i++ % 1000) == 0)
        DEBUG((DEBUG_ERROR, "."));
      gBS->Stall(1000);
      Data = Mdio->Read(Mdio, PhyDev->Addr,
          MIIM_88E1xxx_PHY_STATUS);
    }
    DEBUG((DEBUG_ERROR," done\n"));
    gBS->Stall(500000);
  } else {
    if (Data & MIIM_88E1xxx_PHYSTAT_LINK) {
      DEBUG((DEBUG_ERROR, "MvPhyDxe: link up, "));
      PhyDev->LinkUp = TRUE;
    } else {
      DEBUG((DEBUG_ERROR, "MvPhyDxe: link down, "));
      PhyDev->LinkUp = FALSE;
    }
  }

  if (Data & MIIM_88E1xxx_PHYSTAT_DUPLEX) {
    DEBUG((DEBUG_ERROR, "full duplex, "));
    PhyDev->FullDuplex = TRUE;
  } else {
    DEBUG((DEBUG_ERROR, "half duplex, "));
    PhyDev->FullDuplex = FALSE;
  }

  Speed = Data & MIIM_88E1xxx_PHYSTAT_SPEED;

  switch (Speed) {
  case MIIM_88E1xxx_PHYSTAT_GBIT:
    DEBUG((DEBUG_ERROR, "speed 1000\n"));
    PhyDev->Speed = SPEED_1000;
    break;
  case MIIM_88E1xxx_PHYSTAT_100:
    DEBUG((DEBUG_ERROR, "speed 100\n"));
    PhyDev->Speed = SPEED_100;
    break;
  default:
    DEBUG((DEBUG_ERROR, "speed 10\n"));
    PhyDev->Speed = SPEED_10;
    break;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MvPhyInit1512 (
    IN CONST EFI_PHY_PROTOCOL *Snp,
    IN UINT32 PhyAddr,
    IN OUT PHY_DEVICE *PhyDev
    )
{
  UINT32 Data;
  INTN i;

  MvPhyM88e1111sConfig (PhyDev);

  /* autonegotiation on startup is not always required */
  if (!PcdGetBool (PcdPhyStartupAutoneg))
    return EFI_SUCCESS;

  Data = Mdio->Read(Mdio, PhyAddr, MII_BMSR);

  if ((Data & BMSR_ANEGCAPABLE) && !(Data & BMSR_ANEGCOMPLETE)) {

    DEBUG((DEBUG_ERROR, "MvPhyDxe: Waiting for PHY auto negotiation... "));
    for (i = 0; !(Data & BMSR_ANEGCOMPLETE); i++) {
      if (i > PHY_ANEG_TIMEOUT) {
        DEBUG((DEBUG_ERROR, "timeout\n"));
        PhyDev->LinkUp = FALSE;
        return EFI_TIMEOUT;
      }

      gBS->Stall(1000);  /* 1 ms */
      Data = Mdio->Read(Mdio, PhyAddr, MII_BMSR);
    }
    PhyDev->LinkUp = TRUE;
    DEBUG((DEBUG_INFO, "MvPhyDxe: link up\n"));
  } else {
    Data = Mdio->Read(Mdio, PhyAddr, MII_BMSR);

    if (Data & BMSR_LSTATUS) {
      PhyDev->LinkUp = TRUE;
      DEBUG((DEBUG_INFO, "MvPhyDxe: link up\n"));
    } else {
      PhyDev->LinkUp = FALSE;
      DEBUG((DEBUG_INFO, "MvPhyDxe: link down\n"));
    }
  }
  MvPhyParseStatus (PhyDev);

  return EFI_SUCCESS;
}

EFI_STATUS
MvPhyInit (
  IN CONST EFI_PHY_PROTOCOL *Snp,
  IN UINT32 PhyAddr,
  IN OUT PHY_DEVICE **OutPhyDev
  )
{
  EFI_STATUS Status;
  PHY_DEVICE *PhyDev;
  UINT8 *ConnectionTypes;
  UINT8 *DeviceIds;
  INTN i;

  Status = gBS->LocateProtocol (
      &gEfiMdioProtocolGuid,
      NULL,
      (VOID **) &Mdio
      );
  if (EFI_ERROR(Status))
    return Status;

  /* perform setup common for all PHYs */
  PhyDev = AllocateZeroPool (sizeof (PHY_DEVICE));
  PhyDev->Addr = PhyAddr;
  DEBUG((DEBUG_INFO, "MvPhyDxe: PhyAddr is %d\n", PhyAddr));
  ConnectionTypes = PcdGetPtr (PcdPhyConnectionTypes);
  if (PhyAddr >= PcdGetSize (PcdPhyConnectionTypes)) {
    DEBUG((DEBUG_ERROR, "MvPhyDxe: wrong phy address\n"));
    Status = EFI_INVALID_PARAMETER;
    goto error;
  }
  PhyDev->Connection = ConnectionTypes[PhyAddr];
  *OutPhyDev = PhyDev;

  DeviceIds = PcdGetPtr (PcdPhyDeviceIds);
  for (i = 0; i < PcdGetSize (PcdPhyDeviceIds); i++) {
    /* find MvPhyDevices fitting entry */
    if (MvPhyDevices[i].DevId == DeviceIds[i]) {
      ASSERT (MvPhyDevices[i].DevInit != NULL);
      /* proceed with PHY-specific initialization */
      return MvPhyDevices[i].DevInit(Snp, PhyAddr, PhyDev);
    }
  }

  /* if we are here, no matching DevId was found */
  Status = EFI_INVALID_PARAMETER;
error:
  FreePool (PhyDev);
  return Status;
}

EFI_STATUS
MvPhyStatus (
  IN CONST EFI_PHY_PROTOCOL *This,
  IN PHY_DEVICE  *PhyDev
  )
{
  UINT32 Data;

  Data = Mdio->Read(Mdio, PhyDev->Addr, MII_BMSR);
  Data = Mdio->Read(Mdio, PhyDev->Addr, MII_BMSR);

  if ((Data & BMSR_LSTATUS) == 0) {
    PhyDev->LinkUp = FALSE;
  } else {
    PhyDev->LinkUp = TRUE;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
MvPhyDxeInitialise (
  IN EFI_HANDLE  ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_PHY_PROTOCOL *Phy;
  EFI_STATUS Status;
  EFI_HANDLE Handle = NULL;

  Phy = AllocateZeroPool (sizeof (EFI_PHY_PROTOCOL));
  Phy->Status = MvPhyStatus;
  Phy->Init = MvPhyInit;

  Status = gBS->InstallMultipleProtocolInterfaces (
                  &Handle,
                  &gEfiPhyProtocolGuid, Phy,
                  NULL
                  );

  if (EFI_ERROR(Status)) {
    DEBUG((DEBUG_ERROR, "Failed to install interfaces\n"));
    return Status;
  }
  DEBUG((DEBUG_ERROR, "Succesfully installed protocol interfaces\n"));

  return EFI_SUCCESS;
}
