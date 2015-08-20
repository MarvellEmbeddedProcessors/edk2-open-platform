/** @file
  Initialize SATA Phy, Serdes, and Controller.

  Copyright (c) 2014 - 2016, AMD Inc. All rights reserved.<BR>

  This program and the accompanying materials are licensed and made available
  under the terms and conditions of the BSD License which accompanies this
  distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
**/

#include "SataController.h"
#include <Library/IoLib.h>
#include "SataRegisters.h"
#include "PciEmulation.h"
#include <Library/AmdSataInitLib.h>

VOID
ResetSataController (
  EFI_PHYSICAL_ADDRESS    AhciBaseAddr
  )
{
  UINT32                  RegVal;

  // Make a minimal global reset for HBA regiser
  RegVal = MmioRead32(AhciBaseAddr + EFI_AHCI_GHC_OFFSET);
  RegVal |= EFI_AHCI_GHC_RESET;
  MmioWrite32(AhciBaseAddr + EFI_AHCI_GHC_OFFSET, RegVal);

  // Clear all interrupts
  MmioWrite32(AhciBaseAddr + EFI_AHCI_PORT_IS, EFI_AHCI_PORT_IS_CLEAR);

  // Turn on interrupts and ensure that the HBA is working in AHCI mode
  RegVal = MmioRead32(AhciBaseAddr + EFI_AHCI_GHC_OFFSET);
  RegVal |= EFI_AHCI_GHC_IE | EFI_AHCI_GHC_ENABLE;
  MmioWrite32(AhciBaseAddr + EFI_AHCI_GHC_OFFSET, RegVal);
}


VOID
SetSataCapabilities (
  EFI_PHYSICAL_ADDRESS    AhciBaseAddr
  )
{
  UINT32                  Capability;

  Capability = MmioRead32(AhciBaseAddr + EFI_AHCI_CAPABILITY_OFFSET);
  if (PcdGetBool(PcdSataSssSupport)) // Staggered Spin-Up Support bit
    Capability |= EFI_AHCI_CAP_SSS;
  if (PcdGetBool(PcdSataSmpsSupport)) // Mechanical Presence Support bit
    Capability |= EFI_AHCI_CAP_SMPS;
  MmioWrite32(AhciBaseAddr + EFI_AHCI_CAPABILITY_OFFSET, Capability);
}


VOID
InitializeSataPorts (
  EFI_PHYSICAL_ADDRESS    AhciBaseAddr
  )
{
  INTN                    PortNum;
  UINT32                  SataPi;
  BOOLEAN                 IsPortImplemented;
  BOOLEAN                 IsCpd;
  BOOLEAN                 IsMpsp;
  UINT32                  PortRegAddr;
  UINT32                  RegVal;

  // Set Ports Implemented (PI)
  SataPi = PcdGet32(PcdSataPi);
  MmioWrite32(AhciBaseAddr + EFI_AHCI_PI_OFFSET, SataPi);

  IsCpd = PcdGetBool(PcdSataPortCpd);
  IsMpsp = PcdGetBool(PcdSataPortMpsp);
  for (PortNum = 0; PortNum < EFI_AHCI_MAX_PORTS; PortNum++) {
    IsPortImplemented = (SataPi & (1 << PortNum)) ? 1 : 0;
    if (IsPortImplemented && (IsCpd || IsMpsp)) {
      PortRegAddr = EFI_AHCI_PORT_OFFSET(PortNum) + EFI_AHCI_PORT_CMD;
      RegVal = MmioRead32(AhciBaseAddr + PortRegAddr);
      if (IsCpd)
        RegVal |= EFI_AHCI_PORT_CMD_CPD;
      else
        RegVal &= ~EFI_AHCI_PORT_CMD_CPD;
      if (IsMpsp)
        RegVal |= EFI_AHCI_PORT_CMD_MPSP;
      else
        RegVal &= ~EFI_AHCI_PORT_CMD_MPSP;
      RegVal |= EFI_AHCI_PORT_CMD_HPCP;
      MmioWrite32(AhciBaseAddr + PortRegAddr, RegVal);
    }
  }
}


EFI_STATUS
InitializeSataController (
  VOID
  )
{
  EFI_PHYSICAL_ADDRESS    AhciBaseAddr;
  UINT8              SataPortCount;
  UINT8              SataChPerSerdes;
  UINT32             StartPort;
  UINT32             PortNum;
  UINT32             EvenPort;
  UINT32             OddPort;
  EFI_STATUS         Status;

#ifdef BUILD_FOR_SATA1
  AhciBaseAddr = PcdGet32(PcdSata1CtrlAxiSlvPort);
  SataPortCount = PcdGet8(PcdSata1PortCount);
  StartPort = PcdGet8(PcdSataPortCount);
#else
  AhciBaseAddr = PcdGet32(PcdSataCtrlAxiSlvPort);
  SataPortCount = PcdGet8(PcdSataPortCount);
  StartPort = 0;
#endif

  SataChPerSerdes = PcdGet8(PcdSataNumChPerSerdes);

  //
  // Perform SATA workarounds
  //
  for (PortNum = 0; PortNum < SataPortCount; PortNum++) {
#ifdef BUILD_FOR_SATA1
      SetCwMinSata1(PortNum);
#else
      SetCwMinSata0(PortNum);
#endif
  }

  for (PortNum = 0; PortNum < SataPortCount; PortNum += SataChPerSerdes) {
      EvenPort = (UINT32)(PcdGet16(PcdSataPortMode) >> (PortNum * 2)) & 3;
      OddPort = (UINT32)(PcdGet16(PcdSataPortMode) >> ((PortNum+1) * 2)) & 3;
      SataPhyInit((StartPort + PortNum)/SataChPerSerdes, EvenPort, OddPort);
  }

  for (PortNum = 0; PortNum < SataPortCount; PortNum++) {
#ifdef BUILD_FOR_SATA1
      SetPrdSingleSata1(PortNum);
#else
      SetPrdSingleSata0(PortNum);
#endif
  }

  //
  // Reset SATA controller
  //
  ResetSataController(AhciBaseAddr);

  //
  // Set SATA capabilities
  //
  SetSataCapabilities(AhciBaseAddr);

  //
  // Set and intialize the Sata ports
  //
  InitializeSataPorts(AhciBaseAddr);

  //
  // Emulate a PCI configuration for this device
  //
  Status = PciEmulationEntryPoint(AhciBaseAddr);
  ASSERT_EFI_ERROR (Status);

  return EFI_SUCCESS;
}

