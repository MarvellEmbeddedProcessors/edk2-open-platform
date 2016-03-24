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

#ifndef __PHY_H__
#define __PHY_H__

#define EFI_PHY_PROTOCOL_GUID { 0x0d728a1e, 0x45b5, 0x4feb, { 0x98, 0xc8, 0x31, 0x3d, 0xa8, 0x11, 0x74, 0x62 }}

typedef struct _EFI_PHY_PROTOCOL EFI_PHY_PROTOCOL;

typedef enum {
  PHY_CONNECTION_RGMII,
  PHY_CONNECTION_RGMII_ID,
  PHY_CONNECTION_RGMII_TXID,
  PHY_CONNECTION_RGMII_RXID,
  PHY_CONNECTION_SGMII,
  PHY_CONNECTION_RTBI,
  PHY_CONNECTION_XAUI,
  PHY_CONNECTION_RXAUI
} PHY_CONNECTION;

typedef enum {
  NO_SPEED,
  SPEED_10,
  SPEED_100,
  SPEED_1000
} PHY_SPEED;

typedef struct {
  UINT32          Addr;
  BOOLEAN         LinkUp;
  BOOLEAN         Duplex;
  BOOLEAN         AutoNegotiation;
  PHY_SPEED       Speed;
  PHY_CONNECTION  Connection;
} PHY_DEVICE;

typedef
EFI_STATUS
(EFIAPI *EFI_PHY_STATUS) (
  IN CONST EFI_PHY_PROTOCOL *This,
  IN PHY_DEVICE  *PhyDev
  );

typedef
EFI_STATUS
(EFIAPI *EFI_PHY_INIT) (
  IN CONST EFI_PHY_PROTOCOL *This,
  IN UINT32 PhyAddr,
  IN OUT PHY_DEVICE **PhyDev
  );

struct _EFI_PHY_PROTOCOL {
  EFI_PHY_STATUS Status;
  EFI_PHY_INIT Init;
};

extern EFI_GUID gEfiPhyProtocolGuid;
#endif
