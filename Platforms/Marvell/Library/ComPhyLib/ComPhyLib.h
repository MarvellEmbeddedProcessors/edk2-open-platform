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

#ifndef __COMPHY_H__
#define __COMPHY_H__

#include <Uefi.h>
#include <Library/ArmLib.h>
#include <Library/ArmPlatformLib.h>
#include <Library/DebugLib.h>
#include <Library/PcdLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/MvComPhyLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include <Library/UefiBootServicesTableLib.h>

#include <Protocol/BoardDesc.h>

#define MAX_LANE_OPTIONS          10

/***** Parsing PCD *****/
#define GET_LANE_TYPE(id)         PcdGetPtr(PcdChip##id##ComPhyTypes)
#define GET_LANE_SPEED(id)        PcdGetPtr(PcdChip##id##ComPhySpeeds)
#define GET_LANE_INV(id)          PcdGetPtr(PcdChip##id##ComPhyInvFlags)

#define GetComPhyPcd(lane_struct, id) {                      \
  lane_struct[id].Type = (UINT8 *)GET_LANE_TYPE(id);         \
  lane_struct[id].SpeedValue = (UINT8 *)GET_LANE_SPEED(id);  \
  lane_struct[id].InvFlag = (UINT8 *)GET_LANE_SPEED(id);     \
}

#define COMPHY_SPEED_1_25G                           0
#define COMPHY_SPEED_2_5G                            1
#define COMPHY_SPEED_3_125G                          2
#define COMPHY_SPEED_5G                              3
#define COMPHY_SPEED_5_15625G                        4
#define COMPHY_SPEED_6G                              5
#define COMPHY_SPEED_10_3125G                        6
#define COMPHY_SPEED_MAX                             7
#define COMPHY_SPEED_INVALID                         0xff
/* The  default speed for IO with fixed known speed */
#define COMPHY_SPEED_DEFAULT                         0x3F

#define COMPHY_TYPE_UNCONNECTED                      0
#define COMPHY_TYPE_PCIE0                            1
#define COMPHY_TYPE_PCIE1                            2
#define COMPHY_TYPE_PCIE2                            3
#define COMPHY_TYPE_PCIE3                            4
#define COMPHY_TYPE_SATA0                            5
#define COMPHY_TYPE_SATA1                            6
#define COMPHY_TYPE_SATA2                            7
#define COMPHY_TYPE_SATA3                            8
#define COMPHY_TYPE_SGMII0                           9
#define COMPHY_TYPE_SGMII1                           10
#define COMPHY_TYPE_SGMII2                           11
#define COMPHY_TYPE_SGMII3                           12
#define COMPHY_TYPE_QSGMII                           13
#define COMPHY_TYPE_USB3_HOST0                       14
#define COMPHY_TYPE_USB3_HOST1                       15
#define COMPHY_TYPE_USB3_DEVICE                      16
#define COMPHY_TYPE_XAUI0                            17
#define COMPHY_TYPE_XAUI1                            18
#define COMPHY_TYPE_XAUI2                            19
#define COMPHY_TYPE_XAUI3                            20
#define COMPHY_TYPE_RXAUI0                           21
#define COMPHY_TYPE_RXAUI1                           22
#define COMPHY_TYPE_SFI                              23
#define COMPHY_TYPE_MAX                              24
#define COMPHY_TYPE_INVALID                          0xff

#define COMPHY_SATA_MODE            0x1
#define COMPHY_SGMII_MODE           0x2   /* SGMII 1G */
#define COMPHY_HS_SGMII_MODE        0x3   /* SGMII 2.5G */
#define COMPHY_USB3H_MODE           0x4
#define COMPHY_USB3D_MODE           0x5
#define COMPHY_PCIE_MODE            0x6
#define COMPHY_RXAUI_MODE           0x7
#define COMPHY_XFI_MODE             0x8
#define COMPHY_SFI_MODE             0x9
#define COMPHY_USB3_MODE            0xa
#define COMPHY_AP_MODE              0xb

/* Comphy unit index macro */
#define COMPHY_UNIT_ID0         0
#define COMPHY_UNIT_ID1         1
#define COMPHY_UNIT_ID2         2
#define COMPHY_UNIT_ID3         3

/* Firmware related definitions used for SMC calls */
#define MV_SIP_CPMPHY_POWER_ON      0x82000001
#define MV_SIP_CPMPHY_POWER_OFF     0x82000002
#define MV_SIP_COMPHY_PLL_LOCK      0x82000003

#define COMPHY_FW_FORMAT(mode, idx, speeds)  (((mode) << 12) | ((idx) << 8) | ((speeds) << 2))
#define COMPHY_FW_PCIE_FORMAT(pcie_width, mode, idx, speeds) \
                    (((pcie_width) << 18) | COMPHY_FW_FORMAT(mode, idx, speeds))


/***** SATA registers *****/
#define SATA3_VENDOR_ADDRESS                      0xA0
#define SATA3_VENDOR_ADDR_OFSSET                  0
#define SATA3_VENDOR_ADDR_MASK                    (0xFFFFFFFF << SATA3_VENDOR_ADDR_OFSSET)
#define SATA3_VENDOR_DATA                         0xA4

#define SATA_CONTROL_REG                          0x0
#define SATA3_CTRL_SATA0_PD_OFFSET                6
#define SATA3_CTRL_SATA0_PD_MASK                  (1 << SATA3_CTRL_SATA0_PD_OFFSET)
#define SATA3_CTRL_SATA1_PD_OFFSET                14
#define SATA3_CTRL_SATA1_PD_MASK                  (1 << SATA3_CTRL_SATA1_PD_OFFSET)
#define SATA3_CTRL_SATA1_ENABLE_OFFSET            22
#define SATA3_CTRL_SATA1_ENABLE_MASK              (1 << SATA3_CTRL_SATA1_ENABLE_OFFSET)
#define SATA3_CTRL_SATA_SSU_OFFSET                23
#define SATA3_CTRL_SATA_SSU_MASK                  (1 << SATA3_CTRL_SATA_SSU_OFFSET)

#define SATA_MBUS_SIZE_SELECT_REG                 0x4
#define SATA_MBUS_REGRET_EN_OFFSET                7
#define SATA_MBUS_REGRET_EN_MASK                  (0x1 << SATA_MBUS_REGRET_EN_OFFSET)

/***************************/

typedef struct _CHIP_COMPHY_CONFIG CHIP_COMPHY_CONFIG;

typedef struct {
  UINT32 Type;
  UINT32 MuxValue;
} COMPHY_MUX_OPTIONS;

typedef struct {
  UINT8 Type;
  UINT8 Speed;
  UINT8 Invert;
} COMPHY_MAP;

typedef struct {
  UINT8 *Type;
  UINT8 *SpeedValue;
  UINT8 *InvFlag;
} PCD_LANE_MAP;

typedef
VOID
(*COMPHY_CHIP_INIT) (
  IN CHIP_COMPHY_CONFIG *PtrChipCfg
  );

struct _CHIP_COMPHY_CONFIG {
  MV_COMPHY_CHIP_TYPE ChipType;
  COMPHY_MAP MapData[MAX_LANE_OPTIONS];
  EFI_PHYSICAL_ADDRESS ComPhyBaseAddr;
  EFI_PHYSICAL_ADDRESS Hpipe3BaseAddr;
  COMPHY_CHIP_INIT Init;
  UINT32 LanesCount;
  UINT32 MuxBitCount;
  UINT8 ChipId;
};

VOID
ComPhyMuxInit (
  IN CHIP_COMPHY_CONFIG *PtrChipCfg,
  IN COMPHY_MAP *ComPhyMapData,
  IN EFI_PHYSICAL_ADDRESS SelectorBase
  );

VOID
ComPhyCp110Init (
  IN CHIP_COMPHY_CONFIG * First
  );

VOID
RegSet (
  IN EFI_PHYSICAL_ADDRESS Addr,
  IN UINT32 Data,
  IN UINT32 Mask
  );

VOID
RegSetSilent (
  IN EFI_PHYSICAL_ADDRESS Addr,
  IN UINT32 Data,
  IN UINT32 Mask
  );

#endif // __COMPHY_H__
