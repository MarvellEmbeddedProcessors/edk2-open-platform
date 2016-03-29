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

#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/ArmLib.h>

#define ICU_REG_BASE           0x1E0000

#define ICU_AXI_ATTR           0x0
#define ICU_SET_SPI_AL(x)      (0x10 + (0x10 * x))
#define ICU_SET_SPI_AH(x)      (0x14 + (0x10 * x))
#define ICU_CLR_SPI_AL(x)      (0x18 + (0x10 * x))
#define ICU_CLR_SPI_AH(x)      (0x1c + (0x10 * x))
#define ICU_INT_CFG(x)         (0x100 + 4 * x)

#define ICU_INT_ENABLE_OFFSET  (24)
#define ICU_IS_EDGE_OFFSET     (28)
#define ICU_GROUP_OFFSET       (29)

#define NS_MULTI_IRQS          40
#define NS_SINGLE_IRQS         23
#define REI_IRQS               10
#define SEI_IRQS               20
#define MAX_ICU_IRQS           207

typedef enum {
  ICU_GRP_NSR  = 0,
  ICU_GRP_SR   = 1,
  ICU_GRP_LPI  = 2,
  ICU_GRP_VLPI = 3,
  ICU_GRP_SEI  = 4,
  ICU_GRP_REI  = 5,
  ICU_GRP_MAX,
} ICU_GROUP;

typedef struct {
  UINT8 IcuId;
  UINT8 SpiId;
  UINT8 IsEdge;
} ICU_IRQ;

typedef struct {
  ICU_GROUP Group;
  UINTN SetSpiAddr;
  UINTN ClrSpiAddr;
} ICU_MSI;

VOID
IcuInit (
  VOID
  );
