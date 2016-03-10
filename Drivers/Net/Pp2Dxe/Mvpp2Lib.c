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

#include "Mvpp2LibHw.h"
#include "Pp2Dxe.h"
#include "Mvpp2Lib.h"

/* Parser configuration routines */

/* Update parser tcam and sram hw entries */
STATIC INT32 Mvpp2PrsHwWrite(MVPP2_SHARED *priv, MVPP2_PRS_ENTRY *pe)
{
  INT32 i;

  if (pe->index > MVPP2_PRS_TCAM_SRAM_SIZE - 1)
    return MVPP2_EINVAL;

  /* Clear entry invalidation bit */
  pe->tcam.word[MVPP2_PRS_TCAM_INV_WORD] &= ~MVPP2_PRS_TCAM_INV_MASK;

  /* Write tcam index - indirect access */
  Mvpp2Write(priv, MVPP2_PRS_TCAM_IDX_REG, pe->index);
  for (i = 0; i < MVPP2_PRS_TCAM_WORDS; i++)
    Mvpp2Write(priv, MVPP2_PRS_TCAM_DATA_REG(i), pe->tcam.word[i]);

  /* Write sram index - indirect access */
  Mvpp2Write(priv, MVPP2_PRS_SRAM_IDX_REG, pe->index);
  for (i = 0; i < MVPP2_PRS_SRAM_WORDS; i++)
    Mvpp2Write(priv, MVPP2_PRS_SRAM_DATA_REG(i), pe->sram.word[i]);

  return 0;
}

/* Read tcam entry from hw */
STATIC INT32 Mvpp2PrsHwRead(MVPP2_SHARED *priv, MVPP2_PRS_ENTRY *pe)
{
  INT32 i;

  if (pe->index > MVPP2_PRS_TCAM_SRAM_SIZE - 1)
    return MVPP2_EINVAL;

  /* Write tcam index - indirect access */
  Mvpp2Write(priv, MVPP2_PRS_TCAM_IDX_REG, pe->index);

  pe->tcam.word[MVPP2_PRS_TCAM_INV_WORD] = Mvpp2Read(priv,
            MVPP2_PRS_TCAM_DATA_REG(MVPP2_PRS_TCAM_INV_WORD));
  if (pe->tcam.word[MVPP2_PRS_TCAM_INV_WORD] & MVPP2_PRS_TCAM_INV_MASK)
    return MVPP2_PRS_TCAM_ENTRY_INVALID;

  for (i = 0; i < MVPP2_PRS_TCAM_WORDS; i++)
    pe->tcam.word[i] = Mvpp2Read(priv, MVPP2_PRS_TCAM_DATA_REG(i));

  /* Write sram index - indirect access */
  Mvpp2Write(priv, MVPP2_PRS_SRAM_IDX_REG, pe->index);
  for (i = 0; i < MVPP2_PRS_SRAM_WORDS; i++)
    pe->sram.word[i] = Mvpp2Read(priv, MVPP2_PRS_SRAM_DATA_REG(i));

  return 0;
}

/* Invalidate tcam hw entry */
STATIC VOID Mvpp2PrsHwInv(MVPP2_SHARED *priv, INT32 index)
{
  /* Write index - indirect access */
  Mvpp2Write(priv, MVPP2_PRS_TCAM_IDX_REG, index);
  Mvpp2Write(priv, MVPP2_PRS_TCAM_DATA_REG(MVPP2_PRS_TCAM_INV_WORD),
        MVPP2_PRS_TCAM_INV_MASK);
}

/* Enable shadow table entry and set its lookup ID */
STATIC VOID Mvpp2PrsShadowSet(MVPP2_SHARED *priv, INT32 index, INT32 lu)
{
  priv->PrsShadow[index].valid = TRUE;
  priv->PrsShadow[index].lu = lu;
}

/* Update ri fields in shadow table entry */
STATIC VOID Mvpp2PrsShadowRiSet(MVPP2_SHARED *priv, INT32 index,
            UINT32 ri, UINT32 RiMask)
{
  priv->PrsShadow[index].RiMask = RiMask;
  priv->PrsShadow[index].ri = ri;
}

/* Update lookup field in tcam sw entry */
STATIC VOID Mvpp2PrsTcamLuSet(MVPP2_PRS_ENTRY *pe, UINT32 lu)
{
  INT32 EnableOff = MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_LU_BYTE);

  pe->tcam.byte[MVPP2_PRS_TCAM_LU_BYTE] = lu;
  pe->tcam.byte[EnableOff] = MVPP2_PRS_LU_MASK;
}

/* Update mask for single port in tcam sw entry */
STATIC VOID Mvpp2PrsTcamPortSet(MVPP2_PRS_ENTRY *pe,
            UINT32 port, BOOLEAN add)
{
  INT32 EnableOff = MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_PORT_BYTE);

  if (add)
    pe->tcam.byte[EnableOff] &= ~(1 << port);
  else
    pe->tcam.byte[EnableOff] |= 1 << port;
}

/* Update port map in tcam sw entry */
STATIC VOID Mvpp2PrsTcamPortMapSet(MVPP2_PRS_ENTRY *pe,
          UINT32 ports)
{
  UINT8 PortMask = MVPP2_PRS_PORT_MASK;
  INT32 EnableOff = MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_PORT_BYTE);

  pe->tcam.byte[MVPP2_PRS_TCAM_PORT_BYTE] = 0;
  pe->tcam.byte[EnableOff] &= ~PortMask;
  pe->tcam.byte[EnableOff] |= ~ports & MVPP2_PRS_PORT_MASK;
}

/* Obtain port map from tcam sw entry */
STATIC UINT32 Mvpp2PrsTcamPortMapGet(MVPP2_PRS_ENTRY *pe)
{
  INT32 EnableOff = MVPP2_PRS_TCAM_EN_OFFS(MVPP2_PRS_TCAM_PORT_BYTE);

  return ~(pe->tcam.byte[EnableOff]) & MVPP2_PRS_PORT_MASK;
}

/* Set byte of data and its enable bits in tcam sw entry */
STATIC VOID Mvpp2PrsTcamDataByteSet(MVPP2_PRS_ENTRY *pe,
           UINT32 offs, UINT8 byte,
           UINT8 enable)
{
  pe->tcam.byte[MVPP2_PRS_TCAM_DATA_BYTE(offs)] = byte;
  pe->tcam.byte[MVPP2_PRS_TCAM_DATA_BYTE_EN(offs)] = enable;
}

/* Get byte of data and its enable bits from tcam sw entry */
STATIC VOID Mvpp2PrsTcamDataByteGet(MVPP2_PRS_ENTRY *pe,
           UINT32 offs, UINT8 *byte,
           UINT8 *enable)
{
  *byte = pe->tcam.byte[MVPP2_PRS_TCAM_DATA_BYTE(offs)];
  *enable = pe->tcam.byte[MVPP2_PRS_TCAM_DATA_BYTE_EN(offs)];
}

/* Compare tcam data bytes with a pattern */
STATIC BOOLEAN Mvpp2PrsTcamDataCmp(MVPP2_PRS_ENTRY *pe, INT32 offs,
            UINT16 data)
{
  INT32 off = MVPP2_PRS_TCAM_DATA_BYTE(offs);
  UINT16 TcamData;

  TcamData = (8 << pe->tcam.byte[off + 1]) | pe->tcam.byte[off];
  if (TcamData != data)
    return FALSE;
  return TRUE;
}

/* Update ai bits in tcam sw entry */
STATIC VOID Mvpp2PrsTcamAiUpdate(MVPP2_PRS_ENTRY *pe,
             UINT32 bits, UINT32 enable)
{
  INT32 i, AiIdx = MVPP2_PRS_TCAM_AI_BYTE;

  for (i = 0; i < MVPP2_PRS_AI_BITS; i++) {

    if (!(enable & BIT(i)))
      continue;

    if (bits & BIT(i))
      pe->tcam.byte[AiIdx] |= 1 << i;
    else
      pe->tcam.byte[AiIdx] &= ~(1 << i);
  }

  pe->tcam.byte[MVPP2_PRS_TCAM_EN_OFFS(AiIdx)] |= enable;
}

/* Get ai bits from tcam sw entry */
STATIC INT32 Mvpp2PrsTcamAiGet(MVPP2_PRS_ENTRY *pe)
{
  return pe->tcam.byte[MVPP2_PRS_TCAM_AI_BYTE];
}

/* Get dword of data and its enable bits from tcam sw entry */
STATIC VOID Mvpp2PrsTcamDataDwordGet(MVPP2_PRS_ENTRY *pe,
           UINT32 offs, UINT32 *word,
           UINT32 *enable)
{
  INT32 index, offset;
  UINT8 byte, mask;

  for (index = 0; index < 4; index++) {
    offset = (offs * 4) + index;
    Mvpp2PrsTcamDataByteGet(pe, offset,  &byte, &mask);
    ((UINT8 *)word)[index] = byte;
    ((UINT8 *)enable)[index] = mask;
  }
}

/* Set ethertype in tcam sw entry */
STATIC VOID Mvpp2PrsMatchEtype(MVPP2_PRS_ENTRY *pe, INT32 offset,
          UINT16 ethertype)
{
  Mvpp2PrsTcamDataByteSet(pe, offset + 0, ethertype >> 8, 0xff);
  Mvpp2PrsTcamDataByteSet(pe, offset + 1, ethertype & 0xff, 0xff);
}

/* Set bits in sram sw entry */
STATIC VOID Mvpp2PrsSramBitsSet(MVPP2_PRS_ENTRY *pe,
               INT32 BitNum, INT32 val)
{
  pe->sram.byte[MVPP2_BIT_TO_BYTE(BitNum)] |= (val << (BitNum % 8));
}

/* Clear bits in sram sw entry */
STATIC VOID Mvpp2PrsSramBitsClear(MVPP2_PRS_ENTRY *pe,
           INT32 BitNum, INT32 val)
{
  pe->sram.byte[MVPP2_BIT_TO_BYTE(BitNum)] &= ~(val << (BitNum % 8));
}

/* Update ri bits in sram sw entry */
STATIC VOID Mvpp2PrsSramRiUpdate(MVPP2_PRS_ENTRY *pe,
             UINT32 bits, UINT32 mask)
{
  UINT32 i;

  for (i = 0; i < MVPP2_PRS_SRAM_RI_CTRL_BITS; i++) {
    INT32 RiOff = MVPP2_PRS_SRAM_RI_OFFS;

    if (!(mask & BIT(i)))
      continue;

    if (bits & BIT(i))
      Mvpp2PrsSramBitsSet(pe, RiOff + i, 1);
    else
      Mvpp2PrsSramBitsClear(pe, RiOff + i, 1);

    Mvpp2PrsSramBitsSet(pe, MVPP2_PRS_SRAM_RI_CTRL_OFFS + i, 1);
  }
}

/* Obtain ri bits from sram sw entry */
STATIC INT32 Mvpp2PrsSramRiGet(MVPP2_PRS_ENTRY *pe)
{
  return pe->sram.word[MVPP2_PRS_SRAM_RI_WORD];
}

/* Update ai bits in sram sw entry */
STATIC VOID Mvpp2PrsSramAiUpdate(MVPP2_PRS_ENTRY *pe,
             UINT32 bits, UINT32 mask)
{
  UINT32 i;
  INT32 AiOff = MVPP2_PRS_SRAM_AI_OFFS;

  for (i = 0; i < MVPP2_PRS_SRAM_AI_CTRL_BITS; i++) {

    if (!(mask & BIT(i)))
      continue;

    if (bits & BIT(i))
      Mvpp2PrsSramBitsSet(pe, AiOff + i, 1);
    else
      Mvpp2PrsSramBitsClear(pe, AiOff + i, 1);

    Mvpp2PrsSramBitsSet(pe, MVPP2_PRS_SRAM_AI_CTRL_OFFS + i, 1);
  }
}

/* Read ai bits from sram sw entry */
STATIC INT32 Mvpp2PrsSramAiGet(MVPP2_PRS_ENTRY *pe)
{
  UINT8 bits;
  INT32 AiOff = MVPP2_BIT_TO_BYTE(MVPP2_PRS_SRAM_AI_OFFS);
  INT32 AiEnOff = AiOff + 1;
  INT32 AiShift = MVPP2_PRS_SRAM_AI_OFFS % 8;

  bits = (pe->sram.byte[AiOff] >> AiShift) |
         (pe->sram.byte[AiEnOff] << (8 - AiShift));

  return bits;
}

/* In sram sw entry set lookup ID field of the tcam key to be used in the next
 * lookup INT32eration
 */
STATIC VOID Mvpp2PrsSramNextLuSet(MVPP2_PRS_ENTRY *pe,
               UINT32 lu)
{
  INT32 SramNextOff = MVPP2_PRS_SRAM_NEXT_LU_OFFS;

  Mvpp2PrsSramBitsClear(pe, SramNextOff,
          MVPP2_PRS_SRAM_NEXT_LU_MASK);
  Mvpp2PrsSramBitsSet(pe, SramNextOff, lu);
}

/* In the sram sw entry set sign and value of the next lookup offset
 * and the offset value generated to the classifier
 */
STATIC VOID Mvpp2PrsSramShiftSet(MVPP2_PRS_ENTRY *pe, INT32 shift,
             UINT32 op)
{
  /* Set sign */
  if (shift < 0) {
    Mvpp2PrsSramBitsSet(pe, MVPP2_PRS_SRAM_SHIFT_SIGN_BIT, 1);
    shift = 0 - shift;
  } else {
    Mvpp2PrsSramBitsClear(pe, MVPP2_PRS_SRAM_SHIFT_SIGN_BIT, 1);
  }

  /* Set value */
  pe->sram.byte[MVPP2_BIT_TO_BYTE(MVPP2_PRS_SRAM_SHIFT_OFFS)] =
                 (UINT8)shift;

  /* Reset and set operation */
  Mvpp2PrsSramBitsClear(pe, MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS,
          MVPP2_PRS_SRAM_OP_SEL_SHIFT_MASK);
  Mvpp2PrsSramBitsSet(pe, MVPP2_PRS_SRAM_OP_SEL_SHIFT_OFFS, op);

  /* Set base offset as current */
  Mvpp2PrsSramBitsClear(pe, MVPP2_PRS_SRAM_OP_SEL_BASE_OFFS, 1);
}

/* In the sram sw entry set sign and value of the user defined offset
 * generated to the classifier
 */
STATIC VOID Mvpp2PrsSramOffsetSet(MVPP2_PRS_ENTRY *pe,
              UINT32 type, INT32 offset,
              UINT32 op)
{
  /* Set sign */
  if (offset < 0) {
    Mvpp2PrsSramBitsSet(pe, MVPP2_PRS_SRAM_UDF_SIGN_BIT, 1);
    offset = 0 - offset;
  } else {
    Mvpp2PrsSramBitsClear(pe, MVPP2_PRS_SRAM_UDF_SIGN_BIT, 1);
  }

  /* Set value */
  Mvpp2PrsSramBitsClear(pe, MVPP2_PRS_SRAM_UDF_OFFS,
          MVPP2_PRS_SRAM_UDF_MASK);
  Mvpp2PrsSramBitsSet(pe, MVPP2_PRS_SRAM_UDF_OFFS, offset);
  pe->sram.byte[MVPP2_BIT_TO_BYTE(MVPP2_PRS_SRAM_UDF_OFFS +
          MVPP2_PRS_SRAM_UDF_BITS)] &=
        ~(MVPP2_PRS_SRAM_UDF_MASK >> (8 - (MVPP2_PRS_SRAM_UDF_OFFS % 8)));
  pe->sram.byte[MVPP2_BIT_TO_BYTE(MVPP2_PRS_SRAM_UDF_OFFS +
          MVPP2_PRS_SRAM_UDF_BITS)] |=
        (offset >> (8 - (MVPP2_PRS_SRAM_UDF_OFFS % 8)));

  /* Set offset type */
  Mvpp2PrsSramBitsClear(pe, MVPP2_PRS_SRAM_UDF_TYPE_OFFS,
          MVPP2_PRS_SRAM_UDF_TYPE_MASK);
  Mvpp2PrsSramBitsSet(pe, MVPP2_PRS_SRAM_UDF_TYPE_OFFS, type);

  /* Set offset operation */
  Mvpp2PrsSramBitsClear(pe, MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS,
          MVPP2_PRS_SRAM_OP_SEL_UDF_MASK);
  Mvpp2PrsSramBitsSet(pe, MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS, op);

  pe->sram.byte[MVPP2_BIT_TO_BYTE(MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS +
          MVPP2_PRS_SRAM_OP_SEL_UDF_BITS)] &=
               ~(MVPP2_PRS_SRAM_OP_SEL_UDF_MASK >>
            (8 - (MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS % 8)));

  pe->sram.byte[MVPP2_BIT_TO_BYTE(MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS +
          MVPP2_PRS_SRAM_OP_SEL_UDF_BITS)] |=
           (op >> (8 - (MVPP2_PRS_SRAM_OP_SEL_UDF_OFFS % 8)));

  /* Set base offset as current */
  Mvpp2PrsSramBitsClear(pe, MVPP2_PRS_SRAM_OP_SEL_BASE_OFFS, 1);
}

/* Find parser flow entry */
STATIC MVPP2_PRS_ENTRY *Mvpp2PrsFlowFind(MVPP2_SHARED *priv,
               INT32 flow)
{
  MVPP2_PRS_ENTRY *pe;
  INT32 tid;
  UINT32 dword, enable;

  pe = Mvpp2Alloc(sizeof(*pe));
  if (!pe)
    return MVPP2_NULL;
  Mvpp2PrsTcamLuSet(pe, MVPP2_PRS_LU_FLOWS);

  /* Go through the all entires with MVPP2_PRS_LU_FLOWS */
  for (tid = MVPP2_PRS_TCAM_SRAM_SIZE - 1; tid >= 0; tid--) {
    UINT8 bits;

    if (!priv->PrsShadow[tid].valid ||
        priv->PrsShadow[tid].lu != MVPP2_PRS_LU_FLOWS)
      continue;

    pe->index = tid;
    Mvpp2PrsHwRead(priv, pe);

    /* Check result info, because there maybe
    *   several TCAM lines to generate the same flow */
    Mvpp2PrsTcamDataDwordGet(pe, 0, &dword, &enable);
    if ((dword != 0) || (enable != 0))
      continue;

    bits = Mvpp2PrsSramAiGet(pe);

    /* Sram store classification lookup ID in AI bits [5:0] */
    if ((bits & MVPP2_PRS_FLOW_ID_MASK) == flow)
      return pe;
  }
  Mvpp2Free(pe);

  return MVPP2_NULL;
}

/* Return first free tcam index, seeking from start to end */
STATIC INT32 Mvpp2PrsTcamFirstFree(MVPP2_SHARED *priv, UINT8 start,
             UINT8 end)
{
  INT32 tid;

  if (start > end)
    Mvpp2Swap(start, end);

  if (end >= MVPP2_PRS_TCAM_SRAM_SIZE)
    end = MVPP2_PRS_TCAM_SRAM_SIZE - 1;

  for (tid = start; tid <= end; tid++) {
    if (!priv->PrsShadow[tid].valid)
      return tid;
  }

  return MVPP2_EINVAL;
}

/* Enable/disable dropping all mac da's */
STATIC VOID Mvpp2PrsMacDropAllSet(MVPP2_SHARED *priv, INT32 port,
               BOOLEAN add)
{
  MVPP2_PRS_ENTRY pe;

  if (priv->PrsShadow[MVPP2_PE_DROP_ALL].valid) {
    /* Entry exist - update port only */
    pe.index = MVPP2_PE_DROP_ALL;
    Mvpp2PrsHwRead(priv, &pe);
  } else {
    /* Entry doesn't exist - create new */
    Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
    Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_MAC);
    pe.index = MVPP2_PE_DROP_ALL;

    /* Non-promiscuous mode for all ports - DROP unknown packets */
    Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_DROP_MASK,
           MVPP2_PRS_RI_DROP_MASK);

    Mvpp2PrsSramBitsSet(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
    Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_FLOWS);

    /* Update shadow table */
    Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_MAC);

    /* Mask all ports */
    Mvpp2PrsTcamPortMapSet(&pe, 0);
  }

  /* Update port mask */
  Mvpp2PrsTcamPortSet(&pe, port, add);

  Mvpp2PrsHwWrite(priv, &pe);
}

/* Set port to promiscuous mode */
VOID Mvpp2PrsMacPromiscSet(MVPP2_SHARED *priv, INT32 port, BOOLEAN add)
{
  MVPP2_PRS_ENTRY pe;

  /* Promiscuous mode - Accept unknown packets */

  if (priv->PrsShadow[MVPP2_PE_MAC_PROMISCUOUS].valid) {
    /* Entry exist - update port only */
    pe.index = MVPP2_PE_MAC_PROMISCUOUS;
    Mvpp2PrsHwRead(priv, &pe);
  } else {
    /* Entry doesn't exist - create new */
    Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
    Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_MAC);
    pe.index = MVPP2_PE_MAC_PROMISCUOUS;

    /* Continue - set next lookup */
    Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_DSA);

    /* Set result info bits */
    Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L2_UCAST,
           MVPP2_PRS_RI_L2_CAST_MASK);

    /* Shift to ethertype */
    Mvpp2PrsSramShiftSet(&pe, 2 * MV_ETH_ALEN,
           MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

    /* Mask all ports */
    Mvpp2PrsTcamPortMapSet(&pe, 0);

    /* Update shadow table */
    Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_MAC);
  }

  /* Update port mask */
  Mvpp2PrsTcamPortSet(&pe, port, add);

  Mvpp2PrsHwWrite(priv, &pe);
}

/* Accept multicast */
VOID Mvpp2PrsMacMultiSet(MVPP2_SHARED *priv, INT32 port, INT32 index,
           BOOLEAN add)
{
  MVPP2_PRS_ENTRY pe;
  UINT8 DaMc;

  /* Ethernet multicast address first byte is
   * 0x01 for IPv4 and 0x33 for IPv6
   */
  DaMc = (index == MVPP2_PE_MAC_MC_ALL) ? 0x01 : 0x33;

  if (priv->PrsShadow[index].valid) {
    /* Entry exist - update port only */
    pe.index = index;
    Mvpp2PrsHwRead(priv, &pe);
  } else {
    /* Entry doesn't exist - create new */
    Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
    Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_MAC);
    pe.index = index;

    /* Continue - set next lookup */
    Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_DSA);

    /* Set result info bits */
    Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L2_MCAST,
           MVPP2_PRS_RI_L2_CAST_MASK);

    /* Update tcam entry data first byte */
    Mvpp2PrsTcamDataByteSet(&pe, 0, DaMc, 0xff);

    /* Shift to ethertype */
    Mvpp2PrsSramShiftSet(&pe, 2 * MV_ETH_ALEN,
           MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

    /* Mask all ports */
    Mvpp2PrsTcamPortMapSet(&pe, 0);

    /* Update shadow table */
    Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_MAC);
  }

  /* Update port mask */
  Mvpp2PrsTcamPortSet(&pe, port, add);

  Mvpp2PrsHwWrite(priv, &pe);
}

/* Set entry for dsa packets */
STATIC VOID Mvpp2PrsDsaTagSet(MVPP2_SHARED *priv, INT32 port,
             BOOLEAN add, BOOLEAN tagged,
             BOOLEAN extend)
{
  MVPP2_PRS_ENTRY pe;
  INT32 tid, shift;

  if (extend) {
    tid = tagged ? MVPP2_PE_EDSA_TAGGED : MVPP2_PE_EDSA_UNTAGGED;
    shift = 8;
  } else {
    tid = tagged ? MVPP2_PE_DSA_TAGGED : MVPP2_PE_DSA_UNTAGGED;
    shift = 4;
  }

  if (priv->PrsShadow[tid].valid) {
    /* Entry exist - update port only */
    pe.index = tid;
    Mvpp2PrsHwRead(priv, &pe);
  } else {
    /* Entry doesn't exist - create new */
    Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
    Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_DSA);
    pe.index = tid;

    /* Shift 4 bytes if DSA tag or 8 bytes in case of EDSA tag*/
    Mvpp2PrsSramShiftSet(&pe, shift,
           MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

    /* Update shadow table */
    Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_DSA);

    if (tagged) {
      /* Set tagged bit in DSA tag */
      Mvpp2PrsTcamDataByteSet(&pe, 0,
            MVPP2_PRS_TCAM_DSA_TAGGED_BIT,
            MVPP2_PRS_TCAM_DSA_TAGGED_BIT);
      /* Clear all ai bits for next iteration */
      Mvpp2PrsSramAiUpdate(&pe, 0,
             MVPP2_PRS_SRAM_AI_MASK);
      /* If packet is tagged continue check vlans */
      Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_VLAN);
    } else {
      /* Set result info bits to 'no vlans' */
      Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_VLAN_NONE,
             MVPP2_PRS_RI_VLAN_MASK);
      Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_L2);
    }

    /* Mask all ports */
    Mvpp2PrsTcamPortMapSet(&pe, 0);
  }

  /* Update port mask */
  Mvpp2PrsTcamPortSet(&pe, port, add);

  Mvpp2PrsHwWrite(priv, &pe);
}

/* Set entry for dsa ethertype */
STATIC VOID Mvpp2PrsDsaTagEthertypeSet(MVPP2_SHARED *priv, INT32 port,
              BOOLEAN add, BOOLEAN tagged,
              BOOLEAN extend)
{
  MVPP2_PRS_ENTRY pe;
  INT32 tid, shift, PortMask;

  if (extend) {
    tid = tagged ? MVPP2_PE_ETYPE_EDSA_TAGGED :
          MVPP2_PE_ETYPE_EDSA_UNTAGGED;
    PortMask = 0;
    shift = 8;
  } else {
    tid = tagged ? MVPP2_PE_ETYPE_DSA_TAGGED :
          MVPP2_PE_ETYPE_DSA_UNTAGGED;
    PortMask = MVPP2_PRS_PORT_MASK;
    shift = 4;
  }

  if (priv->PrsShadow[tid].valid) {
    /* Entry exist - update port only */
    pe.index = tid;
    Mvpp2PrsHwRead(priv, &pe);
  } else {
    /* Entry doesn't exist - create new */
    Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
    Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_DSA);
    pe.index = tid;

    /* Set ethertype */
    Mvpp2PrsMatchEtype(&pe, 0, MV_ETH_P_EDSA);
    Mvpp2PrsMatchEtype(&pe, 2, 0);

    Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_DSA_MASK,
           MVPP2_PRS_RI_DSA_MASK);
    /* Shift ethertype + 2 byte reserved + tag*/
    Mvpp2PrsSramShiftSet(&pe, 2 + MVPP2_ETH_TYPE_LEN + shift,
           MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

    /* Update shadow table */
    Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_DSA);

    if (tagged) {
      /* Set tagged bit in DSA tag */
      Mvpp2PrsTcamDataByteSet(&pe,
                 MVPP2_ETH_TYPE_LEN + 2 + 3,
             MVPP2_PRS_TCAM_DSA_TAGGED_BIT,
             MVPP2_PRS_TCAM_DSA_TAGGED_BIT);
      /* Clear all ai bits for next iteration */
      Mvpp2PrsSramAiUpdate(&pe, 0,
             MVPP2_PRS_SRAM_AI_MASK);
      /* If packet is tagged continue check vlans */
      Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_VLAN);
    } else {
      /* Set result info bits to 'no vlans' */
      Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_VLAN_NONE,
             MVPP2_PRS_RI_VLAN_MASK);
      Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_L2);
    }
    /* Mask/unmask all ports, depending on dsa type */
    Mvpp2PrsTcamPortMapSet(&pe, PortMask);
  }

  /* Update port mask */
  Mvpp2PrsTcamPortSet(&pe, port, add);

  Mvpp2PrsHwWrite(priv, &pe);
}

/* Search for existing single/triple vlan entry */
STATIC MVPP2_PRS_ENTRY *Mvpp2PrsVlanFind(MVPP2_SHARED *priv,
               UINT16 tpid, INT32 ai)
{
  MVPP2_PRS_ENTRY *pe;
  INT32 tid;

  pe = Mvpp2Alloc(sizeof(*pe));
  if (!pe)
    return MVPP2_NULL;
  Mvpp2PrsTcamLuSet(pe, MVPP2_PRS_LU_VLAN);

  /* Go through the all entries with MVPP2_PRS_LU_VLAN */
  for (tid = MVPP2_PE_FIRST_FREE_TID;
       tid <= MVPP2_PE_LAST_FREE_TID; tid++) {
    UINT32 RiBits, AiBits;
    BOOLEAN match;

    if (!priv->PrsShadow[tid].valid ||
        priv->PrsShadow[tid].lu != MVPP2_PRS_LU_VLAN)
      continue;

    pe->index = tid;

    Mvpp2PrsHwRead(priv, pe);
    match = Mvpp2PrsTcamDataCmp(pe, 0, Mvpp2Swab16(tpid));
    if (!match)
      continue;

    /* Get vlan type */
    RiBits = Mvpp2PrsSramRiGet(pe);
    RiBits &= MVPP2_PRS_RI_VLAN_MASK;

    /* Get current ai value from tcam */
    AiBits = Mvpp2PrsTcamAiGet(pe);
    /* Clear double vlan bit */
    AiBits &= ~MVPP2_PRS_DBL_VLAN_AI_BIT;

    if (ai != AiBits)
      continue;

    if (RiBits == MVPP2_PRS_RI_VLAN_SINGLE ||
        RiBits == MVPP2_PRS_RI_VLAN_TRIPLE)
      return pe;
  }
  Mvpp2Free(pe);

  return MVPP2_NULL;
}

/* Add/update single/triple vlan entry */
INT32 Mvpp2PrsVlanAdd(MVPP2_SHARED *priv, UINT16 tpid, INT32 ai,
           UINT32 PortMap)
{
  MVPP2_PRS_ENTRY *pe;
  INT32 TidAux, tid;
  INT32 ret = 0;

  pe = Mvpp2PrsVlanFind(priv, tpid, ai);

  if (!pe) {
    /* Create new tcam entry */
    tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_LAST_FREE_TID,
            MVPP2_PE_FIRST_FREE_TID);
    if (tid < 0)
      return tid;

    pe = Mvpp2Alloc(sizeof(*pe));
    if (!pe)
      return MVPP2_ENOMEM;

    /* Get last double vlan tid */
    for (TidAux = MVPP2_PE_LAST_FREE_TID;
         TidAux >= MVPP2_PE_FIRST_FREE_TID; TidAux--) {
      UINT32 RiBits;

      if (!priv->PrsShadow[TidAux].valid ||
          priv->PrsShadow[TidAux].lu != MVPP2_PRS_LU_VLAN)
        continue;

      pe->index = TidAux;
      Mvpp2PrsHwRead(priv, pe);
      RiBits = Mvpp2PrsSramRiGet(pe);
      if ((RiBits & MVPP2_PRS_RI_VLAN_MASK) ==
          MVPP2_PRS_RI_VLAN_DOUBLE)
        break;
    }

    if (tid <= TidAux) {
      ret = MVPP2_EINVAL;
      goto error;
    }

    Mvpp2Memset(pe, 0 , sizeof(MVPP2_PRS_ENTRY));
    Mvpp2PrsTcamLuSet(pe, MVPP2_PRS_LU_VLAN);
    pe->index = tid;

    Mvpp2PrsMatchEtype(pe, 0, tpid);

    Mvpp2PrsSramNextLuSet(pe, MVPP2_PRS_LU_L2);
    /* Shift 4 bytes - skip 1 vlan tag */
    Mvpp2PrsSramShiftSet(pe, MVPP2_VLAN_TAG_LEN,
           MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
    /* Clear all ai bits for next iteration */
    Mvpp2PrsSramAiUpdate(pe, 0, MVPP2_PRS_SRAM_AI_MASK);

    if (ai == MVPP2_PRS_SINGLE_VLAN_AI) {
      Mvpp2PrsSramRiUpdate(pe, MVPP2_PRS_RI_VLAN_SINGLE,
             MVPP2_PRS_RI_VLAN_MASK);
    } else {
      ai |= MVPP2_PRS_DBL_VLAN_AI_BIT;
      Mvpp2PrsSramRiUpdate(pe, MVPP2_PRS_RI_VLAN_TRIPLE,
             MVPP2_PRS_RI_VLAN_MASK);
    }
    Mvpp2PrsTcamAiUpdate(pe, ai, MVPP2_PRS_SRAM_AI_MASK);

    Mvpp2PrsShadowSet(priv, pe->index, MVPP2_PRS_LU_VLAN);
  }
  /* Update ports' mask */
  Mvpp2PrsTcamPortMapSet(pe, PortMap);

  Mvpp2PrsHwWrite(priv, pe);

error:
  Mvpp2Free(pe);

  return ret;
}

/* Get first free double vlan ai number */
INT32 Mvpp2PrsDoubleVlanAiFreeGet(MVPP2_SHARED *priv)
{
  INT32 i;

  for (i = 1; i < MVPP2_PRS_DBL_VLANS_MAX; i++) {
    if (!priv->PrsDoubleVlans[i])
      return i;
  }

  return MVPP2_EINVAL;
}

/* Search for existing double vlan entry */
MVPP2_PRS_ENTRY *Mvpp2PrsDoubleVlanFind(MVPP2_SHARED *priv,
               UINT16 tpid1,
               UINT16 tpid2)
{
  MVPP2_PRS_ENTRY *pe;
  INT32 tid;

  pe = Mvpp2Alloc(sizeof(*pe));
  if (!pe)
    return MVPP2_NULL;
  Mvpp2PrsTcamLuSet(pe, MVPP2_PRS_LU_VLAN);

  /* Go through the all entries with MVPP2_PRS_LU_VLAN */
  for (tid = MVPP2_PE_FIRST_FREE_TID;
       tid <= MVPP2_PE_LAST_FREE_TID; tid++) {
    UINT32 RiMask;
    BOOLEAN match;

    if (!priv->PrsShadow[tid].valid ||
        priv->PrsShadow[tid].lu != MVPP2_PRS_LU_VLAN)
      continue;

    pe->index = tid;
    Mvpp2PrsHwRead(priv, pe);

    match = Mvpp2PrsTcamDataCmp(pe, 0, Mvpp2Swab16(tpid1))
      && Mvpp2PrsTcamDataCmp(pe, 4, Mvpp2Swab16(tpid2));

    if (!match)
      continue;

    RiMask = Mvpp2PrsSramRiGet(pe) & MVPP2_PRS_RI_VLAN_MASK;
    if (RiMask == MVPP2_PRS_RI_VLAN_DOUBLE)
      return pe;
  }
  Mvpp2Free(pe);

  return MVPP2_NULL;
}

/* Add or update double vlan entry */
INT32 Mvpp2PrsDoubleVlanAdd(MVPP2_SHARED *priv, UINT16 tpid1,
            UINT16 tpid2,
            UINT32 PortMap)
{
  MVPP2_PRS_ENTRY *pe;
  INT32 TidAux, tid, ai, ret = 0;

  pe = Mvpp2PrsDoubleVlanFind(priv, tpid1, tpid2);

  if (!pe) {
    /* Create new tcam entry */
    tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_FIRST_FREE_TID,
        MVPP2_PE_LAST_FREE_TID);
    if (tid < 0)
      return tid;

    pe = Mvpp2Alloc(sizeof(*pe));
    if (!pe)
      return MVPP2_ENOMEM;

    /* Set ai value for new double vlan entry */
    ai = Mvpp2PrsDoubleVlanAiFreeGet(priv);
    if (ai < 0) {
      ret = ai;
      goto error;
    }

    /* Get first single/triple vlan tid */
    for (TidAux = MVPP2_PE_FIRST_FREE_TID;
         TidAux <= MVPP2_PE_LAST_FREE_TID; TidAux++) {
      UINT32 RiBits;

      if (!priv->PrsShadow[TidAux].valid ||
          priv->PrsShadow[TidAux].lu != MVPP2_PRS_LU_VLAN)
        continue;

      pe->index = TidAux;
      Mvpp2PrsHwRead(priv, pe);
      RiBits = Mvpp2PrsSramRiGet(pe);
      RiBits &= MVPP2_PRS_RI_VLAN_MASK;
      if (RiBits == MVPP2_PRS_RI_VLAN_SINGLE ||
          RiBits == MVPP2_PRS_RI_VLAN_TRIPLE)
        break;
    }

    if (tid >= TidAux) {
      ret = MVPP2_ERANGE;
      goto error;
    }

    Mvpp2Memset(pe, 0, sizeof(MVPP2_PRS_ENTRY));
    Mvpp2PrsTcamLuSet(pe, MVPP2_PRS_LU_VLAN);
    pe->index = tid;

    priv->PrsDoubleVlans[ai] = TRUE;

    Mvpp2PrsMatchEtype(pe, 0, tpid1);
    Mvpp2PrsMatchEtype(pe, 4, tpid2);

    Mvpp2PrsSramNextLuSet(pe, MVPP2_PRS_LU_VLAN);
    /* Shift 8 bytes - skip 2 vlan tags */
    Mvpp2PrsSramShiftSet(pe, 2 * MVPP2_VLAN_TAG_LEN,
           MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
    Mvpp2PrsSramRiUpdate(pe, MVPP2_PRS_RI_VLAN_DOUBLE,
           MVPP2_PRS_RI_VLAN_MASK);
    Mvpp2PrsSramAiUpdate(pe, ai | MVPP2_PRS_DBL_VLAN_AI_BIT,
           MVPP2_PRS_SRAM_AI_MASK);

    Mvpp2PrsShadowSet(priv, pe->index, MVPP2_PRS_LU_VLAN);
  }

  /* Update ports' mask */
  Mvpp2PrsTcamPortMapSet(pe, PortMap);
  Mvpp2PrsHwWrite(priv, pe);

error:
  Mvpp2Free(pe);
  return ret;
}

/* IPv4 header parsing for fragmentation and L4 offset */
STATIC INT32 Mvpp2PrsIp4Proto(MVPP2_SHARED *priv, UINT16 proto,
             UINT32 ri, UINT32 RiMask)
{
  MVPP2_PRS_ENTRY pe;
  INT32 tid;

  if ((proto != MV_IPPR_TCP) && (proto != MV_IPPR_UDP) &&
      (proto != MV_IPPR_IGMP))
    return MVPP2_EINVAL;

  /* Fragmented packet */
  tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_FIRST_FREE_TID,
          MVPP2_PE_LAST_FREE_TID);
  if (tid < 0)
    return tid;

  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_IP4);
  pe.index = tid;

  /* Set next lu to IPv4 */
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_IP4);
  Mvpp2PrsSramShiftSet(&pe, 12, MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
  /* Set L4 offset */
  Mvpp2PrsSramOffsetSet(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L4,
          sizeof(Mvpp2Iphdr) - 4,
          MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);
  Mvpp2PrsSramAiUpdate(&pe, MVPP2_PRS_IPV4_DIP_AI_BIT,
         MVPP2_PRS_IPV4_DIP_AI_BIT);
  Mvpp2PrsSramRiUpdate(&pe, ri | MVPP2_PRS_RI_IP_FRAG_MASK,
         RiMask | MVPP2_PRS_RI_IP_FRAG_MASK);

  Mvpp2PrsTcamDataByteSet(&pe, 5, proto, MVPP2_PRS_TCAM_PROTO_MASK);
  Mvpp2PrsTcamAiUpdate(&pe, 0, MVPP2_PRS_IPV4_DIP_AI_BIT);
  /* Unmask all ports */
  Mvpp2PrsTcamPortMapSet(&pe, MVPP2_PRS_PORT_MASK);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_IP4);
  Mvpp2PrsHwWrite(priv, &pe);

  /* Not fragmented packet */
  tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_FIRST_FREE_TID,
          MVPP2_PE_LAST_FREE_TID);
  if (tid < 0)
    return tid;

  pe.index = tid;
  /* Clear ri before updating */
  pe.sram.word[MVPP2_PRS_SRAM_RI_WORD] = 0x0;
  pe.sram.word[MVPP2_PRS_SRAM_RI_CTRL_WORD] = 0x0;
  Mvpp2PrsSramRiUpdate(&pe, ri, RiMask);

  Mvpp2PrsTcamDataByteSet(&pe, 2, 0x00, MVPP2_PRS_TCAM_PROTO_MASK_L);
  Mvpp2PrsTcamDataByteSet(&pe, 3, 0x00, MVPP2_PRS_TCAM_PROTO_MASK);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_IP4);
  Mvpp2PrsHwWrite(priv, &pe);

  return 0;
}

/* IPv4 L3 multicast or broadcast */
STATIC INT32 Mvpp2PrsIp4Cast(MVPP2_SHARED *priv, UINT16 L3Cast)
{
  MVPP2_PRS_ENTRY pe;
  INT32 mask, tid;

  tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_FIRST_FREE_TID,
          MVPP2_PE_LAST_FREE_TID);
  if (tid < 0)
    return tid;

  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_IP4);
  pe.index = tid;

  switch (L3Cast) {
  case MVPP2_PRS_L3_MULTI_CAST:
    Mvpp2PrsTcamDataByteSet(&pe, 0, MVPP2_PRS_IPV4_MC,
               MVPP2_PRS_IPV4_MC_MASK);
    Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L3_MCAST,
           MVPP2_PRS_RI_L3_ADDR_MASK);
    break;
  case  MVPP2_PRS_L3_BROAD_CAST:
    mask = MVPP2_PRS_IPV4_BC_MASK;
    Mvpp2PrsTcamDataByteSet(&pe, 0, mask, mask);
    Mvpp2PrsTcamDataByteSet(&pe, 1, mask, mask);
    Mvpp2PrsTcamDataByteSet(&pe, 2, mask, mask);
    Mvpp2PrsTcamDataByteSet(&pe, 3, mask, mask);
    Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L3_BCAST,
           MVPP2_PRS_RI_L3_ADDR_MASK);
    break;
  default:
    return MVPP2_EINVAL;
  }

  /* Finished: go to flowid generation */
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_FLOWS);
  Mvpp2PrsSramBitsSet(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);

  Mvpp2PrsTcamAiUpdate(&pe, MVPP2_PRS_IPV4_DIP_AI_BIT,
         MVPP2_PRS_IPV4_DIP_AI_BIT);
  /* Unmask all ports */
  Mvpp2PrsTcamPortMapSet(&pe, MVPP2_PRS_PORT_MASK);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_IP4);
  Mvpp2PrsHwWrite(priv, &pe);

  return 0;
}

/* Set entries for protocols over IPv6  */
STATIC INT32 Mvpp2PrsIp6Proto(MVPP2_SHARED *priv, UINT16 proto,
             UINT32 ri, UINT32 RiMask)
{
  MVPP2_PRS_ENTRY pe;
  INT32 tid;

  if ((proto != MV_IPPR_TCP) && (proto != MV_IPPR_UDP) &&
      (proto != MV_IPPR_ICMPV6) && (proto != MV_IPPR_IPIP))
    return MVPP2_EINVAL;

  tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_FIRST_FREE_TID,
          MVPP2_PE_LAST_FREE_TID);
  if (tid < 0)
    return tid;

  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_IP6);
  pe.index = tid;

  /* Finished: go to flowid generation */
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_FLOWS);
  Mvpp2PrsSramBitsSet(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
  Mvpp2PrsSramRiUpdate(&pe, ri, RiMask);
  Mvpp2PrsSramOffsetSet(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L4,
          sizeof(Mvpp2Ipv6hdr) - 6,
          MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

  Mvpp2PrsTcamDataByteSet(&pe, 0, proto, MVPP2_PRS_TCAM_PROTO_MASK);
  Mvpp2PrsTcamAiUpdate(&pe, MVPP2_PRS_IPV6_NO_EXT_AI_BIT,
         MVPP2_PRS_IPV6_NO_EXT_AI_BIT);
  /* Unmask all ports */
  Mvpp2PrsTcamPortMapSet(&pe, MVPP2_PRS_PORT_MASK);

  /* Write HW */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_IP6);
  Mvpp2PrsHwWrite(priv, &pe);

  return 0;
}

/* IPv6 L3 multicast entry */
STATIC INT32 Mvpp2PrsIp6Cast(MVPP2_SHARED *priv, UINT16 L3Cast)
{
  MVPP2_PRS_ENTRY pe;
  INT32 tid;

  if (L3Cast != MVPP2_PRS_L3_MULTI_CAST)
    return MVPP2_EINVAL;

  tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_FIRST_FREE_TID,
          MVPP2_PE_LAST_FREE_TID);
  if (tid < 0)
    return tid;

  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_IP6);
  pe.index = tid;

  /* Finished: go to flowid generation */
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_IP6);
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L3_MCAST,
         MVPP2_PRS_RI_L3_ADDR_MASK);
  Mvpp2PrsSramAiUpdate(&pe, MVPP2_PRS_IPV6_NO_EXT_AI_BIT,
         MVPP2_PRS_IPV6_NO_EXT_AI_BIT);
  /* Shift back to IPv6 NH */
  Mvpp2PrsSramShiftSet(&pe, -18, MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

  Mvpp2PrsTcamDataByteSet(&pe, 0, MVPP2_PRS_IPV6_MC,
             MVPP2_PRS_IPV6_MC_MASK);
  Mvpp2PrsTcamAiUpdate(&pe, 0, MVPP2_PRS_IPV6_NO_EXT_AI_BIT);
  /* Unmask all ports */
  Mvpp2PrsTcamPortMapSet(&pe, MVPP2_PRS_PORT_MASK);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_IP6);
  Mvpp2PrsHwWrite(priv, &pe);

  return 0;
}

/* Parser per-port initialization */
STATIC VOID Mvpp2PrsHwPortInit(MVPP2_SHARED *priv, INT32 port,
              INT32 LuFirst, INT32 LuMax,
              INT32 offset)
{
  UINT32 val;

  /* Set lookup ID */
  val = Mvpp2Read(priv, MVPP2_PRS_INIT_LOOKUP_REG);
  val &= ~MVPP2_PRS_PORT_LU_MASK(port);
  val |=  MVPP2_PRS_PORT_LU_VAL(port, LuFirst);
  Mvpp2Write(priv, MVPP2_PRS_INIT_LOOKUP_REG, val);

  /* Set maximum number of loops for packet received from port */
  val = Mvpp2Read(priv, MVPP2_PRS_MAX_LOOP_REG(port));
  val &= ~MVPP2_PRS_MAX_LOOP_MASK(port);
  val |= MVPP2_PRS_MAX_LOOP_VAL(port, LuMax);
  Mvpp2Write(priv, MVPP2_PRS_MAX_LOOP_REG(port), val);

  /* Set initial offset for packet header extraction for the first
   * searching loop
   */
  val = Mvpp2Read(priv, MVPP2_PRS_INIT_OFFS_REG(port));
  val &= ~MVPP2_PRS_INIT_OFF_MASK(port);
  val |= MVPP2_PRS_INIT_OFF_VAL(port, offset);
  Mvpp2Write(priv, MVPP2_PRS_INIT_OFFS_REG(port), val);
}

/* Default flow entries initialization for all ports */
STATIC VOID Mvpp2PrsDefFlowInit(MVPP2_SHARED *priv)
{
  MVPP2_PRS_ENTRY pe;
  INT32 port;

  for (port = 0; port < MVPP2_MAX_PORTS; port++) {
    Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
    Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_FLOWS);
    pe.index = MVPP2_PE_FIRST_DEFAULT_FLOW - port;

    /* Mask all ports */
    Mvpp2PrsTcamPortMapSet(&pe, 0);

    /* Set flow ID*/
    Mvpp2PrsSramAiUpdate(&pe, port, MVPP2_PRS_FLOW_ID_MASK);
    Mvpp2PrsSramBitsSet(&pe, MVPP2_PRS_SRAM_LU_DONE_BIT, 1);

    /* Update shadow table and hw entry */
    Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_FLOWS);
    Mvpp2PrsHwWrite(priv, &pe);
  }
}

/* Set default entry for Marvell Header field */
STATIC VOID Mvpp2PrsMhInit(MVPP2_SHARED *priv)
{
  MVPP2_PRS_ENTRY pe;

  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));

  pe.index = MVPP2_PE_MH_DEFAULT;
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_MH);
  Mvpp2PrsSramShiftSet(&pe, MVPP2_MH_SIZE,
         MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_MAC);

  /* Unmask all ports */
  Mvpp2PrsTcamPortMapSet(&pe, MVPP2_PRS_PORT_MASK);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_MH);
  Mvpp2PrsHwWrite(priv, &pe);
}

/* Set default entires (place holder) for promiscuous, non-promiscuous and
 * multicast MAC addresses
 */
STATIC VOID Mvpp2PrsMacInit(MVPP2_SHARED *priv)
{
  MVPP2_PRS_ENTRY pe;

  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));

  /* Non-promiscuous mode for all ports - DROP unknown packets */
  pe.index = MVPP2_PE_MAC_NON_PROMISCUOUS;
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_MAC);

  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_DROP_MASK,
         MVPP2_PRS_RI_DROP_MASK);
  Mvpp2PrsSramBitsSet(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_FLOWS);

  /* Unmask all ports */
  Mvpp2PrsTcamPortMapSet(&pe, MVPP2_PRS_PORT_MASK);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_MAC);
  Mvpp2PrsHwWrite(priv, &pe);

  /* place holders only - no ports */
  Mvpp2PrsMacDropAllSet(priv, 0, FALSE);
  Mvpp2PrsMacPromiscSet(priv, 0, FALSE);
  Mvpp2PrsMacMultiSet(priv, MVPP2_PE_MAC_MC_ALL, 0, FALSE);
  Mvpp2PrsMacMultiSet(priv, MVPP2_PE_MAC_MC_IP6, 0, FALSE);
}

/* Set default entries for various types of dsa packets */
STATIC VOID Mvpp2PrsDsaInit(MVPP2_SHARED *priv)
{
  MVPP2_PRS_ENTRY pe;

  /* None tagged EDSA entry - place holder */
  Mvpp2PrsDsaTagSet(priv, 0, FALSE, MVPP2_PRS_UNTAGGED,
            MVPP2_PRS_EDSA);

  /* Tagged EDSA entry - place holder */
  Mvpp2PrsDsaTagSet(priv, 0, FALSE, MVPP2_PRS_TAGGED, MVPP2_PRS_EDSA);

  /* None tagged DSA entry - place holder */
  Mvpp2PrsDsaTagSet(priv, 0, FALSE, MVPP2_PRS_UNTAGGED,
            MVPP2_PRS_DSA);

  /* Tagged DSA entry - place holder */
  Mvpp2PrsDsaTagSet(priv, 0, FALSE, MVPP2_PRS_TAGGED, MVPP2_PRS_DSA);

  /* None tagged EDSA ethertype entry - place holder*/
  Mvpp2PrsDsaTagEthertypeSet(priv, 0, FALSE,
          MVPP2_PRS_UNTAGGED, MVPP2_PRS_EDSA);

  /* Tagged EDSA ethertype entry - place holder*/
  Mvpp2PrsDsaTagEthertypeSet(priv, 0, FALSE,
          MVPP2_PRS_TAGGED, MVPP2_PRS_EDSA);

  /* None tagged DSA ethertype entry */
  Mvpp2PrsDsaTagEthertypeSet(priv, 0, TRUE,
          MVPP2_PRS_UNTAGGED, MVPP2_PRS_DSA);

  /* Tagged DSA ethertype entry */
  Mvpp2PrsDsaTagEthertypeSet(priv, 0, TRUE,
          MVPP2_PRS_TAGGED, MVPP2_PRS_DSA);

  /* Set default entry, in case DSA or EDSA tag not found */
  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_DSA);
  pe.index = MVPP2_PE_DSA_DEFAULT;
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_VLAN);

  /* Shift 0 bytes */
  Mvpp2PrsSramShiftSet(&pe, 0, MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_MAC);

  /* Clear all sram ai bits for next iteration */
  Mvpp2PrsSramAiUpdate(&pe, 0, MVPP2_PRS_SRAM_AI_MASK);

  /* Unmask all ports */
  Mvpp2PrsTcamPortMapSet(&pe, MVPP2_PRS_PORT_MASK);

  Mvpp2PrsHwWrite(priv, &pe);
}

/* Match basic ethertypes */
STATIC INT32 Mvpp2PrsEtypeInit(MVPP2_SHARED *priv)
{
  MVPP2_PRS_ENTRY pe;
  INT32 tid;

  /* Ethertype: PPPoE */
  tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_FIRST_FREE_TID,
          MVPP2_PE_LAST_FREE_TID);
  if (tid < 0)
    return tid;

  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_L2);
  pe.index = tid;

  Mvpp2PrsMatchEtype(&pe, 0, MV_ETH_P_PPP_SES);

  Mvpp2PrsSramShiftSet(&pe, MVPP2_PPPOE_HDR_SIZE,
         MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_PPPOE);
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_PPPOE_MASK,
         MVPP2_PRS_RI_PPPOE_MASK);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_L2);
  priv->PrsShadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
  priv->PrsShadow[pe.index].finish = FALSE;
  Mvpp2PrsShadowRiSet(priv, pe.index, MVPP2_PRS_RI_PPPOE_MASK,
        MVPP2_PRS_RI_PPPOE_MASK);
  Mvpp2PrsHwWrite(priv, &pe);

  /* Ethertype: ARP */
  tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_FIRST_FREE_TID,
          MVPP2_PE_LAST_FREE_TID);
  if (tid < 0)
    return tid;

  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_L2);
  pe.index = tid;

  Mvpp2PrsMatchEtype(&pe, 0, MV_ETH_P_ARP);

  /* Generate flow in the next iteration*/
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_FLOWS);
  Mvpp2PrsSramBitsSet(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L3_ARP,
         MVPP2_PRS_RI_L3_PROTO_MASK);
  /* Set L3 offset */
  Mvpp2PrsSramOffsetSet(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
          MVPP2_ETH_TYPE_LEN,
          MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_L2);
  priv->PrsShadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
  priv->PrsShadow[pe.index].finish = TRUE;
  Mvpp2PrsShadowRiSet(priv, pe.index, MVPP2_PRS_RI_L3_ARP,
        MVPP2_PRS_RI_L3_PROTO_MASK);
  Mvpp2PrsHwWrite(priv, &pe);

  /* Ethertype: LBTD */
  tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_FIRST_FREE_TID,
          MVPP2_PE_LAST_FREE_TID);
  if (tid < 0)
    return tid;

  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_L2);
  pe.index = tid;

  Mvpp2PrsMatchEtype(&pe, 0, MVPP2_IP_LBDT_TYPE);

  /* Generate flow in the next iteration*/
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_FLOWS);
  Mvpp2PrsSramBitsSet(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_CPU_CODE_RX_SPEC |
         MVPP2_PRS_RI_UDF3_RX_SPECIAL,
         MVPP2_PRS_RI_CPU_CODE_MASK |
         MVPP2_PRS_RI_UDF3_MASK);
  /* Set L3 offset */
  Mvpp2PrsSramOffsetSet(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
          MVPP2_ETH_TYPE_LEN,
          MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_L2);
  priv->PrsShadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
  priv->PrsShadow[pe.index].finish = TRUE;
  Mvpp2PrsShadowRiSet(priv, pe.index, MVPP2_PRS_RI_CPU_CODE_RX_SPEC |
        MVPP2_PRS_RI_UDF3_RX_SPECIAL,
        MVPP2_PRS_RI_CPU_CODE_MASK |
        MVPP2_PRS_RI_UDF3_MASK);
  Mvpp2PrsHwWrite(priv, &pe);

  /* Ethertype: IPv4 without options */
  tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_FIRST_FREE_TID,
          MVPP2_PE_LAST_FREE_TID);
  if (tid < 0)
    return tid;

  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_L2);
  pe.index = tid;

  Mvpp2PrsMatchEtype(&pe, 0, MV_ETH_P_IP);
  Mvpp2PrsTcamDataByteSet(&pe, MVPP2_ETH_TYPE_LEN,
             MVPP2_PRS_IPV4_HEAD | MVPP2_PRS_IPV4_IHL,
             MVPP2_PRS_IPV4_HEAD_MASK |
             MVPP2_PRS_IPV4_IHL_MASK);

  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_IP4);
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L3_IP4,
         MVPP2_PRS_RI_L3_PROTO_MASK);
  /* Skip EthType + 4 bytes of IP header */
  Mvpp2PrsSramShiftSet(&pe, MVPP2_ETH_TYPE_LEN + 4,
         MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
  /* Set L3 offset */
  Mvpp2PrsSramOffsetSet(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
          MVPP2_ETH_TYPE_LEN,
          MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_L2);
  priv->PrsShadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
  priv->PrsShadow[pe.index].finish = FALSE;
  Mvpp2PrsShadowRiSet(priv, pe.index, MVPP2_PRS_RI_L3_IP4,
        MVPP2_PRS_RI_L3_PROTO_MASK);
  Mvpp2PrsHwWrite(priv, &pe);

  /* Ethertype: IPv4 with options */
  tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_FIRST_FREE_TID,
          MVPP2_PE_LAST_FREE_TID);
  if (tid < 0)
    return tid;

  pe.index = tid;

  /* Clear tcam data before updating */
  pe.tcam.byte[MVPP2_PRS_TCAM_DATA_BYTE(MVPP2_ETH_TYPE_LEN)] = 0x0;
  pe.tcam.byte[MVPP2_PRS_TCAM_DATA_BYTE_EN(MVPP2_ETH_TYPE_LEN)] = 0x0;

  Mvpp2PrsTcamDataByteSet(&pe, MVPP2_ETH_TYPE_LEN,
             MVPP2_PRS_IPV4_HEAD,
             MVPP2_PRS_IPV4_HEAD_MASK);

  /* Clear ri before updating */
  pe.sram.word[MVPP2_PRS_SRAM_RI_WORD] = 0x0;
  pe.sram.word[MVPP2_PRS_SRAM_RI_CTRL_WORD] = 0x0;
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L3_IP4_OPT,
         MVPP2_PRS_RI_L3_PROTO_MASK);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_L2);
  priv->PrsShadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
  priv->PrsShadow[pe.index].finish = FALSE;
  Mvpp2PrsShadowRiSet(priv, pe.index, MVPP2_PRS_RI_L3_IP4_OPT,
        MVPP2_PRS_RI_L3_PROTO_MASK);
  Mvpp2PrsHwWrite(priv, &pe);

  /* Ethertype: IPv6 without options */
  tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_FIRST_FREE_TID,
          MVPP2_PE_LAST_FREE_TID);
  if (tid < 0)
    return tid;

  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_L2);
  pe.index = tid;

  Mvpp2PrsMatchEtype(&pe, 0, MV_ETH_P_IPV6);

  /* Skip DIP of IPV6 header */
  Mvpp2PrsSramShiftSet(&pe, MVPP2_ETH_TYPE_LEN + 8 +
         MVPP2_MAX_L3_ADDR_SIZE,
         MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_IP6);
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L3_IP6,
         MVPP2_PRS_RI_L3_PROTO_MASK);
  /* Set L3 offset */
  Mvpp2PrsSramOffsetSet(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
          MVPP2_ETH_TYPE_LEN,
          MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_L2);
  priv->PrsShadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
  priv->PrsShadow[pe.index].finish = FALSE;
  Mvpp2PrsShadowRiSet(priv, pe.index, MVPP2_PRS_RI_L3_IP6,
        MVPP2_PRS_RI_L3_PROTO_MASK);
  Mvpp2PrsHwWrite(priv, &pe);

  /* Default entry for MVPP2_PRS_LU_L2 - Unknown ethtype */
  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_L2);
  pe.index = MVPP2_PE_ETH_TYPE_UN;

  /* Unmask all ports */
  Mvpp2PrsTcamPortMapSet(&pe, MVPP2_PRS_PORT_MASK);

  /* Generate flow in the next iteration*/
  Mvpp2PrsSramBitsSet(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_FLOWS);
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L3_UN,
         MVPP2_PRS_RI_L3_PROTO_MASK);
  /* Set L3 offset even it's unknown L3 */
  Mvpp2PrsSramOffsetSet(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
          MVPP2_ETH_TYPE_LEN,
          MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_L2);
  priv->PrsShadow[pe.index].udf = MVPP2_PRS_UDF_L2_DEF;
  priv->PrsShadow[pe.index].finish = TRUE;
  Mvpp2PrsShadowRiSet(priv, pe.index, MVPP2_PRS_RI_L3_UN,
        MVPP2_PRS_RI_L3_PROTO_MASK);
  Mvpp2PrsHwWrite(priv, &pe);

  return 0;
}

/* Configure vlan entries and detect up to 2 successive VLAN tags.
 * Possible options:
 * 0x8100, 0x88A8
 * 0x8100, 0x8100
 * 0x8100
 * 0x88A8
 */
STATIC INT32 Mvpp2PrsVlanInit(MVPP2_SHARED *priv)
{
  MVPP2_PRS_ENTRY pe;
  INT32 err;

  /* Double VLAN: 0x8100, 0x88A8 */
  err = Mvpp2PrsDoubleVlanAdd(priv, MV_ETH_P_8021Q, MV_ETH_P_8021AD,
          MVPP2_PRS_PORT_MASK);
  if (err)
    return err;

  /* Double VLAN: 0x8100, 0x8100 */
  err = Mvpp2PrsDoubleVlanAdd(priv, MV_ETH_P_8021Q, MV_ETH_P_8021Q,
          MVPP2_PRS_PORT_MASK);
  if (err)
    return err;

  /* Single VLAN: 0x88a8 */
  err = Mvpp2PrsVlanAdd(priv, MV_ETH_P_8021AD,
         MVPP2_PRS_SINGLE_VLAN_AI,
         MVPP2_PRS_PORT_MASK);
  if (err)
    return err;

  /* Single VLAN: 0x8100 */
  err = Mvpp2PrsVlanAdd(priv, MV_ETH_P_8021Q, MVPP2_PRS_SINGLE_VLAN_AI,
         MVPP2_PRS_PORT_MASK);
  if (err)
    return err;

  /* Set default double vlan entry */
  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_VLAN);
  pe.index = MVPP2_PE_VLAN_DBL;

  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_L2);
  /* Clear ai for next iterations */
  Mvpp2PrsSramAiUpdate(&pe, 0, MVPP2_PRS_SRAM_AI_MASK);
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_VLAN_DOUBLE,
         MVPP2_PRS_RI_VLAN_MASK);

  Mvpp2PrsTcamAiUpdate(&pe, MVPP2_PRS_DBL_VLAN_AI_BIT,
         MVPP2_PRS_DBL_VLAN_AI_BIT);
  /* Unmask all ports */
  Mvpp2PrsTcamPortMapSet(&pe, MVPP2_PRS_PORT_MASK);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_VLAN);
  Mvpp2PrsHwWrite(priv, &pe);

  /* Set default vlan none entry */
  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_VLAN);
  pe.index = MVPP2_PE_VLAN_NONE;

  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_L2);
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_VLAN_NONE,
         MVPP2_PRS_RI_VLAN_MASK);

  /* Unmask all ports */
  Mvpp2PrsTcamPortMapSet(&pe, MVPP2_PRS_PORT_MASK);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_VLAN);
  Mvpp2PrsHwWrite(priv, &pe);

  return 0;
}

/* Set entries for PPPoE ethertype */
STATIC INT32 Mvpp2PrsPppoeInit(MVPP2_SHARED *priv)
{
  MVPP2_PRS_ENTRY pe;
  INT32 tid;

  /* IPv4 over PPPoE with options */
  tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_FIRST_FREE_TID,
          MVPP2_PE_LAST_FREE_TID);
  if (tid < 0)
    return tid;

  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_PPPOE);
  pe.index = tid;

  Mvpp2PrsMatchEtype(&pe, 0, MV_PPP_IP);

  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_IP4);
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L3_IP4_OPT,
         MVPP2_PRS_RI_L3_PROTO_MASK);
  /* Skip EthType + 4 bytes of IP header */
  Mvpp2PrsSramShiftSet(&pe, MVPP2_ETH_TYPE_LEN + 4,
         MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
  /* Set L3 offset */
  Mvpp2PrsSramOffsetSet(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
          MVPP2_ETH_TYPE_LEN,
          MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_PPPOE);
  Mvpp2PrsHwWrite(priv, &pe);

  /* IPv4 over PPPoE without options */
  tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_FIRST_FREE_TID,
          MVPP2_PE_LAST_FREE_TID);
  if (tid < 0)
    return tid;

  pe.index = tid;

  Mvpp2PrsTcamDataByteSet(&pe, MVPP2_ETH_TYPE_LEN,
             MVPP2_PRS_IPV4_HEAD | MVPP2_PRS_IPV4_IHL,
             MVPP2_PRS_IPV4_HEAD_MASK |
             MVPP2_PRS_IPV4_IHL_MASK);

  /* Clear ri before updating */
  pe.sram.word[MVPP2_PRS_SRAM_RI_WORD] = 0x0;
  pe.sram.word[MVPP2_PRS_SRAM_RI_CTRL_WORD] = 0x0;
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L3_IP4,
         MVPP2_PRS_RI_L3_PROTO_MASK);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_PPPOE);
  Mvpp2PrsHwWrite(priv, &pe);

  /* IPv6 over PPPoE */
  tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_FIRST_FREE_TID,
          MVPP2_PE_LAST_FREE_TID);
  if (tid < 0)
    return tid;

  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_PPPOE);
  pe.index = tid;

  Mvpp2PrsMatchEtype(&pe, 0, MV_PPP_IPV6);

  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_IP6);
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L3_IP6,
         MVPP2_PRS_RI_L3_PROTO_MASK);
  /* Skip EthType + 4 bytes of IPv6 header */
  Mvpp2PrsSramShiftSet(&pe, MVPP2_ETH_TYPE_LEN + 4,
         MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
  /* Set L3 offset */
  Mvpp2PrsSramOffsetSet(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
          MVPP2_ETH_TYPE_LEN,
          MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_PPPOE);
  Mvpp2PrsHwWrite(priv, &pe);

  /* Non-IP over PPPoE */
  tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_FIRST_FREE_TID,
          MVPP2_PE_LAST_FREE_TID);
  if (tid < 0)
    return tid;

  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_PPPOE);
  pe.index = tid;

  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L3_UN,
         MVPP2_PRS_RI_L3_PROTO_MASK);

  /* Finished: go to flowid generation */
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_FLOWS);
  Mvpp2PrsSramBitsSet(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
  /* Set L3 offset even if it's unknown L3 */
  Mvpp2PrsSramOffsetSet(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L3,
          MVPP2_ETH_TYPE_LEN,
          MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_PPPOE);
  Mvpp2PrsHwWrite(priv, &pe);

  return 0;
}

/* Initialize entries for IPv4 */
STATIC INT32 Mvpp2PrsIp4Init(MVPP2_SHARED *priv)
{
  MVPP2_PRS_ENTRY pe;
  INT32 err;

  /* Set entries for TCP, UDP and IGMP over IPv4 */
  err = Mvpp2PrsIp4Proto(priv, MV_IPPR_TCP, MVPP2_PRS_RI_L4_TCP,
          MVPP2_PRS_RI_L4_PROTO_MASK);
  if (err)
    return err;

  err = Mvpp2PrsIp4Proto(priv, MV_IPPR_UDP, MVPP2_PRS_RI_L4_UDP,
          MVPP2_PRS_RI_L4_PROTO_MASK);
  if (err)
    return err;

  err = Mvpp2PrsIp4Proto(priv, MV_IPPR_IGMP,
          MVPP2_PRS_RI_CPU_CODE_RX_SPEC |
          MVPP2_PRS_RI_UDF3_RX_SPECIAL,
          MVPP2_PRS_RI_CPU_CODE_MASK |
          MVPP2_PRS_RI_UDF3_MASK);
  if (err)
    return err;

  /* IPv4 Broadcast */
  err = Mvpp2PrsIp4Cast(priv, MVPP2_PRS_L3_BROAD_CAST);
  if (err)
    return err;

  /* IPv4 Multicast */
  err = Mvpp2PrsIp4Cast(priv, MVPP2_PRS_L3_MULTI_CAST);
  if (err)
    return err;

  /* Default IPv4 entry for unknown protocols */
  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_IP4);
  pe.index = MVPP2_PE_IP4_PROTO_UN;

  /* Set next lu to IPv4 */
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_IP4);
  Mvpp2PrsSramShiftSet(&pe, 12, MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);
  /* Set L4 offset */
  Mvpp2PrsSramOffsetSet(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L4,
          sizeof(Mvpp2Iphdr) - 4,
          MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);
  Mvpp2PrsSramAiUpdate(&pe, MVPP2_PRS_IPV4_DIP_AI_BIT,
         MVPP2_PRS_IPV4_DIP_AI_BIT);
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L4_OTHER,
         MVPP2_PRS_RI_L4_PROTO_MASK);

  Mvpp2PrsTcamAiUpdate(&pe, 0, MVPP2_PRS_IPV4_DIP_AI_BIT);
  /* Unmask all ports */
  Mvpp2PrsTcamPortMapSet(&pe, MVPP2_PRS_PORT_MASK);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_IP4);
  Mvpp2PrsHwWrite(priv, &pe);

  /* Default IPv4 entry for unicast address */
  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_IP4);
  pe.index = MVPP2_PE_IP4_ADDR_UN;

  /* Finished: go to flowid generation */
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_FLOWS);
  Mvpp2PrsSramBitsSet(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L3_UCAST,
         MVPP2_PRS_RI_L3_ADDR_MASK);

  Mvpp2PrsTcamAiUpdate(&pe, MVPP2_PRS_IPV4_DIP_AI_BIT,
         MVPP2_PRS_IPV4_DIP_AI_BIT);
  /* Unmask all ports */
  Mvpp2PrsTcamPortMapSet(&pe, MVPP2_PRS_PORT_MASK);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_IP4);
  Mvpp2PrsHwWrite(priv, &pe);

  return 0;
}

/* Initialize entries for IPv6 */
STATIC INT32 Mvpp2PrsIp6Init(MVPP2_SHARED *priv)
{
  MVPP2_PRS_ENTRY pe;
  INT32 tid, err;

  /* Set entries for TCP, UDP and ICMP over IPv6 */
  err = Mvpp2PrsIp6Proto(priv, MV_IPPR_TCP,
          MVPP2_PRS_RI_L4_TCP,
          MVPP2_PRS_RI_L4_PROTO_MASK);
  if (err)
    return err;

  err = Mvpp2PrsIp6Proto(priv, MV_IPPR_UDP,
          MVPP2_PRS_RI_L4_UDP,
          MVPP2_PRS_RI_L4_PROTO_MASK);
  if (err)
    return err;

  err = Mvpp2PrsIp6Proto(priv, MV_IPPR_ICMPV6,
          MVPP2_PRS_RI_CPU_CODE_RX_SPEC |
          MVPP2_PRS_RI_UDF3_RX_SPECIAL,
          MVPP2_PRS_RI_CPU_CODE_MASK |
          MVPP2_PRS_RI_UDF3_MASK);
  if (err)
    return err;

  /* IPv4 is the last header. This is similar case as 6-TCP or 17-UDP */
  /* Result Info: UDF7=1, DS lite */
  err = Mvpp2PrsIp6Proto(priv, MV_IPPR_IPIP,
          MVPP2_PRS_RI_UDF7_IP6_LITE,
          MVPP2_PRS_RI_UDF7_MASK);
  if (err)
    return err;

  /* IPv6 multicast */
  err = Mvpp2PrsIp6Cast(priv, MVPP2_PRS_L3_MULTI_CAST);
  if (err)
    return err;

  /* Entry for checking hop limit */
  tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_FIRST_FREE_TID,
          MVPP2_PE_LAST_FREE_TID);
  if (tid < 0)
    return tid;

  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_IP6);
  pe.index = tid;

  /* Finished: go to flowid generation */
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_FLOWS);
  Mvpp2PrsSramBitsSet(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L3_UN |
         MVPP2_PRS_RI_DROP_MASK,
         MVPP2_PRS_RI_L3_PROTO_MASK |
         MVPP2_PRS_RI_DROP_MASK);

  Mvpp2PrsTcamDataByteSet(&pe, 1, 0x00, MVPP2_PRS_IPV6_HOP_MASK);
  Mvpp2PrsTcamAiUpdate(&pe, MVPP2_PRS_IPV6_NO_EXT_AI_BIT,
         MVPP2_PRS_IPV6_NO_EXT_AI_BIT);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_IP4);
  Mvpp2PrsHwWrite(priv, &pe);

  /* Default IPv6 entry for unknown protocols */
  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_IP6);
  pe.index = MVPP2_PE_IP6_PROTO_UN;

  /* Finished: go to flowid generation */
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_FLOWS);
  Mvpp2PrsSramBitsSet(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L4_OTHER,
         MVPP2_PRS_RI_L4_PROTO_MASK);
  /* Set L4 offset relatively to our current place */
  Mvpp2PrsSramOffsetSet(&pe, MVPP2_PRS_SRAM_UDF_TYPE_L4,
          sizeof(Mvpp2Ipv6hdr) - 4,
          MVPP2_PRS_SRAM_OP_SEL_UDF_ADD);

  Mvpp2PrsTcamAiUpdate(&pe, MVPP2_PRS_IPV6_NO_EXT_AI_BIT,
         MVPP2_PRS_IPV6_NO_EXT_AI_BIT);
  /* Unmask all ports */
  Mvpp2PrsTcamPortMapSet(&pe, MVPP2_PRS_PORT_MASK);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_IP4);
  Mvpp2PrsHwWrite(priv, &pe);

  /* Default IPv6 entry for unknown ext protocols */
  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_IP6);
  pe.index = MVPP2_PE_IP6_EXT_PROTO_UN;

  /* Finished: go to flowid generation */
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_FLOWS);
  Mvpp2PrsSramBitsSet(&pe, MVPP2_PRS_SRAM_LU_GEN_BIT, 1);
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L4_OTHER,
         MVPP2_PRS_RI_L4_PROTO_MASK);

  Mvpp2PrsTcamAiUpdate(&pe, MVPP2_PRS_IPV6_EXT_AI_BIT,
         MVPP2_PRS_IPV6_EXT_AI_BIT);
  /* Unmask all ports */
  Mvpp2PrsTcamPortMapSet(&pe, MVPP2_PRS_PORT_MASK);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_IP4);
  Mvpp2PrsHwWrite(priv, &pe);

  /* Default IPv6 entry for unicast address */
  Mvpp2Memset(&pe, 0, sizeof(MVPP2_PRS_ENTRY));
  Mvpp2PrsTcamLuSet(&pe, MVPP2_PRS_LU_IP6);
  pe.index = MVPP2_PE_IP6_ADDR_UN;

  /* Finished: go to IPv6 again */
  Mvpp2PrsSramNextLuSet(&pe, MVPP2_PRS_LU_IP6);
  Mvpp2PrsSramRiUpdate(&pe, MVPP2_PRS_RI_L3_UCAST,
         MVPP2_PRS_RI_L3_ADDR_MASK);
  Mvpp2PrsSramAiUpdate(&pe, MVPP2_PRS_IPV6_NO_EXT_AI_BIT,
         MVPP2_PRS_IPV6_NO_EXT_AI_BIT);
  /* Shift back to IPV6 NH */
  Mvpp2PrsSramShiftSet(&pe, -18, MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

  Mvpp2PrsTcamAiUpdate(&pe, 0, MVPP2_PRS_IPV6_NO_EXT_AI_BIT);
  /* Unmask all ports */
  Mvpp2PrsTcamPortMapSet(&pe, MVPP2_PRS_PORT_MASK);

  /* Update shadow table and hw entry */
  Mvpp2PrsShadowSet(priv, pe.index, MVPP2_PRS_LU_IP6);
  Mvpp2PrsHwWrite(priv, &pe);

  return 0;
}

/* Parser default initialization */
INT32 Mvpp2PrsDefaultInit(MVPP2_SHARED *priv)
{
  INT32 err, index, i;

  /* Enable tcam table */
  Mvpp2Write(priv, MVPP2_PRS_TCAM_CTRL_REG, MVPP2_PRS_TCAM_EN_MASK);

  /* Clear all tcam and sram entries */
  for (index = 0; index < MVPP2_PRS_TCAM_SRAM_SIZE; index++) {
    Mvpp2Write(priv, MVPP2_PRS_TCAM_IDX_REG, index);
    for (i = 0; i < MVPP2_PRS_TCAM_WORDS; i++)
      Mvpp2Write(priv, MVPP2_PRS_TCAM_DATA_REG(i), 0);

    Mvpp2Write(priv, MVPP2_PRS_SRAM_IDX_REG, index);
    for (i = 0; i < MVPP2_PRS_SRAM_WORDS; i++)
      Mvpp2Write(priv, MVPP2_PRS_SRAM_DATA_REG(i), 0);
  }

  /* Invalidate all tcam entries */
  for (index = 0; index < MVPP2_PRS_TCAM_SRAM_SIZE; index++)
    Mvpp2PrsHwInv(priv, index);

  /* Always start from lookup = 0 */
  for (index = 0; index < MVPP2_MAX_PORTS; index++)
    Mvpp2PrsHwPortInit(priv, index, MVPP2_PRS_LU_MH,
               MVPP2_PRS_PORT_LU_MAX, 0);

  Mvpp2PrsDefFlowInit(priv);

  Mvpp2PrsMhInit(priv);

  Mvpp2PrsMacInit(priv);

  Mvpp2PrsDsaInit(priv);

  err = Mvpp2PrsEtypeInit(priv);
  if (err)
    return err;

  err = Mvpp2PrsVlanInit(priv);
  if (err)
    return err;

  err = Mvpp2PrsPppoeInit(priv);
  if (err)
    return err;

  err = Mvpp2PrsIp6Init(priv);
  if (err)
    return err;

  err = Mvpp2PrsIp4Init(priv);
  if (err)
    return err;

  return 0;
}

/* Compare MAC DA with tcam entry data */
STATIC BOOLEAN Mvpp2PrsMacRangeEquals(MVPP2_PRS_ENTRY *pe,
               const UINT8 *da, UINT8 *mask)
{
  UINT8 TcamByte, TcamMask;
  INT32 index;

  for (index = 0; index < MV_ETH_ALEN; index++) {
    Mvpp2PrsTcamDataByteGet(pe, index, &TcamByte, &TcamMask);
    if (TcamMask != mask[index])
      return FALSE;

    if ((TcamMask & TcamByte) != (da[index] & mask[index]))
      return FALSE;
  }

  return TRUE;
}

/* Find tcam entry with matched pair <MAC DA, port> */
STATIC MVPP2_PRS_ENTRY *
Mvpp2PrsMacDaRangeFind(MVPP2_SHARED *priv, INT32 pmap, const UINT8 *da,
          UINT8 *mask, INT32 UdfType)
{
  MVPP2_PRS_ENTRY *pe;
  INT32 tid;

  pe = Mvpp2Alloc(sizeof(*pe));
  if (!pe)
    return MVPP2_NULL;
  Mvpp2PrsTcamLuSet(pe, MVPP2_PRS_LU_MAC);

  /* Go through the all entires with MVPP2_PRS_LU_MAC */
  for (tid = MVPP2_PE_FIRST_FREE_TID;
       tid <= MVPP2_PE_LAST_FREE_TID; tid++) {
    UINT32 EntryPmap;

    if (!priv->PrsShadow[tid].valid ||
        (priv->PrsShadow[tid].lu != MVPP2_PRS_LU_MAC) ||
        (priv->PrsShadow[tid].udf != UdfType))
      continue;

    pe->index = tid;
    Mvpp2PrsHwRead(priv, pe);
    EntryPmap = Mvpp2PrsTcamPortMapGet(pe);

    if (Mvpp2PrsMacRangeEquals(pe, da, mask) &&
        EntryPmap == pmap)
      return pe;
  }
  Mvpp2Free(pe);

  return MVPP2_NULL;
}

/* Update parser's mac da entry */
INT32 Mvpp2PrsMacDaAccept(MVPP2_SHARED *priv, INT32 port,
          const UINT8 *da, BOOLEAN add)
{
  MVPP2_PRS_ENTRY *pe;
  UINT32 pmap, len, ri;
  UINT8 mask[MV_ETH_ALEN] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
  INT32 tid;

  /* Scan TCAM and see if entry with this <MAC DA, port> already exist */
  pe = Mvpp2PrsMacDaRangeFind(priv, (1 << port), da, mask,
           MVPP2_PRS_UDF_MAC_DEF);

  /* No such entry */
  if (!pe) {
    if (!add)
      return 0;

    /* Create new TCAM entry */
    /* Find first range mac entry*/
    for (tid = MVPP2_PE_FIRST_FREE_TID;
         tid <= MVPP2_PE_LAST_FREE_TID; tid++)
      if (priv->PrsShadow[tid].valid &&
          (priv->PrsShadow[tid].lu == MVPP2_PRS_LU_MAC) &&
          (priv->PrsShadow[tid].udf ==
                   MVPP2_PRS_UDF_MAC_RANGE))
        break;

    /* Go through the all entries from first to last */
    tid = Mvpp2PrsTcamFirstFree(priv, MVPP2_PE_FIRST_FREE_TID,
            tid - 1);
    if (tid < 0)
      return tid;

    pe = Mvpp2Alloc(sizeof(*pe));
    if (!pe)
      return -1;
    Mvpp2PrsTcamLuSet(pe, MVPP2_PRS_LU_MAC);
    pe->index = tid;

    /* Mask all ports */
    Mvpp2PrsTcamPortMapSet(pe, 0);
  }

  /* Update port mask */
  Mvpp2PrsTcamPortSet(pe, port, add);

  /* Invalidate the entry if no ports are left enabled */
  pmap = Mvpp2PrsTcamPortMapGet(pe);
  if (pmap == 0) {
    if (add) {
      Mvpp2Free(pe);
      return -1;
    }
    Mvpp2PrsHwInv(priv, pe->index);
    priv->PrsShadow[pe->index].valid = FALSE;
    Mvpp2Free(pe);
    return 0;
  }

  /* Continue - set next lookup */
  Mvpp2PrsSramNextLuSet(pe, MVPP2_PRS_LU_DSA);

  /* Set match on DA */
  len = MV_ETH_ALEN;
  while (len--)
    Mvpp2PrsTcamDataByteSet(pe, len, da[len], 0xff);

  /* Set result info bits */
  if (Mvpp2IsBroadcastEtherAddr(da))
    ri = MVPP2_PRS_RI_L2_BCAST;
  else if (Mvpp2IsMulticastEtherAddr(da))
    ri = MVPP2_PRS_RI_L2_MCAST;
  else
    ri = MVPP2_PRS_RI_L2_UCAST | MVPP2_PRS_RI_MAC_ME_MASK;

  Mvpp2PrsSramRiUpdate(pe, ri, MVPP2_PRS_RI_L2_CAST_MASK |
         MVPP2_PRS_RI_MAC_ME_MASK);
  Mvpp2PrsShadowRiSet(priv, pe->index, ri, MVPP2_PRS_RI_L2_CAST_MASK |
        MVPP2_PRS_RI_MAC_ME_MASK);

  /* Shift to ethertype */
  Mvpp2PrsSramShiftSet(pe, 2 * MV_ETH_ALEN,
         MVPP2_PRS_SRAM_OP_SEL_SHIFT_ADD);

  /* Update shadow table and hw entry */
  priv->PrsShadow[pe->index].udf = MVPP2_PRS_UDF_MAC_DEF;
  Mvpp2PrsShadowSet(priv, pe->index, MVPP2_PRS_LU_MAC);
  Mvpp2PrsHwWrite(priv, pe);

  Mvpp2Free(pe);

  return 0;
}

/* Delete all port's multicast simple (not range) entries */
VOID Mvpp2PrsMcastDelAll(MVPP2_SHARED *priv, INT32 port)
{
  MVPP2_PRS_ENTRY pe;
  INT32 index, tid;

  for (tid = MVPP2_PE_FIRST_FREE_TID;
       tid <= MVPP2_PE_LAST_FREE_TID; tid++) {
    UINT8 da[MV_ETH_ALEN], DaMask[MV_ETH_ALEN];

    if (!priv->PrsShadow[tid].valid ||
        (priv->PrsShadow[tid].lu != MVPP2_PRS_LU_MAC) ||
        (priv->PrsShadow[tid].udf != MVPP2_PRS_UDF_MAC_DEF))
      continue;

    /* Only simple mac entries */
    pe.index = tid;
    Mvpp2PrsHwRead(priv, &pe);

    /* Read mac addr from entry */
    for (index = 0; index < MV_ETH_ALEN; index++)
      Mvpp2PrsTcamDataByteGet(&pe, index, &da[index],
                 &DaMask[index]);

    if (Mvpp2IsMulticastEtherAddr(da) &&
        !Mvpp2IsBroadcastEtherAddr(da))
      /* Delete this entry */
      Mvpp2PrsMacDaAccept(priv, port, da, FALSE);
  }
}

INT32 Mvpp2PrsTagModeSet(MVPP2_SHARED *priv, INT32 port, INT32 type)
{
  switch (type) {
  case MVPP2_TAG_TYPE_EDSA:
    /* Add port to EDSA entries */
    Mvpp2PrsDsaTagSet(priv, port, TRUE,
              MVPP2_PRS_TAGGED, MVPP2_PRS_EDSA);
    Mvpp2PrsDsaTagSet(priv, port, TRUE,
              MVPP2_PRS_UNTAGGED, MVPP2_PRS_EDSA);
    /* Remove port from DSA entries */
    Mvpp2PrsDsaTagSet(priv, port, FALSE,
              MVPP2_PRS_TAGGED, MVPP2_PRS_DSA);
    Mvpp2PrsDsaTagSet(priv, port, FALSE,
              MVPP2_PRS_UNTAGGED, MVPP2_PRS_DSA);
    break;

  case MVPP2_TAG_TYPE_DSA:
    /* Add port to DSA entries */
    Mvpp2PrsDsaTagSet(priv, port, TRUE,
              MVPP2_PRS_TAGGED, MVPP2_PRS_DSA);
    Mvpp2PrsDsaTagSet(priv, port, TRUE,
              MVPP2_PRS_UNTAGGED, MVPP2_PRS_DSA);
    /* Remove port from EDSA entries */
    Mvpp2PrsDsaTagSet(priv, port, FALSE,
              MVPP2_PRS_TAGGED, MVPP2_PRS_EDSA);
    Mvpp2PrsDsaTagSet(priv, port, FALSE,
              MVPP2_PRS_UNTAGGED, MVPP2_PRS_EDSA);
    break;

  case MVPP2_TAG_TYPE_MH:
  case MVPP2_TAG_TYPE_NONE:
    /* Remove port form EDSA and DSA entries */
    Mvpp2PrsDsaTagSet(priv, port, FALSE,
              MVPP2_PRS_TAGGED, MVPP2_PRS_DSA);
    Mvpp2PrsDsaTagSet(priv, port, FALSE,
              MVPP2_PRS_UNTAGGED, MVPP2_PRS_DSA);
    Mvpp2PrsDsaTagSet(priv, port, FALSE,
              MVPP2_PRS_TAGGED, MVPP2_PRS_EDSA);
    Mvpp2PrsDsaTagSet(priv, port, FALSE,
              MVPP2_PRS_UNTAGGED, MVPP2_PRS_EDSA);
    break;

  default:
    if ((type < 0) || (type > MVPP2_TAG_TYPE_EDSA))
      return MVPP2_EINVAL;
  }

  return 0;
}

/* Set prs flow for the port */
INT32 Mvpp2PrsDefFlow(PP2DXE_PORT *port)
{
  MVPP2_PRS_ENTRY *pe;
  INT32 tid;

  pe = Mvpp2PrsFlowFind(port->priv, port->id);

  /* Such entry not exist */
  if (!pe) {
    /* Go through the all entires from last to first */
    tid = Mvpp2PrsTcamFirstFree(port->priv,
            MVPP2_PE_LAST_FREE_TID,
            MVPP2_PE_FIRST_FREE_TID);
    if (tid < 0)
      return tid;

    pe = Mvpp2Alloc(sizeof(*pe));
    if (!pe)
      return MVPP2_ENOMEM;

    Mvpp2PrsTcamLuSet(pe, MVPP2_PRS_LU_FLOWS);
    pe->index = tid;

    /* Set flow ID*/
    Mvpp2PrsSramAiUpdate(pe, port->id, MVPP2_PRS_FLOW_ID_MASK);
    Mvpp2PrsSramBitsSet(pe, MVPP2_PRS_SRAM_LU_DONE_BIT, 1);

    /* Update shadow table */
    Mvpp2PrsShadowSet(port->priv, pe->index, MVPP2_PRS_LU_FLOWS);
  }

  Mvpp2PrsTcamPortMapSet(pe, (1 << port->id));
  Mvpp2PrsHwWrite(port->priv, pe);
  Mvpp2Free(pe);

  return 0;
}

/* Classifier configuration routines */

/* Update classification flow table registers */
STATIC VOID Mvpp2ClsFlowWrite(MVPP2_SHARED *priv,
         MVPP2_CLS_FLOW_ENTRY *fe)
{
  Mvpp2Write(priv, MVPP2_CLS_FLOW_INDEX_REG, fe->index);
  Mvpp2Write(priv, MVPP2_CLS_FLOW_TBL0_REG,  fe->data[0]);
  Mvpp2Write(priv, MVPP2_CLS_FLOW_TBL1_REG,  fe->data[1]);
  Mvpp2Write(priv, MVPP2_CLS_FLOW_TBL2_REG,  fe->data[2]);
}

/* Update classification lookup table register */
VOID Mvpp2ClsLookupWrite(MVPP2_SHARED *priv,
          MVPP2_CLS_LOOKUP_ENTRY *le)
{
  UINT32 val;

  val = (le->way << MVPP2_CLS_LKP_INDEX_WAY_OFFS) | le->lkpid;
  Mvpp2Write(priv, MVPP2_CLS_LKP_INDEX_REG, val);
  Mvpp2Write(priv, MVPP2_CLS_LKP_TBL_REG, le->data);
}

/* Classifier default initialization */
VOID Mvpp2ClsInit(MVPP2_SHARED *priv)
{
  MVPP2_CLS_LOOKUP_ENTRY le;
  MVPP2_CLS_FLOW_ENTRY fe;
  INT32 index;

  /* Enable classifier */
  Mvpp2Write(priv, MVPP2_CLS_MODE_REG, MVPP2_CLS_MODE_ACTIVE_MASK);

  /* Clear classifier flow table */
  Mvpp2Memset(&fe.data, 0, MVPP2_CLS_FLOWS_TBL_DATA_WORDS);
  for (index = 0; index < MVPP2_CLS_FLOWS_TBL_SIZE; index++) {
    fe.index = index;
    Mvpp2ClsFlowWrite(priv, &fe);
  }

  /* Clear classifier lookup table */
  le.data = 0;
  for (index = 0; index < MVPP2_CLS_LKP_TBL_SIZE; index++) {
    le.lkpid = index;
    le.way = 0;
    Mvpp2ClsLookupWrite(priv, &le);

    le.way = 1;
    Mvpp2ClsLookupWrite(priv, &le);
  }
}

VOID Mvpp2ClsPortConfig(PP2DXE_PORT *port)
{
  MVPP2_CLS_LOOKUP_ENTRY le;
  UINT32 val;

  /* Set way for the port */
  val = Mvpp2Read(port->priv, MVPP2_CLS_PORT_WAY_REG);
  val &= ~MVPP2_CLS_PORT_WAY_MASK(port->id);
  Mvpp2Write(port->priv, MVPP2_CLS_PORT_WAY_REG, val);

  /* Pick the entry to be accessed in lookup ID decoding table
   * according to the way and lkpid.
   */
  le.lkpid = port->id;
  le.way = 0;
  le.data = 0;

  /* Set initial CPU queue for receiving packets */
  le.data &= ~MVPP2_CLS_LKP_TBL_RXQ_MASK;
  le.data |= port->FirstRxq;

  /* Disable classification engines */
  le.data &= ~MVPP2_CLS_LKP_TBL_LOOKUP_EN_MASK;

  /* Update lookup ID table entry */
  Mvpp2ClsLookupWrite(port->priv, &le);
}

/* Set CPU queue number for oversize packets */
VOID Mvpp2ClsOversizeRxqSet(PP2DXE_PORT *port)
{

  Mvpp2Write(port->priv, MVPP2_CLS_OVERSIZE_RXQ_LOW_REG(port->id),
        port->FirstRxq & MVPP2_CLS_OVERSIZE_RXQ_LOW_MASK);

#ifdef MVPP2_V1
  Mvpp2Write(port->priv, MVPP2_CLS_SWFWD_P2HQ_REG(port->id),
        (port->FirstRxq >> MVPP2_CLS_OVERSIZE_RXQ_LOW_BITS));

  val = Mvpp2Read(port->priv, MVPP2_CLS_SWFWD_PCTRL_REG);
  val |= MVPP2_CLS_SWFWD_PCTRL_MASK(port->id);
  Mvpp2Write(port->priv, MVPP2_CLS_SWFWD_PCTRL_REG, val);
#endif
}

/* BM helper routines */


VOID Mvpp2BmPoolHwCreate(MVPP2_SHARED *priv,
           MVPP2_BMS_POOL *BmPool, INT32 size)
{
#ifdef MVPP2_V1
  UINT32 val;

  Mvpp2Write(priv, MVPP2_BM_POOL_BASE_REG(BmPool->id),
        BmPool->PhysAddr);
  Mvpp2Write(priv, MVPP2_BM_POOL_SIZE_REG(BmPool->id), size);

  val = Mvpp2Read(priv, MVPP2_BM_POOL_CTRL_REG(BmPool->id));
  val |= MVPP2_BM_START_MASK;
  Mvpp2Write(priv, MVPP2_BM_POOL_CTRL_REG(BmPool->id), val);

  BmPool->type = MVPP2_BM_FREE;
  BmPool->size = size;
  BmPool->PktSize = 0;
  BmPool->BufNum = 0;
#else
  BmPool->size = size;

  Mvpp2Write(priv, MVPP2_BM_POOL_BASE_REG(BmPool->id),
        Lower32Bits(BmPool->PhysAddr));

  Mvpp2Write(priv, MVPP22_BM_POOL_BASE_HIGH_REG,
  (Upper32Bits(BmPool->PhysAddr)&
      MVPP22_BM_POOL_BASE_HIGH_REG));
  Mvpp2Write(priv, MVPP2_BM_POOL_SIZE_REG(BmPool->id),
          BmPool->size);
#endif
}

/* Set pool buffer size */
VOID Mvpp2BmPoolBufsizeSet(MVPP2_SHARED *priv,
             MVPP2_BMS_POOL *BmPool,
             INT32 BufSize)
{
  UINT32 val;

  BmPool->BufSize = BufSize;

  val = MVPP2_ALIGN(BufSize, 1 << MVPP2_POOL_BUF_SIZE_OFFSET);
  Mvpp2Write(priv, MVPP2_POOL_BUF_SIZE_REG(BmPool->id), val);
}

VOID Mvpp2BmStop(MVPP2_SHARED *priv, INT32 pool)
{
  UINT32 val, i;

  for (i = 0; i < MVPP2_BM_SIZE; i++)
    Mvpp2Read(priv, MVPP2_BM_PHY_ALLOC_REG(0));

  val = Mvpp2Read(priv, MVPP2_BM_POOL_CTRL_REG(pool));
  val |= MVPP2_BM_STOP_MASK;
  Mvpp2Write(priv, MVPP2_BM_POOL_CTRL_REG(pool), val);

}

VOID Mvpp2BmIrqClear(MVPP2_SHARED *priv, INT32 pool)
{
  /* Mask BM all interrupts */
  Mvpp2Write(priv, MVPP2_BM_INTR_MASK_REG(pool), 0);
  /* Clear BM cause register */
  Mvpp2Write(priv, MVPP2_BM_INTR_CAUSE_REG(pool), 0);
}

/* Attach long pool to rxq */
VOID Mvpp2RxqLongPoolSet(PP2DXE_PORT *port,
           INT32 lrxq, INT32 LongPool)
{
  UINT32 val;
  INT32 prxq;

  /* Get queue physical ID */
  prxq = port->rxqs[lrxq].id;

  val = Mvpp2Read(port->priv, MVPP2_RXQ_CONFIG_REG(prxq));
  val &= ~MVPP2_RXQ_POOL_LONG_MASK;
  val |= ((LongPool << MVPP2_RXQ_POOL_LONG_OFFS) &
        MVPP2_RXQ_POOL_LONG_MASK);

  Mvpp2Write(port->priv, MVPP2_RXQ_CONFIG_REG(prxq), val);
}

/* Attach short pool to rxq */
VOID Mvpp2RxqShortPoolSet(PP2DXE_PORT *port,
            INT32 lrxq, INT32 ShortPool)
{
  UINT32 val;
  INT32 prxq;

  /* Get queue physical ID */
  prxq = port->rxqs[lrxq].id;

  val = Mvpp2Read(port->priv, MVPP2_RXQ_CONFIG_REG(prxq));
  val &= ~MVPP2_RXQ_POOL_SHORT_MASK;
  val |= ((ShortPool << MVPP2_RXQ_POOL_SHORT_OFFS) &
        MVPP2_RXQ_POOL_SHORT_MASK);

  Mvpp2Write(port->priv, MVPP2_RXQ_CONFIG_REG(prxq), val);
}

/* Release multicast buffer */
VOID Mvpp2BmPoolMcPut(PP2DXE_PORT *port, INT32 pool,
        UINT32 BufPhysAddr, UINT32 BufVirtAddr,
        INT32 McId)
{
  UINT32 val = 0;

  val |= (McId & MVPP2_BM_MC_ID_MASK);
  Mvpp2Write(port->priv, MVPP2_BM_MC_RLS_REG, val);

  Mvpp2BmPoolPut(port->priv, pool,
        BufPhysAddr | MVPP2_BM_PHY_RLS_MC_BUFF_MASK,
        BufVirtAddr);
}

/* Refill BM pool */
VOID Mvpp2PoolRefill(PP2DXE_PORT *port, UINT32 bm,
           UINT32 PhysAddr, UINT32 cookie)
{
  INT32 pool = Mvpp2BmCookiePoolGet(bm);

  Mvpp2BmPoolPut(port->priv, pool, PhysAddr, cookie);
}

/* Mask the current CPU's Rx/Tx interrupts */
VOID Mvpp2InterruptsMask(VOID *arg)
{
  PP2DXE_PORT *port = arg;

  Mvpp2Write(port->priv, MVPP2_ISR_RX_TX_MASK_REG(port->id), 0);
}

/* Unmask the current CPU's Rx/Tx interrupts */
VOID Mvpp2InterruptsUnmask(VOID *arg)
{
  PP2DXE_PORT *port = arg;

  Mvpp2Write(port->priv, MVPP2_ISR_RX_TX_MASK_REG(port->id),
        (MVPP2_CAUSE_MISC_SUM_MASK |
         MVPP2_CAUSE_RXQ_OCCUP_DESC_ALL_MASK));
}

/* Port configuration routines */

STATIC VOID Mvpp2PortMiiSet(PP2DXE_PORT *port)
{
  UINT32 val;

  val = Mvpp2GmacRead(port, MVPP2_GMAC_CTRL_2_REG);

  switch (port->PhyInterface) {
  case MV_MODE_SGMII:
    val |= MVPP2_GMAC_INBAND_AN_MASK;
    break;
  case MV_MODE_RGMII:
    val |= MVPP2_GMAC_PORT_RGMII_MASK;
  default:
    val &= ~MVPP2_GMAC_PCS_ENABLE_MASK;
  }

  Mvpp2GmacWrite(port, MVPP2_GMAC_CTRL_2_REG, val);
}

STATIC VOID Mvpp2PortFcAdvEnable(PP2DXE_PORT *port)
{
  UINT32 val;

  val = Mvpp2GmacRead(port, MVPP2_GMAC_AUTONEG_CONFIG);
  val |= MVPP2_GMAC_FC_ADV_EN;
  Mvpp2GmacWrite(port, MVPP2_GMAC_AUTONEG_CONFIG, val);
}

VOID Mvpp2PortEnable(PP2DXE_PORT *port)
{
  UINT32 val;

  val = Mvpp2GmacRead(port, MVPP2_GMAC_CTRL_0_REG);
  val |= MVPP2_GMAC_PORT_EN_MASK;
  val |= MVPP2_GMAC_MIB_CNTR_EN_MASK;
  Mvpp2GmacWrite(port, MVPP2_GMAC_CTRL_0_REG, val);
}

VOID Mvpp2PortDisable(PP2DXE_PORT *port)
{
  UINT32 val;

  val = Mvpp2GmacRead(port, MVPP2_GMAC_CTRL_0_REG);
  val &= ~(MVPP2_GMAC_PORT_EN_MASK);
  Mvpp2GmacWrite(port, MVPP2_GMAC_CTRL_0_REG, val);
}

/* Set IEEE 802.3x Flow Control Xon Packet Transmission Mode */
STATIC VOID Mvpp2PortPeriodicXonDisable(PP2DXE_PORT *port)
{
  UINT32 val;

  val = Mvpp2GmacRead(port, MVPP2_GMAC_CTRL_1_REG) &
            ~MVPP2_GMAC_PERIODIC_XON_EN_MASK;
  Mvpp2GmacWrite(port, MVPP2_GMAC_CTRL_1_REG, val);
}

/* Configure loopback port */
#ifdef MVPP2_V1
STATIC VOID Mvpp2PortLoopbackSet(PP2DXE_PORT *port)
{
  UINT32 val;

  val = Mvpp2GmacRead(port, MVPP2_GMAC_CTRL_1_REG);

  if (port->speed == SPEED_1000)
    val |= MVPP2_GMAC_GMII_LB_EN_MASK;
  else
    val &= ~MVPP2_GMAC_GMII_LB_EN_MASK;

  if (port->PhyInterface == MV_MODE_SGMII)
    val |= MVPP2_GMAC_PCS_LB_EN_MASK;
  else
    val &= ~MVPP2_GMAC_PCS_LB_EN_MASK;

  Mvpp2GmacWrite(port, MVPP2_GMAC_CTRL_1_REG, val);
}
#endif

STATIC VOID Mvpp2PortReset(PP2DXE_PORT *port)
{
  UINT32 val;

  val = Mvpp2GmacRead(port, MVPP2_GMAC_CTRL_2_REG) &
        ~MVPP2_GMAC_PORT_RESET_MASK;
  Mvpp2GmacWrite(port, MVPP2_GMAC_CTRL_2_REG, val);

  while (Mvpp2GmacRead(port, MVPP2_GMAC_CTRL_2_REG) &
         MVPP2_GMAC_PORT_RESET_MASK)
    continue;
}

/* Set defaults to the MVPP2 port */
VOID Mvpp2DefaultsSet(PP2DXE_PORT *port)
{
  INT32 TxPortNum, val, queue, ptxq;

#ifdef MVPP2_V1
  /* Configure port to loopback if needed */
  if (port->flags & MVPP2_F_LOOPBACK)
    Mvpp2PortLoopbackSet(port);

  /* Update TX FIFO MIN Threshold */
  val = Mvpp2GmacRead(port, MVPP2_GMAC_PORT_FIFO_CFG_1_REG);
  val &= ~MVPP2_GMAC_TX_FIFO_MIN_TH_ALL_MASK;
  /* Min. TX threshold must be less than minimal packet length */
  val |= MVPP2_GMAC_TX_FIFO_MIN_TH_MASK(64 - 4 - 2);
  Mvpp2GmacWrite(port, MVPP2_GMAC_PORT_FIFO_CFG_1_REG, val);
#endif

  /* Disable Legacy WRR, Disable EJP, Release from reset */
  TxPortNum = Mvpp2EgressPort(port);
  Mvpp2Write(port->priv, MVPP2_TXP_SCHED_PORT_INDEX_REG,
        TxPortNum);
  Mvpp2Write(port->priv, MVPP2_TXP_SCHED_CMD_1_REG, 0);

  /* Close bandwidth for all queues */
  for (queue = 0; queue < MVPP2_MAX_TXQ; queue++) {
    ptxq = Mvpp2TxqPhys(port->id, queue);
    Mvpp2Write(port->priv,
          MVPP2_TXQ_SCHED_TOKEN_CNTR_REG(ptxq), 0);
  }

  /* Set refill period to 1 usec, refill tokens
   * and bucket size to maximum
   */
  Mvpp2Write(port->priv, MVPP2_TXP_SCHED_PERIOD_REG,
        port->priv->tclk / MVPP2_USEC_PER_SEC);
  val = Mvpp2Read(port->priv, MVPP2_TXP_SCHED_REFILL_REG);
  val &= ~MVPP2_TXP_REFILL_PERIOD_ALL_MASK;
  val |= MVPP2_TXP_REFILL_PERIOD_MASK(1);
  val |= MVPP2_TXP_REFILL_TOKENS_ALL_MASK;
  Mvpp2Write(port->priv, MVPP2_TXP_SCHED_REFILL_REG, val);
  val = MVPP2_TXP_TOKEN_SIZE_MAX;
  Mvpp2Write(port->priv, MVPP2_TXP_SCHED_TOKEN_SIZE_REG, val);

  /* Set MaximumLowLatencyPacketSize value to 256 */
  Mvpp2Write(port->priv, MVPP2_RX_CTRL_REG(port->id),
        MVPP2_RX_USE_PSEUDO_FOR_CSUM_MASK |
        MVPP2_RX_LOW_LATENCY_PKT_SIZE(256));

#ifdef MVPP2_V1
  /* Enable Rx cache snoop */
  INT32 lrxq;
  for (lrxq = 0; lrxq < RxqNumber; lrxq++) {
    queue = port->rxqs[lrxq].id;
    val = Mvpp2Read(port->priv, MVPP2_RXQ_CONFIG_REG(queue));
    val |= MVPP2_SNOOP_PKT_SIZE_MASK |
         MVPP2_SNOOP_BUF_HDR_MASK;
    Mvpp2Write(port->priv, MVPP2_RXQ_CONFIG_REG(queue), val);
  }
#else
  /* Mask all interrupts to all present cpus */
  Mvpp2InterruptsDisable(port, 0x1);
#endif

}

/* Enable/disable receiving packets */
VOID Mvpp2IngressEnable(PP2DXE_PORT *port)
{
  UINT32 val;
  INT32 lrxq, queue;

  for (lrxq = 0; lrxq < RxqNumber; lrxq++) {
    queue = port->rxqs[lrxq].id;
    val = Mvpp2Read(port->priv, MVPP2_RXQ_CONFIG_REG(queue));
    val &= ~MVPP2_RXQ_DISABLE_MASK;
    Mvpp2Write(port->priv, MVPP2_RXQ_CONFIG_REG(queue), val);
  }
}

VOID Mvpp2IngressDisable(PP2DXE_PORT *port)
{
  UINT32 val;
  INT32 lrxq, queue;

  for (lrxq = 0; lrxq < RxqNumber; lrxq++) {
    queue = port->rxqs[lrxq].id;
    val = Mvpp2Read(port->priv, MVPP2_RXQ_CONFIG_REG(queue));
    val |= MVPP2_RXQ_DISABLE_MASK;
    Mvpp2Write(port->priv, MVPP2_RXQ_CONFIG_REG(queue), val);
  }
}

/* Enable transmit via physical egress queue
 * - HW starts take descriptors from DRAM
 */
VOID Mvpp2EgressEnable(PP2DXE_PORT *port)
{
  UINT32 qmap;
  INT32 queue;
  INT32 TxPortNum = Mvpp2EgressPort(port);

  /* Enable all initialized TXs. */
  qmap = 0;
  for (queue = 0; queue < TxqNumber; queue++) {
    MVPP2_TX_QUEUE *txq = &port->txqs[queue];

    if (txq->descs != MVPP2_NULL)
      qmap |= (1 << queue);
  }

  Mvpp2Write(port->priv, MVPP2_TXP_SCHED_PORT_INDEX_REG, TxPortNum);
  Mvpp2Write(port->priv, MVPP2_TXP_SCHED_Q_CMD_REG, qmap);
}

/* Disable transmit via physical egress queue
 * - HW doesn't take descriptors from DRAM
 */
VOID Mvpp2EgressDisable(PP2DXE_PORT *port)
{
  UINT32 RegData;
  INT32 delay;
  INT32 TxPortNum = Mvpp2EgressPort(port);

  /* Issue stop command for active channels only */
  Mvpp2Write(port->priv, MVPP2_TXP_SCHED_PORT_INDEX_REG, TxPortNum);
  RegData = (Mvpp2Read(port->priv, MVPP2_TXP_SCHED_Q_CMD_REG)) &
        MVPP2_TXP_SCHED_ENQ_MASK;
  if (RegData != 0)
    Mvpp2Write(port->priv, MVPP2_TXP_SCHED_Q_CMD_REG,
          (RegData << MVPP2_TXP_SCHED_DISQ_OFFSET));

  /* Wait for all Tx activity to terminate. */
  delay = 0;
  do {
    if (delay >= MVPP2_TX_DISABLE_TIMEOUT_MSEC) {
      Mvpp2Printf("Tx stop timed out, status=0x%08x\n",
             RegData);
      break;
    }
    Mvpp2Mdelay(1);
    delay++;

    /* Check port TX Command register that all
     * Tx queues are stopped
     */
    RegData = Mvpp2Read(port->priv, MVPP2_TXP_SCHED_Q_CMD_REG);
  } while (RegData & MVPP2_TXP_SCHED_ENQ_MASK);
}

/* Rx descriptors helper methods */

/* Set rx queue offset */
STATIC VOID Mvpp2RxqOffsetSet(PP2DXE_PORT *port,
         INT32 prxq, INT32 offset)
{
  UINT32 val;

  /* Convert offset from bytes to units of 32 bytes */
  offset = offset >> 5;

  val = Mvpp2Read(port->priv, MVPP2_RXQ_CONFIG_REG(prxq));
  val &= ~MVPP2_RXQ_PACKET_OFFSET_MASK;

  /* Offset is in */
  val |= ((offset << MVPP2_RXQ_PACKET_OFFSET_OFFS) &
        MVPP2_RXQ_PACKET_OFFSET_MASK);

  Mvpp2Write(port->priv, MVPP2_RXQ_CONFIG_REG(prxq), val);
}

/* Obtain BM cookie information from descriptor */
UINT32 Mvpp2BmCookieBuild(MVPP2_RX_DESC *RxDesc, INT32 cpu)
{
  INT32 pool = (RxDesc->status & MVPP2_RXD_BM_POOL_ID_MASK) >>
       MVPP2_RXD_BM_POOL_ID_OFFS;

  return ((pool & 0xFF) << MVPP2_BM_COOKIE_POOL_OFFS) |
         ((cpu & 0xFF) << MVPP2_BM_COOKIE_CPU_OFFS);
}

/* Tx descriptors helper methods */

INT32 Mvpp2TxqDrainSet(PP2DXE_PORT *port, INT32 txq, BOOLEAN en)
{
  UINT32 RegVal;
  INT32 ptxq = Mvpp2TxqPhys(port->id, txq);

  Mvpp2Write(port->priv, MVPP2_TXQ_NUM_REG, ptxq);
  RegVal = Mvpp2Read(port->priv, MVPP2_TXQ_PREF_BUF_REG);

  if (en)
    RegVal |= MVPP2_TXQ_DRAIN_EN_MASK;
  else
    RegVal &= ~MVPP2_TXQ_DRAIN_EN_MASK;

  Mvpp2Write(port->priv, MVPP2_TXQ_PREF_BUF_REG, RegVal);

  return 0;
}

/* Get number of Tx descriptors waiting to be transmitted by HW */
INT32 Mvpp2TxqPendDescNumGet(PP2DXE_PORT *port,
        MVPP2_TX_QUEUE *txq)
{
  UINT32 val;

  Mvpp2Write(port->priv, MVPP2_TXQ_NUM_REG, txq->id);
  val = Mvpp2Read(port->priv, MVPP2_TXQ_PENDING_REG);

  return val & MVPP2_TXQ_PENDING_MASK;
}

/* Get number of occupied aggregated Tx descriptors */
UINT32 Mvpp2AggrTxqPendDescNumGet(MVPP2_SHARED *pp2, int cpu)
{
  UINT32 RegVal;

  RegVal = Mvpp2Read(pp2, MVPP2_AGGR_TXQ_STATUS_REG(cpu));

  return RegVal & MVPP2_AGGR_TXQ_PENDING_MASK;
}

/* Get pointer to next Tx descriptor to be processed (send) by HW */
MVPP2_TX_DESC *
Mvpp2TxqNextDescGet(MVPP2_TX_QUEUE *txq)
{
  INT32 TxDesc = txq->NextDescToProc;

  txq->NextDescToProc = MVPP2_QUEUE_NEXT_DESC(txq, TxDesc);
  return txq->descs + TxDesc;
}

/* Update HW with number of aggregated Tx descriptors to be sent */
VOID Mvpp2AggrTxqPendDescAdd(PP2DXE_PORT *port, INT32 pending)
{
  /* aggregated access - relevant TXQ number is written in TX desc */
  Mvpp2Write(port->priv, MVPP2_AGGR_TXQ_UPDATE_REG, pending);
}

/* Check if there are enough free descriptors in aggregated txq.
 * If not, update the number of occupied descriptors and repeat the check.
 */
INT32 Mvpp2AggrDescNumCheck(MVPP2_SHARED *priv,
              MVPP2_TX_QUEUE *AggrTxq, INT32 num,
              INT32 cpu)
{
  if ((AggrTxq->count + num) > AggrTxq->size) {
    /* Update number of occupied aggregated Tx descriptors */
    UINT32 val = Mvpp2Read(priv, MVPP2_AGGR_TXQ_STATUS_REG(cpu));

    AggrTxq->count = val & MVPP2_AGGR_TXQ_PENDING_MASK;
  }

  if ((AggrTxq->count + num) > AggrTxq->size)
    return MVPP2_ENOMEM;

  return 0;
}

/* Reserved Tx descriptors allocation request */
INT32 Mvpp2TxqAllocReservedDesc(MVPP2_SHARED *priv,
          MVPP2_TX_QUEUE *txq, INT32 num)
{
  UINT32 val;

  val = (txq->id << MVPP2_TXQ_RSVD_REQ_Q_OFFSET) | num;
  Mvpp2Write(priv, MVPP2_TXQ_RSVD_REQ_REG, val);

  val = Mvpp2Read(priv, MVPP2_TXQ_RSVD_RSLT_REG);

  return val & MVPP2_TXQ_RSVD_RSLT_MASK;
}

/* Release the last allocated Tx descriptor. Useful to handle DMA
 * mapping failures in the Tx path.
 */
VOID Mvpp2TxqDescPut(MVPP2_TX_QUEUE *txq)
{
  if (txq->NextDescToProc == 0)
    txq->NextDescToProc = txq->LastDesc - 1;
  else
    txq->NextDescToProc--;
}

/* Set Tx descriptors fields relevant for CSUM calculation */
UINT32 Mvpp2TxqDescCsum(INT32 L3Offs, INT32 L3Proto,
      INT32 IpHdrLen, INT32 L4Proto)
{
  UINT32 command;

  /* fields: L3_offset, IP_hdrlen, L3_type, G_IPV4Chk,
   * G_L4_chk, L4_type required only for checksum calculation
   */
  command = (L3Offs << MVPP2_TXD_L3_OFF_SHIFT);
  command |= (IpHdrLen << MVPP2_TXD_IP_HLEN_SHIFT);
  command |= MVPP2_TXD_IP_CSUM_DISABLE;

  if (L3Proto == Mvpp2Swab16(MV_ETH_P_IP)) {
    command &= ~MVPP2_TXD_IP_CSUM_DISABLE;  /* enable IPv4 csum */
    command &= ~MVPP2_TXD_L3_IP6;   /* enable IPv4 */
  } else {
    command |= MVPP2_TXD_L3_IP6;    /* enable IPv6 */
  }

  if (L4Proto == MV_IPPR_TCP) {
    command &= ~MVPP2_TXD_L4_UDP;   /* enable TCP */
    command &= ~MVPP2_TXD_L4_CSUM_FRAG; /* generate L4 csum */
  } else if (L4Proto == MV_IPPR_UDP) {
    command |= MVPP2_TXD_L4_UDP;    /* enable UDP */
    command &= ~MVPP2_TXD_L4_CSUM_FRAG; /* generate L4 csum */
  } else {
    command |= MVPP2_TXD_L4_CSUM_NOT;
  }

  return command;
}

VOID Mvpp2TxqSentCounterClear(VOID *arg)
{
  PP2DXE_PORT *port = arg;
  INT32 queue;

  for (queue = 0; queue < TxqNumber; queue++) {
    INT32 id = port->txqs[queue].id;

    Mvpp2Read(port->priv, MVPP2_TXQ_SENT_REG(id));
  }
}

/* Change maximum receive size of the port */
VOID Mvpp2GmacMaxRxSizeSet(PP2DXE_PORT *port)
{
  UINT32 val;

  val = Mvpp2GmacRead(port, MVPP2_GMAC_CTRL_0_REG);
  val &= ~MVPP2_GMAC_MAX_RX_SIZE_MASK;
  val |= (((port->PktSize - MVPP2_MH_SIZE) / 2) <<
                           MVPP2_GMAC_MAX_RX_SIZE_OFFS);
  Mvpp2GmacWrite(port, MVPP2_GMAC_CTRL_0_REG, val);
}

/* Set max sizes for Tx queues */
VOID Mvpp2TxpMaxTxSizeSet(PP2DXE_PORT *port)
{
  UINT32  val, size, mtu;
  INT32 txq, TxPortNum;

  mtu = port->PktSize * 8;
  if (mtu > MVPP2_TXP_MTU_MAX)
    mtu = MVPP2_TXP_MTU_MAX;

  /* WA for wrong Token bucket update: Set MTU value = 3*real MTU value */
  mtu = 3 * mtu;

  /* Indirect access to registers */
  TxPortNum = Mvpp2EgressPort(port);
  Mvpp2Write(port->priv, MVPP2_TXP_SCHED_PORT_INDEX_REG, TxPortNum);

  /* Set MTU */
  val = Mvpp2Read(port->priv, MVPP2_TXP_SCHED_MTU_REG);
  val &= ~MVPP2_TXP_MTU_MAX;
  val |= mtu;
  Mvpp2Write(port->priv, MVPP2_TXP_SCHED_MTU_REG, val);

  /* TXP token size and all TXQs token size must be larger that MTU */
  val = Mvpp2Read(port->priv, MVPP2_TXP_SCHED_TOKEN_SIZE_REG);
  size = val & MVPP2_TXP_TOKEN_SIZE_MAX;
  if (size < mtu) {
    size = mtu;
    val &= ~MVPP2_TXP_TOKEN_SIZE_MAX;
    val |= size;
    Mvpp2Write(port->priv, MVPP2_TXP_SCHED_TOKEN_SIZE_REG, val);
  }

  for (txq = 0; txq < TxqNumber; txq++) {
    val = Mvpp2Read(port->priv,
         MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq));
    size = val & MVPP2_TXQ_TOKEN_SIZE_MAX;

    if (size < mtu) {
      size = mtu;
      val &= ~MVPP2_TXQ_TOKEN_SIZE_MAX;
      val |= size;
      Mvpp2Write(port->priv,
            MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq),
            val);
    }
  }
}

/* Set the number of packets that will be received before Rx INT32errupt
 * will be generated by HW.
 */
VOID Mvpp2RxPktsCoalSet(PP2DXE_PORT *port,
          MVPP2_RX_QUEUE *rxq, UINT32 pkts)
{
  UINT32 val;

  val = (pkts & MVPP2_OCCUPIED_THRESH_MASK);
  Mvpp2Write(port->priv, MVPP2_RXQ_NUM_REG, rxq->id);
  Mvpp2Write(port->priv, MVPP2_RXQ_THRESH_REG, val);

  rxq->PktsCoal = pkts;
}

/* Set the time delay in usec before Rx INT32errupt */
VOID Mvpp2RxTimeCoalSet(PP2DXE_PORT *port,
          MVPP2_RX_QUEUE *rxq, UINT32 usec)
{
  UINT32 val;

  val = (port->priv->tclk / MVPP2_USEC_PER_SEC) * usec;
  Mvpp2Write(port->priv, MVPP2_ISR_RX_THRESHOLD_REG(rxq->id), val);

  rxq->TimeCoal = usec;
}

/* Rx/Tx queue initialization/cleanup methods */

VOID Mvpp2RxqHwInit(PP2DXE_PORT *port,
           MVPP2_RX_QUEUE *rxq)
{
  rxq->LastDesc = rxq->size - 1;

  /* Zero occupied and non-occupied counters - direct access */
  Mvpp2Write(port->priv, MVPP2_RXQ_STATUS_REG(rxq->id), 0);

  /* Set Rx descriptors queue starting address - indirect access */
  Mvpp2Write(port->priv, MVPP2_RXQ_NUM_REG, rxq->id);
#ifdef MVPP2_V1
  Mvpp2Write(port->priv, MVPP2_RXQ_DESC_ADDR_REG,
      rxq->DescsPhys >> MVPP21_DESC_ADDR_SHIFT);
#else
  Mvpp2Write(port->priv, MVPP2_RXQ_DESC_ADDR_REG,
      rxq->DescsPhys >> MVPP22_DESC_ADDR_SHIFT);
#endif
  Mvpp2Write(port->priv, MVPP2_RXQ_DESC_SIZE_REG, rxq->size);
  Mvpp2Write(port->priv, MVPP2_RXQ_INDEX_REG, 0);

  /* Set Offset */
  Mvpp2RxqOffsetSet(port, rxq->id, MVPP2_RXQ_OFFSET);

  /* Set coalescing pkts and time */
  Mvpp2RxPktsCoalSet(port, rxq, MVPP2_RX_COAL_PKTS);
  Mvpp2RxTimeCoalSet(port, rxq, rxq->TimeCoal);

  /* Add number of descriptors ready for receiving packets */
  Mvpp2RxqStatusUpdate(port, rxq->id, 0, rxq->size);
}

/* Push packets received by the RXQ to BM pool */
VOID Mvpp2RxqDropPkts(PP2DXE_PORT *port,
       MVPP2_RX_QUEUE *rxq,
       INT32 cpu)
{
  INT32 RxReceived;

  RxReceived = Mvpp2RxqReceived(port, rxq->id);
  if (!RxReceived)
    return;

#ifdef MVPP2_V1
  INT32 i;
  for (i = 0; i < RxReceived; i++) {
    MVPP2_RX_DESC *RxDesc = Mvpp2RxqNextDescGet(rxq);
    UINT32 bm = Mvpp2BmCookieBuild(RxDesc, cpu);

    Mvpp2PoolRefill(port, bm, RxDesc->BufPhysAddr,
          RxDesc->BufCookie);
  }
#endif
  Mvpp2RxqStatusUpdate(port, rxq->id, RxReceived, RxReceived);
}

VOID Mvpp2RxqHwDeinit(PP2DXE_PORT *port,
       MVPP2_RX_QUEUE *rxq)
{
  rxq->descs             = MVPP2_NULL;
  rxq->LastDesc         = 0;
  rxq->NextDescToProc = 0;
  rxq->DescsPhys        = 0;

  /* Clear Rx descriptors queue starting address and size;
   * free descriptor number
   */
  Mvpp2Write(port->priv, MVPP2_RXQ_STATUS_REG(rxq->id), 0);
  Mvpp2Write(port->priv, MVPP2_RXQ_NUM_REG, rxq->id);
  Mvpp2Write(port->priv, MVPP2_RXQ_DESC_ADDR_REG, 0);
  Mvpp2Write(port->priv, MVPP2_RXQ_DESC_SIZE_REG, 0);
}

VOID Mvpp2TxqHwInit(PP2DXE_PORT *port,
           MVPP2_TX_QUEUE *txq)
{
  INT32 desc, DescPerTxq, TxPortNum;
  UINT32 val;

  txq->LastDesc = txq->size - 1;

  /* Set Tx descriptors queue starting address - indirect access */
  Mvpp2Write(port->priv, MVPP2_TXQ_NUM_REG, txq->id);
  Mvpp2Write(port->priv, MVPP2_TXQ_DESC_ADDR_REG, txq->DescsPhys);
  Mvpp2Write(port->priv, MVPP2_TXQ_DESC_SIZE_REG, txq->size &
               MVPP2_TXQ_DESC_SIZE_MASK);
  Mvpp2Write(port->priv, MVPP2_TXQ_INDEX_REG, 0);
  Mvpp2Write(port->priv, MVPP2_TXQ_RSVD_CLR_REG,
        txq->id << MVPP2_TXQ_RSVD_CLR_OFFSET);
  val = Mvpp2Read(port->priv, MVPP2_TXQ_PENDING_REG);
  val &= ~MVPP2_TXQ_PENDING_MASK;
  Mvpp2Write(port->priv, MVPP2_TXQ_PENDING_REG, val);

  /* Calculate base address in prefetch buffer. We reserve 16 descriptors
   * for each existing TXQ.
   * TCONTS for PON port must be continuous from 0 to MVPP2_MAX_TCONT
   * GBE ports assumed to be continious from 0 to MVPP2_MAX_PORTS
   */
  DescPerTxq = 16;
  desc = (port->id * MVPP2_MAX_TXQ * DescPerTxq) +
         (txq->LogId * DescPerTxq);

  Mvpp2Write(port->priv, MVPP2_TXQ_PREF_BUF_REG,
        MVPP2_PREF_BUF_PTR(desc) | MVPP2_PREF_BUF_SIZE_16 |
        MVPP2_PREF_BUF_THRESH(DescPerTxq/2));

  /* WRR / EJP configuration - indirect access */
  TxPortNum = Mvpp2EgressPort(port);
  Mvpp2Write(port->priv, MVPP2_TXP_SCHED_PORT_INDEX_REG, TxPortNum);

  val = Mvpp2Read(port->priv, MVPP2_TXQ_SCHED_REFILL_REG(txq->LogId));
  val &= ~MVPP2_TXQ_REFILL_PERIOD_ALL_MASK;
  val |= MVPP2_TXQ_REFILL_PERIOD_MASK(1);
  val |= MVPP2_TXQ_REFILL_TOKENS_ALL_MASK;
  Mvpp2Write(port->priv, MVPP2_TXQ_SCHED_REFILL_REG(txq->LogId), val);

  val = MVPP2_TXQ_TOKEN_SIZE_MAX;
  Mvpp2Write(port->priv, MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq->LogId),
        val);
}

VOID Mvpp2TxqHwDeinit(PP2DXE_PORT *port,
       MVPP2_TX_QUEUE *txq)
{
  txq->descs             = MVPP2_NULL;
  txq->LastDesc         = 0;
  txq->NextDescToProc = 0;
  txq->DescsPhys        = 0;

  /* Set minimum bandwidth for disabled TXQs */
  Mvpp2Write(port->priv, MVPP2_TXQ_SCHED_TOKEN_CNTR_REG(txq->id), 0);

  /* Set Tx descriptors queue starting address and size */
  Mvpp2Write(port->priv, MVPP2_TXQ_NUM_REG, txq->id);
  Mvpp2Write(port->priv, MVPP2_TXQ_DESC_ADDR_REG, 0);
  Mvpp2Write(port->priv, MVPP2_TXQ_DESC_SIZE_REG, 0);
}

/* Allocate and initialize descriptors for aggr TXQ */
VOID Mvpp2AggrTxqHwInit(MVPP2_TX_QUEUE *AggrTxq,
          INT32 DescNum, INT32 cpu,
          MVPP2_SHARED *priv)
{
  AggrTxq->LastDesc = AggrTxq->size - 1;

  /* Aggr TXQ no reset WA */
  AggrTxq->NextDescToProc = Mvpp2Read(priv,
             MVPP2_AGGR_TXQ_INDEX_REG(cpu));

  /* Set Tx descriptors queue starting address */
  /* indirect access */
#ifndef MVPP2_V1
  Mvpp2Write(priv, MVPP2_AGGR_TXQ_DESC_ADDR_REG(cpu), AggrTxq->DescsPhys
      >> MVPP22_DESC_ADDR_SHIFT);
#else
  Mvpp2Write(priv, MVPP2_AGGR_TXQ_DESC_ADDR_REG(cpu), AggrTxq->DescsPhys
      >> MVPP21_DESC_ADDR_SHIFT);
#endif
  Mvpp2Write(priv, MVPP2_AGGR_TXQ_DESC_SIZE_REG(cpu), DescNum
      & MVPP2_AGGR_TXQ_DESC_SIZE_MASK);

}

/* Enable gmac */
VOID Mvpp2PortPowerUp(PP2DXE_PORT *port)
{
  Mvpp2PortMiiSet(port);
  Mvpp2PortPeriodicXonDisable(port);
  Mvpp2PortFcAdvEnable(port);
  Mvpp2PortReset(port);
}

/* Initialize Rx FIFO's */
VOID Mvpp2RxFifoInit(MVPP2_SHARED *priv)
{
  INT32 port;

  for (port = 0; port < MVPP2_MAX_PORTS; port++) {
    Mvpp2Write(priv, MVPP2_RX_DATA_FIFO_SIZE_REG(port),
          MVPP2_RX_FIFO_PORT_DATA_SIZE);
    Mvpp2Write(priv, MVPP2_RX_ATTR_FIFO_SIZE_REG(port),
          MVPP2_RX_FIFO_PORT_ATTR_SIZE);
  }

  Mvpp2Write(priv, MVPP2_RX_MIN_PKT_SIZE_REG,
        MVPP2_RX_FIFO_PORT_MIN_PKT);
  Mvpp2Write(priv, MVPP2_RX_FIFO_INIT_REG, 0x1);
}

VOID MvGop110NetcActivePort(PP2DXE_PORT *Pp2Port, UINT32 port, UINT32 val)
{
  UINT32 reg;

  reg = Mvpp2Rfu1Read(Pp2Port->priv, MV_NETCOMP_PORTS_CONTROL_1);
  reg &= ~(NETC_PORTS_ACTIVE_MASK(port));

  val <<= NETC_PORTS_ACTIVE_OFFSET(port);
  val &= NETC_PORTS_ACTIVE_MASK(port);

  reg |= val;

  Mvpp2Rfu1Write(Pp2Port->priv, MV_NETCOMP_PORTS_CONTROL_1, reg);
}

STATIC VOID MvGop110NetcXauiEnable(PP2DXE_PORT *Pp2Port, UINT32 port, UINT32 val)
{
  UINT32 reg;

  reg = Mvpp2Rfu1Read(Pp2Port->priv, SD1_CONTROL_1_REG);
  reg &= ~SD1_CONTROL_XAUI_EN_MASK;

  val <<= SD1_CONTROL_XAUI_EN_OFFSET;
  val &= SD1_CONTROL_XAUI_EN_MASK;

  reg |= val;

  Mvpp2Rfu1Write(Pp2Port->priv, SD1_CONTROL_1_REG, reg);
}

STATIC VOID MvGop110NetcRxaui0Enable(PP2DXE_PORT *Pp2Port, UINT32 port, UINT32 val)
{
  UINT32 reg;

  reg = Mvpp2Rfu1Read(Pp2Port->priv, SD1_CONTROL_1_REG);
  reg &= ~SD1_CONTROL_RXAUI0_L23_EN_MASK;

  val <<= SD1_CONTROL_RXAUI0_L23_EN_OFFSET;
  val &= SD1_CONTROL_RXAUI0_L23_EN_MASK;

  reg |= val;

  Mvpp2Rfu1Write(Pp2Port->priv, SD1_CONTROL_1_REG, reg);
}

STATIC VOID MvGop110NetcRxaui1Enable(PP2DXE_PORT *Pp2Port, UINT32 port, UINT32 val)
{
  UINT32 reg;

  reg = Mvpp2Rfu1Read(Pp2Port->priv, SD1_CONTROL_1_REG);
  reg &= ~SD1_CONTROL_RXAUI1_L45_EN_MASK;

  val <<= SD1_CONTROL_RXAUI1_L45_EN_OFFSET;
  val &= SD1_CONTROL_RXAUI1_L45_EN_MASK;

  reg |= val;

  Mvpp2Rfu1Write(Pp2Port->priv, SD1_CONTROL_1_REG, reg);
}

STATIC VOID MvGop110NetcMiiMode(PP2DXE_PORT *Pp2Port, UINT32 port, UINT32 val)
{
  UINT32 reg;

  reg = Mvpp2Rfu1Read(Pp2Port->priv, MV_NETCOMP_CONTROL_0);
  reg &= ~NETC_GBE_PORT1_MII_MODE_MASK;

  val <<= NETC_GBE_PORT1_MII_MODE_OFFSET;
  val &= NETC_GBE_PORT1_MII_MODE_MASK;

  reg |= val;

  Mvpp2Rfu1Write(Pp2Port->priv, MV_NETCOMP_CONTROL_0, reg);
}

STATIC VOID MvGop110NetcGopReset(PP2DXE_PORT *Pp2Port, UINT32 val)
{
  UINT32 reg;

  reg = Mvpp2Rfu1Read(Pp2Port->priv, MV_GOP_SOFT_RESET_1_REG);
  reg &= ~NETC_GOP_SOFT_RESET_MASK;

  val <<= NETC_GOP_SOFT_RESET_OFFSET;
  val &= NETC_GOP_SOFT_RESET_MASK;

  reg |= val;

  Mvpp2Rfu1Write(Pp2Port->priv, MV_GOP_SOFT_RESET_1_REG, reg);
}

STATIC VOID MvGop110NetcGopClockLogicSet(PP2DXE_PORT *Pp2Port, UINT32 val)
{
  UINT32 reg;

  reg = Mvpp2Rfu1Read(Pp2Port->priv, MV_NETCOMP_PORTS_CONTROL_0);
  reg &= ~NETC_CLK_DIV_PHASE_MASK;

  val <<= NETC_CLK_DIV_PHASE_OFFSET;
  val &= NETC_CLK_DIV_PHASE_MASK;

  reg |= val;

  Mvpp2Rfu1Write(Pp2Port->priv, MV_NETCOMP_PORTS_CONTROL_0, reg);
}

STATIC VOID MvGop110NetcPortRfReset(PP2DXE_PORT *Pp2Port, UINT32 port, UINT32 val)
{
  UINT32 reg;

  reg = Mvpp2Rfu1Read(Pp2Port->priv, MV_NETCOMP_PORTS_CONTROL_1);
  reg &= ~(NETC_PORT_GIG_RF_RESET_MASK(port));

  val <<= NETC_PORT_GIG_RF_RESET_OFFSET(port);
  val &= NETC_PORT_GIG_RF_RESET_MASK(port);

  reg |= val;

  Mvpp2Rfu1Write(Pp2Port->priv, MV_NETCOMP_PORTS_CONTROL_1, reg);
}

STATIC VOID MvGop110NetcGbeSgmiiModeSelect(PP2DXE_PORT *Pp2Port, UINT32 port,
            UINT32 val)
{
  UINT32 reg, mask, offset;

  if (port == 2) {
    mask = NETC_GBE_PORT0_SGMII_MODE_MASK;
    offset = NETC_GBE_PORT0_SGMII_MODE_OFFSET;
  } else {
    mask = NETC_GBE_PORT1_SGMII_MODE_MASK;
    offset = NETC_GBE_PORT1_SGMII_MODE_OFFSET;
  }
  reg = Mvpp2Rfu1Read(Pp2Port->priv, MV_NETCOMP_CONTROL_0);
  reg &= ~mask;

  val <<= offset;
  val &= mask;

  reg |= val;

  Mvpp2Rfu1Write(Pp2Port->priv, MV_NETCOMP_CONTROL_0, reg);
}

STATIC VOID MvGop110NetcBusWidthSelect(PP2DXE_PORT *Pp2Port, UINT32 val)
{
  UINT32 reg;

  reg = Mvpp2Rfu1Read(Pp2Port->priv, MV_NETCOMP_PORTS_CONTROL_0);
  reg &= ~NETC_BUS_WIDTH_SELECT_MASK;

  val <<= NETC_BUS_WIDTH_SELECT_OFFSET;
  val &= NETC_BUS_WIDTH_SELECT_MASK;

  reg |= val;

  Mvpp2Rfu1Write(Pp2Port->priv, MV_NETCOMP_PORTS_CONTROL_0, reg);
}

STATIC VOID MvGop110NetcSampleStagesTiming(PP2DXE_PORT *Pp2Port, UINT32 val)
{
  UINT32 reg;

  reg = Mvpp2Rfu1Read(Pp2Port->priv, MV_NETCOMP_PORTS_CONTROL_0);
  reg &= ~NETC_GIG_RX_DATA_SAMPLE_MASK;

  val <<= NETC_GIG_RX_DATA_SAMPLE_OFFSET;
  val &= NETC_GIG_RX_DATA_SAMPLE_MASK;

  reg |= val;

  Mvpp2Rfu1Write(Pp2Port->priv, MV_NETCOMP_PORTS_CONTROL_0, reg);
}

STATIC VOID MvGop110NetcMacToXgmii(PP2DXE_PORT *Pp2Port, UINT32 port,
          enum MvNetcPhase phase)
{
  switch (phase) {
  case MV_NETC_FIRST_PHASE:
    /* Set Bus Width to HB mode = 1 */
    MvGop110NetcBusWidthSelect(Pp2Port, 1);
    /* Select RGMII mode */
    MvGop110NetcGbeSgmiiModeSelect(Pp2Port, port,
              MV_NETC_GBE_XMII);
    break;
  case MV_NETC_SECOND_PHASE:
    /* De-assert the relevant port HB reset */
    MvGop110NetcPortRfReset(Pp2Port, port, 1);
    break;
  }
}

STATIC VOID MvGop110NetcMacToSgmii(PP2DXE_PORT *Pp2Port, UINT32 port,
          enum MvNetcPhase phase)
{
  switch (phase) {
  case MV_NETC_FIRST_PHASE:
    /* Set Bus Width to HB mode = 1 */
    MvGop110NetcBusWidthSelect(Pp2Port, 1);
    /* Select SGMII mode */
    if (port >= 1)
      MvGop110NetcGbeSgmiiModeSelect(Pp2Port, port,
      MV_NETC_GBE_SGMII);

    /* Configure the sample stages */
    MvGop110NetcSampleStagesTiming(Pp2Port, 0);
    /* Configure the ComPhy Selector */
    /* MvGop110NetcComPhySelectorConfig(netComplex); */
    break;
  case MV_NETC_SECOND_PHASE:
    /* De-assert the relevant port HB reset */
    MvGop110NetcPortRfReset(Pp2Port, port, 1);
    break;
  }
}

STATIC VOID MvGop110NetcMacToRxaui(PP2DXE_PORT *Pp2Port, UINT32 port,
          enum MvNetcPhase phase,
          enum MvNetcLanes lanes)
{
  /* Currently only RXAUI0 supported */
  if (port != 0)
    return;

  switch (phase) {
  case MV_NETC_FIRST_PHASE:
    /* RXAUI Serdes/s Clock alignment */
    if (lanes == MV_NETC_LANE_23)
      MvGop110NetcRxaui0Enable(Pp2Port, port, 1);
    else
      MvGop110NetcRxaui1Enable(Pp2Port, port, 1);
    break;
  case MV_NETC_SECOND_PHASE:
    /* De-assert the relevant port HB reset */
    MvGop110NetcPortRfReset(Pp2Port, port, 1);
    break;
  }
}

STATIC VOID MvGop110NetcMacToXaui(PP2DXE_PORT *Pp2Port, UINT32 port,
          enum MvNetcPhase phase)
{
  switch (phase) {
  case MV_NETC_FIRST_PHASE:
    /* RXAUI Serdes/s Clock alignment */
    MvGop110NetcXauiEnable(Pp2Port, port, 1);
    break;
  case MV_NETC_SECOND_PHASE:
    /* De-assert the relevant port HB reset */
    MvGop110NetcPortRfReset(Pp2Port, port, 1);
    break;
  }
}

INT32 MvGop110NetcInit(PP2DXE_PORT *Pp2Port,
      UINT32 NetCompConfig, enum MvNetcPhase phase)
{
  UINT32 c = NetCompConfig;

  if (c & MV_NETC_GE_MAC0_RXAUI_L23)
    MvGop110NetcMacToRxaui(Pp2Port, 0, phase, MV_NETC_LANE_23);

  if (c & MV_NETC_GE_MAC0_RXAUI_L45)
    MvGop110NetcMacToRxaui(Pp2Port, 0, phase, MV_NETC_LANE_45);

  if (c & MV_NETC_GE_MAC0_XAUI)
    MvGop110NetcMacToXaui(Pp2Port, 0, phase);

  if (c & MV_NETC_GE_MAC2_SGMII)
    MvGop110NetcMacToSgmii(Pp2Port, 2, phase);
  else
    MvGop110NetcMacToXgmii(Pp2Port, 2, phase);
  if (c & MV_NETC_GE_MAC3_SGMII)
    MvGop110NetcMacToSgmii(Pp2Port, 3, phase);
  else {
    MvGop110NetcMacToXgmii(Pp2Port, 3, phase);
    if (c & MV_NETC_GE_MAC3_RGMII)
      MvGop110NetcMiiMode(Pp2Port, 3, MV_NETC_GBE_RGMII);
    else
      MvGop110NetcMiiMode(Pp2Port, 3, MV_NETC_GBE_MII);
  }

  /* Activate gop ports 0, 2, 3 */
  MvGop110NetcActivePort(Pp2Port, 0, 1);
  MvGop110NetcActivePort(Pp2Port, 2, 1);
  MvGop110NetcActivePort(Pp2Port, 3, 1);

  if (phase == MV_NETC_SECOND_PHASE) {
    /* Enable the GOP internal clock logic */
    MvGop110NetcGopClockLogicSet(Pp2Port, 1);
    /* De-assert GOP unit reset */
    MvGop110NetcGopReset(Pp2Port, 1);
  }
  return 0;
}
UINT32 MvpPp2xGop110NetcCfgCreate(PP2DXE_PORT *Pp2Port)
{
  UINT32 val = 0;

    if (Pp2Port->GopIndex == 0) {
      if (Pp2Port->PhyInterface ==
        MV_MODE_XAUI)
        val |= MV_NETC_GE_MAC0_XAUI;
      else if (Pp2Port->PhyInterface ==
        MV_MODE_RXAUI)
        val |= MV_NETC_GE_MAC0_RXAUI_L23;
    }
    if (Pp2Port->GopIndex == 2) {
      if (Pp2Port->PhyInterface ==
        MV_MODE_SGMII)
        val |= MV_NETC_GE_MAC2_SGMII;
    }
    if (Pp2Port->GopIndex == 3) {
      if (Pp2Port->PhyInterface ==
        MV_MODE_SGMII)
        val |= MV_NETC_GE_MAC3_SGMII;
      else if (Pp2Port->PhyInterface ==
        MV_MODE_RGMII)
        val |= MV_NETC_GE_MAC3_RGMII;
    }

  return val;
}

/*
* MvPortInit
*       Init physical port. Configures the port mode and all it's elements
*       accordingly.
*       Does not verify that the selected mode/port number is valid at the
*       core level.
*/
INT32 MvGop110PortInit(PP2DXE_PORT *Pp2Port)
{

  switch (Pp2Port->PhyInterface) {
  case MV_MODE_RGMII:
    MvGop110GmacReset(Pp2Port, RESET);
    /* configure PCS */
    MvGop110GpcsModeCfg(Pp2Port, FALSE);
    MvGop110BypassClkCfg(Pp2Port, TRUE);

    /* configure MAC */
    MvGop110GmacModeCfg(Pp2Port);
    /* pcs unreset */
    MvGop110GpcsReset(Pp2Port, UNRESET);
    /* mac unreset */
    MvGop110GmacReset(Pp2Port, UNRESET);
  break;
  case MV_MODE_SGMII:
  case MV_MODE_QSGMII:
    /* configure PCS */
    MvGop110GpcsModeCfg(Pp2Port, TRUE);

    /* configure MAC */
    MvGop110GmacModeCfg(Pp2Port);
    /* select proper Mac mode */
    MvGop110Xlg2GigMacCfg(Pp2Port);

    /* pcs unreset */
    MvGop110GpcsReset(Pp2Port, UNRESET);
    /* mac unreset */
    MvGop110GmacReset(Pp2Port, UNRESET);
  break;
  default:
    return -1;
  }

  return 0;
}

/* Set the MAC to reset or exit from reset */
INT32 MvGop110GmacReset(PP2DXE_PORT *Pp2Port, enum MvReset reset)
{
  UINT32 RegAddr;
  UINT32 val;

  RegAddr = MVPP2_PORT_CTRL2_REG;

  /* read - modify - write */
  val = MvGop110GmacRead(Pp2Port, RegAddr);
  if (reset == RESET)
    val |= MVPP2_PORT_CTRL2_PORTMACRESET_MASK;
  else
    val &= ~MVPP2_PORT_CTRL2_PORTMACRESET_MASK;
  MvGop110GmacWrite(Pp2Port, RegAddr, val);

  return 0;
}
/*
* MvGop110GpcsModeCfg
*Configure port to working with Gig PCS or don't.
*/
INT32 MvGop110GpcsModeCfg(PP2DXE_PORT *Pp2Port, BOOLEAN en)
{
  UINT32 val;

  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_CTRL2_REG);

  if (en)
    val |= MVPP2_PORT_CTRL2_PCS_EN_MASK;
  else
    val &= ~MVPP2_PORT_CTRL2_PCS_EN_MASK;

  /* enable / disable PCS on this port */
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_CTRL2_REG, val);

  return 0;
}

INT32 MvGop110BypassClkCfg(PP2DXE_PORT *Pp2Port, BOOLEAN en)
{
  UINT32 val;

  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_CTRL2_REG);

  if (en)
    val |= MVPP2_PORT_CTRL2_CLK_125_BYPS_EN_MASK;
  else
    val &= ~MVPP2_PORT_CTRL2_CLK_125_BYPS_EN_MASK;

  /* enable / disable PCS on this port */
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_CTRL2_REG, val);

  return 0;
}

INT32 MvGop110GpcsReset(PP2DXE_PORT *Pp2Port, enum MvReset act)
{
  UINT32 RegData;

  RegData = MvGop110GmacRead(Pp2Port, MVPP2_PORT_CTRL2_REG);
  if (act == RESET)
    U32_SET_FIELD(RegData, MVPP2_PORT_CTRL2_SGMII_MODE_MASK, 0);
  else
    U32_SET_FIELD(RegData, MVPP2_PORT_CTRL2_SGMII_MODE_MASK,
      1 << MVPP2_PORT_CTRL2_SGMII_MODE_OFFS);

  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_CTRL2_REG, RegData);
  return 0;
}

VOID MvGop110Xlg2GigMacCfg(PP2DXE_PORT *Pp2Port)
{
  UINT32 RegVal;

  /* relevant only for MAC0 (XLG0 and GMAC0) */
  if (Pp2Port->GopIndex > 0)
    return;

  /* configure 1Gig MAC mode */
  RegVal = Mvpp2XlgRead(Pp2Port,
          MV_XLG_PORT_MAC_CTRL3_REG);
  U32_SET_FIELD(RegVal, MV_XLG_MAC_CTRL3_MACMODESELECT_MASK,
    (0 << MV_XLG_MAC_CTRL3_MACMODESELECT_OFFS));
  Mvpp2XlgWrite(Pp2Port, MV_XLG_PORT_MAC_CTRL3_REG,
        RegVal);
}
/* Set the internal mux's to the required MAC in the GOP */
INT32 MvGop110GmacModeCfg(PP2DXE_PORT *Pp2Port)
{
  UINT32 RegAddr;
  UINT32 val;

  /* Set TX FIFO thresholds */
  switch (Pp2Port->PhyInterface) {
  case MV_MODE_SGMII:
    if (Pp2Port->speed == MV_PORT_SPEED_2500)
      MvGop110GmacSgmii25Cfg(Pp2Port);
    else
      MvGop110GmacSgmiiCfg(Pp2Port);
  break;
  case MV_MODE_RGMII:
    MvGop110GmacRgmiiCfg(Pp2Port);
  break;
  case MV_MODE_QSGMII:
    MvGop110GmacQsgmiiCfg(Pp2Port);
  break;
  default:
    return -1;
  }

  /* Jumbo frame support - 0x1400*2= 0x2800 bytes */
  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_CTRL0_REG);
  U32_SET_FIELD(val, MVPP2_PORT_CTRL0_FRAMESIZELIMIT_MASK,
    (0x1400 << MVPP2_PORT_CTRL0_FRAMESIZELIMIT_OFFS));
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_CTRL0_REG, val);

  /* PeriodicXonEn disable */
  RegAddr = MVPP2_PORT_CTRL1_REG;
  val = MvGop110GmacRead(Pp2Port, RegAddr);
  val &= ~MVPP2_PORT_CTRL1_EN_PERIODIC_FC_XON_MASK;
  MvGop110GmacWrite(Pp2Port, RegAddr, val);

  /* mask all ports interrupts */
  MvGop110GmacPortLinkEventMask(Pp2Port);

#if MV_PP2x_INTERRUPT
  /* unmask link change interrupt */
  val = MvGop110GmacRead(Pp2Port, MVPP2_INTERRUPT_MASK_REG);
  val |= MVPP2_INTERRUPT_CAUSE_LINK_CHANGE_MASK;
  val |= 1; /* unmask summary bit */
  MvGop110GmacWrite(Pp2Port, MVPP2_INTERRUPT_MASK_REG, val);
#endif
  return 0;
}

VOID MvGop110GmacRgmiiCfg(PP2DXE_PORT *Pp2Port)
{
  UINT32 val, thresh, an;

  /*configure minimal level of the Tx FIFO before the lower part starts to read a packet*/
  thresh = MV_RGMII_TX_FIFO_MIN_TH;
  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_FIFO_CFG_1_REG);
  U32_SET_FIELD(val, MVPP2_PORT_FIFO_CFG_1_TX_FIFO_MIN_TH_MASK,
    (thresh << MVPP2_PORT_FIFO_CFG_1_TX_FIFO_MIN_TH_OFFS));
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_FIFO_CFG_1_REG, val);

  /* Disable bypass of sync module */
  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_CTRL4_REG);
  val |= MVPP2_PORT_CTRL4_SYNC_BYPASS_MASK;
  /* configure DP clock select according to mode */
  val &= ~MVPP2_PORT_CTRL4_DP_CLK_SEL_MASK;
  val |= MVPP2_PORT_CTRL4_QSGMII_BYPASS_ACTIVE_MASK;
  val |= MVPP2_PORT_CTRL4_EXT_PIN_GMII_SEL_MASK;
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_CTRL4_REG, val);

  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_CTRL2_REG);
  val &= ~MVPP2_PORT_CTRL2_DIS_PADING_OFFS;
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_CTRL2_REG, val);

  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_CTRL0_REG);
  /* configure GIG MAC to SGMII mode */
  val &= ~MVPP2_PORT_CTRL0_PORTTYPE_MASK;
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_CTRL0_REG, val);

  /* configure AN 0xb8e8 */
  an = MVPP2_PORT_AUTO_NEG_CFG_AN_BYPASS_EN_MASK |
    MVPP2_PORT_AUTO_NEG_CFG_EN_AN_SPEED_MASK   |
    MVPP2_PORT_AUTO_NEG_CFG_EN_FC_AN_MASK      |
    MVPP2_PORT_AUTO_NEG_CFG_EN_FDX_AN_MASK     |
    MVPP2_PORT_AUTO_NEG_CFG_CHOOSE_SAMPLE_TX_CONFIG_MASK;
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_AUTO_NEG_CFG_REG, an);
}
VOID MvGop110GmacSgmii25Cfg(PP2DXE_PORT *Pp2Port)
{
  UINT32 val, thresh, an;

  /*configure minimal level of the Tx FIFO before the lower part starts to read a packet*/
  thresh = MV_SGMII2_5_TX_FIFO_MIN_TH;
  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_FIFO_CFG_1_REG);
  U32_SET_FIELD(val, MVPP2_PORT_FIFO_CFG_1_TX_FIFO_MIN_TH_MASK,
    (thresh << MVPP2_PORT_FIFO_CFG_1_TX_FIFO_MIN_TH_OFFS));
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_FIFO_CFG_1_REG, val);

  /* Disable bypass of sync module */
  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_CTRL4_REG);
  val |= MVPP2_PORT_CTRL4_SYNC_BYPASS_MASK;
  /* configure DP clock select according to mode */
  val |= MVPP2_PORT_CTRL4_DP_CLK_SEL_MASK;
  /* configure QSGMII bypass according to mode */
  val |= MVPP2_PORT_CTRL4_QSGMII_BYPASS_ACTIVE_MASK;
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_CTRL4_REG, val);

  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_CTRL2_REG);
  val |= MVPP2_PORT_CTRL2_DIS_PADING_OFFS;
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_CTRL2_REG, val);

  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_CTRL0_REG);
  /* configure GIG MAC to 1000Base-X mode connected to a fiber transceiver */
  val |= MVPP2_PORT_CTRL0_PORTTYPE_MASK;
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_CTRL0_REG, val);

  /* configure AN 0x9268 */
  an = MVPP2_PORT_AUTO_NEG_CFG_EN_PCS_AN_MASK |
    MVPP2_PORT_AUTO_NEG_CFG_AN_BYPASS_EN_MASK |
    MVPP2_PORT_AUTO_NEG_CFG_SET_MII_SPEED_MASK  |
    MVPP2_PORT_AUTO_NEG_CFG_SET_GMII_SPEED_MASK     |
    MVPP2_PORT_AUTO_NEG_CFG_ADV_PAUSE_MASK    |
    MVPP2_PORT_AUTO_NEG_CFG_SET_FULL_DX_MASK  |
    MVPP2_PORT_AUTO_NEG_CFG_CHOOSE_SAMPLE_TX_CONFIG_MASK;
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_AUTO_NEG_CFG_REG, an);
}
VOID MvGop110GmacSgmiiCfg(PP2DXE_PORT *Pp2Port)
{
  UINT32 val, thresh, an;

  /*configure minimal level of the Tx FIFO before the lower part starts to read a packet*/
  thresh = MV_SGMII_TX_FIFO_MIN_TH;
  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_FIFO_CFG_1_REG);
  U32_SET_FIELD(val, MVPP2_PORT_FIFO_CFG_1_TX_FIFO_MIN_TH_MASK,
    (thresh << MVPP2_PORT_FIFO_CFG_1_TX_FIFO_MIN_TH_OFFS));
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_FIFO_CFG_1_REG, val);

  /* Disable bypass of sync module */
  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_CTRL4_REG);
  val |= MVPP2_PORT_CTRL4_SYNC_BYPASS_MASK;
  /* configure DP clock select according to mode */
  val &= ~MVPP2_PORT_CTRL4_DP_CLK_SEL_MASK;
  /* configure QSGMII bypass according to mode */
  val |= MVPP2_PORT_CTRL4_QSGMII_BYPASS_ACTIVE_MASK;
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_CTRL4_REG, val);

  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_CTRL2_REG);
  val |= MVPP2_PORT_CTRL2_DIS_PADING_OFFS;
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_CTRL2_REG, val);

  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_CTRL0_REG);
  /* configure GIG MAC to SGMII mode */
  val &= ~MVPP2_PORT_CTRL0_PORTTYPE_MASK;
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_CTRL0_REG, val);

  /* configure AN */
  an = MVPP2_PORT_AUTO_NEG_CFG_EN_PCS_AN_MASK |
    MVPP2_PORT_AUTO_NEG_CFG_AN_BYPASS_EN_MASK |
    MVPP2_PORT_AUTO_NEG_CFG_EN_AN_SPEED_MASK  |
    MVPP2_PORT_AUTO_NEG_CFG_EN_FC_AN_MASK     |
    MVPP2_PORT_AUTO_NEG_CFG_EN_FDX_AN_MASK    |
    MVPP2_PORT_AUTO_NEG_CFG_CHOOSE_SAMPLE_TX_CONFIG_MASK;
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_AUTO_NEG_CFG_REG, an);
}

VOID MvGop110GmacQsgmiiCfg(PP2DXE_PORT *Pp2Port)
{
  UINT32 val, thresh, an;

  /*configure minimal level of the Tx FIFO before the lower part starts to read a packet*/
  thresh = MV_SGMII_TX_FIFO_MIN_TH;
  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_FIFO_CFG_1_REG);
  U32_SET_FIELD(val, MVPP2_PORT_FIFO_CFG_1_TX_FIFO_MIN_TH_MASK,
    (thresh << MVPP2_PORT_FIFO_CFG_1_TX_FIFO_MIN_TH_OFFS));
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_FIFO_CFG_1_REG, val);

  /* Disable bypass of sync module */
  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_CTRL4_REG);
  val |= MVPP2_PORT_CTRL4_SYNC_BYPASS_MASK;
  /* configure DP clock select according to mode */
  val &= ~MVPP2_PORT_CTRL4_DP_CLK_SEL_MASK;
  val &= ~MVPP2_PORT_CTRL4_EXT_PIN_GMII_SEL_MASK;
  /* configure QSGMII bypass according to mode */
  val &= ~MVPP2_PORT_CTRL4_QSGMII_BYPASS_ACTIVE_MASK;
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_CTRL4_REG, val);

  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_CTRL2_REG);
  val &= ~MVPP2_PORT_CTRL2_DIS_PADING_OFFS;
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_CTRL2_REG, val);

  val = MvGop110GmacRead(Pp2Port, MVPP2_PORT_CTRL0_REG);
  /* configure GIG MAC to SGMII mode */
  val &= ~MVPP2_PORT_CTRL0_PORTTYPE_MASK;
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_CTRL0_REG, val);

  /* configure AN 0xB8EC */
  an = MVPP2_PORT_AUTO_NEG_CFG_EN_PCS_AN_MASK |
    MVPP2_PORT_AUTO_NEG_CFG_AN_BYPASS_EN_MASK |
    MVPP2_PORT_AUTO_NEG_CFG_EN_AN_SPEED_MASK  |
    MVPP2_PORT_AUTO_NEG_CFG_EN_FC_AN_MASK     |
    MVPP2_PORT_AUTO_NEG_CFG_EN_FDX_AN_MASK    |
    MVPP2_PORT_AUTO_NEG_CFG_CHOOSE_SAMPLE_TX_CONFIG_MASK;
  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_AUTO_NEG_CFG_REG, an);
}
/*
* MvGopPhyAddrCfg
*/
INT32 Mvpp2SmiPhyAddrCfg(PP2DXE_PORT *Pp2Port, INT32 port, INT32 addr)
{
  Mvpp2SmiWrite(Pp2Port->priv, MV_SMI_PHY_ADDRESS_REG(port), addr);

  return 0;
}
BOOLEAN MvGop110PortIsLinkUp(PP2DXE_PORT *Pp2Port)
{
  switch (Pp2Port->PhyInterface) {
  case MV_MODE_RGMII:
  case MV_MODE_SGMII:
  case MV_MODE_QSGMII:
    return MvGop110GmacLinkStatusGet(Pp2Port);
  break;
  case MV_MODE_XAUI:
  case MV_MODE_RXAUI:
    gBS->Stall(1000);
//    return MvGop110XlgMacLinkStatusGet(Pp2Port);
    return FALSE;
  break;
  default:
    return FALSE;
  }
}
/* Get MAC link status */
BOOLEAN MvGop110GmacLinkStatusGet(PP2DXE_PORT *Pp2Port)
{
  UINT32 RegAddr;
  UINT32 val;

  RegAddr = MVPP2_PORT_STATUS0_REG;

  val = MvGop110GmacRead(Pp2Port, RegAddr);
  return (val & 1) ? TRUE : FALSE;
}

/* BM */
INTN Mvpp2BmPoolCtrl(MVPP2_SHARED *pp2, INTN pool, enum Mvpp2Command cmd)
{
  UINT32 RegVal = 0;
  RegVal = Mvpp2Read(pp2, MVPP2_BM_POOL_CTRL_REG(pool));

  switch (cmd) {
  case MVPP2_START:
    RegVal |= MVPP2_BM_START_MASK;
    break;

  case MVPP2_STOP:
    RegVal |= MVPP2_BM_STOP_MASK;
    break;

  default:
    return -1;
  }
  Mvpp2Write(pp2, MVPP2_BM_POOL_CTRL_REG(pool), RegVal);

  return 0;
}

VOID MvGop110PortDisable(PP2DXE_PORT *Pp2Port)
{

  switch (Pp2Port->PhyInterface) {
  case MV_MODE_RGMII:
  case MV_MODE_SGMII:
  case MV_MODE_QSGMII:
    MvGop110GmacPortDisable(Pp2Port);
  break;
/*
  case MV_MODE_XAUI:
  case MV_MODE_RXAUI:
    MvGop110XlgMacPortDisable(gop, PortNum);
  break;
  */

  default:
    return;
  }
}

VOID MvGop110PortEnable(PP2DXE_PORT *Pp2Port)
{

  switch (Pp2Port->PhyInterface) {
  case MV_MODE_RGMII:
  case MV_MODE_SGMII:
  case MV_MODE_QSGMII:
    MvGop110GmacPortEnable(Pp2Port);
  break;
/*
  case MV_MODE_XAUI:
  case MV_MODE_RXAUI:
    MvGop110XlgMacPortDisable(gop, PortNum);
  break;
  */

  default:
    return;
  }
}

/* Enable port and MIB counters */
VOID MvGop110GmacPortEnable(PP2DXE_PORT *Pp2Port)
{
  UINT32 RegVal;

  RegVal = MvGop110GmacRead(Pp2Port, MVPP2_PORT_CTRL0_REG);
  RegVal |= MVPP2_PORT_CTRL0_PORTEN_MASK;
  RegVal |= MVPP2_PORT_CTRL0_COUNT_EN_MASK;

  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_CTRL0_REG, RegVal);
}

/* Disable port */
VOID MvGop110GmacPortDisable(PP2DXE_PORT *Pp2Port)
{
  UINT32 RegVal;

  /* mask all ports interrupts */
  MvGop110GmacPortLinkEventMask(Pp2Port);

  RegVal = MvGop110GmacRead(Pp2Port, MVPP2_PORT_CTRL0_REG);
  RegVal &= ~MVPP2_PORT_CTRL0_PORTEN_MASK;

  MvGop110GmacWrite(Pp2Port, MVPP2_PORT_CTRL0_REG, RegVal);
}

VOID MvGop110GmacPortLinkEventMask(PP2DXE_PORT *Pp2Port)
{
  UINT32 RegVal;

  RegVal = MvGop110GmacRead(Pp2Port,
        MV_GMAC_INTERRUPT_SUM_MASK_REG);
  RegVal &= ~MV_GMAC_INTERRUPT_SUM_CAUSE_LINK_CHANGE_MASK;
  MvGop110GmacWrite(Pp2Port, MV_GMAC_INTERRUPT_SUM_MASK_REG,
      RegVal);
}

INT32 MvGop110PortEventsMask(PP2DXE_PORT *Pp2Port)
{

  switch (Pp2Port->PhyInterface) {
  case MV_MODE_RGMII:
  case MV_MODE_SGMII:
  case MV_MODE_QSGMII:
    MvGop110GmacPortLinkEventMask(Pp2Port);
  break;
  default:
    return -1;
  }
  return 0;
}

INT32 MvGop110FlCfg(PP2DXE_PORT *Pp2Port)
{

  switch (Pp2Port->PhyInterface) {
  case MV_MODE_RGMII:
  case MV_MODE_SGMII:
  case MV_MODE_QSGMII:
    /* disable AN */
    MvGop110SpeedDuplexSet(Pp2Port, Pp2Port->speed,
              MV_PORT_DUPLEX_FULL);
  break;

  case MV_MODE_XAUI:
  case MV_MODE_RXAUI:
    return 0;

  default:
    return -1;
  }
  return 0;
}
/* set port speed and duplex */
INT32 MvGop110SpeedDuplexSet(PP2DXE_PORT *Pp2Port,
      INT32 speed, enum MvPortDuplex duplex)
{

  switch (Pp2Port->PhyInterface) {
  case MV_MODE_RGMII:
  case MV_MODE_SGMII:
  case MV_MODE_QSGMII:
    MvGop110GmacSpeedDuplexSet(Pp2Port, speed, duplex);
  break;

  case MV_MODE_XAUI:
  case MV_MODE_RXAUI:
  break;

  default:
    return -1;
  }
  return 0;
}

/* Sets port speed to Auto Negotiation / 1000 / 100 / 10 Mbps.
*  Sets port duplex to Auto Negotiation / Full / Half Duplex.
*/
INT32 MvGop110GmacSpeedDuplexSet(PP2DXE_PORT *Pp2Port,
  INT32 speed, enum MvPortDuplex duplex)
{
  UINT32 RegVal;

  RegVal = Mvpp2GmacRead(Pp2Port,
          MVPP2_PORT_AUTO_NEG_CFG_REG);

  switch (speed) {
  case MV_PORT_SPEED_2500:
  case MV_PORT_SPEED_1000:
    RegVal &= ~MVPP2_PORT_AUTO_NEG_CFG_EN_AN_SPEED_MASK;
    RegVal |= MVPP2_PORT_AUTO_NEG_CFG_SET_GMII_SPEED_MASK;
    /* the 100/10 bit doesn't matter in this case */
    break;
  case MV_PORT_SPEED_100:
    RegVal &= ~MVPP2_PORT_AUTO_NEG_CFG_EN_AN_SPEED_MASK;
    RegVal &= ~MVPP2_PORT_AUTO_NEG_CFG_SET_GMII_SPEED_MASK;
    RegVal |= MVPP2_PORT_AUTO_NEG_CFG_SET_MII_SPEED_MASK;
    break;
  case MV_PORT_SPEED_10:
    RegVal &= ~MVPP2_PORT_AUTO_NEG_CFG_EN_AN_SPEED_MASK;
    RegVal &= ~MVPP2_PORT_AUTO_NEG_CFG_SET_GMII_SPEED_MASK;
    RegVal &= ~MVPP2_PORT_AUTO_NEG_CFG_SET_MII_SPEED_MASK;
    break;
  default:
    return MVPP2_EINVAL;
  }

  switch (duplex) {
  case MV_PORT_DUPLEX_AN:
    RegVal  |= MVPP2_PORT_AUTO_NEG_CFG_EN_FDX_AN_MASK;
    /* the other bits don't matter in this case */
    break;
  case MV_PORT_DUPLEX_HALF:
    RegVal &= ~MVPP2_PORT_AUTO_NEG_CFG_EN_FDX_AN_MASK;
    RegVal &= ~MVPP2_PORT_AUTO_NEG_CFG_SET_FULL_DX_MASK;
    break;
  case MV_PORT_DUPLEX_FULL:
    RegVal &= ~MVPP2_PORT_AUTO_NEG_CFG_EN_FDX_AN_MASK;
    RegVal |= MVPP2_PORT_AUTO_NEG_CFG_SET_FULL_DX_MASK;
    break;
  default:
    return MVPP2_EINVAL;
  }

  Mvpp2GmacWrite(Pp2Port, MVPP2_PORT_AUTO_NEG_CFG_REG,
        RegVal);
  return 0;
}

VOID Mvpp2AxiConfig(MVPP2_SHARED *pp2)
{
  /* Config AXI Read&Write Normal and Soop mode  */
  Mvpp2Write(pp2, MVPP22_AXI_BM_WR_ATTR_REG,
    MVPP22_AXI_ATTR_SNOOP_CNTRL_BIT);
  Mvpp2Write(pp2, MVPP22_AXI_BM_RD_ATTR_REG,
    MVPP22_AXI_ATTR_SNOOP_CNTRL_BIT);
  Mvpp2Write(pp2, MVPP22_AXI_AGGRQ_DESCR_RD_ATTR_REG,
    MVPP22_AXI_ATTR_SNOOP_CNTRL_BIT);
  Mvpp2Write(pp2, MVPP22_AXI_TXQ_DESCR_WR_ATTR_REG,
    MVPP22_AXI_ATTR_SNOOP_CNTRL_BIT);
  Mvpp2Write(pp2, MVPP22_AXI_TXQ_DESCR_RD_ATTR_REG,
    MVPP22_AXI_ATTR_SNOOP_CNTRL_BIT);
  Mvpp2Write(pp2, MVPP22_AXI_RXQ_DESCR_WR_ATTR_REG,
    MVPP22_AXI_ATTR_SNOOP_CNTRL_BIT);
  Mvpp2Write(pp2, MVPP22_AXI_RX_DATA_WR_ATTR_REG,
    MVPP22_AXI_ATTR_SNOOP_CNTRL_BIT);
  Mvpp2Write(pp2, MVPP22_AXI_TX_DATA_RD_ATTR_REG,
    MVPP22_AXI_ATTR_SNOOP_CNTRL_BIT);
}

/* Cleanup Tx ports */
VOID Mvpp2TxpClean(PP2DXE_PORT *pp, INT32 txp,
          MVPP2_TX_QUEUE *txq)
{
  INT32 delay, pending;
  UINT32 RegVal;

  Mvpp2Write(pp->priv, MVPP2_TXQ_NUM_REG, txq->id);
  RegVal = Mvpp2Read(pp->priv, MVPP2_TXQ_PREF_BUF_REG);
  RegVal |= MVPP2_TXQ_DRAIN_EN_MASK;
  Mvpp2Write(pp->priv, MVPP2_TXQ_PREF_BUF_REG, RegVal);

  /* The napi queue has been stopped so wait for all packets
   * to be transmitted.
   */
  delay = 0;
  do {
    if (delay >= MVPP2_TX_PENDING_TIMEOUT_MSEC) {
      Mvpp2Printf("port %d: cleaning queue %d timed out\n",
            pp->id, txq->LogId);
      break;
    }
    Mvpp2Mdelay(1);
    delay++;

    pending = Mvpp2TxqPendDescNumGet(pp, txq);
  } while (pending);

  RegVal &= ~MVPP2_TXQ_DRAIN_EN_MASK;
  Mvpp2Write(pp->priv, MVPP2_TXQ_PREF_BUF_REG, RegVal);
}

/* Cleanup all Tx queues */
VOID Mvpp2CleanupTxqs(PP2DXE_PORT *pp)
{
  MVPP2_TX_QUEUE *txq;
  INT32 txp, queue;
  UINT32 RegVal;

  RegVal = Mvpp2Read(pp->priv, MVPP2_TX_PORT_FLUSH_REG);

  /* Reset Tx ports and delete Tx queues */
  for (txp = 0; txp < pp->TxpNum; txp++) {
    RegVal |= MVPP2_TX_PORT_FLUSH_MASK(pp->id);
    Mvpp2Write(pp->priv, MVPP2_TX_PORT_FLUSH_REG, RegVal);

    for (queue = 0; queue < TxqNumber; queue++) {
      txq = &pp->txqs[txp * TxqNumber + queue];
      Mvpp2TxpClean(pp, txp, txq);
      Mvpp2TxqHwDeinit(pp, txq);
    }

    RegVal &= ~MVPP2_TX_PORT_FLUSH_MASK(pp->id);
    Mvpp2Write(pp->priv, MVPP2_TX_PORT_FLUSH_REG, RegVal);
  }
}

/* Cleanup all Rx queues */
VOID Mvpp2CleanupRxqs(PP2DXE_PORT *pp)
{
  INT32 queue;

  for (queue = 0; queue < RxqNumber; queue++)
    Mvpp2RxqHwDeinit(pp, &pp->rxqs[queue]);
}
