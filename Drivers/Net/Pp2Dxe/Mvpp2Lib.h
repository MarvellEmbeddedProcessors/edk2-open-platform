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

#ifndef __MVPP2_LIB_H__
#define __MVPP2_LIB_H__

#include "Mvpp2LibHw.h"
#include "Pp2Dxe.h"

/* Number of RXQs used by single port */
STATIC INT32 RxqNumber = 1;
/* Number of TXQs used by single port */
STATIC INT32 TxqNumber = 1;

VOID Mvpp2PrsMacPromiscSet(MVPP2_SHARED *priv, INT32 port, BOOLEAN add);
VOID Mvpp2PrsMacMultiSet(MVPP2_SHARED *priv, INT32 port, INT32 index,
        BOOLEAN add);
INT32 Mvpp2PrsDefaultInit(MVPP2_SHARED *priv);
INT32 Mvpp2PrsMacDaAccept(MVPP2_SHARED *priv, INT32 port,
            const UINT8 *da, BOOLEAN add);
VOID Mvpp2PrsMcastDelAll(MVPP2_SHARED *priv, INT32 port);
INT32 Mvpp2PrsTagModeSet(MVPP2_SHARED *priv, INT32 port, INT32 type);
INT32 Mvpp2PrsDefFlow(PP2DXE_PORT *port);
VOID Mvpp2ClsInit(MVPP2_SHARED *priv);
VOID Mvpp2ClsPortConfig(PP2DXE_PORT *port);
VOID Mvpp2ClsOversizeRxqSet(PP2DXE_PORT *port);
VOID Mvpp2BmPoolHwCreate(MVPP2_SHARED *priv,
        MVPP2_BMS_POOL *BmPool, INT32 size);
VOID Mvpp2BmPoolBufsizeSet(MVPP2_SHARED *priv,
             MVPP2_BMS_POOL *BmPool,
             INT32 BufSize);
VOID Mvpp2BmStop(MVPP2_SHARED *priv, INT32 pool);
VOID Mvpp2BmIrqClear(MVPP2_SHARED *priv, INT32 pool);
VOID Mvpp2RxqLongPoolSet(PP2DXE_PORT *port,
        INT32 lrxq, INT32 LongPool);
VOID Mvpp2RxqShortPoolSet(PP2DXE_PORT *port,
         INT32 lrxq, INT32 ShortPool);
VOID Mvpp2BmPoolMcPut(PP2DXE_PORT *port, INT32 pool,
           UINT32 BufPhysAddr, UINT32 BufVirtAddr,
           INT32 McId);
VOID Mvpp2PoolRefill(PP2DXE_PORT *port, UINT32 bm,
        UINT32 PhysAddr, UINT32 cookie);
VOID Mvpp2InterruptsMask(VOID *arg);
VOID Mvpp2InterruptsUnmask(VOID *arg);
VOID Mvpp2PortEnable(PP2DXE_PORT *port);
VOID Mvpp2PortDisable(PP2DXE_PORT *port);
VOID Mvpp2DefaultsSet(PP2DXE_PORT *port);
VOID Mvpp2IngressEnable(PP2DXE_PORT *port);
VOID Mvpp2IngressDisable(PP2DXE_PORT *port);
VOID Mvpp2EgressEnable(PP2DXE_PORT *port);
VOID Mvpp2EgressDisable(PP2DXE_PORT *port);
UINT32 Mvpp2BmCookieBuild(MVPP2_RX_DESC *RxDesc, INT32 cpu);
INT32 Mvpp2TxqDrainSet(PP2DXE_PORT *port, INT32 txq, BOOLEAN en);
INT32 Mvpp2TxqPendDescNumGet(PP2DXE_PORT *port,
          MVPP2_TX_QUEUE *txq);
UINT32 Mvpp2AggrTxqPendDescNumGet(MVPP2_SHARED *pp2, int cpu);
MVPP2_TX_DESC *
Mvpp2TxqNextDescGet(MVPP2_TX_QUEUE *txq);
VOID Mvpp2AggrTxqPendDescAdd(PP2DXE_PORT *port, INT32 pending);
INT32 Mvpp2AggrDescNumCheck(MVPP2_SHARED *priv,
        MVPP2_TX_QUEUE *AggrTxq,
        INT32 num, INT32 cpu);
INT32 Mvpp2TxqAllocReservedDesc(MVPP2_SHARED *priv,
            MVPP2_TX_QUEUE *txq, INT32 num);
VOID Mvpp2TxqDescPut(MVPP2_TX_QUEUE *txq);
UINT32 Mvpp2TxqDescCsum(INT32 L3Offs, INT32 L3Proto,
         INT32 IpHdrLen, INT32 L4Proto);
VOID Mvpp2TxqSentCounterClear(VOID *arg);
VOID Mvpp2GmacMaxRxSizeSet(PP2DXE_PORT *port);
VOID Mvpp2TxpMaxTxSizeSet(PP2DXE_PORT *port);
VOID Mvpp2RxPktsCoalSet(PP2DXE_PORT *port,
             MVPP2_RX_QUEUE *rxq, UINT32 pkts);
VOID Mvpp2RxTimeCoalSet(PP2DXE_PORT *port,
             MVPP2_RX_QUEUE *rxq, UINT32 usec);
VOID Mvpp2AggrTxqHwInit(MVPP2_TX_QUEUE *AggrTxq,
             INT32 DescNum, INT32 cpu,
             MVPP2_SHARED *priv);
VOID Mvpp2RxqHwInit(PP2DXE_PORT *port,
        MVPP2_RX_QUEUE *rxq);
VOID Mvpp2RxqDropPkts(PP2DXE_PORT *port,
          MVPP2_RX_QUEUE *rxq,
          INT32 cpu);
VOID Mvpp2TxqHwInit(PP2DXE_PORT *port,
        MVPP2_TX_QUEUE *txq);
VOID Mvpp2TxqHwDeinit(PP2DXE_PORT *port,
          MVPP2_TX_QUEUE *txq);
VOID Mvpp2PortPowerUp(PP2DXE_PORT *port);
VOID Mvpp2RxFifoInit(MVPP2_SHARED *priv);
VOID Mvpp2RxqHwDeinit(PP2DXE_PORT *port,
          MVPP2_RX_QUEUE *rxq);
UINT32 MvpPp2xGop110NetcCfgCreate(PP2DXE_PORT *Pp2Port);
INT32 MvGop110NetcInit(PP2DXE_PORT *mvport,
      UINT32 NetCompConfig, enum MvNetcPhase phase);

UINT32 MvpPp2xGop110NetcCfgCreate(PP2DXE_PORT *Pp2Port);
INT32 MvGop110PortInit(PP2DXE_PORT *Pp2Port);
INT32 MvGop110GmacReset(PP2DXE_PORT *Pp2Port, enum MvReset reset);
INT32 MvGop110GpcsModeCfg(PP2DXE_PORT *Pp2Port, BOOLEAN en);
INT32 MvGop110BypassClkCfg(PP2DXE_PORT *Pp2Port, BOOLEAN en);
INT32 MvGop110GpcsReset(PP2DXE_PORT *Pp2Port, enum MvReset act);
VOID MvGop110Xlg2GigMacCfg(PP2DXE_PORT *Pp2Port);
INT32 MvGop110GmacModeCfg(PP2DXE_PORT *Pp2Port);
VOID MvGop110GmacRgmiiCfg(PP2DXE_PORT *Pp2Port);
VOID MvGop110GmacSgmii25Cfg(PP2DXE_PORT *Pp2Port);
VOID MvGop110GmacSgmiiCfg(PP2DXE_PORT *Pp2Port);
VOID MvGop110GmacQsgmiiCfg(PP2DXE_PORT *Pp2Port);
INT32 Mvpp2SmiPhyAddrCfg(PP2DXE_PORT *Pp2Port, INT32 port, INT32 addr);
BOOLEAN MvGop110PortIsLinkUp(PP2DXE_PORT *Pp2Port);
BOOLEAN MvGop110GmacLinkStatusGet(PP2DXE_PORT *Pp2Port);
INTN Mvpp2BmPoolCtrl(MVPP2_SHARED *pp2, INTN pool,
    enum Mvpp2Command cmd);
VOID MvGop110PortDisable(PP2DXE_PORT *Pp2Port);
VOID MvGop110PortEnable(PP2DXE_PORT *Pp2Port);
VOID MvGop110GmacPortEnable(PP2DXE_PORT *Pp2Port);
VOID MvGop110GmacPortDisable(PP2DXE_PORT *Pp2Port);
VOID MvGop110GmacPortLinkEventMask(PP2DXE_PORT *Pp2Port);
INT32 MvGop110PortEventsMask(PP2DXE_PORT *Pp2Port);
INT32 MvGop110FlCfg(PP2DXE_PORT *Pp2Port);
INT32 MvGop110SpeedDuplexSet(PP2DXE_PORT *Pp2Port,
      INT32 speed, enum MvPortDuplex duplex);
INT32 MvGop110GmacSpeedDuplexSet(PP2DXE_PORT *Pp2Port,
  INT32 speed, enum MvPortDuplex duplex);
VOID Mvpp2AxiConfig(MVPP2_SHARED *pp2);
VOID Mvpp2TxpClean(PP2DXE_PORT *pp, INT32 txp,
          MVPP2_TX_QUEUE *txq);
VOID Mvpp2CleanupTxqs(PP2DXE_PORT *pp);
VOID Mvpp2CleanupRxqs(PP2DXE_PORT *pp);

/* Get number of physical egress port */
STATIC inline INT32 Mvpp2EgressPort(PP2DXE_PORT *port)
{
  return MVPP2_MAX_TCONT + port->id;
}

/* Get number of physical TXQ */
STATIC inline INT32 Mvpp2TxqPhys(INT32 port, INT32 txq)
{
  return (MVPP2_MAX_TCONT + port) * MVPP2_MAX_TXQ + txq;
}

/* Set pool number in a BM cookie */
STATIC inline UINT32 Mvpp2BmCookiePoolSet(UINT32 cookie, INT32 pool)
{
  UINT32 bm;

  bm = cookie & ~(0xFF << MVPP2_BM_COOKIE_POOL_OFFS);
  bm |= ((pool & 0xFF) << MVPP2_BM_COOKIE_POOL_OFFS);

  return bm;
}

/* Get pool number from a BM cookie */
STATIC inline INT32 Mvpp2BmCookiePoolGet(UINT32 cookie)
{
  return (cookie >> MVPP2_BM_COOKIE_POOL_OFFS) & 0xFF;
}

#ifdef MVPP2_V1
/* Release buffer to BM */
STATIC inline VOID Mvpp2BmPoolPut(MVPP2_SHARED *pp2, INT32 pool,
            UINT32 BufPhysAddr, UINT32 BufVirtAddr)
{
  Mvpp2Write(port->priv, MVPP2_BM_VIRT_RLS_REG, BufVirtAddr);
  Mvpp2Write(port->priv, MVPP2_BM_PHY_RLS_REG(pool), BufPhysAddr);
}
#else
STATIC inline VOID Mvpp2BmPoolPut(MVPP2_SHARED *pp2, INT32 pool,
            UINT64 BufPhysAddr, UINT64 BufVirtAddr)
{
  UINT32 val = 0;

  val = (Upper32Bits(BufVirtAddr) & MVPP22_ADDR_HIGH_MASK)
    << MVPP22_BM_VIRT_HIGH_RLS_OFFST;
  val |= (Upper32Bits(BufPhysAddr) & MVPP22_ADDR_HIGH_MASK)
    << MVPP22_BM_PHY_HIGH_RLS_OFFSET;
  Mvpp2Write(pp2, MVPP22_BM_PHY_VIRT_HIGH_RLS_REG, val);
  Mvpp2Write(pp2, MVPP2_BM_VIRT_RLS_REG, (UINT32)BufVirtAddr);
  Mvpp2Write(pp2, MVPP2_BM_PHY_RLS_REG(pool), (UINT32)BufPhysAddr);
}
#endif

STATIC inline VOID Mvpp2InterruptsEnable(PP2DXE_PORT *port,
             INT32 CpuMask)
{
  Mvpp2Write(port->priv, MVPP2_ISR_ENABLE_REG(port->id),
        MVPP2_ISR_ENABLE_INTERRUPT(CpuMask));
}

STATIC inline VOID Mvpp2InterruptsDisable(PP2DXE_PORT *port,
              INT32 CpuMask)
{
  Mvpp2Write(port->priv, MVPP2_ISR_ENABLE_REG(port->id),
        MVPP2_ISR_DISABLE_INTERRUPT(CpuMask));
}

/* Get number of Rx descriptors occupied by received packets */
STATIC inline INT32
Mvpp2RxqReceived(PP2DXE_PORT *port, INT32 RxqId)
{
  UINT32 val = Mvpp2Read(port->priv, MVPP2_RXQ_STATUS_REG(RxqId));

  return val & MVPP2_RXQ_OCCUPIED_MASK;
}

/* Update Rx queue status with the number of occupied and available
 * Rx descriptor slots.
 */
STATIC inline VOID
Mvpp2RxqStatusUpdate(PP2DXE_PORT *port, INT32 RxqId,
      INT32 UsedCount, INT32 FreeCount)
{
  /* Decrement the number of used descriptors and increment count
   * increment the number of free descriptors.
   */
  UINT32 val = UsedCount | (FreeCount << MVPP2_RXQ_NUM_NEW_OFFSET);

  Mvpp2Write(port->priv, MVPP2_RXQ_STATUS_UPDATE_REG(RxqId), val);
}

/* Get pointer to next RX descriptor to be processed by SW */
STATIC inline MVPP2_RX_DESC *
Mvpp2RxqNextDescGet(MVPP2_RX_QUEUE *rxq)
{
  INT32 RxDesc = rxq->NextDescToProc;

  rxq->NextDescToProc = MVPP2_QUEUE_NEXT_DESC(rxq, RxDesc);
  Mvpp2Prefetch(rxq->descs + rxq->NextDescToProc);
  return rxq->descs + RxDesc;
}

/* Get number of sent descriptors and decrement counter.
 * The number of sent descriptors is returned.
 * Per-CPU access
 */
STATIC inline INT32 Mvpp2TxqSentDescProc(PP2DXE_PORT *port,
            MVPP2_TX_QUEUE *txq)
{
  UINT32 val;

  /* Reading status reg resets transmitted descriptor counter */
#ifdef MVPP2V1
  val = Mvpp2Read(port->priv, MVPP2_TXQ_SENT_REG(txq->id));
#else
  val = Mvpp2Read(port->priv, MVPP22_TXQ_SENT_REG(txq->id));
#endif

  return (val & MVPP2_TRANSMITTED_COUNT_MASK) >>
    MVPP2_TRANSMITTED_COUNT_OFFSET;
}

STATIC inline MVPP2_RX_QUEUE *Mvpp2GetRxQueue(PP2DXE_PORT *port,
             UINT32 cause)
{
  INT32 queue = Mvpp2Fls(cause) - 1;

  return &port->rxqs[queue];
}

STATIC inline MVPP2_TX_QUEUE *Mvpp2GetTxQueue(PP2DXE_PORT *port,
             UINT32 cause)
{
  INT32 queue = Mvpp2Fls(cause) - 1;

  return &port->txqs[queue];
}

STATIC inline void Mvpp2x2TxdescPhysAddrSet(DmaAddrT PhysAddr,
  MVPP2_TX_DESC *TxDesc) {
  UINT64 *BufPhysAddrP = &TxDesc->BufPhysAddrHwCmd2;

  *BufPhysAddrP &= ~(MVPP22_ADDR_MASK);
  *BufPhysAddrP |= PhysAddr & MVPP22_ADDR_MASK;
}
#endif /* __MVPP2_LIB_H__ */
