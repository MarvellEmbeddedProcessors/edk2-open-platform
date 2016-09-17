/*******************************************************************************
Copyright (C) 2016 Marvell International Ltd.

This software file (the "File") is owned and distributed by Marvell
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the three
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.

********************************************************************************
Marvell Commercial License Option

If you received this File from Marvell and you have entered into a commercial
license agreement (a "Commercial License") with Marvell, the File is licensed
to you under the terms of the applicable Commercial License.

********************************************************************************
Marvell GPL License Option

This program is free software: you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the Free
Software Foundation, either version 2 of the License, or any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

********************************************************************************
Marvell GNU General Public License FreeRTOS Exception

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File in accordance with the terms and conditions of the Lesser
General Public License Version 2.1 plus the following FreeRTOS exception.
An independent module is a module which is not derived from or based on
FreeRTOS.
Clause 1:
Linking FreeRTOS STATICally or dynamically with other modules is making a
combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
General Public License cover the whole combination.
As a special exception, the copyright holder of FreeRTOS gives you permission
to link FreeRTOS with independent modules that communicate with FreeRTOS solely
through the FreeRTOS API interface, regardless of the license terms of these
independent modules, and to copy and distribute the resulting combined work
under terms of your choice, provided that:
1. Every copy of the combined work is accompanied by a written statement that
details to the recipient the version of FreeRTOS used and an offer by yourself
to provide the FreeRTOS source code (including any modifications you may have
made) should the recipient request it.
2. The combined work is not itself an RTOS, scheduler, kernel or related
product.
3. The independent modules add significant and primary functionality to
FreeRTOS and do not merely extend the existing functionality already present in
FreeRTOS.
Clause 2:
FreeRTOS may not be used for any competitive or comparative purpose, including
the publication of any form of run time or compile time metric, without the
express permission of Real Time Engineers Ltd. (this is the norm within the
industry and is intended to ensure information accuracy).

********************************************************************************
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

#include "mvpp2_lib_hw.h"
#include "Pp2Dxe.h"

/* Number of RXQs used by single port */
STATIC INT32 rxq_number = 1;
/* Number of TXQs used by single port */
STATIC INT32 txq_number = 1;

VOID mvpp2_prs_mac_promisc_set(MVPP2_SHARED *priv, INT32 port, BOOLEAN add);
VOID mvpp2_prs_mac_multi_set(MVPP2_SHARED *priv, INT32 port, INT32 index,
        BOOLEAN add);
INT32 mvpp2_prs_default_init(MVPP2_SHARED *priv);
INT32 mvpp2_prs_mac_da_accept(MVPP2_SHARED *priv, INT32 port,
            const UINT8 *da, BOOLEAN add);
VOID mvpp2_prs_mcast_del_all(MVPP2_SHARED *priv, INT32 port);
INT32 mvpp2_prs_tag_mode_set(MVPP2_SHARED *priv, INT32 port, INT32 type);
INT32 mvpp2_prs_def_flow(PP2DXE_PORT *port);
VOID mvpp2_cls_init(MVPP2_SHARED *priv);
VOID mvpp2_cls_port_config(PP2DXE_PORT *port);
VOID mvpp2_cls_oversize_rxq_set(PP2DXE_PORT *port);
VOID mvpp2_bm_pool_hw_create(MVPP2_SHARED *priv,
        MVPP2_BMS_POOL *bm_pool, INT32 size);
VOID mvpp2_bm_pool_bufsize_set(MVPP2_SHARED *priv,
             MVPP2_BMS_POOL *bm_pool,
             INT32 buf_size);
VOID mvpp2_bm_stop(MVPP2_SHARED *priv, INT32 pool);
VOID mvpp2_bm_irq_clear(MVPP2_SHARED *priv, INT32 pool);
VOID mvpp2_rxq_long_pool_set(PP2DXE_PORT *port,
        INT32 lrxq, INT32 long_pool);
VOID mvpp2_rxq_short_pool_set(PP2DXE_PORT *port,
         INT32 lrxq, INT32 short_pool);
VOID mvpp2_bm_pool_mc_put(PP2DXE_PORT *port, INT32 pool,
           UINT32 buf_phys_addr, UINT32 buf_virt_addr,
           INT32 mc_id);
VOID mvpp2_pool_refill(PP2DXE_PORT *port, UINT32 bm,
        UINT32 phys_addr, UINT32 cookie);
VOID mvpp2_interrupts_mask(VOID *arg);
VOID mvpp2_interrupts_unmask(VOID *arg);
VOID mvpp2_port_enable(PP2DXE_PORT *port);
VOID mvpp2_port_disable(PP2DXE_PORT *port);
VOID mvpp2_defaults_set(PP2DXE_PORT *port);
VOID mvpp2_ingress_enable(PP2DXE_PORT *port);
VOID mvpp2_ingress_disable(PP2DXE_PORT *port);
VOID mvpp2_egress_enable(PP2DXE_PORT *port);
VOID mvpp2_egress_disable(PP2DXE_PORT *port);
UINT32 mvpp2_bm_cookie_build(MVPP2_RX_DESC *rx_desc, INT32 cpu);
INT32 mvpp2_txq_drain_set(PP2DXE_PORT *port, INT32 txq, BOOLEAN en);
INT32 mvpp2_txq_pend_desc_num_get(PP2DXE_PORT *port,
          MVPP2_TX_QUEUE *txq);
UINT32 mvpp2_aggr_txq_pend_desc_num_get(MVPP2_SHARED *pp2, int cpu);
MVPP2_TX_DESC *
mvpp2_txq_next_desc_get(MVPP2_TX_QUEUE *txq);
VOID mvpp2_aggr_txq_pend_desc_add(PP2DXE_PORT *port, INT32 pending);
INT32 mvpp2_aggr_desc_num_check(MVPP2_SHARED *priv,
        MVPP2_TX_QUEUE *aggr_txq,
        INT32 num, INT32 cpu);
INT32 mvpp2_txq_alloc_reserved_desc(MVPP2_SHARED *priv,
            MVPP2_TX_QUEUE *txq, INT32 num);
VOID mvpp2_txq_desc_put(MVPP2_TX_QUEUE *txq);
UINT32 mvpp2_txq_desc_csum(INT32 l3_offs, INT32 l3_proto,
         INT32 ip_hdr_len, INT32 l4_proto);
VOID mvpp2_txq_sent_counter_clear(VOID *arg);
VOID mvpp2_gmac_max_rx_size_set(PP2DXE_PORT *port);
VOID mvpp2_txp_max_tx_size_set(PP2DXE_PORT *port);
VOID mvpp2_rx_pkts_coal_set(PP2DXE_PORT *port,
             MVPP2_RX_QUEUE *rxq, UINT32 pkts);
VOID mvpp2_rx_time_coal_set(PP2DXE_PORT *port,
             MVPP2_RX_QUEUE *rxq, UINT32 usec);
VOID mvpp2_aggr_txq_hw_init(MVPP2_TX_QUEUE *aggr_txq,
             INT32 desc_num, INT32 cpu,
             MVPP2_SHARED *priv);
VOID mvpp2_rxq_hw_init(PP2DXE_PORT *port,
        MVPP2_RX_QUEUE *rxq);
VOID mvpp2_rxq_drop_pkts(PP2DXE_PORT *port,
          MVPP2_RX_QUEUE *rxq,
          INT32 cpu);
VOID mvpp2_txq_hw_init(PP2DXE_PORT *port,
        MVPP2_TX_QUEUE *txq);
VOID mvpp2_txq_hw_deinit(PP2DXE_PORT *port,
          MVPP2_TX_QUEUE *txq);
VOID mvpp2_port_power_up(PP2DXE_PORT *port);
VOID mvpp2_rx_fifo_init(MVPP2_SHARED *priv);
VOID mvpp2_rxq_hw_deinit(PP2DXE_PORT *port,
          MVPP2_RX_QUEUE *rxq);
UINT32 mvp_pp2x_gop110_netc_cfg_create(PP2DXE_PORT *pp2_port);
INT32 mv_gop110_netc_init(PP2DXE_PORT *mvport,
      UINT32 net_comp_config, enum mv_netc_phase phase);

UINT32 mvp_pp2x_gop110_netc_cfg_create(PP2DXE_PORT *pp2_port);
INT32 mv_gop110_port_init(PP2DXE_PORT *pp2_port);
INT32 mv_gop110_gmac_reset(PP2DXE_PORT *pp2_port, enum mv_reset reset);
INT32 mv_gop110_gpcs_mode_cfg(PP2DXE_PORT *pp2_port, BOOLEAN en);
INT32 mv_gop110_bypass_clk_cfg(PP2DXE_PORT *pp2_port, BOOLEAN en);
INT32 mv_gop110_gpcs_reset(PP2DXE_PORT *pp2_port, enum mv_reset act);
VOID mv_gop110_xlg_2_gig_mac_cfg(PP2DXE_PORT *pp2_port);
INT32 mv_gop110_gmac_mode_cfg(PP2DXE_PORT *pp2_port);
VOID mv_gop110_gmac_rgmii_cfg(PP2DXE_PORT *pp2_port);
VOID mv_gop110_gmac_sgmii2_5_cfg(PP2DXE_PORT *pp2_port);
VOID mv_gop110_gmac_sgmii_cfg(PP2DXE_PORT *pp2_port);
VOID mv_gop110_gmac_qsgmii_cfg(PP2DXE_PORT *pp2_port);
INT32 mvpp2_smi_phy_addr_cfg(PP2DXE_PORT *pp2_port, INT32 port, INT32 addr);
BOOLEAN mv_gop110_port_is_link_up(PP2DXE_PORT *pp2_port);
BOOLEAN mv_gop110_gmac_link_status_get(PP2DXE_PORT *pp2_port);
INTN mvpp2_bm_pool_ctrl(MVPP2_SHARED *pp2, INTN pool,
    enum mvpp2_command cmd);
VOID mv_gop110_port_disable(PP2DXE_PORT *pp2_port);
VOID mv_gop110_port_enable(PP2DXE_PORT *pp2_port);
VOID mv_gop110_gmac_port_enable(PP2DXE_PORT *pp2_port);
VOID mv_gop110_gmac_port_disable(PP2DXE_PORT *pp2_port);
VOID mv_gop110_gmac_port_link_event_mask(PP2DXE_PORT *pp2_port);
INT32 mv_gop110_port_events_mask(PP2DXE_PORT *pp2_port);
INT32 mv_gop110_fl_cfg(PP2DXE_PORT *pp2_port);
INT32 mv_gop110_speed_duplex_set(PP2DXE_PORT *pp2_port,
      INT32 speed, enum mv_port_duplex duplex);
INT32 mv_gop110_gmac_speed_duplex_set(PP2DXE_PORT *pp2_port,
  INT32 speed, enum mv_port_duplex duplex);
VOID mvpp2_axi_config(MVPP2_SHARED *pp2);
VOID mvpp2_txp_clean(PP2DXE_PORT *pp, INT32 txp,
          MVPP2_TX_QUEUE *txq);
VOID mvpp2_cleanup_txqs(PP2DXE_PORT *pp);
VOID mvpp2_cleanup_rxqs(PP2DXE_PORT *pp);

/* Get number of physical egress port */
STATIC inline INT32 mvpp2_egress_port(PP2DXE_PORT *port)
{
  return MVPP2_MAX_TCONT + port->id;
}

/* Get number of physical TXQ */
STATIC inline INT32 mvpp2_txq_phys(INT32 port, INT32 txq)
{
  return (MVPP2_MAX_TCONT + port) * MVPP2_MAX_TXQ + txq;
}

/* Set pool number in a BM cookie */
STATIC inline UINT32 mvpp2_bm_cookie_pool_set(UINT32 cookie, INT32 pool)
{
  UINT32 bm;

  bm = cookie & ~(0xFF << MVPP2_BM_COOKIE_POOL_OFFS);
  bm |= ((pool & 0xFF) << MVPP2_BM_COOKIE_POOL_OFFS);

  return bm;
}

/* Get pool number from a BM cookie */
STATIC inline INT32 mvpp2_bm_cookie_pool_get(UINT32 cookie)
{
  return (cookie >> MVPP2_BM_COOKIE_POOL_OFFS) & 0xFF;
}

#ifdef MVPP2_V1
/* Release buffer to BM */
STATIC inline VOID mvpp2_bm_pool_put(MVPP2_SHARED *pp2, INT32 pool,
            UINT32 buf_phys_addr, UINT32 buf_virt_addr)
{
  mvpp2_write(port->priv, MVPP2_BM_VIRT_RLS_REG, buf_virt_addr);
  mvpp2_write(port->priv, MVPP2_BM_PHY_RLS_REG(pool), buf_phys_addr);
}
#else
STATIC inline VOID mvpp2_bm_pool_put(MVPP2_SHARED *pp2, INT32 pool,
            UINT64 buf_phys_addr, UINT64 buf_virt_addr)
{
  UINT32 val = 0;

  val = (upper_32_bits(buf_virt_addr) & MVPP22_ADDR_HIGH_MASK)
    << MVPP22_BM_VIRT_HIGH_RLS_OFFST;
  val |= (upper_32_bits(buf_phys_addr) & MVPP22_ADDR_HIGH_MASK)
    << MVPP22_BM_PHY_HIGH_RLS_OFFSET;
  mvpp2_write(pp2, MVPP22_BM_PHY_VIRT_HIGH_RLS_REG, val);
  mvpp2_write(pp2, MVPP2_BM_VIRT_RLS_REG, (UINT32)buf_virt_addr);
  mvpp2_write(pp2, MVPP2_BM_PHY_RLS_REG(pool), (UINT32)buf_phys_addr);
}
#endif

STATIC inline VOID mvpp2_interrupts_enable(PP2DXE_PORT *port,
             INT32 cpu_mask)
{
  mvpp2_write(port->priv, MVPP2_ISR_ENABLE_REG(port->id),
        MVPP2_ISR_ENABLE_INTERRUPT(cpu_mask));
}

STATIC inline VOID mvpp2_interrupts_disable(PP2DXE_PORT *port,
              INT32 cpu_mask)
{
  mvpp2_write(port->priv, MVPP2_ISR_ENABLE_REG(port->id),
        MVPP2_ISR_DISABLE_INTERRUPT(cpu_mask));
}

/* Get number of Rx descriptors occupied by received packets */
STATIC inline INT32
mvpp2_rxq_received(PP2DXE_PORT *port, INT32 rxq_id)
{
  UINT32 val = mvpp2_read(port->priv, MVPP2_RXQ_STATUS_REG(rxq_id));

  return val & MVPP2_RXQ_OCCUPIED_MASK;
}

/* Update Rx queue status with the number of occupied and available
 * Rx descriptor slots.
 */
STATIC inline VOID
mvpp2_rxq_status_update(PP2DXE_PORT *port, INT32 rxq_id,
      INT32 used_count, INT32 free_count)
{
  /* Decrement the number of used descriptors and increment count
   * increment the number of free descriptors.
   */
  UINT32 val = used_count | (free_count << MVPP2_RXQ_NUM_NEW_OFFSET);

  mvpp2_write(port->priv, MVPP2_RXQ_STATUS_UPDATE_REG(rxq_id), val);
}

/* Get pointer to next RX descriptor to be processed by SW */
STATIC inline MVPP2_RX_DESC *
mvpp2_rxq_next_desc_get(MVPP2_RX_QUEUE *rxq)
{
  INT32 rx_desc = rxq->next_desc_to_proc;

  rxq->next_desc_to_proc = MVPP2_QUEUE_NEXT_DESC(rxq, rx_desc);
  mvpp2_prefetch(rxq->descs + rxq->next_desc_to_proc);
  return rxq->descs + rx_desc;
}

/* Get number of sent descriptors and decrement counter.
 * The number of sent descriptors is returned.
 * Per-CPU access
 */
STATIC inline INT32 mvpp2_txq_sent_desc_proc(PP2DXE_PORT *port,
            MVPP2_TX_QUEUE *txq)
{
  UINT32 val;

  /* Reading status reg resets transmitted descriptor counter */
#ifdef MVPP2V1
  val = mvpp2_read(port->priv, MVPP2_TXQ_SENT_REG(txq->id));
#else
  val = mvpp2_read(port->priv, MVPP22_TXQ_SENT_REG(txq->id));
#endif

  return (val & MVPP2_TRANSMITTED_COUNT_MASK) >>
    MVPP2_TRANSMITTED_COUNT_OFFSET;
}

STATIC inline MVPP2_RX_QUEUE *mvpp2_get_rx_queue(PP2DXE_PORT *port,
             UINT32 cause)
{
  INT32 queue = mvpp2_fls(cause) - 1;

  return &port->rxqs[queue];
}

STATIC inline MVPP2_TX_QUEUE *mvpp2_get_tx_queue(PP2DXE_PORT *port,
             UINT32 cause)
{
  INT32 queue = mvpp2_fls(cause) - 1;

  /* XXX - added reference */
  return &port->txqs[queue];
}
 
STATIC inline void mvpp2x2_txdesc_phys_addr_set(dma_addr_t phys_addr,
  MVPP2_TX_DESC *tx_desc) {
  UINT64 *buf_phys_addr_p = &tx_desc->buf_phys_addr_hw_cmd2;

  *buf_phys_addr_p &= ~(MVPP22_ADDR_MASK);
  *buf_phys_addr_p |= phys_addr & MVPP22_ADDR_MASK;
}
#endif /* __MVPP2_LIB_H__ */
