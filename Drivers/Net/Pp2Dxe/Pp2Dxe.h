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

#ifndef __PP2_DXE_H__
#define __PP2_DXE_H__

#include <Protocol/Cpu.h>
#include <Protocol/DriverBinding.h>
#include <Protocol/SimpleNetwork.h>
#include <Protocol/DevicePath.h>
#include <Protocol/MvPhy.h>
#include <Protocol/Ip4.h>
#include <Protocol/Ip6.h>

#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UncachedMemoryAllocationLib.h>
#include <Library/IoLib.h>
#include <Library/DebugLib.h>
#include <Library/PcdLib.h>
#include <Library/NetLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiBootServicesTableLib.h>

#include "mvpp2_lib_hw.h"

#define PP2DXE_MAX_PHY  2

#define PP2DXE_DEFAULT_MAC_ADDR { 0x0, 0xaf, 0x1c, 0xdd, 0xe, 0x2 }

#define MVPP2_TX_SEND_TIMEOUT    10000

#define NET_SKB_PAD 0
#define PP2DXE_SIGNATURE  SIGNATURE_32('P', 'P', '2', 'D')
#define INSTANCE_FROM_SNP(a)  CR((a), PP2DXE_CONTEXT, Snp, PP2DXE_SIGNATURE)

/* RX buffer constants */
#define MVPP2_SKB_SHINFO_SIZE \
  SKB_DATA_ALIGN(sizeof(struct skb_shared_info))

#define MVPP2_RX_PKT_SIZE(mtu) \
  ALIGN((mtu) + MVPP2_MH_SIZE + MVPP2_VLAN_TAG_LEN + \
        ETH_HLEN + ETH_FCS_LEN, MVPP2_CPU_D_CACHE_LINE_SIZE)

#define MVPP2_RX_BUF_SIZE(pkt_size)  ((pkt_size) + NET_SKB_PAD)
#define MVPP2_RX_TOTAL_SIZE(buf_size)  ((buf_size) + MVPP2_SKB_SHINFO_SIZE)
#define MVPP2_RX_MAX_PKT_SIZE(total_size) \
  ((total_size) - NET_SKB_PAD - MVPP2_SKB_SHINFO_SIZE)
#define MVPP2_RXQ_OFFSET  NET_SKB_PAD

#define IS_NOT_ALIGN(number, align)  ((number) & ((align) - 1))
/* Macro for alignment up. For example, ALIGN_UP(0x0330, 0x20) = 0x0340 */
#define ALIGN(x, a)     (((x) + ((a) - 1)) & ~((a) - 1))
#define ALIGN_UP(number, align) (((number) & ((align) - 1)) ? \
    (((number) + (align)) & ~((align)-1)) : (number))

/* Linux API */
#define mvpp2_alloc(v)    AllocateZeroPool(v)
#define mvpp2_free(p)    FreePool(p)
#define mvpp2_memset(a, v, s)  SetMem((a), (s), (v))
#define mvpp2_mdelay(t)    gBS->Stall((t)*1000)
#define mvpp2_prefetch(v)  do {} while(0);
#define mvpp2_fls(v)    1
#define mvpp2_is_broadcast_ether_addr(da)  \
        1
#define mvpp2_is_multicast_ether_addr(da)  \
        1
#define mvpp2_printf(...)  do {} while(0);
#define mvpp2_swap(a,b)    do { typeof(a) __tmp = (a); (a) = (b); (b) = __tmp; } while (0)
#define mvpp2_swab16(x) \
  ((UINT16)( \
    (((UINT16)(x) & (UINT16)0x00ffU) << 8) | \
    (((UINT16)(x) & (UINT16)0xff00U) >> 8) ))
#define mvpp2_iphdr    EFI_IP4_HEADER
#define mvpp2_ipv6hdr  EFI_IP6_HEADER
#define MVPP2_ALIGN(x, m)  ALIGN((x), (m))
#define MVPP2_ALIGN_UP(number, align) ALIGN_UP((number), (align))
#define MVPP2_NULL    NULL
#define MVPP2_ENOMEM    -1
#define MVPP2_EINVAL    -2
#define MVPP2_ERANGE    -3
#define MVPP2_USEC_PER_SEC  1000000L

#define dma_addr_t    UINT64
#define phys_addr_t    UINT64

#define upper_32_bits(n) ((UINT32)(((n) >> 16) >> 16))
#define lower_32_bits(n) ((UINT32)(n))

/* Port speeds */
#define MV_PORT_SPEED_10        SPEED_10
#define MV_PORT_SPEED_100       SPEED_100
#define MV_PORT_SPEED_1000      SPEED_1000
#define MV_PORT_SPEED_2500      SPEED_2500
#define MV_PORT_SPEED_10000     SPEED_10000

/* L2 and L3 protocol macros */
#define MV_IPPR_TCP    0
#define MV_IPPR_UDP    1
#define MV_IPPR_IPIP    2
#define MV_IPPR_ICMPV6    3
#define MV_IPPR_IGMP    4
#define MV_ETH_P_IP    5
#define MV_ETH_P_IPV6    6
#define MV_ETH_P_PPP_SES  7
#define MV_ETH_P_ARP    8
#define MV_ETH_P_8021Q    9
#define MV_ETH_P_8021AD    10
#define MV_ETH_P_EDSA    11
#define MV_PPP_IP    12
#define MV_PPP_IPV6    13
#define MV_ETH_ALEN    6

/* PHY modes */
#define MV_MODE_SGMII    PHY_CONNECTION_SGMII
#define MV_MODE_RGMII    PHY_CONNECTION_RGMII
#define MV_MODE_XAUI    PHY_CONNECTION_XAUI
#define MV_MODE_RXAUI    PHY_CONNECTION_RXAUI
#define MV_MODE_QSGMII    100

/* AXI Bridge Registers */
#define MVPP22_AXI_BM_WR_ATTR_REG    0x4100
#define MVPP22_AXI_BM_RD_ATTR_REG    0x4104
#define MVPP22_AXI_AGGRQ_DESCR_RD_ATTR_REG  0x4110
#define MVPP22_AXI_TXQ_DESCR_WR_ATTR_REG  0x4114
#define MVPP22_AXI_TXQ_DESCR_RD_ATTR_REG  0x4118
#define MVPP22_AXI_RXQ_DESCR_WR_ATTR_REG  0x411c
#define MVPP22_AXI_RX_DATA_WR_ATTR_REG    0x4120
#define MVPP22_AXI_TX_DATA_RD_ATTR_REG    0x4130
#define MVPP22_AXI_RD_NORMAL_CODE_REG    0x4150
#define MVPP22_AXI_RD_SNP_CODE_REG    0x4154
#define MVPP22_AXI_WR_NORMAL_CODE_REG    0x4160
#define MVPP22_AXI_WR_SNP_CODE_REG    0x4164

#define MVPP22_AXI_RD_CODE_MASK      0x33
#define MVPP22_AXI_WR_CODE_MASK      0x33

#define MVPP22_AXI_ATTR_CACHE_OFFS    0
#define MVPP22_AXI_ATTR_CACHE_SIZE    4
#define MVPP22_AXI_ATTR_CACHE_MASK    0x0000000F

#define MVPP22_AXI_ATTR_QOS_OFFS    4
#define MVPP22_AXI_ATTR_QOS_SIZE    4
#define MVPP22_AXI_ATTR_QOS_MASK    0x000000F0

#define MVPP22_AXI_ATTR_TC_OFFS      8
#define MVPP22_AXI_ATTR_TC_SIZE      4
#define MVPP22_AXI_ATTR_TC_MASK      0x00000F00

#define MVPP22_AXI_ATTR_DOMAIN_OFFS    12
#define MVPP22_AXI_ATTR_DOMAIN_SIZE    2
#define MVPP22_AXI_ATTR_DOMAIN_MASK    0x00003000

#define MVPP22_AXI_ATTR_SNOOP_CNTRL_BIT    BIT(16)

/* Gop related define */
/* Sets the field located at the specified in data.     */
#define U32_SET_FIELD(data, mask, val)  \
        ((data) = (((data) & ~(mask)) | (val)))
#define MV_RGMII_TX_FIFO_MIN_TH    (0x41)
#define MV_SGMII_TX_FIFO_MIN_TH    (0x5)
#define MV_SGMII2_5_TX_FIFO_MIN_TH  (0xB)

/* BM configuration */
#define MVPP2_BM_POOL      0
#define MVPP2_BM_SIZE      32

/* BM constants */
#define MVPP2_BM_POOLS_NUM    8
#define MVPP2_BM_LONG_BUF_NUM    1024
#define MVPP2_BM_SHORT_BUF_NUM    2048
#define MVPP2_BM_POOL_SIZE_MAX    (16*1024 - MVPP2_BM_POOL_PTR_ALIGN/4)
#define MVPP2_BM_POOL_PTR_ALIGN    128
#define MVPP2_BM_SWF_LONG_POOL(port)  ((port > 2) ? 2 : port)
#define MVPP2_BM_SWF_SHORT_POOL    3

/* BM cookie (32 bits) definition */
#define MVPP2_BM_COOKIE_POOL_OFFS  8
#define MVPP2_BM_COOKIE_CPU_OFFS  24

/*
 * Page table entries are set to 1MB, or multiples of 1MB
 * (not < 1MB). driver uses less bd's so use 1MB bdspace.
 */
#define BD_SPACE  (1 << 20)

/* buffer has to be aligned to 1M */
#define MVPP2_BUFFER_ALIGN_SIZE  (1 << 20)

#define __iomem

enum mvpp2_command {
  MVPP2_START,    /* Start     */
  MVPP2_STOP,    /* Stop     */
  MVPP2_PAUSE,    /* Pause    */
  MVPP2_RESTART    /* Restart  */
};

#define ARCH_DMA_MINALIGN 64

/* rx buffer size */
#define BUFF_HDR_OFFS  32
#define BM_ALIGN       32
#define ETH_HLEN       14
#define ETH_ALEN       6
/* 2(HW hdr) 14(MAC hdr) 4(CRC) 32(extra for cache prefetch) */
#define WRAP      (2 + ETH_HLEN + 4 + 32)
#define MTU      1500
#define RX_BUFFER_SIZE    (ALIGN(MTU + WRAP, ARCH_DMA_MINALIGN))

/* Structures */

typedef struct mvpp2 MVPP2_SHARED;

typedef struct {
  /* Physical number of this Tx queue */
  UINT8 id;

  /* Logical number of this Tx queue */
  UINT8 log_id;

  /* Number of Tx DMA descriptors in the descriptor ring */
  INT32 size;

  /* Number of currently used Tx DMA descriptor in the descriptor ring */
  INT32 count;

  UINT32 done_pkts_coal;

  /* Virtual address of thex Tx DMA descriptors array */
  MVPP2_TX_DESC *descs;

  /* DMA address of the Tx DMA descriptors array */
  dma_addr_t descs_phys;

  /* Index of the last Tx DMA descriptor */
  INT32 last_desc;

  /* Index of the next Tx DMA descriptor to process */
  INT32 next_desc_to_proc;
} MVPP2_TX_QUEUE;

typedef struct {
  /* RX queue number, in the range 0-31 for physical RXQs */
  UINT8 id;

  /* Num of rx descriptors in the rx descriptor ring */
  INT32 size;

  UINT32 pkts_coal;
  UINT32 time_coal;

  /* Virtual address of the RX DMA descriptors array */
  MVPP2_RX_DESC *descs;

  /* DMA address of the RX DMA descriptors array */
  dma_addr_t descs_phys;

  /* Index of the last RX DMA descriptor */
  INT32 last_desc;

  /* Index of the next RX DMA descriptor to process */
  INT32 next_desc_to_proc;

  /* ID of port to which physical RXQ is mapped */
  INT32 port;

  /* Port's logic RXQ number to which physical RXQ is mapped */
  INT32 logic_rxq;
} MVPP2_RX_QUEUE;

enum mvpp2_bm_type {
  MVPP2_BM_FREE,
  MVPP2_BM_SWF_LONG,
  MVPP2_BM_SWF_SHORT
};

typedef struct {
  /* Pool number in the range 0-7 */
  INT32 id;
  enum mvpp2_bm_type type;

  /* Buffer Pointers Pool External (BPPE) size */
  INT32 size;
  /* Number of buffers for this pool */
  INT32 buf_num;
  /* Pool buffer size */
  INT32 buf_size;
  /* Packet size */
  INT32 pkt_size;

  /* BPPE virtual base address */
  UINT32 *virt_addr;
  /* BPPE physical base address */
  dma_addr_t phys_addr;

  /* Ports using BM pool */
  UINT32 port_map;
} MVPP2_BMS_POOL;

/* Individual port structure */
typedef struct {
  UINT8 id;
  UINT8 gop_index;

  INT32 irq;

  MVPP2_SHARED *priv;

  /* Per-port registers' base address */
  UINT64 gmac_base;
  UINT64 xlg_base;

  MVPP2_RX_QUEUE *rxqs;
  MVPP2_TX_QUEUE *txqs;

  INT32 pkt_size;

  UINT32 pending_cause_rx;

  /* Per-CPU port control */

  /* Flags */
  UINTN flags;

  UINT16 tx_ring_size;
  UINT16 rx_ring_size;

  INT32 phy_interface;
  BOOLEAN link;
  BOOLEAN duplex;
  BOOLEAN always_up;
  PHY_SPEED speed;

  MVPP2_BMS_POOL *pool_long;
  MVPP2_BMS_POOL *pool_short;

  UINT8 txp_num;

  /* Index of first port's physical RXQ */
  UINT8 first_rxq;
} PP2DXE_PORT;

/* Shared Packet Processor resources */
struct mvpp2 {
  /* Shared registers' base addresses */
  UINT64 __iomem base;
  UINT64 __iomem rfu1_base;
  UINT64 __iomem smi_base;
  VOID __iomem *lms_base;

  /* List of pointers to port structures */
  PP2DXE_PORT **port_list;

  /* Aggregated TXQs */
  MVPP2_TX_QUEUE *aggr_txqs;

  /* BM pools */
  MVPP2_BMS_POOL *bm_pools;

  /* PRS shadow table */
  MVPP2_PRS_SHADOW *prs_shadow;
  /* PRS auxiliary table for double vlan entries control */
  BOOLEAN *prs_double_vlans;

  /* Tclk value */
  UINT32 tclk;
};

/* Structure for preallocation for buffer */
typedef struct {
  MVPP2_TX_DESC *tx_descs;
  MVPP2_TX_DESC *aggr_tx_descs;
  MVPP2_RX_DESC *rx_descs;
  dma_addr_t rx_buffers;
} BUFFER_LOCATION;

#define QUEUE_DEPTH 64
typedef struct {
  UINT32                      Signature;
  INTN                        Instance;
  EFI_HANDLE                  Controller;
  EFI_LOCK                    Lock;
  EFI_SIMPLE_NETWORK_PROTOCOL Snp;
  MARVELL_PHY_PROTOCOL        *Phy;
  PHY_DEVICE                  *PhyDev;
  PP2DXE_PORT                 Port;
  BOOLEAN                     Initialized;
  BOOLEAN                     LateInitialized;
  VOID                        *CompletionQueue[QUEUE_DEPTH];
  UINTN                       CompletionQueueHead;
  UINTN                       CompletionQueueTail;
  EFI_EVENT                   EfiExitBootServicesEvent;
} PP2DXE_CONTEXT;

static inline VOID mvpp2_write(MVPP2_SHARED *priv, UINT32 offset,
          UINT32 data)
{
  ASSERT (priv->base != 0);
  MmioWrite32 (priv->base + offset, data);
}

static inline UINT32 mvpp2_read(MVPP2_SHARED *priv, UINT32 offset)
{
  ASSERT (priv->base != 0);
  return MmioRead32 (priv->base + offset);
}

static inline UINT32 mvpp2_rfu1_read(MVPP2_SHARED *priv, UINT32 offset)
{
  ASSERT (priv->rfu1_base != 0);
  return MmioRead32 (priv->rfu1_base + offset);
}

static inline UINT32 mvpp2_rfu1_write(MVPP2_SHARED *priv, UINT32 offset,
              UINT32 data)
{
  ASSERT (priv->rfu1_base != 0);
  return MmioWrite32 (priv->rfu1_base + offset, data);
}

static inline UINT32 mvpp2_smi_read(MVPP2_SHARED *priv, UINT32 offset)
{
  ASSERT (priv->smi_base != 0);
  return MmioRead32 (priv->smi_base + offset);
}

static inline UINT32 mvpp2_smi_write(MVPP2_SHARED *priv, UINT32 offset,
              UINT32 data)
{
  ASSERT (priv->smi_base != 0);
  return MmioWrite32 (priv->smi_base + offset, data);
}

static inline VOID mvpp2_gmac_write(PP2DXE_PORT *port, UINT32 offset,
               UINT32 data)
{
  ASSERT (port->priv->base != 0);
  MmioWrite32 (port->priv->base + offset, data);
}

static inline UINT32 mvpp2_gmac_read(PP2DXE_PORT *port, UINT32 offset)
{
  ASSERT (port->priv->base != 0);
  return MmioRead32 (port->priv->base + offset);
}

static inline VOID mv_gop110_gmac_write(PP2DXE_PORT *port, UINT32 offset,
               UINT32 data)
{
  ASSERT (port->gmac_base != 0);
  MmioWrite32 (port->gmac_base + offset, data);
}

static inline UINT32 mv_gop110_gmac_read(PP2DXE_PORT *port, UINT32 offset)
{
  ASSERT (port->gmac_base != 0);
  return MmioRead32 (port->gmac_base + offset);
}

static inline VOID mvpp2_xlg_write(PP2DXE_PORT *port, UINT32 offset,
               UINT32 data)
{
  ASSERT (port->xlg_base != 0);
  MmioWrite32 (port->xlg_base + offset, data);
}

static inline UINT32 mvpp2_xlg_read(PP2DXE_PORT *port, UINT32 offset)
{
  ASSERT (port->xlg_base != 0);
  return MmioRead32 (port->xlg_base + offset);
}
#endif
