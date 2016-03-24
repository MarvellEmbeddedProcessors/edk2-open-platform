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
#include <Protocol/SimpleNetwork.h>
#include <Protocol/DevicePath.h>
#include <Protocol/Phy.h>

#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/IoLib.h>
#include <Library/DebugLib.h>
#include <Library/PcdLib.h>
#include <Library/NetLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiBootServicesTableLib.h>

#include "Pp2Dxe.h"
#include "mvpp2_lib.h"

typedef struct {
  MAC_ADDR_DEVICE_PATH      Pp2Mac;
  EFI_DEVICE_PATH_PROTOCOL  End;
} PP2_DEVICE_PATH;

MVPP2_SHARED *Mvpp2Shared;
BUFFER_LOCATION BufferLocation;

PP2_DEVICE_PATH Pp2DevicePathTemplate = {
  {
    {
      MESSAGING_DEVICE_PATH, MSG_MAC_ADDR_DP,
      { (UINT8) (sizeof(MAC_ADDR_DEVICE_PATH)), (UINT8) ((sizeof(MAC_ADDR_DEVICE_PATH)) >> 8) }
    },
    { { 0 } },
    0
  },
  {
    END_DEVICE_PATH_TYPE,
    END_ENTIRE_DEVICE_PATH_SUBTYPE,
    { sizeof(EFI_DEVICE_PATH_PROTOCOL), 0 }
  }
};

STATIC
EFI_STATUS
Pp2DxeBmPoolInit (
  VOID
  )
{
  INTN i;
  UINT8 *pool_addr;

  for (i = 0; i < MVPP2_BM_POOLS_NUM; i++) {
    /* bm_irq_clear */
    mvpp2_bm_irq_clear(Mvpp2Shared, i);
  }

  Mvpp2Shared->bm_pools = AllocateZeroPool (sizeof(struct mvpp2_bm_pool));

  if (!Mvpp2Shared->bm_pools)
    return EFI_OUT_OF_RESOURCES;

  pool_addr = AllocateZeroPool ((sizeof(VOID*) * MVPP2_BM_SIZE)*2 +
      MVPP2_BM_POOL_PTR_ALIGN);

  if (!pool_addr) {
    return EFI_OUT_OF_RESOURCES;
  }
  if (IS_NOT_ALIGN((UINT64)pool_addr,
    MVPP2_BM_POOL_PTR_ALIGN))
    pool_addr =
    (UINT8 *)ALIGN_UP((UINT64)pool_addr,
            MVPP2_BM_POOL_PTR_ALIGN);

  Mvpp2Shared->bm_pools->id = MVPP2_BM_POOL;
  Mvpp2Shared->bm_pools->virt_addr = (MV_U32*)pool_addr;
  Mvpp2Shared->bm_pools->phys_addr = (UINT64)pool_addr;

  mvpp2_bm_pool_hw_create(Mvpp2Shared, Mvpp2Shared->bm_pools, MVPP2_BM_SIZE);

  return EFI_SUCCESS;
}
STATIC
VOID
Pp2DxeAxiConfig (
  VOID
  )
{
  /* Config AXI Read&Write Normal and Soop mode  */
  mvpp2_write(Mvpp2Shared, MVPP22_AXI_RD_NORMAL_CODE_REG,
    MVPP22_AXI_RD_CODE_MASK);
  mvpp2_write(Mvpp2Shared, MVPP22_AXI_RD_SNP_CODE_REG, MVPP22_AXI_RD_CODE_MASK);
  mvpp2_write(Mvpp2Shared, MVPP22_AXI_WR_NORMAL_CODE_REG,
    MVPP22_AXI_WR_CODE_MASK);
  mvpp2_write(Mvpp2Shared, MVPP22_AXI_WR_SNP_CODE_REG, MVPP22_AXI_WR_CODE_MASK);
}

/*
 * mvpp2_bm_start
 *   enable and fill BM pool
 */
STATIC
EFI_STATUS
Pp2DxeBmStart (
  IN PP2DXE_PORT *Port
  )
{
  UINT8 *buff, *buff_phys;
  INTN i;

  mvpp2_bm_pool_ctrl(Mvpp2Shared, MVPP2_BM_POOL, MVPP2_START);

  mvpp2_bm_pool_bufsize_set(Mvpp2Shared, Mvpp2Shared->bm_pools, RX_BUFFER_SIZE);

  /* fill BM pool with buffers */
  for (i = 0; i < MVPP2_BM_SIZE; i++) {
    buff = (UINT8 *)(BufferLocation.rx_buffers
      + (i * RX_BUFFER_SIZE));
    if (!buff)
      return EFI_OUT_OF_RESOURCES;

    buff_phys = (UINT8 *)ALIGN_UP((UINT64)buff,
      BM_ALIGN);
    mvpp2_bm_pool_put(Port, MVPP2_BM_POOL,
      (UINT64)buff_phys, (UINT64)buff_phys);
  }

  return EFI_SUCCESS;
}

STATIC
VOID
Pp2DxeStartDev (
  IN PP2DXE_CONTEXT *Pp2Context
  )
{
  PP2DXE_PORT *Port = &Pp2Context->Port;

  mvpp2_ingress_enable(Port);

  /* Config classifier decoding table */
  mvpp2_cls_port_config(Port);
  mvpp2_cls_oversize_rxq_set(Port);
  mv_gop110_port_events_mask(Port);
  mv_gop110_port_enable(Port);

  gBS->Stall(2000);
  mvpp2_egress_enable(Port);
}

STATIC
EFI_STATUS
Pp2DxeSetupRxqs (
  IN PP2DXE_CONTEXT *Pp2Context
  )
{
  INTN Queue;
  EFI_STATUS Status;
  struct mvpp2_rx_queue *rxq;

  for (Queue = 0; Queue < rxq_number; Queue++) {
    rxq = &Pp2Context->Port.rxqs[Queue];
    rxq->descs_phys = (dma_addr_t)rxq->descs;
    if (!rxq->descs) {
      Status = EFI_OUT_OF_RESOURCES;
      goto err_cleanup;
    }

    mvpp2_rxq_hw_init(&Pp2Context->Port, rxq);
  }
  return EFI_SUCCESS;

err_cleanup:
  /* TODO */
  return Status;
}

STATIC
EFI_STATUS
Pp2DxeSetupTxqs (
  IN PP2DXE_CONTEXT *Pp2Context
  )
{
  INTN Queue;
  struct mvpp2_tx_queue *txq;

  for (Queue = 0; Queue < txq_number; Queue++) {
    txq = &Pp2Context->Port.txqs[Queue];
    txq->descs_phys = (dma_addr_t) txq->descs;
    mvpp2_txq_hw_init(&Pp2Context->Port, txq);
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
Pp2DxeSetupAggrTxqs (
  IN PP2DXE_CONTEXT *Pp2Context
  )
{
  struct mvpp2_tx_queue *aggr_txq;

  aggr_txq = Mvpp2Shared->aggr_txqs;
  aggr_txq->descs_phys = (dma_addr_t)aggr_txq->descs;
  if (!aggr_txq->descs)
    return EFI_OUT_OF_RESOURCES;
  mvpp2_aggr_txq_hw_init(aggr_txq, aggr_txq->size, 0, Mvpp2Shared);
  return EFI_SUCCESS;

}

#define ETH_ALEN 6
STATIC
EFI_STATUS
Pp2DxeOpen (
  IN PP2DXE_CONTEXT *Pp2Context
  )
{
  PP2DXE_PORT *Port = &Pp2Context->Port;
  UINT8 mac_bcast[NET_ETHER_ADDR_LEN] = { 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff };
  UINT8 dev_addr[NET_ETHER_ADDR_LEN] = { 0x00, 0x00, 0x00,
        0x00, 0x00, 0x02 };
  INTN ret;
  EFI_STATUS Status;

  DEBUG((DEBUG_INFO, "Pp2Dxe: Open\n"));
  ret = mvpp2_prs_mac_da_accept(Mvpp2Shared, Port->id, mac_bcast, TRUE);
  if (ret) {
    return EFI_DEVICE_ERROR;
  }
  ret = mvpp2_prs_mac_da_accept(Mvpp2Shared, Port->id, dev_addr, TRUE);
  if (ret) {
    return EFI_DEVICE_ERROR;
  }
  ret = mvpp2_prs_tag_mode_set(Mvpp2Shared, Port->id, MVPP2_TAG_TYPE_MH);
  if (ret) {
    return EFI_DEVICE_ERROR;
  }
  ret = mvpp2_prs_def_flow(Port);
  if (ret) {
    return EFI_DEVICE_ERROR;
  }

  Status = Pp2DxeSetupRxqs(Pp2Context);
  if (EFI_ERROR(Status))
    return Status;

  Status = Pp2DxeSetupTxqs(Pp2Context);
  if (EFI_ERROR(Status))
    return Status;

  Status = Pp2DxeSetupAggrTxqs(Pp2Context);
  if (EFI_ERROR(Status))
    return Status;

  Pp2DxeStartDev(Pp2Context);

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
Pp2DxeLatePortInitialize (
  IN PP2DXE_CONTEXT *Pp2Context
  )
{
  PP2DXE_PORT *Port = &Pp2Context->Port;
  INTN Queue;

  DEBUG((DEBUG_INFO, "Pp2Dxe: LatePortInitialize\n"));
  Port->tx_ring_size = MVPP2_MAX_TXD;
  Port->rx_ring_size = MVPP2_MAX_RXD;

  mvpp2_write(Mvpp2Shared, MVPP22_AXI_BM_WR_ATTR_REG,
        MVPP22_AXI_ATTR_SNOOP_CNTRL_BIT);
  mvpp2_write(Mvpp2Shared, MVPP22_AXI_BM_RD_ATTR_REG,
        MVPP22_AXI_ATTR_SNOOP_CNTRL_BIT);
  mvpp2_write(Mvpp2Shared, MVPP22_AXI_AGGRQ_DESCR_RD_ATTR_REG,
        MVPP22_AXI_ATTR_SNOOP_CNTRL_BIT);
  mvpp2_write(Mvpp2Shared, MVPP22_AXI_TXQ_DESCR_WR_ATTR_REG,
        MVPP22_AXI_ATTR_SNOOP_CNTRL_BIT);
  mvpp2_write(Mvpp2Shared, MVPP22_AXI_TXQ_DESCR_RD_ATTR_REG,
        MVPP22_AXI_ATTR_SNOOP_CNTRL_BIT);
  mvpp2_write(Mvpp2Shared, MVPP22_AXI_RXQ_DESCR_WR_ATTR_REG,
        MVPP22_AXI_ATTR_SNOOP_CNTRL_BIT);
  mvpp2_write(Mvpp2Shared, MVPP22_AXI_RX_DATA_WR_ATTR_REG,
        MVPP22_AXI_ATTR_SNOOP_CNTRL_BIT);
  mvpp2_write(Mvpp2Shared, MVPP22_AXI_TX_DATA_RD_ATTR_REG,
        MVPP22_AXI_ATTR_SNOOP_CNTRL_BIT);

  mvpp2_egress_disable(Port);
  mv_gop110_port_events_mask(Port);
  mv_gop110_port_disable(Port);

  Port->txqs = AllocateZeroPool (sizeof(struct mvpp2_tx_queue) * txq_number);
  if (Port->txqs == NULL) {
    DEBUG((DEBUG_ERROR, "Failed to allocate txqs\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  /* Use preallocated area */
  Port->txqs[0].descs = BufferLocation.tx_descs;

  for (Queue = 0; Queue < txq_number; Queue++) {
    struct mvpp2_tx_queue *txq = &Port->txqs[Queue];

    txq->id = mvpp2_txq_phys(Port->id, Queue);
    txq->log_id = Queue;
    txq->size = Port->tx_ring_size;
  }

  Mvpp2Shared->aggr_txqs = AllocateZeroPool (sizeof(struct mvpp2_tx_queue));
  if (Mvpp2Shared->aggr_txqs == NULL) {
    DEBUG((DEBUG_ERROR, "Failed to allocate aggregated txqs\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  Mvpp2Shared->aggr_txqs->descs = BufferLocation.aggr_tx_descs;
  Mvpp2Shared->aggr_txqs->id = 0;
  Mvpp2Shared->aggr_txqs->log_id = 0;
  Mvpp2Shared->aggr_txqs->size = MVPP2_AGGR_TXQ_SIZE;

  Port->rxqs = AllocateZeroPool (sizeof(struct mvpp2_rx_queue) * rxq_number);
  if (Port->rxqs == NULL) {
    /* TODO free resources */
    DEBUG((DEBUG_ERROR, "Failed to allocate rxqs\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  Port->rxqs[0].descs = BufferLocation.rx_descs;

  for (Queue = 0; Queue < txq_number; Queue++) {
    struct mvpp2_rx_queue *rxq = &Port->rxqs[Queue];

    rxq->id = Queue;
    rxq->size = Port->rx_ring_size;
  }

  mvpp2_ingress_disable(Port);

  mvpp2_defaults_set(Port);

  Pp2DxeOpen(Pp2Context);

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
Pp2DxeLateInitialize (
  IN PP2DXE_CONTEXT *Pp2Context
  )
{
  PP2DXE_PORT *Port = &Pp2Context->Port;
  EFI_STATUS Status;

  DEBUG((DEBUG_INFO, "Pp2Dxe: Pp2DxeLateInitialize\n"));

  mvpp2_write(Port->priv, 0xf5060, 1);
  if (!Pp2Context->LateInitialized) {
    Status = Pp2DxeBmStart(Port);
    if (EFI_ERROR(Status)) {
      DEBUG((DEBUG_ERROR, "Pp2Dxe: bm start error\n"));
      return Status;
    }
    /* Full init on first call */
    Pp2DxeLatePortInitialize(Pp2Context);
    /* Attach pool to rxq */
    mvpp2_rxq_long_pool_set(Port, 0, MVPP2_BM_POOL);
    mvpp2_rxq_short_pool_set(Port, 0, MVPP2_BM_POOL);
    /* mark this port being fully inited,
     * otherwise it will be inited again
     * during next networking transaction,
     * including memory allocatation for
     * TX/RX queue, PHY connect/configuration
     * and address decode configuration.
     */
    Pp2Context->LateInitialized = TRUE;
  } else {
    /* Upon all following calls, this is enough */
    Pp2DxeBmStart(Port);
    mvpp2_txq_drain_set(Port, 0, FALSE);
    mv_gop110_port_events_mask(Port);
    mv_gop110_port_enable(Port);
    mvpp2_ingress_enable(Port);
    mvpp2_egress_enable(Port);
  }
  return 0;
}

EFI_STATUS
Pp2DxePhyInitialize (
  PP2DXE_CONTEXT *Pp2Context
  )
{
  EFI_STATUS Status;
  UINT8 *PhyAddresses;

  PhyAddresses = PcdGetPtr (PcdPhySmiAddresses);
  DEBUG((DEBUG_ERROR, "PhyAddresses are: %d %d\n", PhyAddresses[0], PhyAddresses[1]));
  Status = gBS->LocateProtocol (
      &gEfiPhyProtocolGuid,
      NULL,
      (VOID **) &Pp2Context->Phy
      );
  if (EFI_ERROR(Status))
    return Status;

  Status = Pp2Context->Phy->Init(
            Pp2Context->Phy,
            PhyAddresses[Pp2Context->Instance],
            &Pp2Context->PhyDev
            );
  if (EFI_ERROR(Status) && Status != EFI_TIMEOUT)
    return Status;
  Pp2Context->Phy->Status(Pp2Context->Phy, Pp2Context->PhyDev);
  DEBUG((DEBUG_ERROR,
    "PHY%d: ",
    Pp2Context->PhyDev->Addr));
  DEBUG((DEBUG_ERROR,
    Pp2Context->PhyDev->LinkUp ? "link up, " : "link down, "));
  DEBUG((DEBUG_ERROR,
    Pp2Context->PhyDev->Duplex ? "duplex, " : "no duplex, "));
  DEBUG((DEBUG_ERROR,
    Pp2Context->PhyDev->Speed == SPEED_10 ? "speed 10\n" : (Pp2Context->PhyDev->Speed == SPEED_100 ? "speed 100\n" : "speed 1000\n")));

  mvpp2_smi_phy_addr_cfg(&Pp2Context->Port, Pp2Context->Port.gop_index, Pp2Context->PhyDev->Addr);

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
Pp2DxeSnpInitialize (
  IN EFI_SIMPLE_NETWORK_PROTOCOL                    *This,
  IN UINTN                                          ExtraRxBufferSize  OPTIONAL,
  IN UINTN                                          ExtraTxBufferSize  OPTIONAL
  )
{
  EFI_STATUS Status;
  PP2DXE_CONTEXT *Pp2Context;
  Pp2Context = INSTANCE_FROM_SNP(This);

  DEBUG((DEBUG_INFO, "Pp2Dxe%d: initialize\n", Pp2Context->Instance));
  if (ExtraRxBufferSize != 0 || ExtraTxBufferSize != 0) {
    DEBUG((DEBUG_ERROR, "Pp2Dxe%d: non-zero buffer requests\n", Pp2Context->Instance));
    return EFI_UNSUPPORTED;
  }

  switch (This->Mode->State) {
  case EfiSimpleNetworkStarted:
  DEBUG((DEBUG_INFO, "Pp2Dxe%d: started state\n", Pp2Context->Instance));
    break;
  case EfiSimpleNetworkInitialized:
    DEBUG((DEBUG_INFO, "Pp2Dxe%d: already initialized\n", Pp2Context->Instance));
    return (EFI_SUCCESS);
  case EfiSimpleNetworkStopped:
    DEBUG((DEBUG_INFO, "Pp2Dxe%d: network stopped\n", Pp2Context->Instance));
    return (EFI_NOT_STARTED);
  default:
    DEBUG((DEBUG_INFO, "Pp2Dxe%d: wrong state\n", Pp2Context->Instance));
    return (EFI_DEVICE_ERROR);
  }

  This->Mode->State = EfiSimpleNetworkInitialized;

  if (Pp2Context->Initialized)
    return EFI_SUCCESS;

  Pp2Context->Initialized = TRUE;

  Status = Pp2DxePhyInitialize(Pp2Context);
  if (EFI_ERROR(Status))
    return Status;

  Status = Pp2DxeLateInitialize(Pp2Context);
  return Status;
}

EFI_STATUS
EFIAPI
Pp2SnpStart (
  IN EFI_SIMPLE_NETWORK_PROTOCOL  *This
  )
{
  PP2DXE_CONTEXT *Pp2Context;
  Pp2Context = INSTANCE_FROM_SNP(This);
  DEBUG((DEBUG_INFO, "Pp2Dxe%d: started\n", Pp2Context->Instance));
  switch (This->Mode->State) {
  case EfiSimpleNetworkStopped:
  DEBUG((DEBUG_INFO, "Pp2Dxe%d: stopped state\n", Pp2Context->Instance));
    break;
  case EfiSimpleNetworkStarted:
  case EfiSimpleNetworkInitialized:
    DEBUG((DEBUG_INFO, "Pp2Dxe: Driver already started\n"));
    return (EFI_ALREADY_STARTED);
  default:
    DEBUG((DEBUG_ERROR, "Pp2Dxe: Driver in an invalid state: %u\n",
          (UINTN)This->Mode->State));
    return (EFI_DEVICE_ERROR);
  }
  This->Mode->State = EfiSimpleNetworkStarted;
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
Pp2SnpStop (
  IN EFI_SIMPLE_NETWORK_PROTOCOL  *This
  )
{
  DEBUG((DEBUG_INFO, "Pp2SnpStop \n"));
  switch (This->Mode->State) {
  case EfiSimpleNetworkStarted:
  case EfiSimpleNetworkInitialized:
    break;
  case EfiSimpleNetworkStopped:
    return (EFI_NOT_STARTED);
  default:
    return (EFI_DEVICE_ERROR);
  }
  This->Mode->State = EfiSimpleNetworkStopped;
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
Pp2SnpReset (
  IN EFI_SIMPLE_NETWORK_PROTOCOL   *This,
  IN BOOLEAN                       ExtendedVerification
  )
{
  DEBUG((DEBUG_INFO, "Pp2SnpReset \n"));
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
Pp2SnpShutdown (
  IN EFI_SIMPLE_NETWORK_PROTOCOL  *This
  )
{
  switch (This->Mode->State) {
  case EfiSimpleNetworkInitialized:
    break;
  case EfiSimpleNetworkStarted:
    return (EFI_DEVICE_ERROR);
  case EfiSimpleNetworkStopped:
    return (EFI_NOT_STARTED);
  default:
    return (EFI_DEVICE_ERROR);
  }

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
Pp2SnpReceiveFilters (
  IN EFI_SIMPLE_NETWORK_PROTOCOL                             *This,
  IN UINT32                                                  Enable,
  IN UINT32                                                  Disable,
  IN BOOLEAN                                                 ResetMCastFilter,
  IN UINTN                                                   MCastFilterCnt     OPTIONAL,
  IN EFI_MAC_ADDRESS                                         *MCastFilter OPTIONAL
  )
{
  DEBUG((DEBUG_INFO, "Pp2SnpReceiveFilt \n"));
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
Pp2SnpNetStat (
  IN EFI_SIMPLE_NETWORK_PROTOCOL          *This,
  IN BOOLEAN                              Reset,
  IN OUT UINTN                            *StatisticsSize   OPTIONAL,
  OUT EFI_NETWORK_STATISTICS              *StatisticsTable  OPTIONAL
  )
{
  DEBUG((DEBUG_INFO, "Pp2SnpNetStat \n"));
  return EFI_UNSUPPORTED;
}

EFI_STATUS
EFIAPI
Pp2SnpIpToMac (
  IN EFI_SIMPLE_NETWORK_PROTOCOL          *This,
  IN BOOLEAN                              IPv6,
  IN EFI_IP_ADDRESS                       *IP,
  OUT EFI_MAC_ADDRESS                     *MAC
  )
{
  DEBUG((DEBUG_INFO, "Pp2SnpIpToMac \n"));
  return EFI_UNSUPPORTED;
}

EFI_STATUS
EFIAPI
Pp2SnpNvData (
  IN EFI_SIMPLE_NETWORK_PROTOCOL          *This,
  IN BOOLEAN                              ReadWrite,
  IN UINTN                                Offset,
  IN UINTN                                BufferSize,
  IN OUT VOID                             *Buffer
  )
{
  DEBUG((DEBUG_INFO, "Pp2SnpNvData \n"));
  return EFI_UNSUPPORTED;
}

EFI_STATUS
EFIAPI
Pp2SnpGetStatus (
  IN EFI_SIMPLE_NETWORK_PROTOCOL          *Snp,
  OUT UINT32                              *InterruptStatus OPTIONAL,
  OUT VOID                                **TxBuf OPTIONAL
  )
{
  PP2DXE_CONTEXT *Pp2Context = INSTANCE_FROM_SNP(Snp);
  BOOLEAN LinkUp;

  if (!Pp2Context->Initialized)
    return EFI_NOT_READY;
  LinkUp = mv_gop110_port_is_link_up(&Pp2Context->Port);
  if (LinkUp != Snp->Mode->MediaPresent) {
    DEBUG((DEBUG_ERROR, "Pp2Dxe%d: Link ", Pp2Context->Instance));
    DEBUG((DEBUG_ERROR, LinkUp ? "up\n" : "down\n"));
  }
  Snp->Mode->MediaPresent = LinkUp;
  return EFI_SUCCESS;
}

#define MVPP2_TX_SEND_TIMEOUT    10000
EFI_STATUS
EFIAPI
Pp2SnpTransmit (
  IN EFI_SIMPLE_NETWORK_PROTOCOL          *This,
  IN UINTN                                HeaderSize,
  IN UINTN                                BufferSize,
  IN VOID                                 *Buffer,
  IN EFI_MAC_ADDRESS                      *SrcAddr  OPTIONAL,
  IN EFI_MAC_ADDRESS                      *DestAddr OPTIONAL,
  IN UINT16                               *Protocol OPTIONAL
  )
{
  PP2DXE_CONTEXT *Pp2Context = INSTANCE_FROM_SNP(This);
  PP2DXE_PORT *Port = &Pp2Context->Port;
  struct mvpp2_tx_queue *aggr_txq = Mvpp2Shared->aggr_txqs;
  struct mvpp2_tx_desc *tx_desc;
  INTN timeout = 0;
  INTN tx_done;

  tx_desc = mvpp2_txq_next_desc_get(aggr_txq);
  if (!tx_desc) {
    DEBUG((DEBUG_ERROR, "No tx descriptor to use\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  /* set descriptor fields */
  tx_desc->command =  MVPP2_TXD_IP_CSUM_DISABLE |
  MVPP2_TXD_L4_CSUM_NOT | MVPP2_TXD_F_DESC | MVPP2_TXD_L_DESC;
  tx_desc->data_size = BufferSize;

  tx_desc->packet_offset = (phys_addr_t)Buffer & MVPP2_TX_DESC_ALIGN;

  mvpp2x2_txdesc_phys_addr_set(
    (phys_addr_t)Buffer & ~MVPP2_TX_DESC_ALIGN, tx_desc);
  tx_desc->phys_txq = mvpp2_txq_phys(Port->id, 0);

  /* iowmb */
  __asm__ __volatile__ ("" : : : "memory");
  /* send */
  mvpp2_aggr_txq_pend_desc_add(Port, 1);

  /* Tx done processing */
  /* wait for agrregated to physical TXQ transfer */
  tx_done = mvpp2_aggr_txq_pend_desc_num_get(Mvpp2Shared, 0);
  do {
    if (timeout++ > MVPP2_TX_SEND_TIMEOUT) {
    }
    tx_done = mvpp2_aggr_txq_pend_desc_num_get(Mvpp2Shared, 0);
  } while (tx_done);

  timeout = 0;
  tx_done = mvpp2_txq_sent_desc_proc(Port, &Port->txqs[0]);
  /* wait for packet to be transmitted */
  while (!tx_done) {
    if (timeout++ > MVPP2_TX_SEND_TIMEOUT) {
      DEBUG((DEBUG_ERROR, "Pp2Dxe: transmit timeout\n"));
      return EFI_TIMEOUT;
    }
    tx_done = mvpp2_txq_sent_desc_proc(Port, &Port->txqs[0]);
  }
  /* tx_done has increased - hw sent packet */

  DEBUG((DEBUG_ERROR, "Pp2Dxe: sent\n"));
  return EFI_SUCCESS;
}

VOID
Pp2DumpPacket (
  IN UINT8 *DataPtr,
  IN UINTN PktLength
  )
{
  INTN i;

  DEBUG((DEBUG_ERROR, "Received packet, length is 0x%x\n", PktLength));

  for (i = 0; i < PktLength; i++) {
    DEBUG((DEBUG_ERROR, "%2x", DataPtr[i]));
    if (i % 2 == 1)
      DEBUG((DEBUG_ERROR, " "));
    if (i % 8 == 7)
      DEBUG((DEBUG_ERROR, "\n"));
  }
  DEBUG((DEBUG_ERROR, "\n"));
}

EFI_STATUS
EFIAPI
Pp2SnpReceive (
  IN EFI_SIMPLE_NETWORK_PROTOCOL          *This,
  OUT UINTN                               *HeaderSize OPTIONAL,
  IN OUT UINTN                            *BufferSize,
  OUT VOID                                *Buffer,
  OUT EFI_MAC_ADDRESS                     *SrcAddr    OPTIONAL,
  OUT EFI_MAC_ADDRESS                     *DstAddr   OPTIONAL,
  OUT UINT16                              *Protocol   OPTIONAL
  )
{
  INTN ReceivedPackets;
  PP2DXE_CONTEXT *Pp2Context = INSTANCE_FROM_SNP(This);
  PP2DXE_PORT *Port = &Pp2Context->Port;
  UINT64 PhysAddr, VirtAddr;
  UINT32 Status;
  INTN PoolId;
  UINTN PktLength;
  UINT8 *DataPtr;
  struct mvpp2_rx_desc *RxDesc;
  struct mvpp2_rx_queue *Rxq = &Port->rxqs[0];

  ReceivedPackets = mvpp2_rxq_received(Port, 0);

  if (ReceivedPackets == 0) {
    return EFI_NOT_READY;
  }

  /* process one packet per call */
  RxDesc = mvpp2_rxq_next_desc_get(Rxq);

  Status = RxDesc->status;

  /* drop packets with error or with buffer header (MC, SG) */
  if ((Status & MVPP2_RXD_BUF_HDR) ||
    (Status & MVPP2_RXD_ERR_SUMMARY))
    goto drop;

  /* give packet to stack - skip on first
    * 2 bytes + buffer header */
  PhysAddr = RxDesc->buf_phys_addr_key_hash &
  MVPP22_ADDR_MASK;
  VirtAddr = RxDesc->buf_cookie_bm_qset_cls_info &
  MVPP22_ADDR_MASK;

  PktLength = (UINTN) RxDesc->data_size - 2;
  if (PktLength > *BufferSize)
    /* TODO */
    return EFI_BUFFER_TOO_SMALL;
  CopyMem (Buffer, (VOID*) (PhysAddr + 2), PktLength);
  *BufferSize = PktLength;

  if (HeaderSize != NULL) {
    *HeaderSize = Pp2Context->Snp.Mode->MediaHeaderSize;
  }

  DataPtr = (UINT8 *)Buffer;
  Pp2DumpPacket ((UINT8*)DataPtr, PktLength);

  // Extract the destination address
  if (DstAddr != NULL) {
    CopyMem (DstAddr, &DataPtr[0], NET_ETHER_ADDR_LEN);
  }

  // Get the source address
  if (SrcAddr != NULL) {
    CopyMem (SrcAddr, &DataPtr[6], NET_ETHER_ADDR_LEN);
  }

  // Get the protocol
  if (Protocol != NULL) {
    *Protocol = NTOHS (*(UINT16*)(&DataPtr[12]));
  }

drop:
  /* refill: pass packet back to BM */
  PoolId = (Status & MVPP2_RXD_BM_POOL_ID_MASK) >>
  MVPP2_RXD_BM_POOL_ID_OFFS;
  mvpp2_bm_pool_put(Port, PoolId, PhysAddr, VirtAddr);

  mvpp2_rxq_status_update(Port, 0, 1, 1);
  return EFI_SUCCESS;
}

EFI_STATUS
Pp2DxeSnpInstall (
  IN PP2DXE_CONTEXT *Pp2Context
  )
{
  EFI_HANDLE Handle = NULL;
  EFI_STATUS Status;
  PP2_DEVICE_PATH *Pp2DevicePath;

  DEBUG((DEBUG_INFO, "Pp2Dxe%d: Installing protocols\n", Pp2Context->Instance));
  Pp2Context->Snp.Mode = AllocateZeroPool (sizeof (EFI_SIMPLE_NETWORK_MODE));
  Pp2DevicePath = AllocateCopyPool (sizeof (PP2_DEVICE_PATH), &Pp2DevicePathTemplate);
  Pp2DevicePath->Pp2Mac.MacAddress.Addr[0] = Pp2Context->Instance+2; /* TODO */
  Pp2Context->Signature = PP2DXE_SIGNATURE;
  Pp2Context->Snp.Initialize = Pp2DxeSnpInitialize;
  Pp2Context->Snp.Start = Pp2SnpStart;
  Pp2Context->Snp.Stop = Pp2SnpStop;
  Pp2Context->Snp.Reset = Pp2SnpReset;
  Pp2Context->Snp.Shutdown = Pp2SnpShutdown;
  Pp2Context->Snp.ReceiveFilters = Pp2SnpReceiveFilters;
  Pp2Context->Snp.Statistics = Pp2SnpNetStat;
  Pp2Context->Snp.MCastIpToMac = Pp2SnpIpToMac;
  Pp2Context->Snp.NvData = Pp2SnpNvData;
  Pp2Context->Snp.GetStatus = Pp2SnpGetStatus;
  Pp2Context->Snp.Transmit = Pp2SnpTransmit;
  Pp2Context->Snp.Receive = Pp2SnpReceive;
  Pp2Context->Snp.Revision = EFI_SIMPLE_NETWORK_PROTOCOL_REVISION;

  Pp2Context->Snp.Mode->CurrentAddress = Pp2DevicePath->Pp2Mac.MacAddress;
  Pp2Context->Snp.Mode->State = EfiSimpleNetworkStopped;
  Pp2Context->Snp.Mode->IfType = NET_IFTYPE_ETHERNET;
  Pp2Context->Snp.Mode->HwAddressSize = NET_ETHER_ADDR_LEN;
  Pp2Context->Snp.Mode->MediaHeaderSize = sizeof (ETHER_HEAD);
  Pp2Context->Snp.Mode->MaxPacketSize = EFI_PAGE_SIZE;
  Pp2Context->Snp.Mode->ReceiveFilterMask = EFI_SIMPLE_NETWORK_RECEIVE_UNICAST |
             EFI_SIMPLE_NETWORK_RECEIVE_MULTICAST |
             EFI_SIMPLE_NETWORK_RECEIVE_BROADCAST |
             EFI_SIMPLE_NETWORK_RECEIVE_PROMISCUOUS |
             EFI_SIMPLE_NETWORK_RECEIVE_PROMISCUOUS_MULTICAST;
  Pp2Context->Snp.Mode->ReceiveFilterSetting = EFI_SIMPLE_NETWORK_RECEIVE_UNICAST |
          EFI_SIMPLE_NETWORK_RECEIVE_MULTICAST |
          EFI_SIMPLE_NETWORK_RECEIVE_BROADCAST;
  Pp2Context->Snp.Mode->MaxMCastFilterCount = MAX_MCAST_FILTER_CNT;
  Pp2Context->Snp.Mode->MCastFilterCount = 0;
  Pp2Context->Snp.Mode->MediaPresentSupported = TRUE;
  Pp2Context->Snp.Mode->MediaPresent = FALSE;
  ZeroMem (&Pp2Context->Snp.Mode->MCastFilter, MAX_MCAST_FILTER_CNT * sizeof(EFI_MAC_ADDRESS));
  SetMem (&Pp2Context->Snp.Mode->BroadcastAddress, sizeof (EFI_MAC_ADDRESS), 0xFF);

  Pp2DevicePath->Pp2Mac.IfType = Pp2Context->Snp.Mode->IfType;
  Status = gBS->InstallMultipleProtocolInterfaces (
      &Handle,
      &gEfiSimpleNetworkProtocolGuid, &Pp2Context->Snp,
      &gEfiDevicePathProtocolGuid, Pp2DevicePath,
      NULL
      );

  if (EFI_ERROR(Status))
    DEBUG((DEBUG_ERROR, "Failed to install protocols.\n"));

  return Status;
}

EFI_STATUS
EFIAPI
Pp2DxeInitialise (
  IN EFI_HANDLE  ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  PP2DXE_CONTEXT *Pp2Context;
  EFI_STATUS Status;
  INTN i;
  UINT8 *PortIds, *GopIndexes;
  VOID *BufferSpace;

  Mvpp2Shared = AllocateZeroPool (sizeof (MVPP2_SHARED));
  if (Mvpp2Shared == NULL) {
    DEBUG((DEBUG_ERROR, "Allocation fail.\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  Mvpp2Shared->base = PcdGet64 (PcdPp2SharedAddress);
  Mvpp2Shared->rfu1_base = PcdGet64 (PcdPp2Rfu1BaseAddress);
  Mvpp2Shared->smi_base = PcdGet64 (PcdPp2SmiBaseAddress);
  Mvpp2Shared->tclk = PcdGet32 (PcdPp2ClockFrequency);
  DEBUG((DEBUG_ERROR, "Pp2Dxe: shared base is 0x%lx\n", Mvpp2Shared->base));
  DEBUG((DEBUG_ERROR, "Pp2Dxe: RFU1 base is 0x%lx\n", Mvpp2Shared->rfu1_base));
  DEBUG((DEBUG_ERROR, "Pp2Dxe: SMI base is 0x%lx\n", Mvpp2Shared->smi_base));

  BufferSpace = UncachedAllocateAlignedPool (BD_SPACE, MVPP2_BUFFER_ALIGN_SIZE);
  if (BufferSpace == NULL) {
    DEBUG((DEBUG_ERROR, "Failed to allocate buffer space\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  BufferLocation.tx_descs = (struct mvpp2_tx_desc *)BufferSpace;

  BufferLocation.aggr_tx_descs = (struct mvpp2_tx_desc *)
    ((UINT64)BufferSpace + MVPP2_MAX_TXD
    * sizeof(struct mvpp2_tx_desc));

  BufferLocation.rx_descs = (struct mvpp2_rx_desc *)
    ((UINT64)BufferSpace +
    (MVPP2_MAX_TXD + MVPP2_AGGR_TXQ_SIZE)
    * sizeof(struct mvpp2_tx_desc));

  BufferLocation.rx_buffers = (UINT64)
    (BufferSpace + (MVPP2_MAX_TXD + MVPP2_AGGR_TXQ_SIZE)
    * sizeof(struct mvpp2_tx_desc) +
    MVPP2_MAX_RXD * sizeof(struct mvpp2_rx_desc));

  Pp2DxeAxiConfig();
  Pp2DxeBmPoolInit();
  mvpp2_rx_fifo_init(Mvpp2Shared);

  Mvpp2Shared->prs_shadow = AllocateZeroPool (sizeof(struct mvpp2_prs_shadow)
                * MVPP2_PRS_TCAM_SRAM_SIZE);
  if (Mvpp2Shared->prs_shadow == NULL) {
    DEBUG((DEBUG_ERROR, "Failed to allocate prs_shadow\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  if (mvpp2_prs_default_init(Mvpp2Shared)) {
    DEBUG((DEBUG_ERROR, "Failed to intialize prs\n"));
    return EFI_DEVICE_ERROR;
  }

  mvpp2_cls_init(Mvpp2Shared);

  for (i = 0; i < PcdGet32 (PcdPp2PortNumber); i++) {
    UINT32 NetCompConfig;

    Pp2Context = AllocateZeroPool (sizeof (PP2DXE_CONTEXT));
    if (Pp2Context == NULL) {
      DEBUG((DEBUG_ERROR, "Allocation fail.\n"));
      return EFI_OUT_OF_RESOURCES;
    }

    /* Instances are enumerated from 0 */
    Pp2Context->Instance = i;

    /* Install SNP protocol */
    Status = Pp2DxeSnpInstall(Pp2Context);

    /* TODO: free resources */
    if (EFI_ERROR(Status))
      return Status;

    /* Inlined mv_pp2x_initialize_dev */
    PortIds = PcdGetPtr (PcdPp2PortIds);
    GopIndexes = PcdGetPtr (PcdPp2GopIndexes);
    ASSERT (PcdGetSize (PcdPp2GopIndexes) == PcdGetSize (PcdPp2PortIds));
    Pp2Context->Port.id = PortIds[Pp2Context->Instance];
    Pp2Context->Port.gop_index = GopIndexes[Pp2Context->Instance];
    Pp2Context->Port.priv = Mvpp2Shared;
    Pp2Context->Port.phy_interface = MV_MODE_RGMII;
    Pp2Context->Port.gmac_base = PcdGet64 (PcdPp2GmacBaseAddress) +
      PcdGet32 (PcdPp2GmacObjSize) * Pp2Context->Port.gop_index;
    Pp2Context->Port.xlg_base = PcdGet64 (PcdPp2XlgBaseAddress) +
      PcdGet32 (PcdPp2XlgObjSize) * Pp2Context->Port.gop_index;
    DEBUG((DEBUG_ERROR, "Pp2Dxe%d: port%d - gmac at 0x%lx, xlg at 0x%lx\n", Pp2Context->Instance, Pp2Context->Port.id,
      Pp2Context->Port.gmac_base, Pp2Context->Port.xlg_base));

    NetCompConfig = mvp_pp2x_gop110_netc_cfg_create(&Pp2Context->Port);
    mv_gop110_netc_init(&Pp2Context->Port, NetCompConfig,
      MV_NETC_FIRST_PHASE);
    mv_gop110_netc_init(&Pp2Context->Port, NetCompConfig,
      MV_NETC_SECOND_PHASE);

    mv_gop110_port_init(&Pp2Context->Port);
  }

  return EFI_SUCCESS;
}
