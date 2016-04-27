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
#include <Library/CacheMaintenanceLib.h>

#include "Pp2Dxe.h"
#include "mvpp2_lib.h"

#define ReturnUnlock(tpl, status) do { gBS->RestoreTPL (tpl); return (status); } while(0)
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
    { PP2DXE_DEFAULT_MAC_ADDR },
    0
  },
  {
    END_DEVICE_PATH_TYPE,
    END_ENTIRE_DEVICE_PATH_SUBTYPE,
    { sizeof(EFI_DEVICE_PATH_PROTOCOL), 0 }
  }
};

#define QueueNext(off)  ((((off) + 1) >= QUEUE_DEPTH) ? 0 : ((off) + 1))

STATIC
EFI_STATUS
QueueInsert (
  IN PP2DXE_CONTEXT *Pp2Context,
  IN VOID *Buffer
  )
{

  if (QueueNext (Pp2Context->CompletionQueueTail) ==
      Pp2Context->CompletionQueueHead) {
    return EFI_OUT_OF_RESOURCES;
  }

  Pp2Context->CompletionQueue[Pp2Context->CompletionQueueTail] = Buffer;
  Pp2Context->CompletionQueueTail = QueueNext (Pp2Context->CompletionQueueTail);

  return EFI_SUCCESS;
}

STATIC
VOID *
QueueRemove (
  IN PP2DXE_CONTEXT *Pp2Context
  )
{
  VOID *Buffer;

  if (Pp2Context->CompletionQueueTail == Pp2Context->CompletionQueueHead) {
    return NULL;
  }

  Buffer = Pp2Context->CompletionQueue[Pp2Context->CompletionQueueHead];
  Pp2Context->CompletionQueue[Pp2Context->CompletionQueueHead] = NULL;
  Pp2Context->CompletionQueueHead = QueueNext (Pp2Context->CompletionQueueHead);

  return Buffer;
}

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

/*
 * mvpp2_bm_start
 *   enable and fill BM pool
 */
STATIC
EFI_STATUS
Pp2DxeBmStart (
  VOID
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
    mvpp2_bm_pool_put(Mvpp2Shared, MVPP2_BM_POOL,
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
  mvpp2_cleanup_rxqs(&Pp2Context->Port);
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
  EFI_STATUS Status;

  for (Queue = 0; Queue < txq_number; Queue++) {
    txq = &Pp2Context->Port.txqs[Queue];
    txq->descs_phys = (dma_addr_t) txq->descs;
    if (!txq->descs_phys) {
      Status = EFI_OUT_OF_RESOURCES;
      goto err_cleanup;
    }
    mvpp2_txq_hw_init(&Pp2Context->Port, txq);
  }

  return EFI_SUCCESS;

err_cleanup:
  mvpp2_cleanup_txqs(&Pp2Context->Port);
  return Status;
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

STATIC
EFI_STATUS
Pp2DxeOpen (
  IN PP2DXE_CONTEXT *Pp2Context
  )
{
  PP2DXE_PORT *Port = &Pp2Context->Port;
  UINT8 mac_bcast[NET_ETHER_ADDR_LEN] = { 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff };
  UINT8 dev_addr[NET_ETHER_ADDR_LEN];
  INTN ret;
  EFI_STATUS Status;

  DEBUG((DEBUG_INFO, "Pp2Dxe: Open\n"));

  CopyMem (dev_addr, Pp2Context->Snp.Mode->CurrentAddress.Addr, NET_ETHER_ADDR_LEN);

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

  Port->rxqs = AllocateZeroPool (sizeof(struct mvpp2_rx_queue) * rxq_number);
  if (Port->rxqs == NULL) {
    DEBUG((DEBUG_ERROR, "Failed to allocate rxqs\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  Port->rxqs[0].descs = BufferLocation.rx_descs;

  for (Queue = 0; Queue < txq_number; Queue++) {
    struct mvpp2_rx_queue *rxq = &Port->rxqs[Queue];

    rxq->id = Queue + Port->first_rxq;
    rxq->size = Port->rx_ring_size;
  }

  mvpp2_ingress_disable(Port);

  mvpp2_defaults_set(Port);

  return Pp2DxeOpen(Pp2Context);
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

  if (!Pp2Context->LateInitialized) {
    /* Full init on first call */
    Status = Pp2DxeLatePortInitialize(Pp2Context);
    if (EFI_ERROR(Status)) {
      DEBUG((DEBUG_ERROR, "Pp2Dxe: late initialization failed\n"));
      return Status;
    }
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
    mv_gop110_port_events_mask(Port);
    mv_gop110_port_enable(Port);
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
  Status = gBS->LocateProtocol (
      &gEfiPhyProtocolGuid,
      NULL,
      (VOID **) &Pp2Context->Phy
      );
  if (EFI_ERROR(Status))
    return Status;

  if (PhyAddresses[Pp2Context->Instance] == 0xff)
    /* PHY iniitalization not required */
    return EFI_SUCCESS;

  Status = Pp2Context->Phy->Init(
            Pp2Context->Phy,
            PhyAddresses[Pp2Context->Instance],
	    Pp2Context->Port.phy_interface,
            &Pp2Context->PhyDev
            );
  if (EFI_ERROR(Status) && Status != EFI_TIMEOUT)
    return Status;
  Pp2Context->Phy->Status(Pp2Context->Phy, Pp2Context->PhyDev);
  DEBUG((DEBUG_INFO,
    "PHY%d: ",
    Pp2Context->PhyDev->Addr));
  DEBUG((DEBUG_INFO,
    Pp2Context->PhyDev->LinkUp ? "link up, " : "link down, "));
  DEBUG((DEBUG_INFO,
    Pp2Context->PhyDev->FullDuplex ? "full duplex, " : "half duplex, "));
  DEBUG((DEBUG_INFO,
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
  EFI_TPL SavedTpl;

  DEBUG((DEBUG_INFO, "Pp2Dxe%d: initialize\n", Pp2Context->Instance));
  if (ExtraRxBufferSize != 0 || ExtraTxBufferSize != 0) {
    DEBUG((DEBUG_ERROR, "Pp2Dxe%d: non-zero buffer requests\n", Pp2Context->Instance));
    return EFI_UNSUPPORTED;
  }

  SavedTpl = gBS->RaiseTPL (TPL_CALLBACK);

  switch (This->Mode->State) {
  case EfiSimpleNetworkStarted:
  DEBUG((DEBUG_INFO, "Pp2Dxe%d: started state\n", Pp2Context->Instance));
    break;
  case EfiSimpleNetworkInitialized:
    DEBUG((DEBUG_INFO, "Pp2Dxe%d: already initialized\n", Pp2Context->Instance));
    ReturnUnlock (SavedTpl, EFI_SUCCESS);
  case EfiSimpleNetworkStopped:
    DEBUG((DEBUG_INFO, "Pp2Dxe%d: network stopped\n", Pp2Context->Instance));
    ReturnUnlock (SavedTpl, EFI_NOT_STARTED);
  default:
    DEBUG((DEBUG_INFO, "Pp2Dxe%d: wrong state\n", Pp2Context->Instance));
    ReturnUnlock (SavedTpl, EFI_DEVICE_ERROR);
  }

  This->Mode->State = EfiSimpleNetworkInitialized;

  if (Pp2Context->Initialized)
    ReturnUnlock(SavedTpl, EFI_SUCCESS);

  Pp2Context->Initialized = TRUE;

  Status = Pp2DxePhyInitialize(Pp2Context);
  if (EFI_ERROR(Status))
    ReturnUnlock (SavedTpl, Status);

  ReturnUnlock (SavedTpl, Pp2DxeLateInitialize(Pp2Context));
}

EFI_STATUS
EFIAPI
Pp2SnpStart (
  IN EFI_SIMPLE_NETWORK_PROTOCOL  *This
  )
{
  PP2DXE_CONTEXT *Pp2Context;
  EFI_TPL SavedTpl;

  SavedTpl = gBS->RaiseTPL (TPL_CALLBACK);
  Pp2Context = INSTANCE_FROM_SNP(This);

  DEBUG((DEBUG_INFO, "Pp2Dxe%d: started\n", Pp2Context->Instance));
  switch (This->Mode->State) {
  case EfiSimpleNetworkStopped:
  DEBUG((DEBUG_INFO, "Pp2Dxe%d: stopped state\n", Pp2Context->Instance));
    break;
  case EfiSimpleNetworkStarted:
  case EfiSimpleNetworkInitialized:
    DEBUG((DEBUG_INFO, "Pp2Dxe: Driver already started\n"));
    ReturnUnlock (SavedTpl, EFI_ALREADY_STARTED);
  default:
    DEBUG((DEBUG_ERROR, "Pp2Dxe: Driver in an invalid state: %u\n",
          (UINTN)This->Mode->State));
    ReturnUnlock (SavedTpl, EFI_DEVICE_ERROR);
  }
  This->Mode->State = EfiSimpleNetworkStarted;
  ReturnUnlock (SavedTpl, EFI_SUCCESS);
}

EFI_STATUS
EFIAPI
Pp2SnpStop (
  IN EFI_SIMPLE_NETWORK_PROTOCOL  *This
  )
{
  EFI_TPL SavedTpl;
  SavedTpl = gBS->RaiseTPL (TPL_CALLBACK);

  DEBUG((DEBUG_INFO, "Pp2SnpStop \n"));
  switch (This->Mode->State) {
  case EfiSimpleNetworkStarted:
  case EfiSimpleNetworkInitialized:
    break;
  case EfiSimpleNetworkStopped:
    ReturnUnlock (SavedTpl, EFI_NOT_STARTED);
  default:
    ReturnUnlock (SavedTpl, EFI_DEVICE_ERROR);
  }
  This->Mode->State = EfiSimpleNetworkStopped;
  ReturnUnlock (SavedTpl, EFI_SUCCESS);
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
  EFI_TPL SavedTpl;
  SavedTpl = gBS->RaiseTPL (TPL_CALLBACK);

  switch (This->Mode->State) {
  case EfiSimpleNetworkInitialized:
    break;
  case EfiSimpleNetworkStarted:
    ReturnUnlock (SavedTpl, EFI_DEVICE_ERROR);
  case EfiSimpleNetworkStopped:
    ReturnUnlock (SavedTpl, EFI_NOT_STARTED);
  default:
    ReturnUnlock (SavedTpl, EFI_DEVICE_ERROR);
  }

  ReturnUnlock (SavedTpl, EFI_SUCCESS);
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
  PP2DXE_PORT *Port = &Pp2Context->Port;
  BOOLEAN LinkUp;
  EFI_TPL SavedTpl;

  SavedTpl = gBS->RaiseTPL (TPL_CALLBACK);

  if (!Pp2Context->Initialized)
    ReturnUnlock(SavedTpl, EFI_NOT_READY);

  LinkUp = Port->always_up ? TRUE : mv_gop110_port_is_link_up(Port);

  if (LinkUp != Snp->Mode->MediaPresent) {
    DEBUG((DEBUG_INFO, "Pp2Dxe%d: Link ", Pp2Context->Instance));
    DEBUG((DEBUG_INFO, LinkUp ? "up\n" : "down\n"));
  }
  Snp->Mode->MediaPresent = LinkUp;

  if (TxBuf != NULL) {
    *TxBuf = QueueRemove (Pp2Context);
  }

  ReturnUnlock(SavedTpl, EFI_SUCCESS);
}

EFI_STATUS
EFIAPI
Pp2SnpTransmit (
  IN EFI_SIMPLE_NETWORK_PROTOCOL          *This,
  IN UINTN                                HeaderSize,
  IN UINTN                                BufferSize,
  IN VOID                                 *Buffer,
  IN EFI_MAC_ADDRESS                      *SrcAddr  OPTIONAL,
  IN EFI_MAC_ADDRESS                      *DestAddr OPTIONAL,
  IN UINT16                               *ProtocolPtr OPTIONAL
  )
{
  PP2DXE_CONTEXT *Pp2Context = INSTANCE_FROM_SNP(This);
  PP2DXE_PORT *Port = &Pp2Context->Port;
  struct mvpp2_tx_queue *aggr_txq = Mvpp2Shared->aggr_txqs;
  struct mvpp2_tx_desc *tx_desc;
  INTN timeout = 0;
  INTN tx_done;
  UINT8 *DataPtr = Buffer;
  UINT16 Protocol;
  EFI_TPL SavedTpl;

  if (This == NULL || Buffer == NULL) {
    DEBUG((DEBUG_ERROR, "Pp2Dxe: null Snp or Buffer\n"));
    return EFI_INVALID_PARAMETER;
  }

  if (HeaderSize != 0) {
    ASSERT (HeaderSize == This->Mode->MediaHeaderSize);
    ASSERT (ProtocolPtr != NULL);
    ASSERT (DestAddr != NULL);
  }

  SavedTpl = gBS->RaiseTPL (TPL_CALLBACK);

  switch (This->Mode->State) {
  case EfiSimpleNetworkInitialized:
    break;
  case EfiSimpleNetworkStarted:
    DEBUG((DEBUG_WARN, "Pp2Dxe: Driver not yet initialized\n"));
    ReturnUnlock(SavedTpl, EFI_DEVICE_ERROR);
  case EfiSimpleNetworkStopped:
    DEBUG((DEBUG_WARN, "Pp2Dxe: Driver not started\n"));
    ReturnUnlock(SavedTpl, EFI_NOT_STARTED);
  default:
    DEBUG((DEBUG_ERROR, "Pp2Dxe: Driver in an invalid state\n"));
    ReturnUnlock(SavedTpl, EFI_DEVICE_ERROR);
  }

  if (!This->Mode->MediaPresent) {
    DEBUG((DEBUG_ERROR, "Pp2Dxe: link not ready\n"));
    ReturnUnlock(SavedTpl, EFI_NOT_READY);
  }

  Protocol = HTONS(*ProtocolPtr);

  tx_desc = mvpp2_txq_next_desc_get(aggr_txq);

  if (!tx_desc) {
    DEBUG((DEBUG_ERROR, "No tx descriptor to use\n"));
    ReturnUnlock(SavedTpl, EFI_OUT_OF_RESOURCES);
  }

  if (HeaderSize != 0) {
    CopyMem(DataPtr, DestAddr, NET_ETHER_ADDR_LEN);

    if (SrcAddr != NULL)
      CopyMem(DataPtr + NET_ETHER_ADDR_LEN, SrcAddr, NET_ETHER_ADDR_LEN);
    else
      CopyMem(DataPtr + NET_ETHER_ADDR_LEN, &This->Mode->CurrentAddress, NET_ETHER_ADDR_LEN);

    CopyMem(DataPtr + NET_ETHER_ADDR_LEN * 2, &Protocol, 2);
  }

  /* set descriptor fields */
  tx_desc->command =  MVPP2_TXD_IP_CSUM_DISABLE |
  MVPP2_TXD_L4_CSUM_NOT | MVPP2_TXD_F_DESC | MVPP2_TXD_L_DESC;
  tx_desc->data_size = BufferSize;

  tx_desc->packet_offset = (phys_addr_t)DataPtr & MVPP2_TX_DESC_ALIGN;

  mvpp2x2_txdesc_phys_addr_set(
    (phys_addr_t)DataPtr & ~MVPP2_TX_DESC_ALIGN, tx_desc);
  tx_desc->phys_txq = mvpp2_txq_phys(Port->id, 0);

  InvalidateDataCacheRange (DataPtr, BufferSize);

  /* iowmb */
  __asm__ __volatile__ ("" : : : "memory");
  /* send */
  mvpp2_aggr_txq_pend_desc_add(Port, 1);

  /* Tx done processing */
  /* wait for agrregated to physical TXQ transfer */
  tx_done = mvpp2_aggr_txq_pend_desc_num_get(Mvpp2Shared, 0);
  do {
    if (timeout++ > MVPP2_TX_SEND_TIMEOUT) {
      DEBUG((DEBUG_ERROR, "Pp2Dxe: transmit timeout\n"));
      ReturnUnlock(SavedTpl, EFI_TIMEOUT);
    }
    tx_done = mvpp2_aggr_txq_pend_desc_num_get(Mvpp2Shared, 0);
  } while (tx_done);

  timeout = 0;
  tx_done = mvpp2_txq_sent_desc_proc(Port, &Port->txqs[0]);
  /* wait for packet to be transmitted */
  while (!tx_done) {
    if (timeout++ > MVPP2_TX_SEND_TIMEOUT) {
      DEBUG((DEBUG_ERROR, "Pp2Dxe: transmit timeout\n"));
      ReturnUnlock(SavedTpl, EFI_TIMEOUT);
    }
    tx_done = mvpp2_txq_sent_desc_proc(Port, &Port->txqs[0]);
  }
  /* tx_done has increased - hw sent packet */

  /* add buffer to completion queue and return */
  ReturnUnlock (SavedTpl, QueueInsert (Pp2Context, Buffer));
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
  EFI_STATUS Status = EFI_SUCCESS;
  EFI_TPL SavedTpl;
  UINT32 StatusReg;
  INTN PoolId;
  UINTN PktLength;
  UINT8 *DataPtr;
  struct mvpp2_rx_desc *RxDesc;
  struct mvpp2_rx_queue *Rxq = &Port->rxqs[0];

  SavedTpl = gBS->RaiseTPL (TPL_CALLBACK);
  ReceivedPackets = mvpp2_rxq_received(Port, Rxq->id);

  if (ReceivedPackets == 0) {
    ReturnUnlock(SavedTpl, EFI_NOT_READY);
  }

  /* process one packet per call */
  RxDesc = mvpp2_rxq_next_desc_get(Rxq);

  StatusReg = RxDesc->status;

  /* extract addresses from descriptor */
  PhysAddr = RxDesc->buf_phys_addr_key_hash &
  MVPP22_ADDR_MASK;
  VirtAddr = RxDesc->buf_cookie_bm_qset_cls_info &
  MVPP22_ADDR_MASK;

  /* drop packets with error or with buffer header (MC, SG) */
  if ((StatusReg & MVPP2_RXD_BUF_HDR) ||
    (StatusReg & MVPP2_RXD_ERR_SUMMARY)) {
    DEBUG((DEBUG_WARN, "Pp2Dxe: dropping packet\n"));
    Status = EFI_DEVICE_ERROR;
    goto drop;
  }

  PktLength = (UINTN) RxDesc->data_size - 2;
  if (PktLength > *BufferSize) {
    *BufferSize = PktLength;
    DEBUG((DEBUG_ERROR, "Pp2Dxe: buffer too small\n"));
    ReturnUnlock(SavedTpl, EFI_BUFFER_TOO_SMALL);
  }

  CopyMem (Buffer, (VOID*) (PhysAddr + 2), PktLength);
  *BufferSize = PktLength;

  if (HeaderSize != NULL) {
    *HeaderSize = Pp2Context->Snp.Mode->MediaHeaderSize;
  }

    DataPtr = (UINT8 *)Buffer;

  /* extract the destination address */
  if (DstAddr != NULL) {
    ZeroMem (DstAddr, sizeof(EFI_MAC_ADDRESS));
    CopyMem (DstAddr, &DataPtr[0], NET_ETHER_ADDR_LEN);
  }

  /* get the source address */
  if (SrcAddr != NULL) {
    ZeroMem (SrcAddr, sizeof(EFI_MAC_ADDRESS));
    CopyMem (SrcAddr, &DataPtr[6], NET_ETHER_ADDR_LEN);
  }

  /* get the protocol */
  if (Protocol != NULL) {
    *Protocol = NTOHS (*(UINT16*)(&DataPtr[12]));
  }

drop:
  /* refill: pass packet back to BM */
  PoolId = (StatusReg & MVPP2_RXD_BM_POOL_ID_MASK) >>
  MVPP2_RXD_BM_POOL_ID_OFFS;
  mvpp2_bm_pool_put(Mvpp2Shared, PoolId, PhysAddr, VirtAddr);

  /* iowmb */
  __asm__ __volatile__ ("" : : : "memory");

  ASSERT (Port != NULL);
  ASSERT (Rxq != NULL);
  mvpp2_rxq_status_update(Port, Rxq->id, 1, 1);
  ReturnUnlock(SavedTpl, Status);
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
  /* change MAC address depending on interface */
  Pp2DevicePath->Pp2Mac.MacAddress.Addr[5] += Pp2Context->Instance;
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
  Pp2Context->Snp.Mode->PermanentAddress = Pp2DevicePath->Pp2Mac.MacAddress;
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

STATIC
VOID
Pp2DxeParsePortPcd (
  IN PP2DXE_CONTEXT *Pp2Context
  )
{
  UINT8 *PortIds, *GopIndexes, *PhyConnectionTypes, *AlwaysUp, *Speed;

  PortIds = PcdGetPtr (PcdPp2PortIds);
  GopIndexes = PcdGetPtr (PcdPp2GopIndexes);
  PhyConnectionTypes = PcdGetPtr (PcdPhyConnectionTypes);
  AlwaysUp = PcdGetPtr (PcdPp2InterfaceAlwaysUp);
  Speed = PcdGetPtr (PcdPp2InterfaceSpeed);

  ASSERT (PcdGetSize (PcdPp2GopIndexes) == PcdGetSize (PcdPp2PortIds));
  ASSERT (PcdGetSize (PcdPhyConnectionTypes) == PcdGetSize (PcdPp2PortIds));
  ASSERT (PcdGetSize (PcdPp2InterfaceAlwaysUp) == PcdGetSize (PcdPp2PortIds));
  ASSERT (PcdGetSize (PcdPp2InterfaceSpeed) == PcdGetSize (PcdPp2PortIds));

  Pp2Context->Port.id = PortIds[Pp2Context->Instance];
  Pp2Context->Port.gop_index = GopIndexes[Pp2Context->Instance];
  Pp2Context->Port.phy_interface = PhyConnectionTypes[Pp2Context->Instance];
  Pp2Context->Port.always_up = AlwaysUp[Pp2Context->Instance];
  Pp2Context->Port.speed = Speed[Pp2Context->Instance];
  Pp2Context->Port.gmac_base = PcdGet64 (PcdPp2GmacBaseAddress) +
    PcdGet32 (PcdPp2GmacObjSize) * Pp2Context->Port.gop_index;
  Pp2Context->Port.xlg_base = PcdGet64 (PcdPp2XlgBaseAddress) +
    PcdGet32 (PcdPp2XlgObjSize) * Pp2Context->Port.gop_index;
}

EFI_STATUS
EFIAPI
Pp2DxeInitialise (
  IN EFI_HANDLE  ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  PP2DXE_CONTEXT *Pp2Context = NULL;
  EFI_STATUS Status;
  INTN i;
  VOID *BufferSpace;
  UINT32 NetCompConfig = 0;

  Mvpp2Shared = AllocateZeroPool (sizeof (MVPP2_SHARED));
  if (Mvpp2Shared == NULL) {
    DEBUG((DEBUG_ERROR, "Allocation fail.\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  Mvpp2Shared->base = PcdGet64 (PcdPp2SharedAddress);
  Mvpp2Shared->rfu1_base = PcdGet64 (PcdPp2Rfu1BaseAddress);
  Mvpp2Shared->smi_base = PcdGet64 (PcdPp2SmiBaseAddress);
  Mvpp2Shared->tclk = PcdGet32 (PcdPp2ClockFrequency);
  DEBUG((DEBUG_INFO, "Pp2Dxe: shared base is 0x%lx\n", Mvpp2Shared->base));
  DEBUG((DEBUG_INFO, "Pp2Dxe: RFU1 base is 0x%lx\n", Mvpp2Shared->rfu1_base));
  DEBUG((DEBUG_INFO, "Pp2Dxe: SMI base is 0x%lx\n", Mvpp2Shared->smi_base));

  BufferSpace = UncachedAllocateAlignedPool (BD_SPACE, MVPP2_BUFFER_ALIGN_SIZE);
  if (BufferSpace == NULL) {
    DEBUG((DEBUG_ERROR, "Failed to allocate buffer space\n"));
    return EFI_OUT_OF_RESOURCES;
  }
  SetMem (BufferSpace, BD_SPACE, 0x0);

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

  mvpp2_axi_config(Mvpp2Shared);
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

  Status = Pp2DxeBmStart();
  if (EFI_ERROR(Status)) {
    DEBUG((DEBUG_ERROR, "Pp2Dxe: bm start error\n"));
    return Status;
  }

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

  Mvpp2Shared->aggr_txqs = AllocateZeroPool (sizeof(struct mvpp2_tx_queue));
  if (Mvpp2Shared->aggr_txqs == NULL) {
    DEBUG((DEBUG_ERROR, "Failed to allocate aggregated txqs\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  Mvpp2Shared->aggr_txqs->descs = BufferLocation.aggr_tx_descs;
  Mvpp2Shared->aggr_txqs->id = 0;
  Mvpp2Shared->aggr_txqs->log_id = 0;
  Mvpp2Shared->aggr_txqs->size = MVPP2_AGGR_TXQ_SIZE;

  if (PcdGet32 (PcdPp2PortNumber) == 0) {
    DEBUG((DEBUG_ERROR, "Pp2Dxe: port number set to 0\n"));
    return EFI_INVALID_PARAMETER;
  }

  for (i = 0; i < PcdGet32 (PcdPp2PortNumber); i++) {

    Pp2Context = AllocateZeroPool (sizeof (PP2DXE_CONTEXT));
    if (Pp2Context == NULL) {
      /*
       * If allocation fails, all resources allocated before will get freed
       * at ExitBootServices, as only EfiBootServicesData is used.
       */
      DEBUG((DEBUG_ERROR, "Allocation fail.\n"));
      return EFI_OUT_OF_RESOURCES;
    }

    /* Instances are enumerated from 0 */
    Pp2Context->Instance = i;

    /* Install SNP protocol */
    Status = Pp2DxeSnpInstall(Pp2Context);

    if (EFI_ERROR(Status))
      return Status;

    Pp2DxeParsePortPcd(Pp2Context);
    Pp2Context->Port.txp_num = 1;
    Pp2Context->Port.priv = Mvpp2Shared;
    Pp2Context->Port.first_rxq = 4 * Pp2Context->Instance;
    DEBUG((DEBUG_INFO, "Pp2Dxe%d: port%d - gmac at 0x%lx, xlg at 0x%lx\n", Pp2Context->Instance, Pp2Context->Port.id,
      Pp2Context->Port.gmac_base, Pp2Context->Port.xlg_base));

    NetCompConfig |= mvp_pp2x_gop110_netc_cfg_create(&Pp2Context->Port);

    mv_gop110_port_init(&Pp2Context->Port);
    mv_gop110_fl_cfg(&Pp2Context->Port);
  }

  mv_gop110_netc_init(&Pp2Context->Port, NetCompConfig,
    MV_NETC_FIRST_PHASE);
  mv_gop110_netc_init(&Pp2Context->Port, NetCompConfig,
    MV_NETC_SECOND_PHASE);

  return EFI_SUCCESS;
}
