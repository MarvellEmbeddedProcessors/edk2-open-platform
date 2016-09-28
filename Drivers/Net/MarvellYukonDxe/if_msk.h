/**  <at> file
*  API to ported msk driver
*
*  Copyright (c) 2011-2016, ARM Limited. All rights reserved.
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#include <Uefi.h>

#define MAX_SUPPORTED_PACKET_SIZE   (1566) /* No jumbo frame size support */

EFI_STATUS mskc_probe (EFI_PCI_IO_PROTOCOL *PciIo);

EFI_STATUS mskc_attach (EFI_PCI_IO_PROTOCOL  *PciIo, EFI_MAC_ADDRESS *Mac);
void mskc_detach (void);

EFI_STATUS mskc_init (void);
void mskc_shutdown (void);

void
mskc_rxfilter (
    IN UINT32                      FilterFlags,
    IN UINTN                       MCastFilterCnt,
    IN EFI_MAC_ADDRESS             *MCastFilter
    );

EFI_STATUS
mskc_transmit (
    IN UINTN                       BufferSize,
    IN VOID                        *Buffer
    );

EFI_STATUS
mskc_receive (
    IN OUT UINTN                   *BufferSize,
    OUT VOID                       *Buffer
    );

void
mskc_getstatus (
    OUT UINT32                     *InterruptStatus, OPTIONAL
    OUT VOID                       **TxBuf           OPTIONAL
    );
