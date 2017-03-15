/********************************************************************************
Copyright (C) 2017 Marvell International Ltd.

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

#ifndef __MVHWDESCLIB_H__
#define __MVHWDESCLIB_H__

#include <Library/NonDiscoverableDeviceRegistrationLib.h>

//
// Platform storage description
//
typedef struct {
  // XHCI
  UINT8 XhciDevCount;
  UINTN XhciBaseAddresses[4];
  UINTN XhciMemSize[4];
  NON_DISCOVERABLE_DEVICE_DMA_TYPE XhciDmaType[4];
  // AHCI
  UINT8 AhciDevCount;
  UINTN AhciBaseAddresses[4];
  UINTN AhciMemSize[4];
  NON_DISCOVERABLE_DEVICE_DMA_TYPE AhciDmaType[4];
  // SDHCI
  UINT8 SdhciDevCount;
  UINTN SdhciBaseAddresses[4];
  UINTN SdhciMemSize[4];
  NON_DISCOVERABLE_DEVICE_DMA_TYPE SdhciDmaType[4];
} MVHW_STORAGE_DESC;

#define DECLARE_A7K8K_STORAGE_TEMPLATE   \
STATIC MVHW_STORAGE_DESC mA7k8kStorageDescTemplate = {\
  4, /* XHCI */\
  { 0xF2500000, 0xF2510000, 0xF4500000, 0xF4510000 },\
  { SIZE_16KB, SIZE_16KB, SIZE_16KB, SIZE_16KB },\
  { NonDiscoverableDeviceDmaTypeCoherent, NonDiscoverableDeviceDmaTypeCoherent,\
    NonDiscoverableDeviceDmaTypeCoherent, NonDiscoverableDeviceDmaTypeCoherent },\
  2, /* AHCI */\
  { 0xF2540000, 0xF4540000 },\
  { SIZE_8KB, SIZE_8KB },\
  { NonDiscoverableDeviceDmaTypeCoherent, NonDiscoverableDeviceDmaTypeCoherent },\
  2, /* SDHCI */\
  { 0xF06E0000, 0xF2780000 },\
  { SIZE_1KB, SIZE_1KB },\
  { NonDiscoverableDeviceDmaTypeCoherent, NonDiscoverableDeviceDmaTypeCoherent }\
}

#endif /* __MVHWDESCLIB_H__ */
