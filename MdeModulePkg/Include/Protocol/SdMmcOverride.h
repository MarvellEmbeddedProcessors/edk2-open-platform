/** @file
  Protocol to describe overrides required to support non-standard SDHCI
  implementations

  Copyright (c) 2017 - 2018, Linaro, Ltd. All rights reserved.<BR>

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __SD_MMC_OVERRIDE_H__
#define __SD_MMC_OVERRIDE_H__

#include <Bus/Pci/SdMmcPciHcDxe/SdMmcPciHci.h>
#include <Protocol/SdMmcPassThru.h>

#define EDKII_SD_MMC_OVERRIDE_PROTOCOL_GUID \
  { 0xeaf9e3c1, 0xc9cd, 0x46db, { 0xa5, 0xe5, 0x5a, 0x12, 0x4c, 0x83, 0x23, 0x23 } }

#define EDKII_SD_MMC_OVERRIDE_PROTOCOL_VERSION    0x1
#define EDKII_SD_MMC_OVERRIDE_PROTOCOL_VERSION2   0x2

typedef struct _EDKII_SD_MMC_OVERRIDE EDKII_SD_MMC_OVERRIDE;

typedef enum {
  EdkiiSdMmcResetPre,
  EdkiiSdMmcResetPost,
  EdkiiSdMmcInitHostPre,
  EdkiiSdMmcInitHostPost,
} EDKII_SD_MMC_PHASE_TYPE;

/**

  Override function for SDHCI capability bits

  @param[in]      ControllerHandle      The EFI_HANDLE of the controller.
  @param[in]      Slot                  The 0 based slot index.
  @param[in,out]  SdMmcHcSlotCapability The SDHCI capability structure.

  @retval EFI_SUCCESS           The override function completed successfully.
  @retval EFI_NOT_FOUND         The specified controller or slot does not exist.
  @retval EFI_INVALID_PARAMETER SdMmcHcSlotCapability is NULL

**/
typedef
EFI_STATUS
(EFIAPI * EDKII_SD_MMC_CAPABILITY) (
  IN      EFI_HANDLE                      ControllerHandle,
  IN      UINT8                           Slot,
  IN  OUT VOID                            *SdMmcHcSlotCapability
  );

/**

  Override function for SDHCI controller operations

  @param[in]      ControllerHandle      The EFI_HANDLE of the controller.
  @param[in]      Slot                  The 0 based slot index.
  @param[in]      PhaseType             The type of operation and whether the
                                        hook is invoked right before (pre) or
                                        right after (post)

  @retval EFI_SUCCESS           The override function completed successfully.
  @retval EFI_NOT_FOUND         The specified controller or slot does not exist.
  @retval EFI_INVALID_PARAMETER PhaseType is invalid

**/
typedef
EFI_STATUS
(EFIAPI * EDKII_SD_MMC_NOTIFY_PHASE) (
  IN      EFI_HANDLE                      ControllerHandle,
  IN      UINT8                           Slot,
  IN      EDKII_SD_MMC_PHASE_TYPE         PhaseType
  );

/**

  Override function for uhs signaling

  @param[in]      ControllerHandle      The EFI_HANDLE of the controller.
  @param[in]      Slot                  The 0 based slot index.
  @param[in]      Timing                The timing which should be set by
                                        host controller.

  @retval EFI_SUCCESS           The override function completed successfully.
  @retval EFI_NOT_FOUND         The specified controller or slot does not exist.

**/
typedef
EFI_STATUS
(EFIAPI * EDKII_SD_MMC_UHS_SIGNALING) (
  IN      EFI_HANDLE                      ControllerHandle,
  IN      UINT8                           Slot,
  IN      SD_MMC_UHS_TIMING               Timing
  );

/**

  Additional operations specific for host controller

  @param[in]      ControllerHandle      The EFI_HANDLE of the controller.
  @param[in]      Slot                  The 0 based slot index.
  @param[in]      Timing                The timing which should be set by
                                        host controller.

  @retval EFI_SUCCESS           The override function completed successfully.
  @retval EFI_NOT_FOUND         The specified controller or slot does not exist.

**/
typedef
EFI_STATUS
(EFIAPI * EDKII_SD_MMC_POST_CLOCK_FREQ_SWITCH) (
  IN      EFI_HANDLE                      ControllerHandle,
  IN      UINT8                           Slot,
  IN      SD_MMC_UHS_TIMING               Timing
  );

struct _EDKII_SD_MMC_OVERRIDE {
  //
  // Protocol version of this implementation
  //
  UINTN                                Version;
  //
  // Callback to override SD/MMC host controller capability bits
  //
  EDKII_SD_MMC_CAPABILITY              Capability;
  //
  // Callback to invoke SD/MMC override hooks
  //
  EDKII_SD_MMC_NOTIFY_PHASE            NotifyPhase;
  //
  // Callback to override SD/MMC host controller uhs signaling
  //
  EDKII_SD_MMC_UHS_SIGNALING           UhsSignaling;
  //
  // Callback to add host controller specific operations after SwitchClockFreq
  //
  EDKII_SD_MMC_POST_CLOCK_FREQ_SWITCH  SwitchClockFreqPost;
};

extern EFI_GUID gEdkiiSdMmcOverrideProtocolGuid;

#endif
