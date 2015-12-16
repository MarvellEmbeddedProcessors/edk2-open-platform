/** @file
*
*  Copyright (c) 2015, Hisilicon Limited. All rights reserved.
*  Copyright (c) 2015, Linaro Limited. All rights reserved.
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

#ifndef _OEM_NIC_PROTOCOL_H_
#define _OEM_NIC_PROTOCOL_H_

#define OEM_NIC_PROTOCOL_GUID   \
    { 0xb5903955, 0x31e9, 0x4aaf, { 0xb2, 0x83, 0x7, 0x9f, 0x3c, 0xc4, 0x71, 0x66 } }

#define OEM_XGE_STATUS_PROTOCOL_GUID   \
        { 0xa6b8ed0e, 0xd8cc, 0x4853, { 0xaa, 0x39, 0x2c, 0x3e, 0xcd, 0x7c, 0xa5, 0x97 } }

typedef
EFI_STATUS
(EFIAPI *OEM_NIC_GET_MAC_ADDRESS) (
  IN OUT EFI_MAC_ADDRESS *Mac,
  IN UINTN Port
  );

typedef
EFI_STATUS
(EFIAPI *OEM_NIC_SET_MAC_ADDRESS) (
  IN EFI_MAC_ADDRESS *Mac,
  IN UINTN Port
  );

typedef struct {
  OEM_NIC_GET_MAC_ADDRESS GetMac;
  OEM_NIC_SET_MAC_ADDRESS SetMac;
} OEM_NIC_PROTOCOL;

typedef
VOID
(*OEM_FEEDBACK_XGE_STATUS) (
  BOOLEAN IsLinkup,
  BOOLEAN IsActOK,
  UINT32 port
  );

typedef struct {
  OEM_FEEDBACK_XGE_STATUS FeedbackXgeStatus;
} OEM_XGE_STATUS_PROTOCOL;


extern EFI_GUID gOemNicProtocolGuid;
extern EFI_GUID gOemXgeStatusProtocolGuid;


#endif
