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

#ifndef __EEPROM_H__
#define __EEPROM_H__

#define EFI_EEPROM_PROTOCOL_GUID { 0xcd728a1f, 0x45b5, 0x4feb, { 0x98, 0xc8, 0x31, 0x3d, 0xa8, 0x11, 0x74, 0x62 }}

typedef struct _EFI_EEPROM_PROTOCOL EFI_EEPROM_PROTOCOL;

#define EEPROM_READ   0x1
#define EEPROM_WRITE  0x0
typedef
EFI_STATUS
(EFIAPI *EFI_EEPROM_TRANSFER) (
  IN CONST EFI_EEPROM_PROTOCOL *This,
  IN UINT16 Address,
  IN UINT32 Length,
  IN UINT8 *Buffer,
  IN UINT8 Operation
  );

struct _EFI_EEPROM_PROTOCOL {
  EFI_EEPROM_TRANSFER Transfer;
  UINT8 Identifier;
};

extern EFI_GUID gEfiEepromProtocolGuid;
#endif
