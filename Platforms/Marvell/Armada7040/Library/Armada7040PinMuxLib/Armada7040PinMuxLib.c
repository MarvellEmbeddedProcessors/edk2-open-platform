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

#include <Library/ArmLib.h>
#include <Library/ArmPlatformLib.h>
#include <Library/DebugLib.h>
#include <Library/PcdLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/IoLib.h>

#define MPP_PIN_VAL(pin,func)    (((func) & 0xf) << ((pin) * 4))
#define MPP_MAX_REGS             8
#define MPP_PINS_PER_REG         8

#define AP_CHIP_NAME             Ap
#define CP_CHIP_NAME             Cp
#define MAX_CHIPS                1

#define GET_PCD_PTR(chip,id,num) _PCD_GET_MODE_PTR_##Pcd##chip##id##MppSel##num
#define GET_REG_COUNT(chip,id)   _PCD_GET_MODE_32_Pcd##chip##id##MppRegCount
#define GET_BASE(chip,id)        _PCD_GET_MODE_64_Pcd##chip##id##PinMuxBaseAddress
#define GET_REV_FLAG(chip,id)    _PCD_GET_MODE_BOOL_Pcd##chip##id##PinMuxReverseFlag

/* We get chip name and number */
#define GetMppPcd(chip,id) {                   \
  RegCount[id] = GET_REG_COUNT(chip,id);       \
/* Fall through */                             \
  switch (RegCount[id]) {                      \
  case 8:                                      \
    MppRegPcd[id][7] = GET_PCD_PTR(chip,id,7); \
  case 7:                                      \
    MppRegPcd[id][6] = GET_PCD_PTR(chip,id,6); \
  case 6:                                      \
    MppRegPcd[id][5] = GET_PCD_PTR(chip,id,5); \
  case 5:                                      \
    MppRegPcd[id][4] = GET_PCD_PTR(chip,id,4); \
  case 4:                                      \
    MppRegPcd[id][3] = GET_PCD_PTR(chip,id,3); \
  case 3:                                      \
    MppRegPcd[id][2] = GET_PCD_PTR(chip,id,2); \
  case 2:                                      \
    MppRegPcd[id][1] = GET_PCD_PTR(chip,id,1); \
  case 1:                                      \
    MppRegPcd[id][0] = GET_PCD_PTR(chip,id,0); \
  }                                            \
  BaseAddr[id] = GET_BASE(chip,id);            \
  ReverseFlag[id] = GET_REV_FLAG(chip,id);     \
}

VOID
SetRegisterValueInv (
  UINT8 RegCount,
  UINT8 **MppRegPcd,
  UINTN BaseAddr
  )
{
  UINT32 i, j, CtrlVal;

  for (i = 0; i < RegCount; i++) {
    CtrlVal = 0;
    for (j = 0; j < MPP_PINS_PER_REG; j++) {
      CtrlVal |= MPP_PIN_VAL(7 - j, MppRegPcd[i][7 - j]);
    }
    MmioWrite32 (BaseAddr - 4 * i, CtrlVal);
  }
}

VOID
SetRegisterValue (
  UINT8 RegCount,
  UINT8 **MppRegPcd,
  UINTN BaseAddr
  )
{
  UINT32 i, j, CtrlVal;

  for (i = 0; i < RegCount; i++) {
    CtrlVal = 0;
    for (j = 0; j < MPP_PINS_PER_REG; j++) {
      CtrlVal |= MPP_PIN_VAL(j, MppRegPcd[i][j]);
    }
    MmioWrite32 (BaseAddr + 4 * i, CtrlVal);
  }
}

EFI_STATUS
PinMuxInitialize (
  )
{
  UINTN BaseAddr[MAX_CHIPS], RegCount[MAX_CHIPS];
  BOOLEAN ReverseFlag[MAX_CHIPS];
  UINT8 *MppRegPcd[MAX_CHIPS][MPP_MAX_REGS];
  UINT32 i, ApChipCount, CpChipCount;

  /* Obtain number of APN806 and CP110 chips in the system */
  ApChipCount = PcdGet32 (PcdApPinMuxChipCount);
  CpChipCount = PcdGet32 (PcdCpPinMuxChipCount);

  /* Read all needed PCD for MPP configuration for APN806 chips */
  GetMppPcd(AP_CHIP_NAME, 0);

  for (i = 0; i < MAX_CHIPS; i++) {
    if (i == ApChipCount)
      break;
    if (ReverseFlag[i]) {
      SetRegisterValueInv (RegCount[i], MppRegPcd[i], BaseAddr[i]);
    } else {
      SetRegisterValue (RegCount[i], MppRegPcd[i], BaseAddr[i]);
    }
  }

  if (CpChipCount == 0) {
    return EFI_SUCCESS;
  }

  /* Process CP110 chips' MPP configuration */
  GetMppPcd(CP_CHIP_NAME, 0);

  for (i = 0; i < MAX_CHIPS; i++) {
    if (i == CpChipCount)
      break;
    if (ReverseFlag[i]) {
      SetRegisterValueInv (RegCount[i], MppRegPcd[i], BaseAddr[i]);
    } else {
      SetRegisterValue (RegCount[i], MppRegPcd[i], BaseAddr[i]);
    }
  }

  return EFI_SUCCESS;
}
