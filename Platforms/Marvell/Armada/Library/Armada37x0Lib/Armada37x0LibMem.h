/*******************************************************************************
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

#define DRAM_CH0_MMAP_LOW_REG(cs)       (0xD0000200 + (cs) * 0x8)
#define DRAM_CS_VALID_ENABLED_MASK      0x1
#define DRAM_AREA_LENGTH_OFFS           16
#define DRAM_AREA_LENGTH_MASK           (0x1f << DRAM_AREA_LENGTH_OFFS)
#define DRAM_START_ADDRESS_L_OFFS       23
#define DRAM_START_ADDRESS_L_MASK       (0x1ff << DRAM_START_ADDRESS_L_OFFS)
#define DRAM_CH0_MMAP_HIGH_REG(cs)      (0xD0000204 + (cs) * 0x8)
#define DRAM_START_ADDR_HTOL_OFFS       32

#define DRAM_MAX_CS_NUM                 2

#define DRAM_CS_ENABLED(Cs) \
          (MmioRead32 (DRAM_CH0_MMAP_LOW_REG (Cs)) & DRAM_CS_VALID_ENABLED_MASK)
#define GET_DRAM_REGION_BASE(Cs) \
          ((UINT64)MmioRead32 (DRAM_CH0_MMAP_HIGH_REG ((Cs))) << \
           DRAM_START_ADDR_HTOL_OFFS) | \
          (MmioRead32 (DRAM_CH0_MMAP_LOW_REG (Cs)) & DRAM_START_ADDRESS_L_MASK);
#define GET_DRAM_REGION_SIZE_CODE(Cs) \
          (MmioRead32 (DRAM_CH0_MMAP_LOW_REG ((Cs))) & \
           DRAM_AREA_LENGTH_MASK) >> DRAM_AREA_LENGTH_OFFS
#define DRAM_REGION_SIZE_EVEN(C)        (((C) >= 7) && ((C) <= 26))
#define GET_DRAM_REGION_SIZE_EVEN(C)    ((UINT64)1 << ((C) + 16))
#define DRAM_REGION_SIZE_ODD(C)         ((C) <= 4)
#define GET_DRAM_REGION_SIZE_ODD(C)     ((UINT64)0x18000000 << (C))

