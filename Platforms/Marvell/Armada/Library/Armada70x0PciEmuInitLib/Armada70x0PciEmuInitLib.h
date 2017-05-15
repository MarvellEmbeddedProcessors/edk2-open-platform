/**
*
*  Copyright (c) 2017, Linaro Ltd. All rights reserved.
*  Copyright (C) 2017, Marvell International Ltd. and its affiliates
*
*  This program and the accompanying materials are licensed and made available
*  under the terms and conditions of the BSD License which accompanies this
*  distribution. The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#define GPIO_BASE                      FixedPcdGet64 (PcdChip1MppBaseAddress) + 0x100
#define GPIO_DIR_OFFSET(n)             (((n) >> 5) * 0x40)
#define GPIO_ENABLE_OFFSET(n)          (((n) >> 5) * 0x40 + 0x4)

#define GPIO_PIN_MASK(n)               (1 << ((n) & 0x1f))

#define ARMADA_8040_MCBIN_VBUS_GPIO    47
