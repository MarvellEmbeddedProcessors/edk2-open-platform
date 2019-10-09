/** @file

  Differentiated System Description Table Fields (DSDT)

  Copyright (c) 2018, Linaro Ltd. All rights reserved.<BR>
  Copyright (C) 2019, Marvell International Ltd. and its affiliates.<BR>

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#include "Cn913xDbA/Pcie.h"
#include "IcuInterrupts.h"

DefinitionBlock ("DSDT.aml", "DSDT", 2, "MVEBU ", "CN9130", 3)
{
    Scope (_SB)
    {
        Device (CPU0)
        {
            Name (_HID, "ACPI0007" /* Processor Device */)  // _HID: Hardware ID
            Name (_UID, 0x000)  // _UID: Unique ID
        }
        Device (CPU1)
        {
            Name (_HID, "ACPI0007" /* Processor Device */)  // _HID: Hardware ID
            Name (_UID, 0x001)  // _UID: Unique ID
        }
        Device (CPU2)
        {
            Name (_HID, "ACPI0007" /* Processor Device */)  // _HID: Hardware ID
            Name (_UID, 0x100)  // _UID: Unique ID
        }
        Device (CPU3)
        {
            Name (_HID, "ACPI0007" /* Processor Device */)  // _HID: Hardware ID
            Name (_UID, 0x101)  // _UID: Unique ID
        }

        Device (AHC0)
        {
            Name (_HID, "LNRO001E")     // _HID: Hardware ID
            Name (_UID, 0x00)           // _UID: Unique ID
            Name (_CCA, 0x01)           // _CCA: Cache Coherency Attribute
            Name (_CLS, Package (0x03)  // _CLS: Class Code
            {
                0x01,
                0x06,
                0x01
            })

            Name (_CRS, ResourceTemplate ()  // _CRS: Current Resource Settings
            {
                Memory32Fixed (ReadWrite,
                    0xF2540000,         // Address Base (MMIO)
                    0x00030000,         // Address Length
                    )
                Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive, ,, )
                {
                  CP_GIC_SPI_CP0_SATA_H0
                }
            })
        }

        Device (XHC0)
        {
            Name (_HID, "PNP0D10")      // _HID: Hardware ID
            Name (_UID, 0x00)           // _UID: Unique ID
            Name (_CCA, 0x01)           // _CCA: Cache Coherency Attribute

            Name (_CRS, ResourceTemplate ()  // _CRS: Current Resource Settings
            {
                Memory32Fixed (ReadWrite,
                    0xF2500000,         // Address Base (MMIO)
                    0x00004000,         // Address Length
                    )
                Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive, ,, )
                {
                  CP_GIC_SPI_CP0_USB_H0
                }
            })
        }

        Device (XHC1)
        {
            Name (_HID, "PNP0D10")      // _HID: Hardware ID
            Name (_UID, 0x01)           // _UID: Unique ID
            Name (_CCA, 0x01)           // _CCA: Cache Coherency Attribute

            Name (_CRS, ResourceTemplate ()  // _CRS: Current Resource Settings
            {
                Memory32Fixed (ReadWrite,
                    0xF2510000,         // Address Base (MMIO)
                    0x00004000,         // Address Length
                    )
                Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive, ,, )
                {
                  CP_GIC_SPI_CP0_USB_H1
                }
            })
        }

        Device (COM1)
        {
            Name (_HID, "MRVL0001")                             // _HID: Hardware ID
            Name (_CID, "HISI0031")                             // _CID: Compatible ID
            Name (_ADR, FixedPcdGet64(PcdSerialRegisterBase))   // _ADR: Address
            Name (_CRS, ResourceTemplate ()                     // _CRS: Current Resource Settings
            {
                Memory32Fixed (ReadWrite,
                    FixedPcdGet64(PcdSerialRegisterBase),       // Address Base
                    0x00000100,                                 // Address Length
                    )
                Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive, ,, )
                {
                  51
                }
            })
            Name (_DSD, Package () {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package () {
                      Package () { "clock-frequency", FixedPcdGet32 (PcdSerialClockRate) },
                      Package () { "reg-io-width", 1 },
                      Package () { "reg-shift", 2 },
                }
            })
        }

        Device (PP20)
        {
            Name (_HID, "MRVL0110")                             // _HID: Hardware ID
            Name (_CCA, 0x01)                                   // Cache-coherent controller
            Name (_UID, 0x00)                                   // _UID: Unique ID
            Name (_CRS, ResourceTemplate ()
            {
                Memory32Fixed (ReadWrite, 0xf2000000 , 0x100000)
                Memory32Fixed (ReadWrite, 0xf2129000 , 0xb000)
            })
            Name (_DSD, Package () {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package () {
                  Package () { "clock-frequency", 333333333 },
                }
            })
            Device (ETH0)
            {
              Name (_ADR, 0x0)
              Name (_CRS, ResourceTemplate ()
              {
                  Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive, ,, )
                  {
                    CP_GIC_SPI_PP2_CP0_PORT0
                  }
              })
              Name (_DSD, Package () {
                  ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                  Package () {
                    Package () { "port-id", 0 },
                    Package () { "gop-port-id", 0 },
                    Package () { "phy-mode", "10gbase-kr"},
                  }
              })
            }
            Device (ETH1)
            {
              Name (_ADR, 0x0)
              Name (_CRS, ResourceTemplate ()
              {
                  Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive, ,, )
                  {
                    CP_GIC_SPI_PP2_CP0_PORT1
                  }
              })
              Name (_DSD, Package () {
                  ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                  Package () {
                    Package () { "port-id", 1 },
                    Package () { "gop-port-id", 2 },
                    Package () { "phy-mode", "rgmii-id"},
                  }
              })
            }
            Device (ETH2)
            {
              Name (_ADR, 0x0)
              Name (_CRS, ResourceTemplate ()
              {
                  Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive, ,, )
                  {
                    CP_GIC_SPI_PP2_CP0_PORT2
                  }
              })
              Name (_DSD, Package () {
                  ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                  Package () {
                    Package () { "port-id", 2 },
                    Package () { "gop-port-id", 3 },
                    Package () { "phy-mode", "rgmii-id"},
                  }
              })
            }
        }

        Device (RNG0)
        {
            Name (_HID, "PRP0001")                                 // _HID: Hardware ID
            Name (_UID, 0x00)                                      // _UID: Unique ID
            Name (_CRS, ResourceTemplate ()
            {
                Memory32Fixed (ReadWrite, 0xF2760000, 0x7D)
                Interrupt (ResourceConsumer, Level, ActiveHigh, Shared)
                {
                  CP_GIC_SPI_CP0_EIP_RNG0
                }
            })
            Name (_DSD, Package () {
                ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
                Package () {
                    Package () { "compatible", "inside-secure,safexcel-eip76" },
                }
            })
        }

        //
        // PCIe Root Bus
        //
        Device (PCI0)
        {
            Name (_HID, "PNP0A08" /* PCI Express Bus */)  // _HID: Hardware ID
            Name (_CID, "PNP0A03" /* PCI Bus */)  // _CID: Compatible ID
            Name (_SEG, 0x00)  // _SEG: PCI Segment
            Name (_BBN, 0x00)  // _BBN: BIOS Bus Number
            Name (_CCA, 0x01)  // _CCA: Cache Coherency Attribute
            Name (_PRT, Package ()  // _PRT: PCI Routing Table
            {
                Package () { 0xFFFF, 0x0, 0x0, 0x40 },
                Package () { 0xFFFF, 0x1, 0x0, 0x40 },
                Package () { 0xFFFF, 0x2, 0x0, 0x40 },
                Package () { 0xFFFF, 0x3, 0x0, 0x40 }
            })

            Method (_CRS, 0, Serialized)  // _CRS: Current Resource Settings
            {
                Name (RBUF, ResourceTemplate ()
                {
                    WordBusNumber (ResourceProducer, MinFixed, MaxFixed, PosDecode,
                        0x0000,                             // Granularity
                        PCI_BUS_MIN,                        // Range Minimum
                        PCI_BUS_MAX,                        // Range Maximum
                        0x0000,                             // Translation Offset
                        PCI_BUS_COUNT                       // Length
                        )
                    DWordMemory (ResourceProducer, PosDecode, MinFixed, MaxFixed, NonCacheable, ReadWrite,
                        0x00000000,                         // Granularity
                        PCI_MMIO32_BASE,                    // Range Minimum
                        0xCFFFFFFF,                         // Range Maximum
                        0x00000000,                         // Translation Offset
                        PCI_MMIO32_SIZE                     // Length
                        )
                    DWordIo (ResourceProducer, MinFixed, MaxFixed, PosDecode, EntireRange,
                        0x00000000,                         // Granularity
                        PCI_IO_BASE,                        // Range Minimum
                        0x0000FFFF,                         // Range Maximum
                        PCI_IO_TRANSLATION,                 // Translation Address
                        PCI_IO_SIZE,                        // Length
                        ,
                        ,
                        ,
                        TypeTranslation
                        )
                })
                Return (RBUF) /* \_SB_.PCI0._CRS.RBUF */
            } // Method(_CRS)

            Device (RES0)
            {
                Name (_HID, "PNP0C02")
                Name (_CRS, ResourceTemplate ()
                {
                    Memory32Fixed (ReadWrite,
                                   PCI_ECAM_BASE,
                                   PCI_ECAM_SIZE
                                   )
                })
            }
            Name (SUPP, 0x00)
            Name (CTRL, 0x00)
            Method (_OSC, 4, NotSerialized)  // _OSC: Operating System Capabilities
            {
                CreateDWordField (Arg3, 0x00, CDW1)
                If (LEqual (Arg0, ToUUID ("33db4d5b-1ff7-401c-9657-7441c03dd766") /* PCI Host Bridge Device */))
                {
                    CreateDWordField (Arg3, 0x04, CDW2)
                    CreateDWordField (Arg3, 0x08, CDW3)
                    Store (CDW2, SUPP) /* \_SB_.PCI0.SUPP */
                    Store (CDW3, CTRL) /* \_SB_.PCI0.CTRL */
                    If (LNotEqual (And (SUPP, 0x16), 0x16))
                    {
                        And (CTRL, 0x1E, CTRL) /* \_SB_.PCI0.CTRL */
                    }

                    And (CTRL, 0x1D, CTRL) /* \_SB_.PCI0.CTRL */
                    If (LNotEqual (Arg1, One))
                    {
                        Or (CDW1, 0x08, CDW1) /* \_SB_.PCI0._OSC.CDW1 */
                    }

                    If (LNotEqual (CDW3, CTRL))
                    {
                        Or (CDW1, 0x10, CDW1) /* \_SB_.PCI0._OSC.CDW1 */
                    }

                    Store (CTRL, CDW3) /* \_SB_.PCI0._OSC.CDW3 */
                    Return (Arg3)
                }
                Else
                {
                    Or (CDW1, 0x04, CDW1) /* \_SB_.PCI0._OSC.CDW1 */
                    Return (Arg3)
                }
            } // Method(_OSC)
        }
    }
}
