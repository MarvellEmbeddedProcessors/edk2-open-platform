/** @file

  Differentiated System Description Table Fields (DSDT)

  Copyright (c) 2017, Linaro Ltd. All rights reserved.<BR>

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

DefinitionBlock ("DSDT.aml", "DSDT", 2, "MVEBU ", "ARMADA8K", 3)
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
                    0x00002000,         // Address Length
                    )
                Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive, ,, )
                {
                  95
                }
            })
        }

        Device (AHC1)
        {
            Name (_HID, "LNRO001E")     // _HID: Hardware ID
            Name (_UID, 0x01)           // _UID: Unique ID
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
                    0xF4540000,         // Address Base (MMIO)
                    0x00002000,         // Address Length
                    )
                Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive, ,, )
                {
                  319
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
                  94
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
                  93
                }
            })
        }

        Device (XHC2)
        {
            Name (_HID, "PNP0D10")      // _HID: Hardware ID
            Name (_UID, 0x02)           // _UID: Unique ID
            Name (_CCA, 0x01)           // _CCA: Cache Coherency Attribute

            Name (_CRS, ResourceTemplate ()  // _CRS: Current Resource Settings
            {
                Memory32Fixed (ReadWrite,
                    0xF4500000,         // Address Base (MMIO)
                    0x00004000,         // Address Length
                    )
                Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive, ,, )
                {
                  318
                }
            })
        }

        Device (COM1)
        {
            Name (_HID, "HISI0031")                             // _HID: Hardware ID
            Name (_CID, "8250dw")                               // _CID: Compatible ID
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
                        FixedPcdGet32 (PcdPciBusMin),       // Range Minimum
                        FixedPcdGet32 (PcdPciBusMax),       // Range Minimum
                        0x0000,                             // Translation Offset
                        0xFF                                // Length
                        )
                    DWordMemory (ResourceProducer, PosDecode, MinFixed, MaxFixed, NonCacheable, ReadWrite,
                        0x00000000,                         // Granularity
                        FixedPcdGet32 (PcdPciMmio32Base),   // Range Minimum
                        0xDFFFFFFF,                         // Range Maximum
                        0x00000000,                         // Translation Offset
                        FixedPcdGet32 (PcdPciMmio32Size)    // Length
                        )
                    QWordMemory (ResourceProducer, PosDecode, MinFixed, MaxFixed, NonCacheable, ReadWrite,
                        0x0000000000000000, // Granularity
                        FixedPcdGet64 (PcdPciMmio64Base),   // Range Minimum
                        0x8FFFFFFFF,                        // Range Maximum
                        0x00000000,                         // Translation Offset
                        FixedPcdGet64 (PcdPciMmio64Size)    // Length
                        )
                    DWordIo (ResourceProducer, MinFixed, MaxFixed, PosDecode, EntireRange,
                        0x00000000,                         // Granularity
                        FixedPcdGet32 (PcdPciIoBase),       // Range Minimum
                        0x0000FFFF,                         // Range Maximum
                        FixedPcdGet32 (PcdPciIoTranslation),// Translation Address
                        FixedPcdGet32 (PcdPciIoSize),       // Length
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
                                   FixedPcdGet32 (PcdPciExpressBaseAddress),
                                   0x10000000
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

            //
            // Device-Specific Methods
            //
            Method(_DSM, 0x4, NotSerialized) {
              If (LEqual(Arg0, ToUUID("E5C937D0-3553-4d7a-9117-EA4D19C3434D"))) {
                switch (ToInteger(Arg2)) {
                  //
                  // Function 0: Return supported functions
                  //
                  case(0) {
                    Return (Buffer() {0xFF})
                  }

                  //
                  // Function 1: Return PCIe Slot Information
                  //
                  case(1) {
                    Return (Package(2) {
                      One, // Success
                      Package(3) {
                        0x1,  // x1 PCIe link
                        0x1,  // PCI express card slot
                        0x1   // WAKE# signal supported
                      }
                    })
                  }

                  //
                  // Function 2: Return PCIe Slot Number.
                  //
                  case(2) {
                    Return (Package(1) {
                      Package(4) {
                        2,  // Source ID
                        4,  // Token ID: ID refers to a slot
                        0,  // Start bit of the field to use.
                        7   // End bit of the field to use.
                      }
                    })
                  }

                  //
                  // Function 3: Return Vendor-specific Token ID Strings.
                  //
                  case(3) {
                    Return (Package(0) {})
                  }

                  //
                  // Function 4: Return PCI Bus Capabilities
                  //
                  case(4) {
                    Return (Package(2) {
                      One, // Success
                      Buffer() {
                        1,0,            // Version
                        0,0,            // Status, 0:Success
                        24,0,0,0,       // Length
                        1,0,            // PCI
                        16,0,           // Length
                        0,              // Attributes
                        0x0D,           // Current Speed/Mode
                        0x3F,0,         // Supported Speeds/Modes
                        0,              // Voltage
                        0,0,0,0,0,0,0   // Reserved
                      }
                    })
                  }

                  //
                  // Function 5: Return Ignore PCI Boot Configuration
                  //
                  case(5) {
                    Return (Package(1) {1})
                  }

                  //
                  // Function 6: Return LTR Maximum Latency
                  //
                  case(6) {
                    Return (Package(4) {
                      Package(1){0},  // Maximum Snoop Latency Scale
                      Package(1){0},  // Maximum Snoop Latency Value
                      Package(1){0},  // Maximum No-Snoop Latency Scale
                      Package(1){0}   // Maximum No-Snoop Latency Value
                    })
                  }

                  //
                  // Function 7: Return PCI Express Naming
                  //
                  case(7) {
                    Return (Package(2) {
                      Package(1) {0},
                      Package(1) {Unicode("PCI0")}
                    })
                  }

                  //
                  // Not supported
                  //
                  default {
                  }
                }
              }
              Return (Buffer(){0})
            } // Method(_DSM)

            //
            // Root-Complex 0
            //
            Device (RP0)
            {
                Name (_ADR, FixedPcdGet32 (PcdPciExpressBaseAddress))  // _ADR: Bus 0, Dev 0, Func 0
            }
        }
    }
}
