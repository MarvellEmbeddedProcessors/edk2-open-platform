/** @file
  Differentiated System Description Table Fields (DSDT)

  Copyright (c) 2014, ARM Ltd. All rights reserved.<BR>
    This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

Scope(_SB)
{
  // Mbi-gen pcie subsys
  Device(MBI0) {
    Name(_HID, "HISI0152")
    Name(_CID, "MBIGen")
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, 0xa0080000, 0x10000)
    })

   Name(_DSD, Package () {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package ()
        {
          Package () {"num-pins", 2}
        }
   })
  }

  // Mbi-gen sas1 intc
  Device(MBI1) {
    Name(_HID, "HISI0152")
    Name(_CID, "MBIGen")
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, 0xa0080000, 0x10000)
    })
   Name(_DSD, Package () {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package ()
        {
          Package () {"num-pins", 128}
        }
   })
  }

  Device(MBI2) {          // Mbi-gen sas2 intc
    Name(_HID, "HISI0152")
    Name(_CID, "MBIGen")
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, 0xa0080000, 0x10000)
    })
   Name(_DSD, Package () {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package ()
        {
          Package () {"num-pins", 128}
        }
   })
  }

  Device(MBI3) {          // Mbi-gen dsa0 srv intc
    Name(_HID, "HISI0152")
    Name(_CID, "MBIGen")
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, 0xc0080000, 0x10000)
    })
   Name(_DSD, Package () {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package ()
        {
          Package () {"num-pins", 409}
        }
   })
  }

  Device(MBI4) {          // Mbi-gen dsa1 dbg0 intc
    Name(_HID, "HISI0152")
    Name(_CID, "MBIGen")
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, 0xc0080000, 0x10000)
    })
   Name(_DSD, Package () {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package ()
        {
          Package () {"num-pins", 9}
        }
   })
  }

  Device(MBI5) {          // Mbi-gen dsa2 dbg1 intc
    Name(_HID, "HISI0152")
    Name(_CID, "MBIGen")
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, 0xc0080000, 0x10000)
    })
   Name(_DSD, Package () {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package ()
        {
          Package () {"num-pins", 9}
        }
   })
  }

  Device(MBI6) {          // Mbi-gen dsa sas0 intc
    Name(_HID, "HISI0152")
    Name(_CID, "MBIGen")
    Name(_CRS, ResourceTemplate() {
      Memory32Fixed(ReadWrite, 0xc0080000, 0x10000)
    })
   Name(_DSD, Package () {
        ToUUID("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
        Package ()
        {
          Package () {"num-pins", 128}
        }
   })
  }

}
