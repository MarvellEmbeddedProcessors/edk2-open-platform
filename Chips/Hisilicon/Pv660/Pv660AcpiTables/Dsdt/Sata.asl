//
// SATA AHCI
//

Device (SATA0)
{
  Name(_HID, "HISI0600")	// HiSi AHCI
  Name (_CCA, 1)       		// Cache-coherent controller
  Name (_CLS, Package (3)	// How to identify direct attached AHCI to Windows
  {
    0x01,       // Base Class:  Mass Storage
    0x06,       // Sub-Class: Serial ATA
    0x01,       // Interface: AHCI
  })
  Name (_CRS, ResourceTemplate ()
  {
    Memory32Fixed (ReadWrite, %AHCI_ADDRESS%, 0x00010000)
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive,,,) { %AHCI_GSIV% }
  })
}

