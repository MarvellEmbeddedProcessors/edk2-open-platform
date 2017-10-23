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

#include <Base.h>
#include <IndustryStandard/Pci.h>
#include <Library/SerialPortLib.h>
#include <Library/PcdLib.h>
#include <Library/IoLib.h>
#include <Library/PciLib.h>
#include <Library/PlatformHookLib.h>
#include <Library/BaseLib.h>

#define UART_RBR                0x00       /* Receiver Buffer */
#define RBR_BRK_DET             (1 << 15)  /* Break Detect */
#define RBR_FRM_ERR_DET         (1 << 14)  /* Frame Error Detect */
#define RBR_PAR_ERR_DET         (1 << 13)  /* Parity Error Detect */
#define RBR_OVR_ERR_DET         (1 << 12)  /* Overrun Error */

#define UART_TSH                0x04       /* Transmitter Holding Register */

#define UART_CTRL               0x08       /* Control Register */
#define CTRL_SOFT_RST           (1 << 31)  /* Soft Reset */
#define CTRL_TX_FIFO_RST        (1 << 15)  /* TX FIFO Reset */
#define CTRL_RX_FIFO_RST        (1 << 14)  /* RX FIFO Reset */
#define CTRL_ST_MIRR_EN         (1 << 13)  /* Status Mirror Enable */
#define CTRL_LPBK_EN            (1 << 12)  /* Loopback Mode Enable */
#define CTRL_SND_BRK_SEQ        (1 << 11)  /* Send Break Sequence */
#define CTRL_PAR_EN             (1 << 10)  /* Parity Enable */
#define CTRL_TWO_STOP           (1 << 9)   /* Two Stop Bits */
#define CTRL_TX_HALF_INT        (1 << 8)   /* TX Half-Full Interrupt Enable */
#define CTRL_RX_HALF_INT        (1 << 7)   /* RX Half-Full Interrupt Enable */
#define CTRL_TX_EMPT_INT        (1 << 6)   /* TX Empty Interrupt Enable */
#define CTRL_TX_RDY_INT         (1 << 5)   /* TX Ready Interrupt Enable */
#define CTRL_RX_RDY_INT         (1 << 4)   /* RX Ready Interrupt Enable */
#define CTRL_BRK_DET_INT        (1 << 3)   /* Break Detect Interrupt Enable */
#define CTRL_FRM_ERR_INT        (1 << 2)   /* Frame Error Interrupt Enable */
#define CTRL_PAR_ERR_INT        (1 << 1)   /* Parity Error Interrupt Enable */
#define CTRL_OVR_ERR_INT        (1 << 0)   /* Overrun Error Interrupt Enable */
#define CTRL_INTR_MASK          0x1ff
#define CTRL_TX_IDLE_INT        CTRL_TX_RDY_INT
#define CTRL_IPEND_MASK         (CTRL_OVR_ERR_INT | CTRL_BRK_DET_INT | \
                                 CTRL_RX_RDY_INT)

#define UART_STAT               0x0c       /* Status Register */
#define STAT_TX_FIFO_EMPT       (1 << 13)  /* TX FIFO Empty */
#define STAT_RX_FIFO_EMPT       (1 << 12)  /* RX FIFO Empty */
#define STAT_TX_FIFO_FULL       (1 << 11)  /* TX FIFO Full */
#define STAT_TX_FIFO_HALF       (1 << 10)  /* TX FIFO Half Full */
#define STAT_RX_TOGL            (1 << 9)   /* RX Toogled */
#define STAT_RX_FIFO_FULL       (1 << 8)   /* RX FIFO Full */
#define STAT_RX_FIFO_HALF       (1 << 7)   /* RX FIFO Half Full */
#define STAT_TX_EMPT            (1 << 6)   /* TX Empty */
#define STAT_TX_RDY             (1 << 5)   /* TX Ready */
#define STAT_RX_RDY             (1 << 4)   /* RX Ready */
#define STAT_BRK_DET            (1 << 3)   /* Break Detect */
#define STAT_FRM_ERR            (1 << 2)   /* Frame Error */
#define STAT_PAR_ERR            (1 << 1)   /* Parity Error */
#define STAT_OVR_ERR            (1 << 0)   /* Overrun Error */
#define STAT_TX_IDLE            STAT_TX_RDY
#define STAT_TRANS_MASK         (STAT_OVR_ERR | STAT_BRK_DET | STAT_RX_RDY)

#define UART_CCR                0x10       /* Clock Control Register */
#define CCR_BAUDRATE_DIV        0x3ff      /* Baud Rate Divisor */

/**
  Retrieve the I/O or MMIO base address register for the UART device.

  @return  The base address register of the UART device.

**/
UINTN
GetSerialRegisterBase (
  VOID
  )
{
    return (UINTN)PcdGet64 (PcdSerialRegisterBase);
}

/**
  Return whether the hardware flow control signal allows writing.

  @param  SerialRegisterBase The base address register of UART device.

  @retval TRUE  The serial port is writable.
  @retval FALSE The serial port is not writable.
**/
BOOLEAN
SerialPortWritable (
  UINTN  SerialRegisterBase
  )
{
  UINTN RegVal;
  UINTN RegBase;

  RegBase = GetSerialRegisterBase();

  RegVal = MmioRead32(RegBase + UART_STAT);

  return (RegVal & STAT_TX_RDY);
}

/**
  Initialize the serial device hardware.

  If no initialization is required, then return RETURN_SUCCESS.
  If the serial device was successfully initialized, then return RETURN_SUCCESS.
  If the serial device could not be initialized, then return RETURN_DEVICE_ERROR.

  @retval RETURN_SUCCESS        The serial device was initialized.
  @retval RETURN_DEVICE_ERROR   The serial device could not be initialized.

**/
RETURN_STATUS
EFIAPI
SerialPortInitialize (
  VOID
  )
{
  UINTN RegBase;
  UINTN Divisor;

  //
  // Get the base address of the serial port in either I/O or MMIO space
  //
  RegBase = GetSerialRegisterBase ();
  if (RegBase ==0) {
    return RETURN_DEVICE_ERROR;
  }

  //
  // Calculate divisor for baud generator
  //    Ref_Clk_Rate / Baud_Rate / 16
  //
  Divisor = PcdGet32 (PcdSerialClockRate) / (PcdGet32 (PcdSerialBaudRate) * 16);

  //
  // Get the base address of the serial port in either I/O or MMIO space
  //
  RegBase = GetSerialRegisterBase ();
  if (RegBase ==0) {
    return RETURN_DEVICE_ERROR;
  }

  //
  // Configure baud rate
  //
  MmioWrite32(RegBase + UART_CCR, Divisor);

  //
  // wait for the TX FIFO is empty
  //
  while (!(MmioRead32(RegBase + UART_STAT) & STAT_TX_FIFO_EMPT))
          ;

  /* reset FIFOs */
  MmioWrite32(RegBase + UART_CTRL, CTRL_TX_FIFO_RST | CTRL_RX_FIFO_RST);

  /* No Parity, 1 Stop */
  MmioWrite32(RegBase + UART_CTRL, 0);

  return RETURN_SUCCESS;
}

/**
  Write data from buffer to serial device.

  Writes NumberOfBytes data bytes from Buffer to the serial device.
  The number of bytes actually written to the serial device is returned.
  If the return value is less than NumberOfBytes, then the write operation failed.

  If Buffer is NULL, then ASSERT().

  If NumberOfBytes is zero, then return 0.

  @param  Buffer           Pointer to the data buffer to be written.
  @param  NumberOfBytes    Number of bytes to written to the serial device.

  @retval 0                NumberOfBytes is 0.
  @retval >0               The number of bytes written to the serial device.
                           If this value is less than NumberOfBytes, then the write operation failed.

**/
UINTN
EFIAPI
SerialPortWrite (
  IN UINT8     *Buffer,
  IN UINTN     NumberOfBytes
  )
{
  UINTN Result;
  UINTN RegBase;

  //
  // Get the base address of the serial port in either I/O or MMIO space
  //
  RegBase = GetSerialRegisterBase ();

  for(Result = 0; Result < NumberOfBytes; Result++) {
    while (MmioRead32(RegBase + UART_STAT) & STAT_TX_FIFO_FULL)
        ;
    MmioWrite32(RegBase + UART_TSH, *(Buffer + Result));
  }

  return Result;
}

/**
  Polls a serial device to see if there is any data waiting to be read.

  Polls aserial device to see if there is any data waiting to be read.
  If there is data waiting to be read from the serial device, then TRUE is returned.
  If there is no data waiting to be read from the serial device, then FALSE is returned.

  @retval TRUE             Data is waiting to be read from the serial device.
  @retval FALSE            There is no data waiting to be read from the serial device.

**/
BOOLEAN
EFIAPI
SerialPortPoll (
  VOID
  )
{
  UINTN RegVal;
  UINTN RegBase;

  //
  // Get the base address of the serial port in either I/O or MMIO space
  //
  RegBase = GetSerialRegisterBase ();

  RegVal = MmioRead32(RegBase + UART_STAT);

  return (RegVal & STAT_RX_RDY);
}

/**
  Reads data from a serial device into a buffer.

  @param  Buffer           Pointer to the data buffer to store the data read from the serial device.
  @param  NumberOfBytes    Number of bytes to read from the serial device.

  @retval 0                NumberOfBytes is 0.
  @retval >0               The number of bytes read from the serial device.
                           If this value is less than NumberOfBytes, then the read operation failed.

**/
UINTN
EFIAPI
SerialPortRead (
  OUT UINT8     *Buffer,
  IN  UINTN     NumberOfBytes
  )
{
  UINT8 Result;
  UINTN RegBase;

  //
  // Get the base address of the serial port in either I/O or MMIO space
  //
  RegBase = GetSerialRegisterBase ();

  for(Result = 0; Result < NumberOfBytes; Result++) {
    while (!SerialPortPoll())
            ;
    *(Buffer + Result) = (MmioRead32(RegBase + UART_RBR) & 0xff);
  }

  return Result;
}

/**
  Sets the control bits on a serial device.

  @param Control                Sets the bits of Control that are settable.

  @retval RETURN_SUCCESS        The new control bits were set on the serial device.
  @retval RETURN_UNSUPPORTED    The serial device does not support this operation.
  @retval RETURN_DEVICE_ERROR   The serial device is not functioning correctly.

**/
RETURN_STATUS
EFIAPI
SerialPortSetControl (
  IN UINT32 Control
  )
{
  return RETURN_SUCCESS;
}

/**
  Retrieve the status of the control bits on a serial device.

  @param Control                A pointer to return the current control signals from the serial device.

  @retval RETURN_SUCCESS        The control bits were read from the serial device.
  @retval RETURN_UNSUPPORTED    The serial device does not support this operation.
  @retval RETURN_DEVICE_ERROR   The serial device is not functioning correctly.

**/
RETURN_STATUS
EFIAPI
SerialPortGetControl (
  OUT UINT32 *Control
  )
{
  UINTN RegVal;
  UINTN RegBase;

  //
  // Get the base address of the serial port in either I/O or MMIO space
  //
  RegBase = GetSerialRegisterBase ();

  *Control = 0;

  RegVal = MmioRead32(RegBase + UART_STAT);

  if (RegVal & STAT_RX_FIFO_EMPT)
    *Control |= EFI_SERIAL_INPUT_BUFFER_EMPTY;

  if (RegVal & STAT_TX_FIFO_EMPT)
    *Control |= EFI_SERIAL_OUTPUT_BUFFER_EMPTY;

  if (RegVal & STAT_TX_RDY)
    *Control |= EFI_SERIAL_CLEAR_TO_SEND;

  return RETURN_SUCCESS;
}

/**
  Sets the baud rate, receive FIFO depth, transmit/receice time out, parity,
  data bits, and stop bits on a serial device.

  @param BaudRate           The requested baud rate. A BaudRate value of 0 will use the
                            device's default interface speed.
                            On output, the value actually set.
  @param ReveiveFifoDepth   The requested depth of the FIFO on the receive side of the
                            serial interface. A ReceiveFifoDepth value of 0 will use
                            the device's default FIFO depth.
                            On output, the value actually set.
  @param Timeout            The requested time out for a single character in microseconds.
                            This timeout applies to both the transmit and receive side of the
                            interface. A Timeout value of 0 will use the device's default time
                            out value.
                            On output, the value actually set.
  @param Parity             The type of parity to use on this serial device. A Parity value of
                            DefaultParity will use the device's default parity value.
                            On output, the value actually set.
  @param DataBits           The number of data bits to use on the serial device. A DataBits
                            vaule of 0 will use the device's default data bit setting.
                            On output, the value actually set.
  @param StopBits           The number of stop bits to use on this serial device. A StopBits
                            value of DefaultStopBits will use the device's default number of
                            stop bits.
                            On output, the value actually set.

  @retval RETURN_SUCCESS            The new attributes were set on the serial device.
  @retval RETURN_UNSUPPORTED        The serial device does not support this operation.
  @retval RETURN_INVALID_PARAMETER  One or more of the attributes has an unsupported value.
  @retval RETURN_DEVICE_ERROR       The serial device is not functioning correctly.

**/
RETURN_STATUS
EFIAPI
SerialPortSetAttributes (
  IN OUT UINT64             *BaudRate,
  IN OUT UINT32             *ReceiveFifoDepth,
  IN OUT UINT32             *Timeout,
  IN OUT EFI_PARITY_TYPE    *Parity,
  IN OUT UINT8              *DataBits,
  IN OUT EFI_STOP_BITS_TYPE *StopBits
  )
{
  UINTN RegBase;
  UINTN Divisor;
  UINTN RegVal;

  //
  // Get the base address of the serial port in either I/O or MMIO space
  //
  RegBase = GetSerialRegisterBase ();
  if (RegBase ==0) {
    return RETURN_DEVICE_ERROR;
  }

  //
  // Calculate divisor for baud generator
  //    Ref_Clk_Rate / Baud_Rate / 16
  //
  Divisor = PcdGet32 (PcdSerialClockRate) / ((*BaudRate) * 16);

  //
  // Configure baud rate
  //
  MmioWrite32(RegBase + UART_CCR, Divisor);

  RegVal = MmioRead32(RegBase + UART_CTRL);
  switch (*StopBits) {
  case 2:
    RegVal |= CTRL_TWO_STOP;
    break;
  case 1:
  default:
    RegVal &=~ CTRL_TWO_STOP;
  }

  switch (*Parity) {
  case 3: /* Even parity bit */
    RegVal |= CTRL_PAR_EN;
    break;
  default:
    RegVal &=~ CTRL_PAR_EN;
  }

  MmioWrite32(RegBase + UART_CTRL, RegVal);

  return RETURN_SUCCESS;
}
