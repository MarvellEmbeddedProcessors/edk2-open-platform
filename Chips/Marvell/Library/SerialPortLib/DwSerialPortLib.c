/** @file
  Marvell DesignWare 8250 compatible Serial port driver

  Copyright (C) Marvell International Ltd. and its affiliates

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php.

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Base.h>
#include <Library/IoLib.h>
#include <Library/SerialPortLib.h>

/* aliases - for registers which has the same offsets */
#define RBR    0x0
#define IER    0x4
#define FCR    0x8
#define LCR    0xc
#define MCR    0x10
#define LSR    0x14
#define MSR    0x18
#define SCR    0x1c

#define THR RBR
#define IIR FCR
#define DLL RBR
#define DLM IER

/* registers feilds */
#define FCR_FIFO_EN        BIT0        /* FIFO enable*/
#define FCR_RXSR           BIT1        /* Receiver soft reset*/
#define FCR_TXSR           BIT2        /* transmitter soft reset*/

#define MCR_RTS            BIT1        /* ready to send */

#define LCR_WLS_OFFS        0
#define LCR_WLS_MASK        (0x3 << LCR_WLS_OFFS)    /* character length mask  */
#define LCR_WLS_5           (0x0 << LCR_WLS_OFFS)    /* 5 bit character length */
#define LCR_WLS_6           (0x1 << LCR_WLS_OFFS)    /* 6 bit character length */
#define LCR_WLS_7           (0x2 << LCR_WLS_OFFS)    /* 7 bit character length */
#define LCR_WLS_8           (0x3 << LCR_WLS_OFFS)    /* 8 bit character length */
#define LCR_STP_OFFS        2
#define LCR_1_STB           (0x0 << LCR_STP_OFFS)    /* Number of stop Bits */
#define LCR_2_STB           (0x1 << LCR_STP_OFFS)    /* Number of stop Bits */
#define LCR_PEN             (0x8            /* Parity enable*/
#define LCR_PS_OFFS         4
#define LCR_EPS             (0x1 << LCR_PS_OFFS)    /* Even Parity Select*/
#define LCR_OPS             (0x0 << LCR_PS_OFFS)    /* Odd Parity Select*/
#define LCR_SBRK_OFFS       0x6
#define LCR_SBRK            (0x1 << LCR_SBRK_OFFS)    /* Set Break*/
#define LCR_DIVL_OFFS       7
#define LCR_DIVL_EN         (0x1 << LCR_DIVL_OFFS)    /* Divisor latch enable*/

#define LSR_DR            BIT0    /* Data ready */
#define LSR_OE            BIT1    /* Overrun */
#define LSR_PE            BIT2    /* Parity error */
#define LSR_FE            BIT3    /* Framing error */
#define LSR_BI            BIT4    /* Break */
#define LSR_THRE          BIT5    /* Xmit holding register empty */
#define LSR_TEMT          BIT6    /* Xmitter empty */
#define LSR_ERR           BIT7    /* Error */

/* useful defaults for LCR*/
#define LCR_8N1            (LCR_WLS_8 | LCR_1_STB)

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
  UINTN Regs;
  UINT32 Divisor;
  UINT32 Baudrate;
  UINT32 Clock;

  Regs = (UINTN)PcdGet64(PcdSerialRegisterBase);
  Baudrate = (UINT32)PcdGet64(PcdUartDefaultBaudRate);
  Clock  = (UINT32)PcdGet32(PcdSerialClockRate);

  Divisor = Clock / (Baudrate * 16);

  MmioWrite8(Regs + LCR, LCR_DIVL_EN); /* Enable dividor update */

  MmioWrite8(Regs + DLL, Divisor & 0xFF);
  MmioWrite8(Regs + DLM, (Divisor >> 8) & 0xFF);
  MmioWrite8(Regs + LCR, LCR_8N1); /* 8 data, 1 stop, no parity, clear dividor update */

  /* Clear & enable FIFOs */
  MmioWrite8(Regs + FCR, FCR_FIFO_EN | FCR_RXSR | FCR_TXSR);

  return RETURN_SUCCESS;
}

/**
  Write data from buffer to serial device.

  Writes NumberOfBytes data bytes from Buffer to the serial device.
  The number of bytes actually written to the serial device is returned.
  If the return value is less than NumberOfBytes, then the write operation failed.
  If Buffer is NULL, then ASSERT().
  If NumberOfBytes is zero, then return 0.

  @param  Buffer           The pointer to the data buffer to be written.
  @param  NumberOfBytes    The number of bytes to written to the serial device.

  @retval 0                NumberOfBytes is 0.
  @retval >0               The number of bytes written to the serial device.
                           If this value is less than NumberOfBytes, then the read operation failed.

**/
UINTN
EFIAPI
SerialPortWrite (
  IN UINT8     *Buffer,
  IN UINTN     NumberOfBytes
)
{
  UINTN Regs;
  UINT8 Lsr;
  UINTN Bytes = NumberOfBytes;

  Regs = (UINTN)PcdGet64(PcdSerialRegisterBase);

  while(NumberOfBytes--) {
    do {
      Lsr = MmioRead8(Regs + LSR);
    } while ((Lsr & LSR_THRE) == 0);

    MmioWrite8(Regs + THR, *Buffer);
    Buffer++;
  }

  return Bytes;
}

/**
  Read data from serial device and save the datas in buffer.

  Reads NumberOfBytes data bytes from a serial device into the buffer
  specified by Buffer. The number of bytes actually read is returned.
  If the return value is less than NumberOfBytes, then the rest operation failed.
  If Buffer is NULL, then ASSERT().
  If NumberOfBytes is zero, then return 0.

  @param  Buffer           The pointer to the data buffer to store the data read from the serial device.
  @param  NumberOfBytes    The number of bytes which will be read.

  @retval 0                Read data failed; No data is to be read.
  @retval >0               The actual number of bytes read from serial device.

**/
UINTN
EFIAPI
SerialPortRead (
  OUT UINT8     *Buffer,
  IN  UINTN     NumberOfBytes
)
{
  UINTN Regs;
  UINT8 Lsr;
  UINTN Bytes = NumberOfBytes;

  Regs = (UINTN)PcdGet64(PcdSerialRegisterBase);

  while(NumberOfBytes--) {
    do {
      Lsr = MmioRead8(Regs + LSR);
    } while ((Lsr & LSR_DR) == 0);

    *Buffer = MmioRead8(Regs + RBR);
    Buffer++;
  }

  return Bytes;
}

/**
  Polls a serial device to see if there is any data waiting to be read.

  Polls a serial device to see if there is any data waiting to be read.
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
  UINTN Regs = (UINTN)PcdGet64(PcdSerialRegisterBase);
  return MmioRead8(Regs + LSR) & LSR_DR;
}

/**
  Sets the control bits on a serial device.

  @param[in] Control            Sets the bits of Control that are settable.

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
  return RETURN_UNSUPPORTED;
}

/**
  Retrieve the status of the control bits on a serial device.

  @param[out] Control           A pointer to return the current control signals from the serial device.

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
  *Control = 0;
  if (!SerialPortPoll ()) {
    *Control = EFI_SERIAL_INPUT_BUFFER_EMPTY;
  }
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
  return RETURN_UNSUPPORTED;
}
