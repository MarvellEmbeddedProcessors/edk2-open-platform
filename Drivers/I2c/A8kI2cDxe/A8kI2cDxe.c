/*-
 * Copyright (C) 2008 MARVELL INTERNATIONAL LTD.
 * Copyright (C) 2016 MARVELL INTERNATIONAL LTD.
 * All rights reserved.
 *
 * Developed by Semihalf.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of MARVELL nor the names of contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <Protocol/I2cMaster.h>
#include <Protocol/I2cEnumerate.h>
#include <Protocol/I2cBusConfigurationManagement.h>
#include <Protocol/DevicePath.h>

#include <Library/BaseLib.h>
#include <Library/IoLib.h>
#include <Library/DebugLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>

#include "A8kI2cDxe.h"

STATIC A8K_I2C_BAUD_RATE baud_rate[IIC_FASTEST + 1];

A8K_I2C_DEVICE_PATH gDevicePathProtocol = {
  {
    {
      HARDWARE_DEVICE_PATH,
      HW_VENDOR_DP,
      {
  (UINT8) (sizeof(VENDOR_DEVICE_PATH)),
  (UINT8) (sizeof(VENDOR_DEVICE_PATH) >> 8),
      },
    },
    EFI_CALLER_ID_GUID
  },
  {
    END_DEVICE_PATH_TYPE,
    END_ENTIRE_DEVICE_PATH_SUBTYPE,
    {
      sizeof(EFI_DEVICE_PATH_PROTOCOL),
      0
    }
  }
};

STATIC
UINT32
TWSI_READ(
  IN I2C_MASTER_CONTEXT *I2cMasterContext,
  IN UINTN off)
{
  ASSERT (I2cMasterContext != NULL);
  return MmioRead32 (I2cMasterContext->BaseAddress + off);
}

STATIC
EFI_STATUS
TWSI_WRITE (
  IN I2C_MASTER_CONTEXT *I2cMasterContext,
  IN UINTN off,
  IN UINT32 val)
{
  ASSERT (I2cMasterContext != NULL);
  return MmioWrite32 (I2cMasterContext->BaseAddress + off, val);
}

EFI_STATUS
EFIAPI
A8kI2cInitialise (
  IN EFI_HANDLE  ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS Status;
  I2C_MASTER_CONTEXT *I2cMasterContext;

  /* if attachment succeeds, this gets freed at ExitBootServices */
  I2cMasterContext = AllocateZeroPool (sizeof (I2C_MASTER_CONTEXT));
  if (I2cMasterContext == NULL) {
    DEBUG((DEBUG_ERROR, "Allocation fail.\n"));
    return EFI_OUT_OF_RESOURCES;
  }
  I2cMasterContext->Signature = I2C_MASTER_SIGNATURE;
  I2cMasterContext->I2cMaster.Reset = A8kI2cReset;
  I2cMasterContext->I2cMaster.StartRequest = A8kI2cStartRequest;
  I2cMasterContext->I2cEnumerate.Enumerate = A8kI2cEnumerate;
  I2cMasterContext->I2cBusConf.EnableI2cBusConfiguration = A8kI2cEnableConf;
  I2cMasterContext->TclkFrequency = PcdGet32 (PcdTclkFrequency);
  I2cMasterContext->BaseAddress = PcdGet64 (PcdI2cBaseAddress);

  EfiInitializeLock(&I2cMasterContext->Lock, TPL_NOTIFY);

  /* checks if protocol is *not yet* installed */
  ASSERT_PROTOCOL_ALREADY_INSTALLED(NULL, &gEfiI2cMasterProtocolGuid);
  ASSERT_PROTOCOL_ALREADY_INSTALLED(NULL, &gEfiI2cEnumerateProtocolGuid);
  ASSERT_PROTOCOL_ALREADY_INSTALLED(NULL, &gEfiI2cBusConfigurationManagementProtocolGuid);

  A8kI2cCalBaudRate(TWSI_BAUD_RATE_SLOW, &baud_rate[IIC_SLOW], I2cMasterContext->TclkFrequency);
  A8kI2cCalBaudRate(TWSI_BAUD_RATE_FAST, &baud_rate[IIC_FAST], I2cMasterContext->TclkFrequency);

  Status = gBS->InstallMultipleProtocolInterfaces(
      &I2cMasterContext->Controller,
      &gEfiI2cMasterProtocolGuid,
      &I2cMasterContext->I2cMaster,
      &gEfiI2cEnumerateProtocolGuid,
      &I2cMasterContext->I2cEnumerate,
      &gEfiI2cBusConfigurationManagementProtocolGuid,
      &I2cMasterContext->I2cBusConf,
      &gEfiDevicePathProtocolGuid,
      (EFI_DEVICE_PATH_PROTOCOL *) &gDevicePathProtocol,
      NULL);

  if (EFI_ERROR(Status)) {
    DEBUG((DEBUG_ERROR, "Installing protocol interfaces failed!\n"));
    goto fail;
  }

  return EFI_SUCCESS;

fail:
  FreePool(I2cMasterContext);
  return Status;
}

STATIC
VOID
A8kI2cControlClear (
  IN I2C_MASTER_CONTEXT *I2cMasterContext,
  IN UINT32 mask)
{
  UINT32 val;

  val = TWSI_READ(I2cMasterContext, TWSI_CONTROL);
  val &= ~mask;
  TWSI_WRITE(I2cMasterContext, TWSI_CONTROL, val);
}

STATIC
VOID
A8kI2cControlSet (
  IN I2C_MASTER_CONTEXT *I2cMasterContext,
  IN UINT32 mask)
{
  UINT32 val;

  val = TWSI_READ(I2cMasterContext, TWSI_CONTROL);
  val |= mask;
  TWSI_WRITE(I2cMasterContext, TWSI_CONTROL, val);
}

STATIC
VOID
A8kI2cClearIflg (
 IN I2C_MASTER_CONTEXT *I2cMasterContext
 )
{

  gBS->Stall(1000);
  A8kI2cControlClear(I2cMasterContext, TWSI_CONTROL_IFLG);
  gBS->Stall(1000);
}


/*
 * timeout given in us
 * returns
 *   0 on sucessfull mask change
 *   non-zero on timeout
 */
STATIC
UINTN
A8kI2cPollCtrl (
  IN I2C_MASTER_CONTEXT *I2cMasterContext,
  IN UINTN timeout,
  IN UINT32 mask)
{

  timeout /= 10;
  while (!(TWSI_READ(I2cMasterContext, TWSI_CONTROL) & mask)) {
    gBS->Stall(10);
    if (--timeout == 0)
      return (timeout);
  }
  return (0);
}


/*
 * 'timeout' is given in us. Note also that timeout handling is not exact --
 * A8kI2cLockedStart() total wait can be more than 2 x timeout
 * (A8kI2cPollCtrl() is called twice). 'mask' can be either TWSI_STATUS_START
 * or TWSI_STATUS_RPTD_START
 */
STATIC
EFI_STATUS
A8kI2cLockedStart (
  IN I2C_MASTER_CONTEXT *I2cMasterContext,
  IN INT32 mask,
  IN UINT8 slave,
  IN UINTN timeout
  )
{
  UINTN read_access, iflg_set = 0;
  UINT32 status;

  if (mask == TWSI_STATUS_RPTD_START)
    /* read IFLG to know if it should be cleared later; from NBSD */
    iflg_set = TWSI_READ(I2cMasterContext, TWSI_CONTROL) & TWSI_CONTROL_IFLG;

  A8kI2cControlSet(I2cMasterContext, TWSI_CONTROL_START);

  if (mask == TWSI_STATUS_RPTD_START && iflg_set) {
    DEBUG((DEBUG_INFO, "IFLG set, clearing\n"));
    A8kI2cClearIflg(I2cMasterContext);
  }

  /*
   * Without this delay we timeout checking IFLG if the timeout is 0.
   * NBSD driver always waits here too.
   */
  gBS->Stall(1000);

  if (A8kI2cPollCtrl(I2cMasterContext, timeout, TWSI_CONTROL_IFLG)) {
    DEBUG((DEBUG_ERROR, "timeout sending %sSTART condition\n",
        mask == TWSI_STATUS_START ? "" : "repeated "));
    return EFI_NO_RESPONSE;
  }

  status = TWSI_READ(I2cMasterContext, TWSI_STATUS);
  if (status != mask) {
    DEBUG((DEBUG_ERROR, "wrong status (%02x) after sending %sSTART condition\n",
        status, mask == TWSI_STATUS_START ? "" : "repeated "));
    return EFI_DEVICE_ERROR;
  }

  TWSI_WRITE(I2cMasterContext, TWSI_DATA, slave);
  gBS->Stall(1000);
  A8kI2cClearIflg(I2cMasterContext);

  if (A8kI2cPollCtrl(I2cMasterContext, timeout, TWSI_CONTROL_IFLG)) {
    DEBUG((DEBUG_ERROR, "timeout sending slave address\n"));
    return EFI_NO_RESPONSE;
  }

  read_access = (slave & 0x1) ? 1 : 0;
  status = TWSI_READ(I2cMasterContext, TWSI_STATUS);
  if (status != (read_access ?
      TWSI_STATUS_ADDR_R_ACK : TWSI_STATUS_ADDR_W_ACK)) {
    DEBUG((DEBUG_ERROR, "no ACK (status: %02x) after sending slave address\n",
        status));
    return EFI_NO_RESPONSE;
  }

  return EFI_SUCCESS;
}

#define  ABSSUB(a,b)  (((a) > (b)) ? (a) - (b) : (b) - (a))
STATIC
VOID
A8kI2cCalBaudRate (
  IN CONST UINT32 target,
  IN OUT A8K_I2C_BAUD_RATE *rate,
  UINT32 clk
  )
{
  UINT32 cur, diff, diff0;
  UINTN m, n, m0, n0;

  /* Calculate baud rate. */
  m0 = n0 = 4;  /* Default values on reset */
  diff0 = 0xffffffff;

  for (n = 0; n < 8; n++) {
    for (m = 0; m < 16; m++) {
      cur = TWSI_BAUD_RATE_RAW(clk,m,n);
      diff = ABSSUB(target, cur);
      if (diff < diff0) {
        m0 = m;
        n0 = n;
        diff0 = diff;
      }
    }
  }
  rate->raw = TWSI_BAUD_RATE_RAW(clk, m0, n0);
  rate->param = TWSI_BAUD_RATE_PARAM(m0, n0);
  rate->m = m0;
  rate->n = n0;
}

EFI_STATUS
EFIAPI
A8kI2cReset (
  IN CONST EFI_I2C_MASTER_PROTOCOL *This
  )
{
  UINT32 param;
  I2C_MASTER_CONTEXT *I2cMasterContext = I2C_SC_FROM_MASTER(This);

  param = baud_rate[IIC_FAST].param;

  EfiAcquireLock (&I2cMasterContext->Lock);
  TWSI_WRITE(I2cMasterContext, TWSI_SOFT_RESET, 0x0);
  gBS->Stall(2000);
  TWSI_WRITE(I2cMasterContext, TWSI_BAUD_RATE, param);
  TWSI_WRITE(I2cMasterContext, TWSI_CONTROL, TWSI_CONTROL_TWSIEN | TWSI_CONTROL_ACK);
  gBS->Stall(1000);
  EfiReleaseLock (&I2cMasterContext->Lock);

  return EFI_SUCCESS;
}

/*
 * timeout is given in us
 */
STATIC
EFI_STATUS
A8kI2cRepeatedStart (
  IN I2C_MASTER_CONTEXT *I2cMasterContext,
  IN UINT8 slave,
  IN UINTN timeout
  )
{
  EFI_STATUS Status;

  EfiAcquireLock (&I2cMasterContext->Lock);
  Status = A8kI2cLockedStart(I2cMasterContext, TWSI_STATUS_RPTD_START, slave,
      timeout);
  EfiReleaseLock (&I2cMasterContext->Lock);

  if (EFI_ERROR(Status)) {
    A8kI2cStop(I2cMasterContext);
  }
  return Status;
}

/*
 * timeout is given in us
 */
STATIC
EFI_STATUS
A8kI2cStart (
  IN I2C_MASTER_CONTEXT *I2cMasterContext,
  IN UINT8 slave,
  IN UINTN timeout
  )
{
  EFI_STATUS Status;

  EfiAcquireLock (&I2cMasterContext->Lock);
  Status = A8kI2cLockedStart(I2cMasterContext, TWSI_STATUS_START, slave, timeout);
  EfiReleaseLock (&I2cMasterContext->Lock);

  if (EFI_ERROR(Status)) {
    A8kI2cStop(I2cMasterContext);
  }
  return Status;
}

STATIC
EFI_STATUS
A8kI2cStop (
  IN I2C_MASTER_CONTEXT *I2cMasterContext
  )
{

  EfiAcquireLock (&I2cMasterContext->Lock);
  A8kI2cControlSet(I2cMasterContext, TWSI_CONTROL_STOP);
  gBS->Stall(1000);
  A8kI2cClearIflg(I2cMasterContext);
  EfiReleaseLock (&I2cMasterContext->Lock);

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
A8kI2cRead (
  IN I2C_MASTER_CONTEXT *I2cMasterContext,
  IN OUT UINT8 *buf,
  IN UINTN len,
  IN OUT UINTN *read,
  IN UINTN last,
  IN UINTN delay
  )
{
  UINT32 status;
  UINTN last_byte;
  EFI_STATUS Status;

  EfiAcquireLock (&I2cMasterContext->Lock);
  *read = 0;
  while (*read < len) {
    /*
     * Check if we are reading last byte of the last buffer,
     * do not send ACK then, per I2C specs
     */
    last_byte = ((*read == len - 1) && last) ? 1 : 0;
    if (last_byte)
      A8kI2cControlClear(I2cMasterContext, TWSI_CONTROL_ACK);
    else
      A8kI2cControlSet(I2cMasterContext, TWSI_CONTROL_ACK);

    gBS->Stall (1000);
    A8kI2cClearIflg(I2cMasterContext);

    if (A8kI2cPollCtrl(I2cMasterContext, delay, TWSI_CONTROL_IFLG)) {
      DEBUG((DEBUG_ERROR, "timeout reading data\n"));
      Status = EFI_NO_RESPONSE;
      goto out;
    }

    status = TWSI_READ(I2cMasterContext, TWSI_STATUS);
    if (status != (last_byte ?
        TWSI_STATUS_DATA_RD_NOACK : TWSI_STATUS_DATA_RD_ACK)) {
      DEBUG((DEBUG_ERROR, "wrong status (%02x) while reading\n", status));
      Status = EFI_DEVICE_ERROR;
      goto out;
    }

    *buf++ = TWSI_READ(I2cMasterContext, TWSI_DATA);
    (*read)++;
  }
  Status = EFI_SUCCESS;
out:
  EfiReleaseLock (&I2cMasterContext->Lock);
  return (Status);
}

STATIC
EFI_STATUS
A8kI2cWrite (
  IN I2C_MASTER_CONTEXT *I2cMasterContext,
  IN OUT CONST UINT8 *buf,
  IN UINTN len,
  IN OUT UINTN *sent,
  IN UINTN timeout
  )
{
  UINT32 status;
  EFI_STATUS Status;

  EfiAcquireLock (&I2cMasterContext->Lock);
  *sent = 0;
  while (*sent < len) {
    TWSI_WRITE(I2cMasterContext, TWSI_DATA, *buf++);

    A8kI2cClearIflg(I2cMasterContext);
    if (A8kI2cPollCtrl(I2cMasterContext, timeout, TWSI_CONTROL_IFLG)) {
      DEBUG((DEBUG_ERROR, "timeout writing data\n"));
      Status = EFI_NO_RESPONSE;
      goto out;
    }

    status = TWSI_READ(I2cMasterContext, TWSI_STATUS);
    if (status != TWSI_STATUS_DATA_WR_ACK) {
      DEBUG((DEBUG_ERROR, "wrong status (%02x) while writing\n", status));
      Status = EFI_DEVICE_ERROR;
      goto out;
    }
    (*sent)++;
  }
  Status = EFI_SUCCESS;
out:
  EfiReleaseLock (&I2cMasterContext->Lock);
  return (Status);
}

/*
 * A8kI2cStartRequest should be called only by I2cHost.
 * I2C device drivers ought to use EFI_I2C_IO_PROTOCOL instead.
 */
STATIC
EFI_STATUS
A8kI2cStartRequest (
  IN CONST EFI_I2C_MASTER_PROTOCOL *This,
  IN UINTN                         SlaveAddress,
  IN EFI_I2C_REQUEST_PACKET        *RequestPacket,
  IN EFI_EVENT                     Event      OPTIONAL,
  OUT EFI_STATUS                   *I2cStatus OPTIONAL
  )
{
  UINTN Count;
  UINTN ReadMode;
  UINTN Transmitted;
  I2C_MASTER_CONTEXT *I2cMasterContext = I2C_SC_FROM_MASTER(This);
  EFI_I2C_OPERATION *Operation;

  ASSERT (RequestPacket != NULL);
  ASSERT (I2cMasterContext != NULL);

  for (Count = 0; Count < RequestPacket->OperationCount; Count++) {
    Operation = &RequestPacket->Operation[Count];
    ReadMode = Operation->Flags & I2C_FLAG_READ;

    if (Count == 0) {
      A8kI2cStart ( I2cMasterContext,
		    (SlaveAddress << 1) | ReadMode,
		    TWSI_TRANSFER_TIMEOUT
	          );
    } else if (!(Operation->Flags & I2C_FLAG_NORESTART)) {
      A8kI2cRepeatedStart ( I2cMasterContext,
		            (SlaveAddress << 1) | ReadMode,
			    TWSI_TRANSFER_TIMEOUT
		          );
    }

    if (ReadMode) {
      A8kI2cRead ( I2cMasterContext,
		   Operation->Buffer,
		   Operation->LengthInBytes,
		   &Transmitted,
		   Count == 1,
		   TWSI_TRANSFER_TIMEOUT
		  );
    } else {
      A8kI2cWrite ( I2cMasterContext,
		   Operation->Buffer,
		   Operation->LengthInBytes,
		   &Transmitted,
		   TWSI_TRANSFER_TIMEOUT
		  );
    }
    if (Count == RequestPacket->OperationCount - 1) {
      A8kI2cStop ( I2cMasterContext );
    }
  }

  if (I2cStatus != NULL)
    I2cStatus = EFI_SUCCESS;
  if (Event != NULL)
    gBS->SignalEvent(Event);
  return EFI_SUCCESS;
}

/*
 * I2C_GUID is embedded in EFI_I2C_DEVICE structure, with last byte set to
 * address of device on I2C bus. Device driver should compare its GUID with
 * offered one in Supported() function.
 */
#define I2C_GUID \
  { \
  0x391fc679, 0x6cb0, 0x4f01, { 0x9a, 0xc7, 0x8e, 0x1b, 0x78, 0x6b, 0x7a, 0x00 } \
  }

STATIC
EFI_STATUS
A8kI2cAllocDevice (
  IN UINT8 SlaveAddress,
  IN OUT CONST EFI_I2C_DEVICE **Device
  )
{
  EFI_STATUS Status;
  EFI_I2C_DEVICE *Dev;
  UINT32 *TmpSlaveArray;
  EFI_GUID DevGuid = I2C_GUID;
  EFI_GUID *TmpGuidP;

  DevGuid.Data4[7] = SlaveAddress;

  Status = gBS->AllocatePool ( EfiBootServicesData,
             sizeof(EFI_I2C_DEVICE),
             (VOID **) &Dev );
  if (EFI_ERROR(Status)) {
    DEBUG((DEBUG_ERROR, "allocate pool fail\n"));
    return Status;
  }
  *Device = Dev;
  Dev->DeviceIndex = SlaveAddress;
  Dev->SlaveAddressCount = 1;
  Dev->I2cBusConfiguration = 0;
  Status = gBS->AllocatePool ( EfiBootServicesData,
             sizeof(UINT32),
             (VOID **) &TmpSlaveArray);
  if (EFI_ERROR(Status)) {
    goto fail1;
  }
  TmpSlaveArray[0] = SlaveAddress;
  Dev->SlaveAddressArray = TmpSlaveArray;

  Status = gBS->AllocatePool ( EfiBootServicesData,
             sizeof(EFI_GUID),
             (VOID **) &TmpGuidP);
  if (EFI_ERROR(Status)) {
    goto fail2;
  }
  *TmpGuidP = DevGuid;
  Dev->DeviceGuid = TmpGuidP;

  DEBUG((DEBUG_INFO, "A8kI2c: allocated device with address %x\n", (UINTN)SlaveAddress));
  return EFI_SUCCESS;

fail2:
  FreePool(TmpSlaveArray);
fail1:
  FreePool(Dev);

  return Status;
}

/*
 * It is called by I2cBus to enumerate devices on I2C bus. In this case,
 * enumeration is based on PCD configuration - all slave addresses specified
 * in PCD get their corresponding EFI_I2C_DEVICE structures here.
 *
 * After enumeration succeeds, Supported() function of drivers that installed
 * DriverBinding protocol is called.
 */
STATIC
EFI_STATUS
EFIAPI
A8kI2cEnumerate (
  IN CONST EFI_I2C_ENUMERATE_PROTOCOL *This,
  IN OUT CONST EFI_I2C_DEVICE         **Device
  )
{
  UINT8 *DevicesPcd;
  UINTN Index;
  UINT8 NextDeviceAddress;

  DevicesPcd = PcdGetPtr (PcdI2cSlaveAddresses);
  if (*Device == NULL) {
    if (DevicesPcd[0] != '\0')
      A8kI2cAllocDevice (DevicesPcd[0], Device);
    return EFI_SUCCESS;
  } else {
    for (Index = 0; DevicesPcd[Index] != '\0'; Index++) {
      if (DevicesPcd[Index] == (*Device)->DeviceIndex) {
  NextDeviceAddress = DevicesPcd[Index + 1];
  if (NextDeviceAddress != '\0') {
    A8kI2cAllocDevice(NextDeviceAddress, Device);
    return EFI_SUCCESS;
  }
  break;
      }
    }
    *Device = NULL;
    return EFI_SUCCESS;
  }
}

STATIC
EFI_STATUS
EFIAPI
A8kI2cEnableConf (
  IN CONST EFI_I2C_BUS_CONFIGURATION_MANAGEMENT_PROTOCOL *This,
  IN UINTN                                               I2cBusConfiguration,
  IN EFI_EVENT                                           Event      OPTIONAL,
  IN EFI_STATUS                                          *I2cStatus OPTIONAL
  )
{
  if (I2cStatus != NULL)
    I2cStatus = EFI_SUCCESS;
  if (Event != NULL)
    gBS->SignalEvent(Event);
  return EFI_SUCCESS;
}
