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
#include <ShellBase.h>
#include <Uefi.h>

#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/FileHandleLib.h>
#include <Library/HiiLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PrintLib.h>
#include <Library/ShellCEntryLib.h>
#include <Library/ShellCommandLib.h>
#include <Library/ShellLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>

#include <Protocol/Mdio.h>

#define CMD_NAME_STRING       L"mdio"

// Marvell soho switch Registers and MDIO address
// Global Register 1 defines:
#define MARVELL_SOHO_SW_GREG1_ADDR            (0x1B)
// Status Operation Register
#define MARVELL_SOHO_SW_GREG1_STS_OP_REG      (0x1D)
// Status Busy Offset
#define MARVELL_SOHO_SW_GREG1_BUSY_OFF        (15)
// Status Operation Offset
#define MARVELL_SOHO_SW_GREG1_STS_OP_OFF      (12)
// Read Counter
#define MARVELL_SOHO_SW_GREG1_RD_CNT          (4)
// Flush Counter per port
#define MARVELL_SOHO_SW_GREG1_FLUSH_CNT       (2)
// Histogram Mode Offset
#define MARVELL_SOHO_SW_GREG1_HISTO_MODE_OFF  (10)
// Bidirection
#define MARVELL_SOHO_SW_GREG1_BI_DIR          (3)
// Status Port Offset
#define MARVELL_SOHO_SW_GREG1_STS_PORT_OFF    (5)

// Status Counter Register 3 & 2
#define MARVELL_SOHO_SW_GREG1_STS_CNT1_REG    (0x1E)
// Status Counter Register 1 & 0
#define MARVELL_SOHO_SW_GREG1_STS_CNT0_REG    (0x1F)

// Marvell soho switch Counter ID:
#define IN_GOOD_OCTETS_LOW   (0x0)
#define IN_GOOD_OCTETS_HIGH  (0x1)
#define IN_BAD_OCTETS        (0x2)
#define IN_UNICAST           (0x4)
#define IN_BROADCAST         (0x6)
#define IN_MULTICAST         (0x7)
#define IN_PAUSE             (0x16)
#define IN_UNDERSIZE         (0x18)
#define IN_FRAGMENT          (0x19)
#define IN_OVERSIZE          (0x1A)
#define IN_JABBER            (0x1B)
#define IN_RXERR             (0x1C)
#define IN_FCSERR            (0x1D)

#define OUT_OCTETSLOW        (0xE)
#define OUT_OCTETSHIGH       (0xF)
#define OUT_UNICAST          (0x10)
#define OUT_BROADCAST        (0x13)
#define OUT_MULTICAST        (0x12)
#define OUT_PAUSE            (0x15)
#define COLLISIONS           (0x1E)
#define DEFERRED             (0x5)
#define SINGLE               (0x14)
#define MULTIPLE             (0x17)
#define EXCESSIVE            (0x11)
#define OUTFCSERR            (0x03)
#define LATE                 (0x1F)

STATIC MARVELL_MDIO_PROTOCOL *MdioProtocol;

STATIC CONST CHAR16 gShellMdioFileName[] = L"ShellCommands";
STATIC EFI_HANDLE gShellMdioHiiHandle = NULL;

STATIC CONST SHELL_PARAM_ITEM ParamList[] = {
  {L"read", TypeFlag},
  {L"write", TypeFlag},
  {L"counter", TypeFlag},
  {L"help", TypeFlag},
  {NULL , TypeMax}
  };

STATIC
SHELL_STATUS
EFIAPI
MdioReadCounter (
  IN UINT32        BusNo,
  IN UINT32        Port,
  IN UINT8         CounterId,
  OUT UINT32       *Data
  )
{
  SHELL_STATUS Status;
  UINT32       Val;
  UINT32       Data1, Data2;

  while(1) {
    Status = MdioProtocol->Read (MdioProtocol,
                                 MARVELL_SOHO_SW_GREG1_ADDR,
                                 BusNo,
                                 MARVELL_SOHO_SW_GREG1_STS_OP_REG,
                                 &Val);
    // Wait for Status Busy bit to be cleared.
    if (!(Val >> MARVELL_SOHO_SW_GREG1_BUSY_OFF))
      break;
  }

  Val = (1 << MARVELL_SOHO_SW_GREG1_BUSY_OFF) |
        (MARVELL_SOHO_SW_GREG1_RD_CNT << MARVELL_SOHO_SW_GREG1_STS_OP_OFF) |
        (MARVELL_SOHO_SW_GREG1_BI_DIR << MARVELL_SOHO_SW_GREG1_HISTO_MODE_OFF) |
        ((Port + 1) << MARVELL_SOHO_SW_GREG1_STS_PORT_OFF) |
         CounterId;
  Status = MdioProtocol->Write (MdioProtocol,
                                MARVELL_SOHO_SW_GREG1_ADDR,
                                BusNo,
                                MARVELL_SOHO_SW_GREG1_STS_OP_REG,
                                Val);

  if (EFI_ERROR (Status)) {
    Print (L"%s: Returned failure %d\n", CMD_NAME_STRING, Status);
    return SHELL_ABORTED;
  } else {
    Status = MdioProtocol->Read (MdioProtocol,
                                 MARVELL_SOHO_SW_GREG1_ADDR,
                                 BusNo,
                                 MARVELL_SOHO_SW_GREG1_STS_CNT1_REG,
                                 &Data1);
    Status = MdioProtocol->Read (MdioProtocol,
                                 MARVELL_SOHO_SW_GREG1_ADDR,
                                 BusNo,
                                 MARVELL_SOHO_SW_GREG1_STS_CNT0_REG,
                                 &Data2);

    *Data = ((Data1 & 0xFF00) << 24) | ((Data1 & 0xFF) << 16) | ((Data2 & 0xFF00) << 8) | (Data2 & 0xFF);
  }
  return EFI_SUCCESS;
}

/**
  Return the file name of the help text file if not using HII.

  @return The string pointer to the file name.
**/
STATIC
CONST CHAR16*
EFIAPI
ShellCommandGetManFileNameMdio (
  VOID
  )
{
  return gShellMdioFileName;
}

STATIC
VOID
MdioUsage (
  VOID
  )
{
  Print (L"\nMdio Bus Access command\n"
         "mdio read <BusNo> <Address> <Reg>\n\n"
         "mdio write <BusNo> <Address> <Reg> <Value>\n\n"
         "mdio counter <BusNo> <Address>, optional, only "
         "to read counter from Marvell SOHO Switch\n\n"
  );
}

STATIC
SHELL_STATUS
EFIAPI
ShellCommandRunMdio (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  CHAR16                  *ProblemParam;
  LIST_ENTRY              *CheckPackage;
  EFI_STATUS              Status;

  // Locate MDIO protocols
  Status = gBS->LocateProtocol (
                  &gMarvellMdioProtocolGuid,
                  NULL,
                  (VOID **)&MdioProtocol
                  );

  if (EFI_ERROR(Status)) {
    Print (L"%s: Cannot locate MDIO protocol\n", CMD_NAME_STRING);
    return SHELL_ABORTED;
  }


  // Parse command line
  Status = ShellInitialize ();
  if (EFI_ERROR (Status)) {
    Print (L"%s: Error while initializing Shell\n", CMD_NAME_STRING);
    ASSERT_EFI_ERROR (Status);
    return SHELL_ABORTED;
  }

  Status = ShellCommandLineParse (ParamList, &CheckPackage, &ProblemParam, TRUE);
  if (EFI_ERROR (Status)) {
    Print (L"%s: Invalid parameter\n", CMD_NAME_STRING);
    return SHELL_ABORTED;
  }

  if (ShellCommandLineGetFlag (CheckPackage, L"help")) {
    MdioUsage();
    return EFI_SUCCESS;
  }

  if (ShellCommandLineGetFlag (CheckPackage, L"read")) {
    CONST CHAR16* BusNoStr;
    CONST CHAR16* AddrStr;
    CONST CHAR16* RegStr;
    UINT32 BusNo, Addr, Reg;
    UINT32 Data;

    BusNoStr = ShellCommandLineGetRawValue (CheckPackage, 1);
    AddrStr = ShellCommandLineGetRawValue (CheckPackage, 2);
    RegStr = ShellCommandLineGetRawValue (CheckPackage, 3);

    BusNo = ShellHexStrToUintn (BusNoStr);
    Addr = ShellHexStrToUintn (AddrStr);
    Reg = ShellHexStrToUintn (RegStr);

    Status = MdioProtocol->Read (MdioProtocol, Addr, BusNo, Reg, &Data);
    if (EFI_ERROR (Status)) {
        Print (L"%s: Returned failure %d\n", CMD_NAME_STRING, Status);
        return SHELL_ABORTED;
    } else {
        Print (L"%s: Read Bus %d Addr 0x%x Reg 0x%x Val 0x%x\n",
               CMD_NAME_STRING, BusNo, Addr, Reg, Data);
    }

  } else if (ShellCommandLineGetFlag (CheckPackage, L"write")) {
      CONST CHAR16* BusNoStr;
      CONST CHAR16* AddrStr;
      CONST CHAR16* RegStr;
      CONST CHAR16* DataStr;
      UINT32 BusNo, Addr, Reg;
      UINT32 Data;

      BusNoStr = ShellCommandLineGetRawValue (CheckPackage, 1);
      AddrStr = ShellCommandLineGetRawValue (CheckPackage, 2);
      RegStr = ShellCommandLineGetRawValue (CheckPackage, 3);
      DataStr = ShellCommandLineGetRawValue (CheckPackage, 4);

      BusNo = ShellHexStrToUintn (BusNoStr);
      Addr = ShellHexStrToUintn (AddrStr);
      Reg = ShellHexStrToUintn (RegStr);
      Data = ShellHexStrToUintn (DataStr);

      Status = MdioProtocol->Write (MdioProtocol, Addr, BusNo, Reg, Data);
      if (EFI_ERROR (Status)) {
          Print (L"%s: Returned failure %d\n", CMD_NAME_STRING, Status);
          return SHELL_ABORTED;
      } else {
          Print (L"%s: Write Bus %d Addr 0x%x Reg 0x%x Val 0x%x OK\n",
                 CMD_NAME_STRING, BusNo, Addr, Reg, Data);
      }

  } else if (ShellCommandLineGetFlag (CheckPackage, L"counter")) {
#if (PcdGet8 (PcdMdioReadCounterEnabled) == 0)
    Print (L"%s: Reading device counter not available\n", CMD_NAME_STRING);
    return SHELL_UNSUPPORTED;
#endif
    CONST CHAR16* BusNoStr;
    CONST CHAR16* AddrStr;
    UINT32 BusNo, Addr;
    UINT32 Data;

    BusNoStr = ShellCommandLineGetRawValue (CheckPackage, 1);
    AddrStr = ShellCommandLineGetRawValue (CheckPackage, 2);

    BusNo = ShellHexStrToUintn (BusNoStr);
    Addr = ShellHexStrToUintn (AddrStr) - 0x10;

    // Ingress Counters
    MdioReadCounter (BusNo, Addr, IN_GOOD_OCTETS_LOW, &Data);
    Print (L"InGoodOctetsLow %d\n", Data);
    MdioReadCounter (BusNo, Addr, IN_GOOD_OCTETS_HIGH, &Data);
    Print (L"InGoodOctetsHigh %d\n", Data);
    MdioReadCounter (BusNo, Addr, IN_BAD_OCTETS, &Data);
    Print (L"InBadOctets %d\n", Data);
    MdioReadCounter (BusNo, Addr, IN_UNICAST, &Data);
    Print (L"InUnicast %d\n", Data);
    MdioReadCounter (BusNo, Addr, IN_BROADCAST, &Data);
    Print (L"InBroadcast %d\n", Data);
    MdioReadCounter (BusNo, Addr, IN_MULTICAST, &Data);
    Print (L"InMulticast %d\n", Data);
    MdioReadCounter (BusNo, Addr, IN_PAUSE, &Data);
    Print (L"InPause %d\n", Data);
    MdioReadCounter (BusNo, Addr, IN_UNDERSIZE, &Data);
    Print (L"InUnderSize %d\n", Data);
    MdioReadCounter (BusNo, Addr, IN_FRAGMENT, &Data);
    Print (L"InFragment %d\n", Data);
    MdioReadCounter (BusNo, Addr, IN_OVERSIZE, &Data);
    Print (L"InOverSize %d\n", Data);
    MdioReadCounter (BusNo, Addr, IN_JABBER, &Data);
    Print (L"InJabber %d\n", Data);
    MdioReadCounter (BusNo, Addr, IN_RXERR, &Data);
    Print (L"InRxErr %d\n", Data);
    MdioReadCounter (BusNo, Addr, IN_FCSERR, &Data);
    Print (L"InFCSErr %d\n", Data);

    // Egress Counters
    MdioReadCounter (BusNo, Addr, OUT_OCTETSLOW, &Data);
    Print (L"OutOctetsLow %d\n", Data);
    MdioReadCounter (BusNo, Addr, OUT_OCTETSHIGH, &Data);
    Print (L"OutOctetsHigh %d\n", Data);
    MdioReadCounter (BusNo, Addr, OUT_UNICAST, &Data);
    Print (L"OutUnicast %d\n", Data);
    MdioReadCounter (BusNo, Addr, OUT_BROADCAST, &Data);
    Print (L"OutBroadcast %d\n", Data);
    MdioReadCounter (BusNo, Addr, OUT_MULTICAST, &Data);
    Print (L"OutMulticast %d\n", Data);
    MdioReadCounter (BusNo, Addr, OUT_PAUSE, &Data);
    Print (L"OutPause %d\n", Data);
    MdioReadCounter (BusNo, Addr, COLLISIONS, &Data);
    Print (L"Collisions %d\n", Data);
    MdioReadCounter (BusNo, Addr, DEFERRED, &Data);
    Print (L"Deferred %d\n", Data);
    MdioReadCounter (BusNo, Addr, SINGLE, &Data);
    Print (L"Single %d\n", Data);
    MdioReadCounter (BusNo, Addr, MULTIPLE, &Data);
    Print (L"Multiple %d\n", Data);
    MdioReadCounter (BusNo, Addr, EXCESSIVE, &Data);
    Print (L"Excessive %d\n", Data);
    MdioReadCounter (BusNo, Addr, OUTFCSERR, &Data);
    Print (L"OutFCSErr %d\n", Data);
    MdioReadCounter (BusNo, Addr, LATE, &Data);
    Print (L"Late %d\n", Data);

    while (1) {
      Status = MdioProtocol->Read (MdioProtocol,
                                   MARVELL_SOHO_SW_GREG1_ADDR,
                                   BusNo,
                                   MARVELL_SOHO_SW_GREG1_STS_OP_REG,
                                   &Data);
      // Wait for Status Busy bit to be cleared.
      if (!(Data >> MARVELL_SOHO_SW_GREG1_BUSY_OFF))
        break;
    }

    Data = (1 << MARVELL_SOHO_SW_GREG1_BUSY_OFF) |
           (MARVELL_SOHO_SW_GREG1_FLUSH_CNT << MARVELL_SOHO_SW_GREG1_STS_OP_OFF) |
           (MARVELL_SOHO_SW_GREG1_BI_DIR << MARVELL_SOHO_SW_GREG1_HISTO_MODE_OFF) |
           ((Addr + 1) << MARVELL_SOHO_SW_GREG1_STS_PORT_OFF) |
           0;
    Status = MdioProtocol->Write (MdioProtocol,
                                  MARVELL_SOHO_SW_GREG1_ADDR,
                                  BusNo,
                                  MARVELL_SOHO_SW_GREG1_STS_OP_REG,
                                  Data);
  } else {
    Print (L"%s: Invalid parameter\n", CMD_NAME_STRING);
    return SHELL_ABORTED;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
ShellMdioCommandConstructor (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS Status;

  gShellMdioHiiHandle = NULL;

  gShellMdioHiiHandle = HiiAddPackages (
                          &gShellMdioHiiGuid,
                          gImageHandle,
                          UefiShellMdioCommandLibStrings,
                          NULL
                          );

  if (gShellMdioHiiHandle == NULL) {
    Print (L"%s: Cannot add Hii package\n", CMD_NAME_STRING);
    return EFI_DEVICE_ERROR;
  }

  Status = ShellCommandRegisterCommandName (
                           CMD_NAME_STRING,
                           ShellCommandRunMdio,
                           ShellCommandGetManFileNameMdio,
                           0,
                           CMD_NAME_STRING,
                           TRUE,
                           gShellMdioHiiHandle,
                           STRING_TOKEN (STR_GET_HELP_MDIO)
                           );

  if (EFI_ERROR(Status)) {
    Print (L"%s: Error while registering command\n", CMD_NAME_STRING);
    return SHELL_ABORTED;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
ShellMdioCommandDestructor (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  if (gShellMdioHiiHandle != NULL) {
    HiiRemovePackages (gShellMdioHiiHandle);
  }

  return EFI_SUCCESS;
}
