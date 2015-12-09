/** @file
*
*  Copyright (c) 2015, Hisilicon Limited. All rights reserved.
*  Copyright (c) 2015, Linaro Limited. All rights reserved.
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#include "UpdateSmbiosType9.h"

/*++

Routine Description:

  Calculate the size of all the strings.

Arguments:

  IN UINT8            *PtrEndOftheData - The Ptr of the end SMBIOS data. (Also the start Ptr of the strings)

Returns:

  0 - When an error occured.
  The total size of all the strings.

--*/
UINTN
GetSmbiosTypeStringsSize (
  IN UINT8            *PtrEndOftheData
)
{
    UINT8               *Temp;
    UINT8               *PtrEndOfString;
    UINT8               Index;

    Temp = PtrEndOftheData;
    PtrEndOfString = NULL;

    for (Index = 0; Index < MAX_SMBIOS_STRING_NUMBER; Index++) {
        //
        // A string set will be ended by a double NULL. (0x0000)
        //
        PtrEndOftheData++;
        PtrEndOfString = PtrEndOftheData + 1;
        if ((*PtrEndOftheData == 0) && (*PtrEndOfString == 0)) {
            break;
        }
    }
    
    if (PtrEndOfString - Temp >= MAX_SMBIOS_STRING_NUMBER) {
        DEBUG ((EFI_D_ERROR, "SMBIOS Calculate the size of the Strings Failed.\n"));
        return 0;
    }
    return PtrEndOfString - Temp;
}

/*++

  更新SMBIOS TYPE9信息。
  
  @param  Type9Record               SMBIOS TYPE9信息。
  
--*/
VOID 
EFIAPI 
UpdateSmbiosType9Info( 
  IN OUT SMBIOS_TABLE_TYPE9             *Type9Record
)
{
    EFI_STATUS                         Status;
    UINTN                              HandleIndex;
    EFI_HANDLE                        *HandleBuffer;
    UINTN                              HandleCount;
    EFI_PCI_IO_PROTOCOL               *PciIo;
    UINTN                              SegmentNumber;
    UINTN                              BusNumber;
    UINTN                              DeviceNumber;
    UINTN                              FunctionNumber;
    
    Status = gBS->LocateHandleBuffer (
          ByProtocol,
          &gEfiPciIoProtocolGuid,
          NULL,
          &HandleCount,
          &HandleBuffer
          );    
    if(EFI_ERROR(Status)) {
        DEBUG((EFI_D_ERROR, " Locate gEfiPciIoProtocol Failed.\n"));           
        gBS->FreePool ((VOID *)HandleBuffer);
        return;
    }

    for (HandleIndex = 0; HandleIndex < HandleCount; HandleIndex++) {
        Status = gBS->HandleProtocol (
            HandleBuffer[HandleIndex],
            &gEfiPciIoProtocolGuid,
            (VOID **)&PciIo
            );
        if (EFI_ERROR (Status)) {
            DEBUG((EFI_D_ERROR, "[%a]:[%dL] Status : %r\n", __FUNCTION__, __LINE__, Status));
            
            continue;
        }

        (VOID)PciIo->GetLocation(PciIo, &SegmentNumber, &BusNumber, &DeviceNumber, &FunctionNumber);
        
        if((BusNumber > Type9Record->BusNum) && (BusNumber < Type9Record->BusNum + 0x40)) {
            DEBUG((EFI_D_ERROR,"PCIe device plot in slot bdf %d %d %d\r\n",BusNumber,DeviceNumber,FunctionNumber));
            Type9Record->SegmentGroupNum   = SegmentNumber;
            Type9Record->BusNum            = BusNumber;
            Type9Record->DevFuncNum        = (DeviceNumber << 3) | FunctionNumber;

            Type9Record->CurrentUsage      = SlotUsageInUse;
            
            break;
        }          
    }

    gBS->FreePool ((VOID *)HandleBuffer);
    
    return;
}


STATIC
VOID
UpdateSmbiosType9Event (
  IN EFI_EVENT                      Event,
  IN VOID                           *Cotext
  )
{
    EFI_STATUS                        Status;
    EFI_SMBIOS_TYPE                   SmbiosType;
    EFI_SMBIOS_HANDLE                 SmbiosHandle;
    EFI_SMBIOS_PROTOCOL               *Smbios;
    EFI_SMBIOS_TABLE_HEADER           *Record;
    SMBIOS_TABLE_TYPE9                *Type9Record;
    SMBIOS_TABLE_TYPE9_WITH_STRING    TempType9Data;
    UINTN                             SmbiosType9Size;
    UINT8                             *MemoryIndex;   

    Status = gBS->LocateProtocol (
                  &gEfiSmbiosProtocolGuid,
                  NULL,
                  (VOID **) &Smbios
                  );    
    if (EFI_ERROR (Status)) {
        DEBUG((EFI_D_ERROR, "[%a]:[%dL] LocateProtocol Failed. Status : %r\n", __FUNCTION__, __LINE__, Status));

        goto Exit;   
    }  

    SmbiosHandle = SMBIOS_HANDLE_PI_RESERVED;
    SmbiosType = EFI_SMBIOS_TYPE_SYSTEM_SLOTS;
    Status = Smbios->GetNext (Smbios, &SmbiosHandle, &SmbiosType, &Record, NULL);
    if (EFI_ERROR(Status)) {
        //DEBUG((EFI_D_ERROR, "[%a]:[%dL] Get System Slot Failed. Status : %r\n", __FUNCTION__, __LINE__, Status));
        
        goto Exit;
    }

    Type9Record = (SMBIOS_TABLE_TYPE9 *) Record;   

    UpdateSmbiosType9Info (Type9Record);

    //
    // Calculate the size of the SMBIOS Type9. (The strings are included.)
    //
    MemoryIndex = (UINT8 *)(VOID*)Type9Record;
    SmbiosType9Size = sizeof(SMBIOS_TABLE_TYPE9) + GetSmbiosTypeStringsSize ((MemoryIndex + sizeof(SMBIOS_TABLE_TYPE9)));
    //
    // Save the SMBIOS Type9 Data temporarily.
    //
    gBS->CopyMem (&TempType9Data, Type9Record, SmbiosType9Size);
    
    //
    // Update SMBIOS Type9.
    //
    Status = Smbios->Remove (Smbios, SmbiosHandle);
    if (EFI_ERROR(Status)) {
        DEBUG((EFI_D_ERROR, "[%a]:[%dL] Smbios Remove Failed. Status : %r\n", __FUNCTION__, __LINE__, Status));        
        goto Exit;
    }
    
    Status = Smbios->Add (Smbios, NULL, &SmbiosHandle, (EFI_SMBIOS_TABLE_HEADER *)&TempType9Data);
    if (EFI_ERROR(Status)) {
        DEBUG((EFI_D_ERROR, "[%a]:[%dL] Smbios Add Failed. Status : %r\n", __FUNCTION__, __LINE__, Status));
        goto Exit;
    }
    
Exit:
    gBS->CloseEvent(Event);

    return;
}

EFI_STATUS
EFIAPI
UpdateSmbiosType9Entry (
  IN EFI_HANDLE            ImageHandle,
  IN EFI_SYSTEM_TABLE     *SystemTable
  )
{
    VOID    *Registration;
    
    (VOID)EfiCreateProtocolNotifyEvent (
                                 &gEfiPciIoProtocolGuid,
                                 TPL_CALLBACK,
                                 UpdateSmbiosType9Event,
                                 NULL,
                                 &Registration
                                 );
    return EFI_SUCCESS;
}

