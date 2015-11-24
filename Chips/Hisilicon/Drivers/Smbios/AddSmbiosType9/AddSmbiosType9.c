#include "AddSmbiosType9.h"

extern SMBIOS_TABLE_TYPE9 gPcieSlotInfo[];
extern UINT8 OemGetPcieSlotNumber ();

EFI_STATUS
EFIAPI
AddSmbiosType9Entry (
  IN EFI_HANDLE            ImageHandle,
  IN EFI_SYSTEM_TABLE     *SystemTable
  )
{
    EFI_STATUS                          Status;
    EFI_SMBIOS_TYPE                     SmbiosType;
    EFI_SMBIOS_HANDLE                   SmbiosHandle;
    EFI_SMBIOS_PROTOCOL                 *Smbios;
    EFI_SMBIOS_TABLE_HEADER             *Record;
    SMBIOS_TABLE_TYPE9                  *Type9Record;
    SMBIOS_TABLE_TYPE9                  *SmbiosRecord = NULL;
    CHAR8                              *OptionalStrStart;

    UINT8                               SmbiosAddType9Number;
    UINT8                               Index;

    CHAR16                              *SlotDesignation = NULL;
    UINTN                               SlotDesignationStrLen;  

    Status = gBS->LocateProtocol (
                  &gEfiSmbiosProtocolGuid,
                  NULL,
                  (VOID **) &Smbios
                  );    
    if (EFI_ERROR (Status)) {
        DEBUG((EFI_D_ERROR, "[%a]:[%dL] LocateProtocol Failed. Status : %r\n", __FUNCTION__, __LINE__, Status));
        return Status;   
    }  

    do {
        SmbiosHandle = SMBIOS_HANDLE_PI_RESERVED;
        SmbiosType = EFI_SMBIOS_TYPE_SYSTEM_SLOTS;
        Status = Smbios->GetNext (Smbios, &SmbiosHandle, &SmbiosType, &Record, NULL);
        if (!EFI_ERROR(Status)) {
            Status = Smbios->Remove (Smbios, SmbiosHandle);
            if (EFI_ERROR(Status)) {
                DEBUG((EFI_D_ERROR, "[%a]:[%dL] Remove System Slot Failed. Status : %r\n", __FUNCTION__, __LINE__, Status));
                break;   
            }
        }
    } while (SmbiosHandle != SMBIOS_HANDLE_PI_RESERVED);

    SmbiosAddType9Number = OemGetPcieSlotNumber();
    
    for (Index = 0; Index < SmbiosAddType9Number; Index++) 
    {
        if (gPcieSlotInfo[Index].Hdr.Type != EFI_SMBIOS_TYPE_SYSTEM_SLOTS)
        {
            continue;
        }
        
        Type9Record = &gPcieSlotInfo[Index];  

        SlotDesignation = AllocateZeroPool ((sizeof (CHAR16)) * SMBIOS_STRING_MAX_LENGTH);
        if (NULL == SlotDesignation)
        {
            Status = EFI_OUT_OF_RESOURCES;
            DEBUG((EFI_D_ERROR, "[%a]:[%dL] AllocateZeroPool Failed. Status : %r\n", __FUNCTION__, __LINE__, Status));
            
            goto Exit;
        }

        SlotDesignationStrLen = UnicodeSPrint (SlotDesignation, SMBIOS_STRING_MAX_LENGTH - 1, L"PCIE Slot%d", Type9Record->SlotID);

        //
        // Two zeros following the last string.
        //
        SmbiosRecord = AllocateZeroPool(sizeof (SMBIOS_TABLE_TYPE9) + SlotDesignationStrLen + 1 + 1);
        if(NULL == SmbiosRecord) 
        {
            Status = EFI_OUT_OF_RESOURCES;
            DEBUG((EFI_D_ERROR, "[%a]:[%dL] AllocateZeroPool Failed. Status : %r\n", __FUNCTION__, __LINE__, Status));
            
            goto Exit;
        }

        (VOID)memcpy_s(SmbiosRecord, sizeof (SMBIOS_TABLE_TYPE9), Type9Record, sizeof (SMBIOS_TABLE_TYPE9));

        SmbiosRecord->Hdr.Length = sizeof (SMBIOS_TABLE_TYPE9);

        OptionalStrStart = (CHAR8 *)(SmbiosRecord + 1);
        UnicodeStrToAsciiStr(SlotDesignation, OptionalStrStart);
        
        //
        // Now we have got the full smbios record, call smbios protocol to add this record.
        //
        SmbiosHandle = SMBIOS_HANDLE_PI_RESERVED;
        Status = Smbios->Add (Smbios, NULL, &SmbiosHandle, (EFI_SMBIOS_TABLE_HEADER *)SmbiosRecord);
        if(EFI_ERROR(Status))
        {
            DEBUG((EFI_D_ERROR, "[%a]:[%dL] Smbios Type09 Table Log Failed! %r \n", __FUNCTION__, __LINE__, Status)); 
            goto Exit;
        }

        FreePool(SmbiosRecord);
        FreePool(SlotDesignation);
    }

    return EFI_SUCCESS;    

Exit:
    if(SmbiosRecord != NULL)
    {
        FreePool(SmbiosRecord);
    }

    if(SlotDesignation != NULL)
    {
        FreePool(SlotDesignation);
    }

    return Status;
}

