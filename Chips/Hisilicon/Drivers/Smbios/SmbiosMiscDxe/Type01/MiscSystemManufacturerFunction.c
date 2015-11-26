/*-----------------------------------------------------------------------*/
/*!!Warning: Huawei key information asset. No spread without permission. */
/*CODEMARK:EG4uRhTwMmgcVFBsBnYHCEm2UPcyllv4D4NOje6cFLSYglw6LvPA978sGAr3yTchgOI0M46H
HZIZCDLcNqR1rbxHHGWmLNp+CRsGfVaxSWS77M+1CS28w31VEQfZSX1k6kWiUvrwZ/IoCE8Z
UyfWHB9xTs8Pof8gYbTn2ltd5ADFjFzBHdL1Z27JuCYwX8WIQPM7FfcusWzFh9BRHCvhWwzP
8DQ7RpkfqlluovOpuSJ5kGX0PpGmxDnREYaNUqaXrZDmg+bmDAK+XaXLQy9leA==#*/
/*--!!Warning: Deleting or modifying the preceding information is prohibited.--*/

/*++

Copyright (c) 2006 - 2011, Intel Corporation. All rights reserved.<BR>
This program and the accompanying materials
are licensed and made available under the terms and conditions of the BSD License
which accompanies this distribution.  The full text of the license may be found at
http://opensource.org/licenses/bsd-license.php

THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

Module Name:

  MiscSystemManufacturerFunction.c

Abstract:

  This driver parses the mMiscSubclassDataTable structure and reports
  any generated data to smbios.

**/

/* Modify list
DATA        AUTHOR            REASON
2015.08.12  y00216284-079     DTS2015081210371，SMBIOS代码检视意见修改 
*/

#include "SmbiosMisc.h"

/**
  This function makes boot time changes to the contents of the
  MiscSystemManufacturer (Type 1).

  @param  RecordData                 Pointer to copy of RecordData from the Data Table.  

  @retval EFI_SUCCESS                All parameters were valid.
  @retval EFI_UNSUPPORTED            Unexpected RecordType value.
  @retval EFI_INVALID_PARAMETER      Invalid parameter was found.

**/
MISC_SMBIOS_TABLE_FUNCTION(MiscSystemManufacturer)
{
    CHAR8                           *OptionalStrStart;
    UINTN                           ManuStrLen;
    UINTN                           VerStrLen;
    UINTN                           PdNameStrLen;
    UINTN                           SerialNumStrLen;
    UINTN                           SKUNumStrLen;
    UINTN                           FamilyStrLen;
    EFI_STRING                      Manufacturer;
    EFI_STRING                      ProductName;
    EFI_STRING                      Version;
    EFI_STRING                      SerialNumber;
    EFI_STRING                      SKUNumber;
    EFI_STRING                      Family;
    STRING_REF                      TokenToGet;
    EFI_SMBIOS_HANDLE               SmbiosHandle;
    SMBIOS_TABLE_TYPE1              *SmbiosRecord;
    SMBIOS_TABLE_TYPE1              *InputData;
    EFI_STATUS                      Status; 
    //uniBIOS_y00216284_018_start DTS2015010906228 2015-1-13 09:08:22
    //Description:通过PCD获取不同单板对应系统产品信息
    STRING_REF                      TokenToUpdate;
    CHAR16                          *Product;
    CHAR16                          *pVersion;
    //uniBIOS_y00216284_018_end 2015-1-13 09:08:22
    

    //
    // First check for invalid parameters.
    //
    if (RecordData == NULL) 
    {
        return EFI_INVALID_PARAMETER;
    }  

    InputData = (SMBIOS_TABLE_TYPE1 *)RecordData;

    //uniBIOS_y00216284_018_start DTS2015010906228 2015-1-13 09:08:22
    //Description:通过PCD获取不同单板对应系统产品信息
    Product = (CHAR16 *) PcdGetPtr (PcdSystemProductName);
    if (StrLen(Product) > 0) 
    {
        TokenToUpdate = STRING_TOKEN (STR_MISC_SYSTEM_PRODUCT_NAME);
        HiiSetString (mHiiHandle, TokenToUpdate, Product, NULL);
    }

    pVersion = (CHAR16 *) PcdGetPtr (PcdSystemVersion);
    if (StrLen(pVersion) > 0) 
    {
        TokenToUpdate = STRING_TOKEN (STR_MISC_SYSTEM_VERSION);
        HiiSetString (mHiiHandle, TokenToUpdate, pVersion, NULL);
    }
    //uniBIOS_y00216284_018_end 2015-1-13 09:07:36

    
    //获取相关字符串的长度
    TokenToGet   = STRING_TOKEN (STR_MISC_SYSTEM_MANUFACTURER);
    Manufacturer = HiiGetPackageString(&gEfiCallerIdGuid, TokenToGet, NULL);
    ManuStrLen   = StrLen(Manufacturer);

    TokenToGet   = STRING_TOKEN (STR_MISC_SYSTEM_PRODUCT_NAME);
    ProductName  = HiiGetPackageString(&gEfiCallerIdGuid, TokenToGet, NULL);
    PdNameStrLen = StrLen(ProductName);

    TokenToGet = STRING_TOKEN (STR_MISC_SYSTEM_VERSION);
    Version    = HiiGetPackageString(&gEfiCallerIdGuid, TokenToGet, NULL);
    VerStrLen  = StrLen(Version);

    TokenToGet      = STRING_TOKEN (STR_MISC_SYSTEM_SERIAL_NUMBER);
    SerialNumber    = HiiGetPackageString(&gEfiCallerIdGuid, TokenToGet, NULL);
    SerialNumStrLen = StrLen(SerialNumber);

    TokenToGet   = STRING_TOKEN (STR_MISC_SYSTEM_SKU_NUMBER);
    SKUNumber    = HiiGetPackageString(&gEfiCallerIdGuid, TokenToGet, NULL);
    SKUNumStrLen = StrLen(SKUNumber);

    TokenToGet   = STRING_TOKEN (STR_MISC_SYSTEM_FAMILY);
    Family       = HiiGetPackageString(&gEfiCallerIdGuid, TokenToGet, NULL);
    FamilyStrLen = StrLen(Family);

    //
    // Two zeros following the last string.
    //
    SmbiosRecord = AllocateZeroPool(sizeof (SMBIOS_TABLE_TYPE1) + ManuStrLen      + 1 
                                                                + PdNameStrLen    + 1 
                                                                + VerStrLen       + 1 
                                                                + SerialNumStrLen + 1 
                                                                + SKUNumStrLen    + 1 
                                                                + FamilyStrLen    + 1 + 1);

    if (NULL == SmbiosRecord)
    {
        Status = EFI_OUT_OF_RESOURCES;
        goto Exit;
    }
    //uniBIOS_c00213799_end  2014-12-11 11:01:25

    (VOID)CopyMem(SmbiosRecord, InputData, sizeof (SMBIOS_TABLE_TYPE1));

    SmbiosRecord->Hdr.Length = sizeof (SMBIOS_TABLE_TYPE1);   
   
    //(VOID)CopyMem((UINT8 *) (&SmbiosRecord->Uuid), sizeof(EFI_GUID), &InputData->Uuid, sizeof(EFI_GUID)); 
    SmbiosRecord->Uuid = InputData->Uuid;

    OptionalStrStart = (CHAR8 *)(SmbiosRecord + 1);
    UnicodeStrToAsciiStr(Manufacturer, OptionalStrStart);
    UnicodeStrToAsciiStr(ProductName,  OptionalStrStart + ManuStrLen + 1);
    UnicodeStrToAsciiStr(Version,      OptionalStrStart + ManuStrLen + 1 + PdNameStrLen + 1);
    UnicodeStrToAsciiStr(SerialNumber, OptionalStrStart + ManuStrLen + 1 + PdNameStrLen + 1 + VerStrLen + 1);
    UnicodeStrToAsciiStr(SKUNumber,    OptionalStrStart + ManuStrLen + 1 + PdNameStrLen + 1 + VerStrLen + 1 + SerialNumStrLen + 1);
    UnicodeStrToAsciiStr(Family,       OptionalStrStart + ManuStrLen + 1 + PdNameStrLen + 1 + VerStrLen + 1 + SerialNumStrLen + 1 + SKUNumStrLen + 1);

    //
    // Now we have got the full smbios record, call smbios protocol to add this record.
    //
    Status = LogSmbiosData( (UINT8*)SmbiosRecord, &SmbiosHandle);  
    if(EFI_ERROR(Status))
    {
        DEBUG((EFI_D_ERROR, "[%a]:[%dL] Smbios Type01 Table Log Failed! %r \n", __FUNCTION__, __LINE__, Status));        
    }
    
    FreePool(SmbiosRecord);
    
Exit:
    if(Manufacturer != NULL)
    {
        FreePool(Manufacturer);   
    } 

    if(ProductName != NULL)
    {
        FreePool(ProductName);   
    }  
    
    if(Version != NULL)
    {
        FreePool(Version);   
    }  
    
    if(SerialNumber != NULL)
    {
        FreePool(SerialNumber);   
    } 
    
    if(SKUNumber != NULL)
    {
        FreePool(SKUNumber);   
    } 

    if(Family != NULL)
    {
        FreePool(Family);   
    }  

    return Status;
}
