/*-----------------------------------------------------------------------*/
/*!!Warning: Huawei key information asset. No spread without permission. */
/*CODEMARK:EG4uRhTwMmgcVFBsBnYHCEm2UPcyllv4D4NOje6cFLSYglw6LvPA978sGAr3yTchgOI0M46H
HZIZCDLcNqR1rbxHHGWmLNp+CRsGfVaxSWS77M+1CS28w31VEQfZSX1k6kWiUvrwZ/IoCE8Z
UyfWHI/Fd02wZ9veRITVo0hpJY5VcC8u6fE5B+OEJC8fn7ME1kdpr88BlUyarLnD5D1ZmANy
I1bistyQpzRZrAaVS0Y+eiJtLdnkXZB12Tf8hkXF0BnLN2R+Oi9GMlhXdymu7g==#*/
/*--!!Warning: Deleting or modifying the preceding information is prohibited.--*/

/** @file

  Copyright (c) 2009 - 2011, Intel Corporation. All rights reserved.<BR>
  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
  
Module Name:

  MiscBaseBoardManufacturerFunction.c
  
Abstract: 

  This driver parses the mSmbiosMiscDataTable structure and reports
  any generated data using SMBIOS protocol.

**/
/* Modify list
DATA        AUTHOR            REASON
2015.08.12  y00216284-079     DTS2015081210371，SMBIOS代码检视意见修改 
*/

#include "SmbiosMisc.h"


/**
  This function makes basic board manufacturer to the contents of the
  Misc Base Board Manufacturer (Type 2).

  @param  RecordData                 Pointer to copy of RecordData from the Data Table.  

  @retval EFI_SUCCESS                All parameters were valid.
  @retval EFI_UNSUPPORTED            Unexpected RecordType value.
  @retval EFI_INVALID_PARAMETER      Invalid parameter was found.

**/
MISC_SMBIOS_TABLE_FUNCTION(MiscBaseBoardManufacturer)
{
    CHAR8                             *OptionalStrStart;
    UINTN                             ManuStrLen;
    UINTN                             ProductNameStrLen;
    UINTN                             VerStrLen;
    UINTN                             SerialNumStrLen;
    UINTN                             AssetTagStrLen; 
    UINTN                             ChassisLocaStrLen;
    UINTN                             HandleCount = 0;
    UINT16                            *HandleArray = NULL;  
    CHAR16                            *BaseBoardManufacturer;
    CHAR16                            *BaseBoardProductName;
    CHAR16                            *Version;
    EFI_STRING                        SerialNumber;
    EFI_STRING                        AssetTag;
    EFI_STRING                        ChassisLocation;
    STRING_REF                        TokenToGet;
    EFI_SMBIOS_HANDLE                 SmbiosHandle;
    SMBIOS_TABLE_TYPE2                *SmbiosRecord;
    SMBIOS_TABLE_TYPE2                *InputData = NULL;
    EFI_STATUS                        Status;

    //uniBIOS_y00216284_018_start DTS2015010906228 2015-1-13 09:08:22
    //Description:通过PCD获取不同单板对应主板相应信息
    STRING_REF                        TokenToUpdate;
    //CHAR16                            *ProductName;
    //CHAR16                            *pVersion;
    //uniBIOS_y00216284_018_end 2015-1-13 09:08:22

    //
    // First check for invalid parameters.
    //
    if (RecordData == NULL) 
    {
        return EFI_INVALID_PARAMETER;
    }

    InputData = (SMBIOS_TABLE_TYPE2*)RecordData; 

    //uniBIOS_y00216284_018_start DTS2015010906228 2015-1-13 09:08:22
    //Description:通过PCD获取不同单板对应主板相应信息
    BaseBoardProductName = (CHAR16 *) PcdGetPtr (PcdBaseBoardProductName);
    if (StrLen(BaseBoardProductName) > 0) 
    {
        TokenToUpdate = STRING_TOKEN (STR_MISC_BASE_BOARD_PRODUCT_NAME);
        HiiSetString (mHiiHandle, TokenToUpdate, BaseBoardProductName, NULL);
    }

    Version = (CHAR16 *) PcdGetPtr (PcdBaseBoardVersion);
    if (StrLen(Version) > 0) 
    {
        TokenToUpdate = STRING_TOKEN (STR_MISC_BASE_BOARD_VERSION);
        HiiSetString (mHiiHandle, TokenToUpdate, Version, NULL);
    }
    //uniBIOS_y00216284_018_end 2015-1-13 09:07:36

    //
    // 获取字符串信息
    //
    TokenToGet = STRING_TOKEN (STR_MISC_BASE_BOARD_MANUFACTURER);
    BaseBoardManufacturer = HiiGetPackageString(&gEfiCallerIdGuid, TokenToGet, NULL);
    ManuStrLen = StrLen(BaseBoardManufacturer);

    TokenToGet = STRING_TOKEN (STR_MISC_BASE_BOARD_PRODUCT_NAME);
    BaseBoardProductName = HiiGetPackageString(&gEfiCallerIdGuid, TokenToGet, NULL);
    ProductNameStrLen = StrLen(BaseBoardProductName);

    TokenToGet = STRING_TOKEN (STR_MISC_BASE_BOARD_VERSION);
    Version = HiiGetPackageString(&gEfiCallerIdGuid, TokenToGet, NULL);
    VerStrLen = StrLen(Version);

    TokenToGet = STRING_TOKEN (STR_MISC_BASE_BOARD_SERIAL_NUMBER);
    SerialNumber = HiiGetPackageString(&gEfiCallerIdGuid, TokenToGet, NULL);
    SerialNumStrLen = StrLen(SerialNumber);

    TokenToGet = STRING_TOKEN (STR_MISC_BASE_BOARD_ASSET_TAG);
    AssetTag = HiiGetPackageString(&gEfiCallerIdGuid, TokenToGet, NULL);
    AssetTagStrLen = StrLen(AssetTag);

    TokenToGet = STRING_TOKEN (STR_MISC_BASE_BOARD_CHASSIS_LOCATION);
    ChassisLocation = HiiGetPackageString(&gEfiCallerIdGuid, TokenToGet, NULL);
    ChassisLocaStrLen = StrLen(ChassisLocation);

    //
    // Two zeros following the last string.
    //
    SmbiosRecord = AllocateZeroPool(sizeof (SMBIOS_TABLE_TYPE2) + ManuStrLen        + 1 
                                                                + ProductNameStrLen + 1 
                                                                + VerStrLen         + 1 
                                                                + SerialNumStrLen   + 1 
                                                                + AssetTagStrLen    + 1 
                                                                + ChassisLocaStrLen + 1 + 1);
    //uniBIOS_c00213799_start DTS2014121003065 2014-12-11 14:47:57
    //Description:coverity
    if (NULL == SmbiosRecord)
    {
        Status = EFI_OUT_OF_RESOURCES;
        goto Exit;
    }
    //uniBIOS_c00213799_end  2014-12-11 14:47:57 
    
    (VOID)CopyMem(SmbiosRecord, InputData, sizeof (SMBIOS_TABLE_TYPE2));
    SmbiosRecord->Hdr.Length        = sizeof (SMBIOS_TABLE_TYPE2);
   
    //
    //  Update Contained objects Handle
    //
    SmbiosRecord->NumberOfContainedObjectHandles = 0;
    GetLinkTypeHandle(EFI_SMBIOS_TYPE_SYSTEM_ENCLOSURE, &HandleArray, &HandleCount);
    if(HandleCount)
    {
        SmbiosRecord->ChassisHandle = HandleArray[0];
    }

    FreePool(HandleArray); 

    OptionalStrStart = (CHAR8 *)(SmbiosRecord + 1);
    UnicodeStrToAsciiStr(BaseBoardManufacturer, OptionalStrStart);
    UnicodeStrToAsciiStr(BaseBoardProductName,  OptionalStrStart + ManuStrLen + 1);
    UnicodeStrToAsciiStr(Version,               OptionalStrStart + ManuStrLen + 1 + ProductNameStrLen + 1);
    UnicodeStrToAsciiStr(SerialNumber,          OptionalStrStart + ManuStrLen + 1 + ProductNameStrLen + 1 + VerStrLen + 1);
    UnicodeStrToAsciiStr(AssetTag,              OptionalStrStart + ManuStrLen + 1 + ProductNameStrLen + 1 + VerStrLen + 1 + SerialNumStrLen + 1 );
    UnicodeStrToAsciiStr(ChassisLocation,       OptionalStrStart + ManuStrLen + 1 + ProductNameStrLen + 1 + VerStrLen + 1 + SerialNumStrLen + 1 + AssetTagStrLen + 1);

    Status = LogSmbiosData( (UINT8*)SmbiosRecord, &SmbiosHandle);  
    if(EFI_ERROR(Status))
    {
        DEBUG((EFI_D_ERROR, "[%a]:[%dL] Smbios Type02 Table Log Failed! %r \n", __FUNCTION__, __LINE__, Status));        
    }

    FreePool(SmbiosRecord);

Exit:
    if(BaseBoardManufacturer != NULL)
    {
        FreePool(BaseBoardManufacturer);   
    } 
    
    if(BaseBoardProductName != NULL)
    {
        FreePool(BaseBoardProductName);   
    }  
    
    if(Version != NULL)
    {
        FreePool(Version);   
    } 
    
    if(SerialNumber != NULL)
    {
        FreePool(SerialNumber);   
    }

    if(AssetTag != NULL)
    {
        FreePool(AssetTag);   
    } 

    if(ChassisLocation != NULL)
    {
        FreePool(ChassisLocation);   
    } 

    return Status;
}
