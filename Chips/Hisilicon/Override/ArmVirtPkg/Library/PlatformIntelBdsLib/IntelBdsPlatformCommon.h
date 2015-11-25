#ifndef _HW_BDS_LIB_H_
#define _HW_BDS_LIB_H_

EFI_STATUS
EFIAPI
HwBdsLibRegisterAppBootOption (
  IN OUT LIST_ENTRY              *BdsBootOptionList,
  IN GUID                        *FileGuid,
  IN CHAR16                      *Description
  );

#endif
