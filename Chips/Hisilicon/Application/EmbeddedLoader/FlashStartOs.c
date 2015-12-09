/** @file
*
*  Copyright (c) 2011-2015, ARM Limited. All rights reserved.
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

#include "CustomLoader.h"

EFI_STATUS 
LzmaDecompressKernel (
  IN  ESL_LINUX LinuxKernel
)
{  
	UINT32              OutputBufferSize;
	UINT32              ScratchBufferSize;
	VOID                *Buffer;
	VOID                *ScratchBuffer;
	INTN 			    Result;
    EFI_STATUS          Status = EFI_SUCCESS;
    
	(VOID)LzmaUefiDecompressGetInfo ((void *)0xA4B00000, 0x500000, &OutputBufferSize, &ScratchBufferSize);
	
	Buffer = AllocatePool(OutputBufferSize);
	if(NULL == Buffer)
	{
		DEBUG ((EFI_D_ERROR, "AllocatePool Fail\n"));  
        return EFI_OUT_OF_RESOURCES;
	}	
	ScratchBuffer = AllocatePool(ScratchBufferSize);
	if(NULL == ScratchBuffer)
	{
	    FreePool(Buffer);
		DEBUG ((EFI_D_ERROR, "Allocate ScratchBuffer Fail\n"));  
        return EFI_OUT_OF_RESOURCES;
	}
	 
	Result = LzmaUefiDecompress((void *)0xA4B00000, OutputBufferSize, Buffer, ScratchBuffer);
    if (0 != Result) 
    {
        DEBUG ((EFI_D_ERROR, "Decompress Failed. Result : %d \n", Result)); 
        Status = EFI_DEVICE_ERROR;
        goto Exit;
    }
	
    gBS->CopyMem((void *)(UINTN)LinuxKernel, Buffer, OutputBufferSize);

Exit:    
	FreePool(Buffer);
	FreePool(ScratchBuffer);

    return Status;
}

EFI_STATUS
EFIAPI
FlashStartOsEntry (
  IN EFI_HANDLE          ImageHandle,
  IN EFI_SYSTEM_TABLE*   SystemTable
  )
{
    UINT32              Index = 0;
    UINTN               FDTConfigTable = 0;
    EFI_STATUS Status;
    ESL_LINUX LinuxKernel = (ESL_LINUX)(0x80000); 

    if (!PcdGet32(PcdIsMPBoot))
    {
        for (Index = 0; Index < gST->NumberOfTableEntries; Index ++) 
        {
            if (CompareGuid (&gFdtTableGuid, &(gST->ConfigurationTable[Index].VendorGuid)))
            {
                FDTConfigTable = (UINTN)gST->ConfigurationTable[Index].VendorTable;
                DEBUG ((EFI_D_ERROR, "FDTConfigTable Address: 0x%lx\n",FDTConfigTable));     
                break;
            }
        }
        gBS->CopyMem((void *)0x6000000,(void *)FDTConfigTable,0x100000);
        MicroSecondDelay(20000);
        
        gBS->CopyMem((void *)LinuxKernel,(void *)0x90100000,0x1F00000);
        MicroSecondDelay(200000);
        
        gBS->CopyMem((void *)0x7000000,(void *)0x92000000,0x4000000);
        MicroSecondDelay(200000);

        DEBUG((EFI_D_ERROR,"Update FDT\n"));
        Status = EFIFdtUpdate(0x06000000);
        if(EFI_ERROR(Status))
        {
            DEBUG((EFI_D_ERROR,"EFIFdtUpdate ERROR\n"));
            goto Exit;
        }
    }
    else
    {
        Status = LzmaDecompressKernel (LinuxKernel);
        if(EFI_ERROR(Status))
        {
            goto Exit;
        }

        gBS->CopyMem((void *)0x6000000, (void *)0xA47C0000, 0x20000);
        MicroSecondDelay(20000);

        gBS->CopyMem((void *)0x7000000, (void *)0xA4000000, 0x7C0000);
        MicroSecondDelay(200000);
    }

    Status = ShutdownUefiBootServices ();
    if(EFI_ERROR(Status)) 
    {
        DEBUG((EFI_D_ERROR,"ERROR: Can not shutdown UEFI boot services. Status=0x%X\n", Status));
        goto Exit;
    }

    //
    // Switch off interrupts, caches, mmu, etc
    //
    Status = PreparePlatformHardware ();
    ASSERT_EFI_ERROR(Status);

    LinuxKernel (0x06000000,0,0,0);
    // Kernel should never exit
    // After Life services are not provided
    ASSERT(FALSE);
    Status = EFI_ABORTED;

Exit:
    // Only be here if we fail to start Linux
    Print (L"ERROR  : Can not start the kernel. Status=%r\n", Status);
  
    // Free Runtimee Memory (kernel and FDT)
    return Status;
}


