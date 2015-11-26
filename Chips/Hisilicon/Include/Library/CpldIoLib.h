#ifndef _CPLD_IO_LIB_H_
#define _CPLD_IO_LIB_H_

VOID WriteCpldReg(UINTN ulRegAddr, UINT8 ulValue);
UINT8 ReadCpldReg(UINTN ulRegAddr);

#endif /* _CPLD_IO_LIB_H_ */
