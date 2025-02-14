//
//  i2c-eeprom.h
//
//  Copyright Bruce McKenney 2025
//  BSD 2-Clause license
//
#ifndef I2C_EEPROM_H_
#define I2C_EEPROM_H_ 1

#include <stdint.h>

#define EEP_ADDRBITS    16
typedef uint16_t eep_addr;          // 16-bit addresses
#define EEP_PAGESIZE    16          // Page size (from the data sheet); power of 2
#define EEP_DMA         0

extern void InitI2C(I2C_Regs *i2cdev, unsigned char eeprom_i2c_address);
extern void EEPROM_ByteWrite(unsigned int Address , unsigned char Data);
extern void EEPROM_PageWrite(unsigned int StartAddress , unsigned char * Data , unsigned int Size);
extern unsigned char EEPROM_RandomRead(unsigned int Address);
extern unsigned char EEPROM_CurrentAddressRead(void);
extern void EEPROM_SequentialRead(unsigned int Address , unsigned char * Data , unsigned int Size);
extern void EEPROM_AckPolling(void);
#if EEP_DMA
extern void InitI2C_DMA(I2C_Regs *i2cdev, unsigned char eeprom_i2c_address, uint8_t chanid);
#define EEP_DMA_NOCHAN  ((uint8_t)-1)
#endif // EEP_DMA

#endif // I2C_EEPROM_H_
