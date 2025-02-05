///
//  main.c
//  Adopted (almost) directly from SLAA208.
//
//  Copyright Bruce McKenney 2025
//  BSD 2-clause license
//

#include "ti_msp_dl_config.h"
#include "i2c-eeprom.h"

#define     SlaveAddress   0x50

unsigned char read_val[150];
unsigned char write_val[150];
unsigned int address;

int
main(void)
{
  unsigned int i;

  SYSCFG_DL_init();
  DL_GPIO_setPins(GPIO_LEDS_PORT,
                  GPIO_LEDS_USER_LED_1_PIN | GPIO_LEDS_USER_TEST_PIN);
#if EEP_DMA
  // We picked CH0 in Sysconfig
  InitI2C_DMA(I2C_INST, SlaveAddress, DMA_CH0_CHAN_ID);          // Initialize I2C module
#else
  InitI2C(I2C_INST, SlaveAddress);          // Initialize I2C module
#endif // EEP_DMA

  EEPROM_ByteWrite(0x0000,0x12);
  EEPROM_AckPolling();                      // Wait for EEPROM write cycle
                                            // completion

  EEPROM_ByteWrite(0x0001,0x34);
  EEPROM_AckPolling();                      // Wait for EEPROM write cycle
                                            // completion
                                            // completion
  EEPROM_ByteWrite(0x0002,0x56);
  EEPROM_AckPolling();                      // Wait for EEPROM write cycle
                                            // completion

  EEPROM_ByteWrite(0x0003,0x78);
  EEPROM_AckPolling();                      // Wait for EEPROM write cycle
                                            // completion

  EEPROM_ByteWrite(0x0004,0x9A);
  EEPROM_AckPolling();                      // Wait for EEPROM write cycle
                                            // completion

  EEPROM_ByteWrite(0x0005,0xBC);
  EEPROM_AckPolling();                      // Wait for EEPROM write cycle
                                            // completion

  read_val[0] = EEPROM_RandomRead(0x0000);  // Read from address 0x0000
  read_val[1] = EEPROM_CurrentAddressRead();// Read from address 0x0001
  read_val[2] = EEPROM_CurrentAddressRead();// Read from address 0x0002
  read_val[3] = EEPROM_CurrentAddressRead();// Read from address 0x0003
  read_val[4] = EEPROM_CurrentAddressRead();// Read from address 0x0004
  read_val[5] = EEPROM_CurrentAddressRead();// Read from address 0x0005

  // Fill write_val array with counter values
  for(i = 0 ; i < sizeof(write_val) ; i++)
  {
    write_val[i] = i;
  }

  address = 0x0000;                         // Set starting address at 0
  // Write a sequence of data array
  EEPROM_PageWrite(address , write_val , sizeof(write_val));
  // Read out a sequence of data from EEPROM
  EEPROM_SequentialRead(address, read_val , sizeof(read_val));

  //    Declare victory
  DL_GPIO_clearPins(GPIO_LEDS_PORT,
                  GPIO_LEDS_USER_LED_1_PIN | GPIO_LEDS_USER_TEST_PIN);

  while (1)
  {
      __WFI();
  }
  /*NOTREACHED*/
  return(0);
}
