/********************************************************************************
* data_memory.h: Contains function declarations and macro definitions for
*                implementation of a 2 kB data memory.
********************************************************************************/
#ifndef DATA_MEMORY_H_
#define DATA_MEMORY_H_

/* Include directives: */
#include "cpu.h"

/* Macro definitions: */
#define DATA_MEMORY_ADDRESS_WIDTH 2000 /* 2000 unique addresses in data memory. */
#define DATA_MEMORY_DATA_WIDTH    8    /* 8 bit storage capacity per address. */

/********************************************************************************
* data_memory_reset: Clears content of the entire data memory.
********************************************************************************/
void data_memory_reset(void);

/********************************************************************************
* data_memory_write: Writes specified 8-bit value to specified address in 
*                    data memory. After successful write, 0 is returned.
*                    If an invalid address is specified, no write is done
*                    and error code 1 is returned.
* 
*                    - address: Write address in data memory.
*                    - value  : 8-bit value to write to specified address.
********************************************************************************/
int data_memory_write(const uint16_t address,
                      const uint8_t value);

/********************************************************************************
* data_memory_read: Returns 8-bit value read from specified read location
*                   in data memory. If an invalid address is specified, 
*                   the value 0x00 is returned.
* 
*                   - address: Read location in data memory.
********************************************************************************/
uint8_t data_memory_read(const uint16_t address);

/********************************************************************************
* data_memory_set_bit: Sets bit in specified data memory address. If an invalid
*                      address is specified, error code 1 is returned,
*                      otherwise success code 0 is returned after write.
*
*                      - address: Write location in data memory.
*                      - bit    : Bit to set in specified write location.
********************************************************************************/
static inline int data_memory_set_bit(const uint16_t address,
                                      const uint8_t bit)
{
   const uint8_t data = data_memory_read(address);
   return data_memory_write(address, data | (1 << bit));
}

/********************************************************************************
* data_memory_clear_bit: Clears bit in specified data memory address. If an 
*                        invalid address is specified, error code 1 is returned,
*                        otherwise success code 0 is returned after write.
*
*                        - address: Write location in data memory.
*                        - bit    : Bit to clear in specified write location.
********************************************************************************/
static inline int data_memory_clear_bit(const uint16_t address,
                                        const uint8_t bit)
{
   const uint8_t data = data_memory_read(address);
   return data_memory_write(address, data & ~(1 << bit));
}

#endif /* DATA_MEMORY_H_ */