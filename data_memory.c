/********************************************************************************
* data_memory.c: Contains function definitions for implementation of a 2 kB 
*                data memory.
********************************************************************************/
#include "data_memory.h"

/********************************************************************************
* data: 2 kB data memory, 2000 x 8 bit storage capacity.
********************************************************************************/
static uint8_t data[DATA_MEMORY_ADDRESS_WIDTH];

/********************************************************************************
* data_memory_reset: Clears content of the entire data memory.
********************************************************************************/
void data_memory_reset(void)
{
   for (uint16_t i = 0; i < DATA_MEMORY_ADDRESS_WIDTH; ++i)
   {
      data[i] = 0x00;
   }
   return;
}

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
                      const uint8_t value)
{
   if (address < DATA_MEMORY_ADDRESS_WIDTH)
   {
      data[address] = value;
      return 0;
   }
   else
   {
      return 1;
   }
}

/********************************************************************************
* data_memory_read: Returns 8-bit value read from specified read location
*                   in data memory. If an invalid address is specified,
*                   the value 0x00 is returned.
*
*                   - address: Read location in data memory.
********************************************************************************/
uint8_t data_memory_read(const uint16_t address)
{
   if (address < DATA_MEMORY_ADDRESS_WIDTH)
   {
      return data[address];
   }
   else
   {
      return 0x00;
   }
}