/********************************************************************************
* stack.c: Contains functions definition for implementation of a 1 kB stack.
********************************************************************************/
#include "stack.h"

/* Static variables: */
static uint8_t stack[STACK_ADDRESS_WIDTH]; /* 1 kB stack (1024 addresses 0 - 1023). */
static uint16_t sp;                        /* Stack pointer, points to last added element. */
static bool stack_empty;                   /* Indicates if the stack is empty. */

/********************************************************************************
* stack_reset: Clears content of the entire stack och sets the stack pointer
*              to the top of the stack.
********************************************************************************/
void stack_reset(void)
{
   for (uint16_t i = 0; i < STACK_ADDRESS_WIDTH; ++i)
   {
      stack[i] = 0x00;
   }

   sp = STACK_ADDRESS_WIDTH - 1;
   stack_empty = true;
   return;
}

/********************************************************************************
* stack_push: Pushes value to the stack, unless it's full. Success code 0 is
*             returned after successful push and the stack pointer is
*             decremented unless the stack was empty before the push.
*             If the stack is full, no push is performed and error code 1
*             is returned.
*
*             - value: 8 bit value to push to the stack.
********************************************************************************/
int stack_push(const uint8_t value)
{
   if (sp == 0)
   {
      return 1;
   }
   else
   {
      if (stack_empty)
      {
         stack[sp] = value;
         stack_empty = false;
      }
      else
      {
         stack[--sp] = value;
      }
      return 0;
   }
}

/********************************************************************************
* stack_pop: Returns value popped from the stack. If the stack is empty,
*            the value 0x00 is returned. The stack pointer is incremeneted
*            after successful pop, unless the stack pointer is already
*            pointing to the top of the stack.
********************************************************************************/
uint8_t stack_pop(void)
{
   if (stack_empty)
   {
      return 0x00;
   }
   else
   {
      if (sp < STACK_ADDRESS_WIDTH - 1)
      {
         return stack[sp++];
      }
      else
      {
         stack_empty = true;
         return stack[sp];
      }
   }
}

/********************************************************************************
* stack_pointer: Returns the address of the stack pointer.
********************************************************************************/
uint16_t stack_pointer(void)
{
   return sp;
}

/********************************************************************************
* stack_last_added_value: Returns the last added value of the stack. If the
*                         stack is empty, the value 0x00 is returned.
********************************************************************************/
uint8_t stack_last_added_value(void)
{
   if (stack_empty)
   {
      return 0x00;
   }
   else
   {
      return stack[sp];
   }
}