/********************************************************************************
* stack.h: Contains function declarations and macro definitions for 
*          implementation of 1 kB stack.
********************************************************************************/
#ifndef STACK_H_
#define STACK_H_

/* Include directives: */
#include "cpu.h"

/* Macro definitions: */
#define STACK_ADDRESS_WIDTH 1024 /* 1024 unique addresses on the stack. */
#define STACK_DATA_WIDTH    8    /* 8 bit storage capacity per address. */

/********************************************************************************
* stack_reset: Clears content of the entire stack och sets the stack pointer
*              to the top of the stack.
********************************************************************************/
void stack_reset(void);

/********************************************************************************
* stack_push: Pushes value to the stack, unless it's full. Success code 0 is
*             returned after successful push and the stack pointer is 
*             decremented unless the stack was empty before the push. 
*             If the stack is full, no push is performed and error code 1 
*             is returned.
* 
*             - value: 8 bit value to push to the stack.
********************************************************************************/
int stack_push(const uint8_t value);

/********************************************************************************
* stack_pop: Returns value popped from the stack. If the stack is empty,
*            the value 0x00 is returned. The stack pointer is incremeneted
*            after successful pop, unless the stack pointer is already 
*            pointing to the top of the stack.
********************************************************************************/
uint8_t stack_pop(void);

/********************************************************************************
* stack_pointer: Returns the address of the stack pointer.
********************************************************************************/
uint16_t stack_pointer(void);

/********************************************************************************
* stack_last_added_value: Returns the last added value of the stack. If the
*                         stack is empty, the value 0x00 is returned.
********************************************************************************/
uint8_t stack_last_added_value(void);

#endif /* STACK_H_ */