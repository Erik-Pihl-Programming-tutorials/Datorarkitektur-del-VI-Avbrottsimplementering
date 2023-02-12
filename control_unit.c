/********************************************************************************
* control_unit.c: Contains static variables and function definitions for 
*                 implementation of an 8-bit control unit.
********************************************************************************/
#include "control_unit.h"

/* Static functions: */
static void monitor_interrupts(void);
static void check_for_irq(void);
static void generate_interrupt(const uint8_t interrupt_vector);
static inline void return_from_interrupt(void);

static inline void monitor_interrupts_pinb(void);
static inline void monitor_interrupts_pinc(void);
static inline void monitor_interrupts_pind(void);

/* Static variables: */
static uint32_t ir; /* Instruction register, stores next instruction to execute. */
static uint8_t pc;  /* Program counter, stores address to next instruction to fetch. */
static uint8_t mar; /* Memory address register, stores address for current instruction. */
static uint8_t sr;  /* Status register, stores status bits ISNZVC. */

static uint8_t op_code; /* Stores OP-code, for example LDI, OUT, JMP etc. */
static uint8_t op1;     /* Stores first operand, most often a destination. */
static uint8_t op2;     /* Stores second operand, most often a value or read address. */

static enum cpu_state state;                    /* Stores current state. */
static uint8_t reg[CPU_REGISTER_ADDRESS_WIDTH]; /* CPU-registers R0 - R31. */

static uint8_t pinb_previous; /* Previous input signals of I/O port B.*/
static uint8_t pinc_previous; /* Previous input signals of I/O port C.*/
static uint8_t pind_previous; /* Previous input signals of I/O port D.*/

/********************************************************************************
* control_unit_reset: Resets control unit registers and corresponding program.
********************************************************************************/
void control_unit_reset(void)
{
   ir = 0x00;
   pc = 0x00;
   mar = 0x00;
   sr = 0x00;

   op_code = 0x00;
   op1 = 0x00;
   op2 = 0x00;

   state = CPU_STATE_FETCH;

   pinb_previous = 0x00;
   pinc_previous = 0x00;
   pind_previous = 0x00;

   for (uint8_t i = 0; i < CPU_REGISTER_ADDRESS_WIDTH; ++i)
   {
      reg[i] = 0x00;
   }

   data_memory_reset();
   stack_reset();
   program_memory_write();
   return;
}

/********************************************************************************
* control_unit_run_next_state: Runs next state in the CPU instruction cycle:
********************************************************************************/
void control_unit_run_next_state(void)
{
   switch (state)
   {
      case CPU_STATE_FETCH:
      {
         ir = program_memory_read(pc); /* Fetches next instruction. */
         mar = pc;                     /* Stores address of current instruction. */
         pc++;                         /* Program counter points to next instruction. */
         state = CPU_STATE_DECODE;     /* Decodes the instruction during next clock cycle. */
         break;
      }
      case CPU_STATE_DECODE:
      {        
         op_code = ir >> 16;           /* Bit 23 downto 16 consists of the OP code. */
         op1 = ir >> 8;                /* Bit 15 downto 8 consists of the first operand. */
         op2 = ir;                     /* Bit 7 downto 0 consists of the second operand. */
         state = CPU_STATE_EXECUTE;    /* Executes the instruction during next clock cycle. */
         break;
      }
      case CPU_STATE_EXECUTE:
      {
         switch (op_code) /* Checks the OP code.*/
         {
         case NOP: /* NOP => do nothing. */
         {
            break; 
         }
         case LDI: /* LDI R16, 0x01 => op_code = LDI, op1 = R16, op2 = 0x01 */
         {
            reg[op1] = op2; 
            break;
         }
         case MOV: /* MOV R17, R16 => op_code = MOV, op1 = R17, op2 = R16 */
         {
            reg[op1] = reg[op2]; 
            break;
         }
         case OUT: /* OUT DDRB, R16 => op_code = OUT, op1 = DDRB, op2 = R16 */
         {
            data_memory_write(op1, reg[op2]);
            break;
         }
         case IN: /* IN R16, PINB => op_code = IN, op1 = R16, op2 = PINB */
         {
            reg[op1] = data_memory_read(op2);
            break;
         }
         case STS: /* STS counter, R16 => op_code = STS, op1 = counter, op2 = R16 */
         {
            data_memory_write(op1 + 256, reg[op2]);
            break;
         }
         case LDS: /* LDS R16, counter => op_code = LDS, op1 = R16, op2 = counter */
         {
            reg[op1] = data_memory_read(op2 + 256);
            break;
         }
         case CLR: /* CLR R16 => op_code = CLR, op1 = R16 */
         {
            reg[op1] = 0x00;
            break;
         }
         case ORI: /* ORI R16, 0x01 => op_code = ORI, op1 = R16, op2 = 0x01 */
         {
            reg[op1] = alu(OR, reg[op1], op2, &sr);  /* reg[op1] = reg[op1] | op2 */
            break;
         }
         case ANDI: /* ANDI R17, 0x05 => op_code = ANDI, op1 = R17, op2 = 0x05 */
         {
            reg[op1] = alu(AND, reg[op1], op2, &sr);  /* reg[op1] = reg[op1] & op2 */
            break;
         }
         case XORI: /* XORI R18, 0xFF => op_code = XORI, op1 = R18, op2 = 0xFF */
         {
            reg[op1] = alu(XOR, reg[op1], op2, &sr);  /* reg[op1] = reg[op1] ^ op2 */
            break;
         }
         case OR: /* OR R16, R17 => op_code = OR, op1 = R16, op2 = R17 */
         {
            reg[op1] = alu(OR, reg[op1], reg[op2], &sr);  /* reg[op1] = reg[op1] | reg[op2] */
            break;
         }
         case AND: /* AND R16, R17 => op_code = AND, op1 = R16, op2 = R17 */
         {
            reg[op1] = alu(AND, reg[op1], reg[op2], &sr); /* reg[op1] = reg[op1] & reg[op2] */
            break;
         }
         case XOR: /* xOR R16, R17 => op_code = XOR, op1 = R16, op2 = R17 */
         {
            reg[op1] = alu(XOR, reg[op1], reg[op2], &sr); /* reg[op1] = reg[op1] ^ reg[op2] */
            break;
         }
         case ADDI: /* ADDI R16, 5 => op_code = ADDI, op1 = R16, op2 = 5 */
         {
            reg[op1] = alu(ADD, reg[op1], op2, &sr); /* reg[op1] = reg[op1] + op2 */
            break;
         }
         case SUBI: /* SUBI R17, 10 => op_code = SUBI, op1 = R17, op2 = 10 */
         {
            reg[op1] = alu(SUB, reg[op1], op2, &sr); /* reg[op1] = reg[op1] - op2. */
            break;
         }
         case ADD: /* ADD R16, R17 => op_code = ADD, op1 = R16, op2 = R17 */
         {
            reg[op1] = alu(ADD, reg[op1], reg[op2], &sr); /* reg[op1] = reg[op1] + reg[op2] */
            break;
         }
         case SUB:
         {
            reg[op1] = alu(SUB, reg[op1], reg[op2], &sr); /* reg[op1] = reg[op1] - reg[op2] */
            break;
         }
         case INC: /* INC R16 => op_code = INC, op1 = R16 */
         {
            reg[op1] = alu(ADD, reg[op1], 0x01, &sr); /* reg[op1] = reg[op1] + 1 */
            break;
         }
         case DEC: /* DEC R16 => op_code = DEC, op1 = R16 */
         {
            reg[op1] = alu(SUB, reg[op1], 0x01, &sr); /* reg[op1] = reg[op1] + 1 */
            break;
         }
         case CPI: /* CPI R16, 10 => op_code = CPI, op1 = R16, op2 = 10 */
         {
            (void)alu(SUB, reg[op1], op2, &sr); /* reg[op1] - op2, updates status flags. */
            break;
         }
         case CP: /* CP R16, R17 => op_code = CP, op1 = R16, op2 = R17 */
         {
            (void)alu(SUB, reg[op1], reg[op2], &sr); /* reg[op1] - reg[op2], updates status flags. */
            break;
         }
         case JMP: /* JMP 0x05 => op_code = JMP, op1 = 0x05 */
         {
            pc = op1; 
            break;
         }
         case BREQ: /* BREQ 0x10 => op_code = BREQ, op1 = 0x10 */
         {
            if (read(sr, Z)) pc = op1; /* Jumps if Z flag is set. */
            break;
         }
         case BRNE: /* BRNE 0x10 => op_code = BRNE, op1 = 0x10 */
         {
            if (!read(sr, Z)) pc = op1; /* Jumps if Z flag is cleared. */
            break;
         }
         case BRGE: /* BRGE 0x10 => op_code = BRGE, op1 = 0x10 */
         {
            if (!read(sr, S)) pc = op1; /* Jumps if S flag is cleared. */
            break;
         }
         case BRGT: /* BRGT 0x10 => op_code = BRGT, op1 = 0x10 */
         {
            if (!read(sr, S) && !read(sr, Z)) pc = op1; /* Jumps if the S and Z flags are cleared. */
            break;
         }
         case BRLE: /* BRLE 0x10 => op_code = BRLE, op1 = 0x10 */
         {
            if (read(sr, S) || read(sr, Z)) pc = op1; /* Jumps if S flag is set or Z flag is set. */
            break;
         }
         case BRLT: /* BRLT 0x10 => op_code = BRLT, op1 = 0x10 */
         {
            if (read(sr, S)) pc = op1; /* Jumps if S flag is set. */
            break;
         }
         case CALL: /* CALL 0x10 => op_code = CALL, op1 = 0x10 */
         {
            stack_push(pc); /* Pushes the return address to the stack. */
            pc = op1;       /* Assigns the address of the subroutine to be called. */
            break;
         }
         case RET: /* RET => op_code = RET */
         {
            pc = stack_pop(); /* Pops the return address from the stack. */
            break;
         }
         case RETI: /* RETI => op_code = RETI */
         {
            return_from_interrupt(); /* Resets program counter and status register. */
            break;
         }
         case PUSH: /* PUSH R16 => op_code = PUSH, op1 = R16 */
         {
            stack_push(reg[op1]); /* Pushes content of specified CPU register to the stack. */
            break;
         }
         case POP: /* POP R16 => op_code = POP, op1 = R16 */
         {
            reg[op1] = stack_pop(); /* Pops content of the stack to specified CPU register. */
            break;
         }
         case LSL: /* LSL R16 => op_code = LSL, op1 = R16 */
         {
            reg[op1] = reg[op1] << 1; /* reg[op1] = reg[op1] << 1; */
            break;
         }
         case LSR: /* LSR R17, op_code = LSR, op1 = R17 */
         {
            reg[op1] = reg[op1] >> 1; /* reg[op1] = reg[op1] >> 1; */
            break;
         }
         case ST: /* ST XREG, R16 => op_code = ST, op1 = XREG, op2 = R16 */
         {
            data_memory_write((reg[op1 + 1] << 8) | reg[op1], reg[op2]);
            break;
         }
         case LD: /* LD R16, XREG => op_code = LD, op1 = R16, op2 = XREG */
         {
            reg[op1] = data_memory_read((reg[op2 + 1] << 8) | reg[op2]);
            break;
         }
         case SEI: /* SEI => op_code = SEI */
         {
            set(sr, I); /* Enables interrupts globally by setting the I-flag in the status register. */
            break;
         }
         case CLI: /* CLI => op_code = CLI */
         {
            clr(sr, I); /* Disables interrupts globally by clearing the I-flag in the status register. */
            break;
         }
         default:
         {
            control_unit_reset(); /* System reset if error occurs. */
            break;
         }
         }

         state = CPU_STATE_FETCH; /* Fetches next instruction during next clock cycle. */
         check_for_irq();         /* Checks for IRQ and generates an interrupt if pending. */
         break;
      }
      default: /* System reset if error occurs. */
      {
         control_unit_reset();
         break;
      }
   }

   monitor_interrupts(); /* Monitors pin change interrupts every clock cycle. */
   return;
}

/********************************************************************************
* control_unit_run_next_state: Runs next CPU instruction cycle, i.e. fetches
*                              a new instruction from program memory, decodes
*                              and executes it.
********************************************************************************/
void control_unit_run_next_instruction_cycle(void)
{
   do
   {
      control_unit_run_next_state();
   } while (state != CPU_STATE_EXECUTE);
   return;
}

/********************************************************************************
* control_unit_print: Prints information about the processor, for instance
*                     current subroutine, instruction, state, content in
*                     CPU-registers and I/O registers DDRB, PORTB and PINB.
********************************************************************************/
void control_unit_print(void)
{
   printf("--------------------------------------------------------------------------------\n");
   printf("Current subroutine:\t\t\t\t%s\n", program_memory_subroutine_name(mar));
   printf("Current instruction:\t\t\t\t%s\n", cpu_instruction_name(op_code));
   printf("Current state:\t\t\t\t\t%s\n", cpu_state_name(state));

   printf("Program counter:\t\t\t\t%hu\n", pc);
   printf("Stack pointer:\t\t\t\t\t%hu\n", stack_pointer());
   printf("Value last added to the stack:\t\t\t%hu\n\n", stack_last_added_value());

   printf("Instruction register:\t\t\t\t%s ", get_binary((ir >> 16) & 0xFF, 8));
   printf("%s ", get_binary((ir >> 8) & 0xFF, 8));
   printf("%s\n", get_binary(ir & 0xFF, 8));

   printf("Status register (ISNZVC):\t\t\t%s\n\n", get_binary(sr, 6));

   printf("Content in CPU register R16:\t\t\t%s\n", get_binary(reg[R16], 8));
   printf("Content in CPU register R17:\t\t\t%s\n", get_binary(reg[R17], 8));
   printf("Content in CPU register R18:\t\t\t%s\n", get_binary(reg[R18], 8));
   printf("Content in CPU register R24:\t\t\t%s\n\n", get_binary(reg[R24], 8));

   printf("Content in X pointer register:\t\t\t%hu\n", (reg[XH] << 8) | reg[XL]);
   printf("Content in Y pointer register:\t\t\t%hu\n", (reg[YH] << 8) | reg[YL]);
   printf("Content in Z pointer register:\t\t\t%hu\n\n", (reg[ZH] << 8) | reg[ZL]);

   printf("Content in data direction register DDRB:\t%s\n", get_binary(data_memory_read(DDRB), 8));
   printf("Content in data register PORTB:\t\t\t%s\n", get_binary(data_memory_read(PORTB), 8));
   printf("Content in pin input register PINB:\t\t%s\n\n", get_binary(data_memory_read(PINB), 8));

   printf("Content in PCI mask register PCMSK0:\t\t%s\n", get_binary(data_memory_read(PCMSK0 + 256), 8));
   printf("Content in PCI control register PCICR:\t\t%s\n", get_binary(data_memory_read(PCICR + 256), 8));
   printf("Content in PCI flag register PCIFR:\t\t%s\n", get_binary(data_memory_read(PCIFR + 256), 8));

   printf("--------------------------------------------------------------------------------\n\n");
   return;
}

/********************************************************************************
* monitor_interrupts: Monitors pin change interrupts on I/O port B - D. 
*                     If a logic change occurs on a pin where pin change
*                     interrupt is enabled, corresponding interrupt flag
*                     is set to indicate an interrupt request (IRQ).
********************************************************************************/
static void monitor_interrupts(void)
{
   monitor_interrupts_pinb();
   monitor_interrupts_pinc();
   monitor_interrupts_pind();
   return;
}

/********************************************************************************
* check_for_irq: Checks for interrupt request by reading the PCI flag bits
*                in the PCIFR register. If a flag bit is set, for instance PCIF0,
*                and the corresponding enable bit is set, for instance PCIE0,
*                an interrupt is generated if the I bit in the status register
*                is set. The flag bit is cleared before the interrupt is 
*                generated.
********************************************************************************/
static void check_for_irq(void)
{
   if (read(sr, I))
   {
      const uint8_t pcifr = data_memory_read(PCIFR + 256);
      const uint8_t pcicr = data_memory_read(PCICR + 256);

      if (read(pcifr, PCIF0) && read(pcicr, PCIE0))
      {
         data_memory_clear_bit(PCIFR + 256, PCIF0);
         generate_interrupt(PCINT0_vect);
      }
      else if (read(pcifr, PCIF1) && read(pcicr, PCIE1))
      {
         data_memory_clear_bit(PCIFR + 256, PCIF1);
         generate_interrupt(PCINT1_vect);
      }
      else if (read(pcifr, PCIF2) && read(pcicr, PCIE2))
      {
         data_memory_clear_bit(PCIFR + 256, PCIF2);
         generate_interrupt(PCINT2_vect);
      }
   }
   return;
}

/********************************************************************************
* generate_interrupt: Generates an interrupt by jumping to the specified
*                     interrupt vector. The content of the program counter 
*                     (the return address) and the status register are
*                     stored on the stack before the jump is executed. 
*                     The I bit in the status register is cleared to disable
*                     interrupts while the current interrupt is executed.
* 
*                     - interrupt_vector: Specified interrupt vector. A jump
*                                         is made to this address to handle
*                                         the interrupt.
********************************************************************************/
static void generate_interrupt(const uint8_t interrupt_vector)
{
   stack_push(pc);
   stack_push(sr);
   clr(sr, I);
   pc = interrupt_vector;
   return;
}

/********************************************************************************
* return_from_interrupt: Loads the return address and the status bits from
*                        the stack to return from current interrupt.
********************************************************************************/
static inline void return_from_interrupt(void)
{
   sr = stack_pop();
   pc = stack_pop();
   return;
}

/********************************************************************************
* monitor_interrupts_pinb: Monitors pin change interrupts on I/O port B by
*                          comparing current and previous input signals on
*                          pins where pin change interrupt is enabled.
* 
*                          If a logic change occurs (current signal is not equal
*                          to previous input signal) on a pin where the 
*                          corresponding pin change mask bit is set, the 
*                          interrupt flag PCIF0 in the flag register PCIFR
*                          is set to indicate an interrupt request (IRQ).
* 
*                          Before terminating the function, the current input
*                          signals are stored as previous input signals for
*                          next monitoring.
********************************************************************************/
static inline void monitor_interrupts_pinb(void)
{
   const uint8_t pinb = data_memory_read(PINB);           
   const uint8_t pcmsk0 = data_memory_read(PCMSK0 + 256); 

   for (uint8_t i = 0; i < DATA_MEMORY_DATA_WIDTH; ++i) 
   {
      if (read(pcmsk0, i) && (read(pinb, i) != read(pinb_previous, i)))
      {
         data_memory_set_bit(PCIFR + 256, PCIF0);
         break;
      }
   }

   pinb_previous = pinb; 
   return;
}

/********************************************************************************
* monitor_interrupts_pinc: Monitors pin change interrupts on I/O port c by
*                          comparing current and previous input signals on
*                          pins where pin change interrupt is enabled.
*
*                          If a logic change occurs (current signal is not equal
*                          to previous input signal) on a pin where the
*                          corresponding pin change mask bit is set, the
*                          interrupt flag PCIF1 in the flag register PCIFR
*                          is set to indicate an interrupt request (IRQ).
*
*                          Before terminating the function, the current input
*                          signals are stored as previous input signals for
*                          next monitoring.
********************************************************************************/
static inline void monitor_interrupts_pinc(void)
{
   const uint8_t pinc = data_memory_read(PINC);           
   const uint8_t pcmsk1 = data_memory_read(PCMSK1 + 256); 

   for (uint8_t i = 0; i < DATA_MEMORY_DATA_WIDTH; ++i)
   {
      if (read(pcmsk1, i) && (read(pinc, i) != read(pinc_previous, i)))
      {
         data_memory_set_bit(PCIFR + 256, PCIF1);
         break;
      }
   }

   pinc_previous = pinc;
   return;
}

/********************************************************************************
* monitor_interrupts_pind: Monitors pin change interrupts on I/O port c by
*                          comparing current and previous input signals on
*                          pins where pin change interrupt is enabled.
*
*                          If a logic change occurs (current signal is not equal
*                          to previous input signal) on a pin where the
*                          corresponding pin change mask bit is set, the
*                          interrupt flag PCIF2 in the flag register PCIFR
*                          is set to indicate an interrupt request (IRQ).
*
*                          Before terminating the function, the current input
*                          signals are stored as previous input signals for
*                          next monitoring.
********************************************************************************/
static inline void monitor_interrupts_pind(void)
{
   const uint8_t pind = data_memory_read(PIND);
   const uint8_t pcmsk2 = data_memory_read(PCMSK2 + 256);

   for (uint8_t i = 0; i < DATA_MEMORY_DATA_WIDTH; ++i)
   {
      if (read(pcmsk2, i) && (read(pind, i) != read(pind_previous, i)))
      {
         data_memory_set_bit(PCIFR + 256, PCIF2);
         break;
      }
   }

   pind_previous = pind;
   return;
}


