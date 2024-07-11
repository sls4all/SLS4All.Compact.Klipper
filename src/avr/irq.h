#ifndef __AVR_IRQ_H
#define __AVR_IRQ_H
// Definitions for irq enable/disable on AVR

#include <avr/interrupt.h> // cli
#include "compiler.h" // barrier

typedef uint8_t irqstatus_t;

void irq_disable(void);
void irq_enable(void);
void irq_restore(irqstatus_t flag);
irqstatus_t irq_save(void);
void irq_wait(void);
void irq_poll(void);

#endif // irq.h
