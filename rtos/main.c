#include "rtos.h"

#include "cpu.h"
#include <stdio.h>
#include <stdint.h>
#include <util/delay.h>

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/iom128.h>
#include <avr/interrupt.h>

#include "port.h"
#include "uart.h"

static semaphore_t tick = {0}; // A semaphore is incremented at every tick.

void task1(void *p)
{
	while(1)
	{
		printf("\nt1");
		delay_ms(50);
	}
}

void task2(void *p)
{
	while(1)
	{
		printf("\nt2");
		delay_ms(100);
	}
}

volatile char x = ' ';
void task3(void *p)
{
	while(1)
	{
		x = uart_getchar();
		delay_ms(1);
	}
}

void task4(void *p)
{
	while(1)
	{
		if(x == 'o')
		printf("OK\n");
		delay_ms(1);
	}
}

void delay_ms(unsigned int t)
{
	wait_for_increment_of(&tick, t, 0, 0);
}

void init_timer(unsigned int hz)
{
	// [+]Set TIMER3_COMPA interrupt to tick every 80,000 clock cycles.
	TCCR3B  = (1<<WGM32) && (1<<CS31); // CTC, clkI/O/8 (From prescaler).
	OCR3A   = (F_CPU / (8 * hz)) - 1;  // Formula: (f_cpu / (64 * desired_f)) - 1, ex: f_pwm=1000Hz (period = 1ms standard).
	ETIFR   = (1<<OCF3A);
	ETIMSK |= (1<<OCIE3A);
}

// This is our idle task - the task that runs when all others are suspended.
// We sleep the CPU - the CPU will automatically awake when the tick interrupt occurs
// This task cannot be stopped, so it is automatically re-started whenever it tries to exit.
void idle_task(void *p)
{
	sleep_enable();
	sei();
	sleep_cpu();
}

// This is a function that runs every tick interrupt - we use it to increment the tick semaphore value by one
// We want a task switch to ALWAYS occur - it is part of the definition of the tick interrupt!
uint8_t tick_interrupt()
{
	increment_semaphore_by(&tick, 1);
	return(1);
}

// [+]Setup the TIMER3_COMPA interrupt - it will be our tick interrupt.
TASK_ISR(TIMER3_COMPA_vect, tick_interrupt());

int main(void)
{
	port_init();
	uart_init(19200);
	printf("\n ~ OS Started ~ \n");

	cli(); // Interrupts should remain disabled - they will be enabled as soon as the first task starts executing.

	// [+]The idle task sleeps the CPU  -  set the sleep mode to IDLE,
	// as we need the sleep to be interruptable by the tick interrupt.
	set_sleep_mode(SLEEP_MODE_IDLE);

	// [+]Create tasks.
	create_task(task1, 0, 0, 65, 100, 0);
	create_task(task2, 0, 0, 65, 100, 0);
	create_task(task3, 0, 0, 65, 50,  0);
	create_task(task4, 0, 0, 65, 50,  0);

	init_timer(5000);

	// [+]Start the RTOS - note that this function will never return.
	task_switcher_start(idle_task, 0, 65, 80);

	return(0);
}
