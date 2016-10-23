#include "stdlib.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/rom.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "inc/hw_nvic.h"
#include "driverlib/timer.h"

extern void SetupUart(void);
extern void SetupPWM(void);
extern void SetupOutputs(void);
extern void SetupTimer(void);
extern void SetupInterrupt(void);
extern void Position( float * Px, float * Py, float * Pz);


int main(void)
{
	// Set clock speed for whole system to 40MHz (PLL)
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

	SetupUart();
	SetupPWM();
	SetupTimer();
	SetupInterrupt();

	// enable PWM for rotate
	ROM_PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);

	SetupOutputs();

	ROM_IntPrioritySet(INT_TIMER3A, 0xD0);
	ROM_IntPrioritySet(INT_TIMER5A, 0xE0);

	// Enable interrupts on GPIO port C
	GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

	// Enable interrupts on GPIO port D
	GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

	// Enable interrupts on GPIO port F
	GPIOIntEnable(GPIO_PORTF_BASE,  GPIO_PIN_0 | GPIO_PIN_4);

	// Enable interrupts on GPIO port A
	GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);

	// Enable interrupts on GPIO port B
	GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3 |GPIO_PIN_6 | GPIO_PIN_7);

	// Enable interrupts on GPIO port B
	GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_0);

	ROM_IntMasterEnable();

	// Start Timer5_A for DEMO check
	ROM_TimerEnable(TIMER5_BASE, TIMER_A);

	// Wait for interrupts
	while (1)
	{

	}

}

