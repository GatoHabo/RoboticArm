#include "stdlib.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"


extern void ButtonPress(volatile uint32_t, volatile uint32_t, volatile uint32_t, volatile uint32_t);

void GPIO_B_Int(void);
void GPIO_C_Int(void);
void GPIO_D_Int(void);
void GPIO_F_Int(void);
void GPIO_A_Int(void);

void Timer0_A_INT(void);
void Timer1_A_INT(void);
void Timer2_A_INT(void);

volatile uint32_t ui16PinStates_C [2] = {0,0}, ui16PinStates_D [2] = {0,0}, ui16PinStates_F [2] = {0,0}, ui16PinStates_A [2] = {0,0},
		ui16PinStates_B [2] = {0,0}, ui16PinStates_E [2] = {0,0};

void GPIO_B_Int(void)
{	// Clear the interrupt
	GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_2 |GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7);

	// Store new states (Button press)
	ui16PinStates_C[0] = ROM_GPIOPinRead(GPIO_PORTC_BASE, 0xF0);
	// Store new states (Button press)
	ui16PinStates_D[0] = ROM_GPIOPinRead(GPIO_PORTD_BASE, 0xC0);
	// Store new states (Button press)
	ui16PinStates_F[0] = ROM_GPIOPinRead(GPIO_PORTF_BASE, 0x10);
	// Store new states (Button press)
	ui16PinStates_A[0] =  ROM_GPIOPinRead(GPIO_PORTA_BASE, 0x1C);
	// Store new states (Button press)
	ui16PinStates_B[0] =  ROM_GPIOPinRead(GPIO_PORTB_BASE, 0xCC);
	//	// Store new states (Button press)
	//	ui16PinStates_E[0] =  ROM_GPIOPinRead(GPIO_PORTE_BASE, 0x01);

	// Start Timer0_A for State check
	ROM_TimerEnable(TIMER0_BASE, TIMER_A);
}

void GPIO_E_Int(void)
{	// Clear the interrupt
	GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_0);

	// Store new states (Button press)
	ui16PinStates_C[0] = ROM_GPIOPinRead(GPIO_PORTC_BASE, 0xF0);
	// Store new states (Button press)
	ui16PinStates_D[0] = ROM_GPIOPinRead(GPIO_PORTD_BASE, 0xC0);
	// Store new states (Button press)
	ui16PinStates_F[0] = ROM_GPIOPinRead(GPIO_PORTF_BASE, 0x10);
	// Store new states (Button press)
	ui16PinStates_A[0] =  ROM_GPIOPinRead(GPIO_PORTA_BASE, 0x1C);
	//	// Store new states (Button press)
	//	ui16PinStates_B[0] =  ROM_GPIOPinRead(GPIO_PORTB_BASE, 0xCC);
	// Store new states (Button press)
	ui16PinStates_E[0] =  ROM_GPIOPinRead(GPIO_PORTE_BASE, 0x01);


	// Start Timer0_A for State check
	ROM_TimerEnable(TIMER0_BASE, TIMER_A);
}


void GPIO_C_Int(void)
{	// Clear the interrupt
	GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

	// Store new states (Button press)
	ui16PinStates_C[0] = ROM_GPIOPinRead(GPIO_PORTC_BASE, 0xF0);
	// Store new states (Button press)
	ui16PinStates_D[0] = ROM_GPIOPinRead(GPIO_PORTD_BASE, 0xC0);
	// Store new states (Button press)
	ui16PinStates_F[0] = ROM_GPIOPinRead(GPIO_PORTF_BASE, 0x10);
	// Store new states (Button press)
	ui16PinStates_A[0] =  ROM_GPIOPinRead(GPIO_PORTA_BASE, 0x1C);
	//	// Store new states (Button press)
	//	ui16PinStates_B[0] =  ROM_GPIOPinRead(GPIO_PORTB_BASE, 0xCC);
	//	// Store new states (Button press)
	//	ui16PinStates_E[0] =  ROM_GPIOPinRead(GPIO_PORTE_BASE, 0x01);


	// Start Timer0_A for State check
	ROM_TimerEnable(TIMER0_BASE, TIMER_A);
}

void GPIO_D_Int(void)
{	// Clear the interrupt
	GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

	// Store new states (Button press)
	ui16PinStates_C[0] = ROM_GPIOPinRead(GPIO_PORTC_BASE, 0xF0);
	// Store new states (Button press)
	ui16PinStates_D[0] = ROM_GPIOPinRead(GPIO_PORTD_BASE, 0xC0);
	// Store new states (Button press)
	ui16PinStates_F[0] = ROM_GPIOPinRead(GPIO_PORTF_BASE, 0x10);
	// Store new states (Button press)
	ui16PinStates_A[0] =  ROM_GPIOPinRead(GPIO_PORTA_BASE, 0x1C);
	//	// Store new states (Button press)
	//	ui16PinStates_B[0] =  ROM_GPIOPinRead(GPIO_PORTB_BASE, 0xCC);
	//	// Store new states (Button press)
	//	ui16PinStates_E[0] =  ROM_GPIOPinRead(GPIO_PORTE_BASE, 0x01);

	// Start Timer0_A for State check
	ROM_TimerEnable(TIMER0_BASE, TIMER_A);
}

void GPIO_F_Int(void)
{	// Clear the interrupt
	GPIOIntClear(GPIO_PORTF_BASE,  GPIO_PIN_0 | GPIO_PIN_4);

	// Store new states (Button press)
	ui16PinStates_C[0] = ROM_GPIOPinRead(GPIO_PORTC_BASE, 0xF0);
	// Store new states (Button press)
	ui16PinStates_D[0] = ROM_GPIOPinRead(GPIO_PORTD_BASE, 0xC0);
	// Store new states (Button press)
	ui16PinStates_F[0] = ROM_GPIOPinRead(GPIO_PORTF_BASE, 0x11);
	// Store new states (Button press)
	ui16PinStates_A[0] =  ROM_GPIOPinRead(GPIO_PORTA_BASE, 0x1C);
	//	// Store new states (Button press)
	//	ui16PinStates_B[0] =  ROM_GPIOPinRead(GPIO_PORTB_BASE, 0xCC);
	//	// Store new states (Button press)
	//	ui16PinStates_E[0] =  ROM_GPIOPinRead(GPIO_PORTE_BASE, 0x01);

	// Start Timer0_A for State check
	ROM_TimerEnable(TIMER0_BASE, TIMER_A);
}

void GPIO_A_Int(void)
{	// Clear the interrupt
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);

	// Store new states (Button press)
	ui16PinStates_C[0] = ROM_GPIOPinRead(GPIO_PORTC_BASE, 0xF0);
	// Store new states (Button press)
	ui16PinStates_D[0] = ROM_GPIOPinRead(GPIO_PORTD_BASE, 0xC0);
	// Store new states (Button press)
	ui16PinStates_F[0] = ROM_GPIOPinRead(GPIO_PORTF_BASE, 0x10);
	// Store new states (Button press)
	ui16PinStates_A[0] =  ROM_GPIOPinRead(GPIO_PORTA_BASE, 0x1C);
	//	// Store new states (Button press)
	//	ui16PinStates_B[0] =  ROM_GPIOPinRead(GPIO_PORTB_BASE, 0xCC);
	//	// Store new states (Button press)
	//	ui16PinStates_E[0] =  ROM_GPIOPinRead(GPIO_PORTE_BASE, 0x01);

	// Start Timer0_A for State check
	ROM_TimerEnable(TIMER0_BASE, TIMER_A);
}

void Timer0_A_INT(void)
{ 	// Clear the timer interrupt
	ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	// Start Timer1_A for Button hold check
	ROM_TimerEnable(TIMER1_BASE, TIMER_A);


	if ( (((ui16PinStates_B[0]) & (ROM_GPIOPinRead(GPIO_PORTB_BASE, 0xCC)))>0) || (((ui16PinStates_F[0]) & (ROM_GPIOPinRead(GPIO_PORTF_BASE, 0x01)))>0)
			|| (((ui16PinStates_E[0]) & (ROM_GPIOPinRead(GPIO_PORTE_BASE, 0x01)))>0) )
	{
		// Turn off outputs
		ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0);

		// Disable Gripper
		ROM_PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, false);
		// Disable Gripper
		ROM_PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);
		// Disable Gripper
		ROM_PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);
		// Disable Gripper
		ROM_PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, false);
		// Disable Gripper
		ROM_PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, false);

		ROM_PWMGenDisable(PWM0_BASE, PWM_GEN_3);
		ROM_PWMGenDisable(PWM0_BASE, PWM_GEN_1);
		ROM_PWMGenDisable(PWM0_BASE, PWM_GEN_2);

		// Enable GPIO interrupt
		ROM_IntDisable(INT_GPIOC);

		// Enable GPIO interrupt
		ROM_IntDisable(INT_GPIOD);

		// Enable GPIO interrupt
		ROM_IntDisable(INT_GPIOF);

		// Enable GPIO interrupt
		ROM_IntDisable(INT_GPIOA);

		// Enable GPIO interrupt
		ROM_IntDisable(INT_GPIOB);

		// Enable GPIO interrupt
		ROM_IntDisable(INT_GPIOE);

		while(1)
		{}

	}

	if ( (((ui16PinStates_C[0]) & (ROM_GPIOPinRead(GPIO_PORTC_BASE, 0xF0)))>0) || (((ui16PinStates_D[0]) & (ROM_GPIOPinRead(GPIO_PORTD_BASE, 0xC0)))>0)
			|| (((ui16PinStates_F[0]) & (ROM_GPIOPinRead(GPIO_PORTF_BASE, 0x10)))>0) ||	(((ui16PinStates_A[0]) & (ROM_GPIOPinRead(GPIO_PORTA_BASE, 0x1C)))>0))
	{

		// Start Timer5_A for DEMO check
		ROM_TimerLoadSet(TIMER5_BASE, TIMER_A, ((ROM_SysCtlClockGet() / 1000)*60000));
		ROM_TimerEnable(TIMER5_BASE, TIMER_A);

		ui16PinStates_C[1]=ui16PinStates_C[0];
		ui16PinStates_D[1]=ui16PinStates_D[0];
		ui16PinStates_F[1]=ui16PinStates_F[0];
		ui16PinStates_A[1]=ui16PinStates_A[0];

		ButtonPress(ui16PinStates_C[0], ui16PinStates_D[0], ui16PinStates_F[0], ui16PinStates_A[0]);
	}

}

void Timer1_A_INT(void)
{ 	// Clear the timer interrupt
	ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	// Start Timer5_A for DEMO check
	ROM_TimerLoadSet(TIMER5_BASE, TIMER_A, ((ROM_SysCtlClockGet() / 1000)*60000));
	ROM_TimerEnable(TIMER5_BASE, TIMER_A);

	if  ( (((ui16PinStates_C[1]) & (ROM_GPIOPinRead(GPIO_PORTC_BASE, 0xF0)))>0) || (((ui16PinStates_D[1]) & (ROM_GPIOPinRead(GPIO_PORTD_BASE, 0xC0)))>0)
			|| (((ui16PinStates_F[1]) & (ROM_GPIOPinRead(GPIO_PORTF_BASE, 0x10)))>0) ||	(((ui16PinStates_A[1]) & (ROM_GPIOPinRead(GPIO_PORTA_BASE, 0x1C)))>0) )
	{
		// Start Timer2_A for Button hold
		ROM_TimerEnable(TIMER2_BASE, TIMER_A);
		ButtonPress((ROM_GPIOPinRead(GPIO_PORTC_BASE, 0xF0)) , (ROM_GPIOPinRead(GPIO_PORTD_BASE, 0xC0)),
				(ROM_GPIOPinRead(GPIO_PORTF_BASE, 0x10)), (ROM_GPIOPinRead(GPIO_PORTA_BASE, 0x1C)));
	}
}

void Timer2_A_INT(void)
{ 	// Clear the timer interrupt
	ROM_TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

	// Start Timer5_A for DEMO check
	ROM_TimerLoadSet(TIMER5_BASE, TIMER_A, ((ROM_SysCtlClockGet() / 1000)*60000));
	ROM_TimerEnable(TIMER5_BASE, TIMER_A);

	if ( (((ui16PinStates_C[1]) & (ROM_GPIOPinRead(GPIO_PORTC_BASE, 0xF0)))>0) || (((ui16PinStates_D[1]) & (ROM_GPIOPinRead(GPIO_PORTD_BASE, 0xC0)))>0)
			|| (((ui16PinStates_F[1]) & (ROM_GPIOPinRead(GPIO_PORTF_BASE, 0x10)))>0) ||	(((ui16PinStates_A[1]) & (ROM_GPIOPinRead(GPIO_PORTA_BASE, 0x1C)))>0) )
	{
		// Start Timer2_A for Button hold
		ROM_TimerEnable(TIMER2_BASE, TIMER_A);
		ButtonPress((ROM_GPIOPinRead(GPIO_PORTC_BASE, 0xF0)) , (ROM_GPIOPinRead(GPIO_PORTD_BASE, 0xC0)),
				(ROM_GPIOPinRead(GPIO_PORTF_BASE, 0x10)), (ROM_GPIOPinRead(GPIO_PORTA_BASE, 0x1C)));
	}

}


