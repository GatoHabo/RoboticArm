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



void SetupUart(void);
void SetupPWM(void);
void SetupOutputs(void);
void SetupTimer(void);
void SetupInterrupt(void);

#define PWM_FREQUENCY 50
#define GPIOD_LOCK 	(*((volatile unsigned long *)0x40007520))
#define GPIOD_CR 	(*((volatile unsigned long *)0x40007524))

#define GPIOF_LOCK 	(*((volatile unsigned long *)0x40025520))
#define GPIOF_CR 	(*((volatile unsigned long *)0x40025524))

void SetupUart(void)
{
	//Enable the UART peripheral
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//Set the Rx/Tx pins as UART pins
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//Configure the UART baud rate, data configuration
	ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
			UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
	SysCtlDelay(10);

	//Configure other UART features (e.g. interrupts, FIFO)
	UARTStdioConfig(0, 115200, ROM_SysCtlClockGet());

	// Send/receive a character
	UARTprintf("UART Initialised\n");
}

void SetupPWM(void)
{
	uint32_t ui32Load;
	uint32_t ui32PWMClock;
	uint16_t ui16PWM_0_ADJUST, ui16PWM_1_ADJUST, ui16PWM_2_ADJUST, ui16PWM_3_ADJUST, ui16PWM_4_ADJUST;
	uint32_t PWMCLKSET;
	uint32_t pwmFreq;
	uint32_t DIVIDE=8;


	// set PWM clock. Use either SYSCTL_PWMDIV_1,SYSCTL_PWMDIV_2, SYSCTL_PWMDIV_4, SYSCTL_PWMDIV_8,
	//SYSCTL_PWMDIV_16, SYSCTL_PWMDIV_32, or SYSCTL_PWMDIV_64.
	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_8);
	SysCtlDelay(10);

	// Enable Peripherals for PWM
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlDelay(10);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlDelay(10);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlDelay(10);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlDelay(10);

	GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_4|GPIO_PIN_5,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
	GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);
	GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_4,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD);

	ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4|GPIO_PIN_5);
	ROM_GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1);
	ROM_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);


	ROM_GPIOPinConfigure(GPIO_PD0_M0PWM6);
	ROM_GPIOPinConfigure(GPIO_PD1_M0PWM7);
	ROM_GPIOPinConfigure(GPIO_PB4_M0PWM2);
	ROM_GPIOPinConfigure(GPIO_PB5_M0PWM3);
	ROM_GPIOPinConfigure(GPIO_PE4_M0PWM4);

	// Print PWM clock
	PWMCLKSET = PWMClockGet(PWM0_BASE);
	if (PWM_SYSCLK_DIV_1 == PWMCLKSET)
	{pwmFreq = SysCtlClockGet();}
	else
	{pwmFreq = SysCtlClockGet()/(PWMCLKSET ^ 0x100);}

	UARTprintf("\n%i\n",pwmFreq);


	// Calculate clock ticks for pwm count
	ui32PWMClock = SysCtlClockGet() / DIVIDE;
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

	// Configure PWM count mode as DOWN or UP/DOWN
	ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
	ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);

	// Configure PWM period
	ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ui32Load);
	ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ui32Load);
	ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, ui32Load/20);


	// 7620 Clock ticks for actual middle = 1524us / 0.2us
	//	(0.2us = 5MHz^-1)

	//	ui16PWM_0_ADJUST = 7620 + (18.59941447*(90-9)) - 1;
	//	ui16PWM_1_ADJUST = 8003.684499848866 ;
	//	ui16PWM_2_ADJUST =  7737.61873102 ;
	//	ui16PWM_3_ADJUST = 7620 - 1 - 200;
	//	ui16PWM_4_ADJUST = ui32Load/40;

	ui16PWM_0_ADJUST = 7620 + (18.59941447*(90)) - 1;

	ui16PWM_1_ADJUST = 7492.5;
	//	ui16PWM_2_ADJUST = 7584.2;
//	ui16PWM_2_ADJUST = 7478.6;
	ui16PWM_2_ADJUST = 7440;
	//	ui16PWM_3_ADJUST = 7420;
	ui16PWM_3_ADJUST = 7500;
	ui16PWM_4_ADJUST = ui32Load/40;

	// configure PWM high pulse
	ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,  ui16PWM_0_ADJUST);
	ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,  ui16PWM_1_ADJUST);
	ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,  ui16PWM_2_ADJUST);
	ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,  ui16PWM_3_ADJUST);
	ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,  ui16PWM_4_ADJUST);

	// enable PWM for arm links
	ROM_PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT|PWM_OUT_7_BIT|PWM_OUT_2_BIT, true);

	// Ensure gripper PWM and rotate is disabled
	ROM_PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT | PWM_OUT_4_BIT, false);

	ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_3);
	ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_1);
	ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_2);
}

void SetupTimer(void)
{
	volatile uint32_t ui32Period1, ui32Period2, ui32Period3,
	ui32Period4,  ui32Period5, ui32Period6;
	volatile uint32_t ui32CheckTime1, ui32CheckTime2, ui32CheckTime3,
	ui32CheckTime4, ui32CheckTime5, ui32CheckTime6;

	// Set timer cycle time in mS
	// TO CHANGE SPEED OF WHOLE ARM, ADJUST ui32CheckTime4. SMALLER IS FASTER
	ui32CheckTime1 = 10;
	ui32CheckTime2 = 100;
	ui32CheckTime3 = 70;
	ui32CheckTime4 = 2;
	ui32CheckTime5 = 700;
	ui32CheckTime6 = 60000;

	// Enable TIMER0 peripheral
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
	SysCtlDelay(10);

	// Enable TIMER1 peripheral
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT);
	SysCtlDelay(10);

	// Enable TIMER2 peripheral
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	ROM_TimerConfigure(TIMER2_BASE, TIMER_CFG_ONE_SHOT);
	SysCtlDelay(10);

	// Enable TIMER3 peripheral
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
	ROM_TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
	SysCtlDelay(10);

	// Enable TIMER4 peripheral
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
	ROM_TimerConfigure(TIMER4_BASE, TIMER_CFG_ONE_SHOT);
	SysCtlDelay(10);

	// Enable TIMER5 peripheral
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
	ROM_TimerConfigure(TIMER5_BASE, TIMER_CFG_ONE_SHOT);
	SysCtlDelay(10);

	// Enable TIMER0_A Clock
	ui32Period1 = (SysCtlClockGet() / 1000)*ui32CheckTime1;
	ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period1);
	SysCtlDelay(10);

	//  Enable TIMER1_A Clock
	ui32Period2 = (SysCtlClockGet() / 1000)*ui32CheckTime2;
	ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period2);
	SysCtlDelay(10);

	//  Enable TIMER2_A Clock
	ui32Period3 = (SysCtlClockGet() / 1000)*ui32CheckTime3;
	ROM_TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period3);
	SysCtlDelay(10);

	//  Enable TIMER3_A Clock
	ui32Period4 = (SysCtlClockGet() / 1000)*ui32CheckTime4;
	ROM_TimerLoadSet(TIMER3_BASE, TIMER_A, ui32Period4);
	SysCtlDelay(10);

	//  Enable TIMER4_A Clock
	ui32Period5 = (SysCtlClockGet() / 1000)*ui32CheckTime5;
	ROM_TimerLoadSet(TIMER4_BASE, TIMER_A, ui32Period5);
	SysCtlDelay(10);

	//  Enable TIMER5_A Clock
	ui32Period6 = (SysCtlClockGet() / 1000)*ui32CheckTime6;
	ROM_TimerLoadSet(TIMER5_BASE, TIMER_A, ui32Period6);
	SysCtlDelay(10);

	// Enable TIMER0_A Interrupt
	ROM_IntEnable(INT_TIMER0A);
	ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	SysCtlDelay(10);

	// Enable TIMER1_A Interrupt
	ROM_IntEnable(INT_TIMER1A);
	ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	SysCtlDelay(10);

	// Enable TIMER2_A Interrupt
	ROM_IntEnable(INT_TIMER2A);
	ROM_TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	SysCtlDelay(10);

	// Enable TIMER3_A Interrupt
	ROM_IntEnable(INT_TIMER3A);
	ROM_TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	SysCtlDelay(10);

	// Enable TIMER4_A Interrupt
	ROM_IntEnable(INT_TIMER4A);
	ROM_TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
	SysCtlDelay(10);

	// Enable TIMER5_A Interrupt
	ROM_IntEnable(INT_TIMER5A);
	ROM_TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
	SysCtlDelay(10);

	// Set TIMER0_A Interrupt priority
	//	ROM_IntPrioritySet(INT_TIMER0A,0x40);

	// Set TIMER0_B Interrupt priority
	//	ROM_IntPrioritySet(INT_TIMER0B,0x20);

	// Set TIMER1_A Interrupt priority
	//	ROM_IntPrioritySet(INT_TIMER1A,0x10);


}


void SetupInterrupt(void)
{
	//Enable periheral GPIOC
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	//Enable periheral GPIOD
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	//Enable periheral GPIOF
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	// Enabled in UART
	//	//Enable periheral GPIOA
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// GPIO port D pin 7 is locked, unlock by direct register writes to GPIOD_LOCK and GPIOD_CR
	GPIOD_LOCK = 0x4C4F434B;

	while (GPIOD_LOCK!=0x0)
	{

	}

	GPIOD_CR|= 0x80;

	// GPIO port F pin 0 is locked, unlock by direct register writes to GPIOF_LOCK and GPIOF_CR
	GPIOF_LOCK = 0x4C4F434B;

	while (GPIOF_LOCK!=0x0)
	{

	}

	GPIOF_CR|= 0x01;

	// Configure GPIO_PORTC_BASE,  GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 pins as input
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTC_BASE,  GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

	// Configure GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7 pins as input
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, 0xF0);

	// Configure GPIO_PORTF_BASE,  GPIO_PIN_4 pins as input
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,   GPIO_PIN_0 | GPIO_PIN_4 );

	// Configure GPIO_PORTA_BASE,  GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4  pins as input
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE,  GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);

	// Configure GPIO_PORTB_BASE,  GPIO_PIN_6 | GPIO_PIN_7 pins as input
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,  GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7);

	// Configure GPIO_PORTE_BASE,  GPIO_PIN_1 pins as input
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);

	// Set up the interrupt trigger mechanism for GPIO_PORTC_BASE,  GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7
	ROM_GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_RISING_EDGE);

	// Set up the interrupt trigger mechanism for GPIO_PORTD_BASE,  GPIO_PIN_6 | GPIO_PIN_7
	ROM_GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_RISING_EDGE);

	// Set up the interrupt trigger mechanism for GPIO_PORTF_BASE,  GPIO_PIN_0 | GPIO_PIN_4
	ROM_GPIOIntTypeSet(GPIO_PORTF_BASE,  GPIO_PIN_0 | GPIO_PIN_4, GPIO_RISING_EDGE);

	// Set up the interrupt trigger mechanism for GPIO_PORTA_BASE,  GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4
	ROM_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 , GPIO_RISING_EDGE);

	// Set up the interrupt trigger mechanism for GPIO_PORTB_BASE, GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7
	ROM_GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_RISING_EDGE);

	// Set up the interrupt trigger mechanism for GPIO_PORTB_BASE, GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7
	ROM_GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_RISING_EDGE);

	// Enable GPIO interrupt
	ROM_IntEnable(INT_GPIOC);

	// Enable GPIO interrupt
	ROM_IntEnable(INT_GPIOD);

	// Enable GPIO interrupt
	ROM_IntEnable(INT_GPIOF);

	// Enable GPIO interrupt
	ROM_IntEnable(INT_GPIOA);

	// Enable GPIO interrupt
	ROM_IntEnable(INT_GPIOB);

	// Enable GPIO interrupt
	ROM_IntEnable(INT_GPIOE);

	//	// Enable interrupts on GPIO port C
	//	GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	//
	//	// Enable interrupts on GPIO port D
	//	GPIOIntEnable(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	//
	//	// Enable interrupts on GPIO port F
	//	GPIOIntEnable(GPIO_PORTF_BASE,  GPIO_PIN_0 | GPIO_PIN_4);
	//
	//	// Enable interrupts on GPIO port A
	//	GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);
	//
	//	// Enable interrupts on GPIO port B
	//	GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_3 |GPIO_PIN_6 | GPIO_PIN_7);
	//
	//	ROM_IntMasterEnable();

}

void SetupOutputs(void)
{
	//	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//	SysCtlDelay(10);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_5);

	///////// Initialise Gripper to open position

	// Gripper Open
	ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);

	// Ensure gripper PWM is enabled
	ROM_PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);

	SysCtlDelay(24000000);
	// Start Timer4_A to disable
	ROM_TimerEnable(TIMER4_BASE, TIMER_A);

}

