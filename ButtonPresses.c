#include "stdlib.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

extern void Position( double *, double *, double *, double *);
extern void DEMO( double *, double *, double *, double *);

void Timer5_A_INT(void);

volatile double Rotate[3] = {90, 90, 90}, GripTicks = (1500/0.2)-1, X[2] ={420,420}, Y[2] = {0,0}, Z[2] = {0,0}, d1=200, d2=220;
int16_t counter_1=0, counter_2=0, movement=10;
uint16_t ui16PWM_3_ADJUST = ((1500/0.2)-1), ui16PWM_4_ADJUST = ((1500/0.2)-1);

void ButtonPress(volatile uint32_t PinStates_C, volatile uint32_t PinStates_D, volatile uint32_t PinStates_F, volatile uint32_t PinStates_A)
{


	if ( (PinStates_C & 0x30) > 0 )
	{
		if ((PinStates_C & 0x30) == 0x30)
		{
			// Do nothing
		}
		else if ( (PinStates_C & 0x30) == 0x10)
		{
			X[0]=X[1]+movement;
		}

		else if ( (PinStates_C & 0x30) == 0x20)
		{
			X[0]=X[1]-movement;
		}
	}


	if ( (PinStates_C & 0xC0) > 0 )
	{
		if ((PinStates_C & 0xC0)== 0xC0)
		{
			// Do nothing
		}
		else if ( (PinStates_C & 0xC0) == 0x40)
		{
			Y[0]=Y[1]+movement;

		}

		else if ( (PinStates_C & 0xC0)== 0x80)
		{
			Y[0]=Y[1]-movement;
		}
	}


	if ( (PinStates_D & 0xC0) > 0 )
	{

		if ((PinStates_D & 0xC0)==0xC0)
		{
			// Do nothing
		}
		else if ( (PinStates_D & 0xC0) == 0x40)
		{
			Z[0]=Z[1]+movement;
		}

		else if ( (PinStates_D & 0xC0) == 0x80)
		{

			if ( (Z[1]-movement) >= -160)
			{
				Z[0]=Z[1]-movement;
			}

		}
	}


	if ( ((PinStates_F | (PinStates_A & 0x04)) & 0x14)>0 )
	{
		if ( ((PinStates_F | (PinStates_A & 0x04)) & 0x14) == 0x14)
		{
			// Do nothing
		}

		else if ( ((PinStates_F | (PinStates_A & 0x04)) & 0x14)  == 0x10)
		{
			if(Rotate[2]<180)
			{
				Rotate[0]=Rotate[2]+(10);
				Rotate[2]=Rotate[0];
			}
		}

		else if ( ((PinStates_F | (PinStates_A & 0x04)) & 0x14)  == 0x04)
		{
			if(Rotate[2]>0)
			{
				Rotate[0]=Rotate[2]-(10);
				Rotate[2]=Rotate[0];
			}
		}

	}

	if ( (X[0]!=X[1]) || (Y[0]!=Y[1]) || (Z[0]!=Z[1]) || (Rotate[1]!=Rotate[0])  )
		{
			Position(X,Y,Z,Rotate);
		}

	if ( (PinStates_A & 0x18) > 0 )
	{
		if ( (PinStates_A & 0x18) == 0x18)
		{
			// Do nothing

		}
		else if ( (PinStates_A & 0x18) == 0x10)
		{
			// Gripper Open
			ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);
			// Ensure gripper PWM is enabled
			ROM_PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);

			// Start Timer4_A to disable
			ROM_TimerEnable(TIMER4_BASE, TIMER_A);

			counter_2--;
		}

		else if ( (PinStates_A & 0x18) == 0x08)
		{
			// Gripper Close
			ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0);
			// Ensure gripper PWM is enabled
			ROM_PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);

			// Start Timer4_A to disable
			ROM_TimerEnable(TIMER4_BASE, TIMER_A);

			counter_2++;
		}
	}


}


void Timer5_A_INT(void)
{
	// Clear the timer interrupt
	ROM_TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
	// Call DEMO
	DEMO(X,Y,Z,Rotate);

}
