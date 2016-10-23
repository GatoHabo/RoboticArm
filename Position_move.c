#include "stdlib.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#define PI 3.141592653589793238462

void Timer3_A_INT(void);
void Timer4_A_INT(void);
void DEMO( double *,double *,double *,double *);

volatile double theta1[4]={0,0,0,0}, phi2[4]={0,0,0,0}, phi3[4]={0,0,0,0}, Rotate1, Rotate2, Rotate3,
		MotorAngles[3]={ (7620 + (18.59941447*(90)) - 1), 7492.5,  7584.2},
		Vel1=0, Vel2=0.15, Vel3=0.15, Vel4=0.15, XTest, YTest, ZTest, count1=0,
		count2=0, count3=0, RotateTicks=7500, RotateEND[3]={90,90,90};

void Position( double * Px, double * Py, double * Pz, double * RotateE)
{
	static double  d1=200, d2=220, P1_1[3] = {0,0,0}, P2_1[3] = {200 ,0,  0},
			P3_1[3] = {420,0,0}, Delta1=0,Delta2=0,	P2_1_rotate[3]={0,0,0},
			P3_1_rotate[3]={0,0,0},	t[3]={0,0,0}, Move1[3] = {0,0,0},
			Move2[3]= {0,0,0}, tol=0.1, b[3] = {0,0,0}, thetaTarget=0,
			phiTarget=0,R2=0,R3=0,R=0, difA=0, ELBOW=1,  RT=420, XSign=1,
			YSign=0, ZSign=0, theta1Sign=0, phi2Sign=0, phi3Sign=0;

	if ( (theta1[1]-theta1[0])<=1 )
	{
		theta1[3]=theta1[0];
	}

	if ( (phi2[1]-phi2[0])<=1 )
	{
		phi2[3]=phi2[0];
	}

	if ( (phi3[1]-phi3[0])<=1 )
	{
		phi3[3]=phi3[0];
	}

	RotateEND[0]=RotateE[0];
	RotateEND[2]=RotateE[2];

	if ((Px[0]==Px[1])&&(Py[0]==Py[1])&&(Pz[0]==Pz[1]))
	{
		ROM_TimerEnable(TIMER3_BASE, TIMER_A);
		return;
	}

	thetaTarget = atan2(Py[0],Px[0]);

	theta1[2]= thetaTarget*180/PI;


	if (theta1[2]<-90)
	{
		theta1[2]=  360 + theta1[2];
	}


	if (Px[0]<0)
	{
		XSign=-1;
	}
	else if (Px[0]>0)
	{
		XSign=1;
	}
	else
	{
		XSign=0;
	}

	if (Py[0]<0)
	{
		YSign=-1;
	}
	else if (Py[0]>0)
	{
		YSign=1;
	}
	else
	{
		YSign=0;
	}

	if (Pz[0]<0)
	{
		ZSign=-1;
	}
	else if (Pz[0]>0)
	{
		ZSign=1;
	}
	else
	{
		ZSign=0;
	}

	if (theta1[0]<0)
	{
		theta1Sign=-1;
	}
	else if (theta1[0]>0)
	{
		theta1Sign=1;
	}
	else
	{
		theta1Sign=0;
	}

	if (phi2[0]<0)
	{
		phi2Sign=-1;
	}
	else if (phi2[0]>0)
	{
		phi2Sign=1;
	}
	else
	{
		phi2Sign=0;
	}

	if (phi3[0]<0)
	{
		phi3Sign=-1;
	}
	else if (phi3[0]>0)
	{
		phi3Sign=1;
	}
	else
	{
		phi3Sign=0;
	}


	if ( ((sqrt( (pow(Px[0],2)) + (pow(Py[0],2)) + (pow(Pz[0],2)) )) > (d1+d2)) )
	{
		phi3[2]=0;
		phi3[0]=0;

		if ( (Px[0]>Px[1]) && (Px[1]<=420) )
		{
			if ( (Py[0]==Py[1]) && (Pz[0]==Pz[1]) )
			{

				if (  ( (theta1[0]<0) && ((theta1[0]+5)>0) )  || ( (theta1[0]>0) && ((theta1[0]-5)<0) ) || (theta1[0]==0)  )
				{
					if (  ( (phi2[0]<0) && ((phi2[0]+5)>0) )  || ( (phi2[0]>0) && ((phi2[0]-5)<0) ) || (phi2[0]==0))
					{
						phi2[2]=0;
					}
					else
					{
						phi2[2]=(abs(phi2[0])-5)*phi2Sign;
					}

					phi2[0]=phi2[2];

					theta1[2]=0;
					theta1[0]=theta1[2];


				}
				else
				{
					if (  ( (phi2[0]<0) && ((phi2[0]+5)>0) )  || ( (phi2[0]>0) && ((phi2[0]-5)<0) ) || (phi2[0]==0))
					{
						phi2[2]=0;
					}
					else
					{
						phi2[2]=(abs(phi2[0])-5)*phi2Sign;
					}

					phi2[0]=phi2[2];
					theta1[2]=(abs(theta1[0])-5)*theta1Sign;
					theta1[0]=theta1[2];
				}
			}

			else if ( (Py[0]>Py[1]) && (Pz[0]==Pz[1]) )
			{

				if (  ( (theta1[0]<45) && ((theta1[0]+5)>45) )  || ( (theta1[0]>45) && ((theta1[0]-5)<45) ) || (theta1[0]==45)  )
				{
					if (  ( (phi2[0]<0) && ((phi2[0]+5)>0) )  || ( (phi2[0]>0) && ((phi2[0]-5)<0) ) || (phi2[0]==0))
					{
						phi2[2]=0;
					}
					else
					{
						phi2[2]=(abs(phi2[0])-5)*phi2Sign;
					}

					phi2[0]=phi2[2];

					theta1[2]=45;
					theta1[0]=theta1[2];
				}
				else
				{
					if (  ( (phi2[0]<0) && ((phi2[0]+5)>0) )  || ( (phi2[0]>0) && ((phi2[0]-5)<0) ) || (phi2[0]==0))
					{
						phi2[2]=0;
					}
					else
					{
						phi2[2]=(abs(phi2[0])-5)*phi2Sign;
					}
					phi2[0]=phi2[2];

					if (theta1[0]<45)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}

					theta1[0]=theta1[2];
				}

			}
			else if ( (Py[0]<Py[1]) && (Pz[0]==Pz[1]) )
			{

				if (  ( (theta1[0]<-43) && ((theta1[0]+5)>-43) )  || ( (theta1[0]>-43) && ((theta1[0]-5)<-43) ) || (theta1[0]==-43)  )
				{
					if (  ( (phi2[0]<0) && ((phi2[0]+5)>0) )  || ( (phi2[0]>0) && ((phi2[0]-5)<0) ) || (phi2[0]==0))
					{
						phi2[2]=0;
					}
					else
					{
						phi2[2]=(abs(phi2[0])-5)*phi2Sign;
					}

					phi2[0]=phi2[2];

					theta1[2]=-43;
					theta1[0]=theta1[2];
				}
				else
				{
					if (  ( (phi2[0]<0) && ((phi2[0]+5)>0) )  || ( (phi2[0]>0) && ((phi2[0]-5)<0) ) || (phi2[0]==0))
					{
						phi2[2]=0;
					}
					else
					{
						phi2[2]=(abs(phi2[0])-5)*phi2Sign;
					}
					phi2[0]=phi2[2];

					if (theta1[0]<-43)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}

					theta1[0]=theta1[2];
				}

			}

			else if ( (Py[0]==Py[1]) && (Pz[0]>Pz[1]) )
			{

				if (  ( (theta1[0]<0) && ((theta1[0]+5)>0) )  || ( (theta1[0]>0) && ((theta1[0]-5)<0) ) || (theta1[0]==0)  )
				{
					if (  ( (phi2[0]<45) && ((phi2[0]+5)>45) )  || ( (phi2[0]>45) && ((phi2[0]-5)<45) ) || (phi2[0]==45))
					{
						phi2[2]=45;
					}
					else if (phi2[0]<45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					theta1[2]=0;
					theta1[0]=theta1[2];
				}
				else
				{
					if (  ( (phi2[0]<45) && ((phi2[0]+5)>45) )  || ( (phi2[0]>45) && ((phi2[0]-5)<45) ) || (phi2[0]==45))
					{
						phi2[2]=45;
					}
					else if (phi2[0]<45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					theta1[2]=(abs(theta1[0])-5)*theta1Sign;
					theta1[0]=theta1[2];
				}

			}
			else if ( (Py[0]==Py[1]) && (Pz[0]<Pz[1]) )
			{

				if (  ( (theta1[0]<0) && ((theta1[0]+5)>0) )  || ( (theta1[0]>0) && ((theta1[0]-5)<0) ) || (theta1[0]==0)  )
				{
					if (  ( (phi2[0]<-45) && ((phi2[0]+5)>-45) )  || ( (phi2[0]>-45) && ((phi2[0]-5)<-45) ) || (phi2[0]==-45))
					{
						phi2[2]=-45;
					}
					else if (phi2[0]<-45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>-45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					theta1[2]=0;
					theta1[0]=theta1[2];
				}
				else
				{
					if (  ( (phi2[0]<-45) && ((phi2[0]+5)>-45) )  || ( (phi2[0]>-45) && ((phi2[0]-5)<-45) ) || (phi2[0]==-45))
					{
						phi2[2]=-45;
					}
					else if (phi2[0]<-45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>-45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					if (theta1[0]<0)
					{
						theta1[2]=(abs(theta1[0])+5)*theta1Sign;
					}
					else
					{
						theta1[2]=(abs(theta1[0])-5)*theta1Sign;
					}

					theta1[0]=theta1[2];
				}

			}
			else if ( (Py[0]>Py[1]) && (Pz[0]>Pz[1]) )
			{

				if (  ( (theta1[0]<45) && ((theta1[0]+5)>45) )  || ( (theta1[0]>45) && ((theta1[0]-5)<45) ) || (theta1[0]==45)  )
				{
					if (  ( (phi2[0]<45) && ((phi2[0]+5)>45) )  || ( (phi2[0]>45) && ((phi2[0]-5)<45) ) || (phi2[0]==45))
					{
						phi2[2]=45;
					}
					else if (phi2[0]<45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					theta1[2]=45;
					theta1[0]=theta1[2];
				}
				else
				{
					if (  ( (phi2[0]<45) && ((phi2[0]+5)>45) )  || ( (phi2[0]>45) && ((phi2[0]-5)<45) ) || (phi2[0]==45))
					{
						phi2[2]=45;
					}
					else if (phi2[0]<45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					if (theta1[0]<45)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}

					theta1[0]=theta1[2];
				}

			}

			else if ( (Py[0]>Py[1]) && (Pz[0]<Pz[1]) )
			{

				if (  ( (theta1[0]<45) && ((theta1[0]+5)>45) )  || ( (theta1[0]>45) && ((theta1[0]-5)<45) ) || (theta1[0]==45)  )
				{
					if (  ( (phi2[0]<-45) && ((phi2[0]+5)>-45) )  || ( (phi2[0]>-45) && ((phi2[0]-5)<-45) ) || (phi2[0]==-45))
					{
						phi2[2]=-45;
					}
					else if (phi2[0]<-45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>-45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					theta1[2]=45;
					theta1[0]=theta1[2];
				}
				else
				{
					if (  ( (phi2[0]<-45) && ((phi2[0]+5)>-45) )  || ( (phi2[0]>-45) && ((phi2[0]-5)<-45) ) || (phi2[0]==-45))
					{
						phi2[2]=-45;
					}
					else if (phi2[0]<-45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>-45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					if (theta1[0]<45)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}

					theta1[0]=theta1[2];
				}

			}


			else if ( (Py[0]<Py[1]) && (Pz[0]<Pz[1]) )
			{

				if (  ( (theta1[0]<-43) && ((theta1[0]+5)>-43) )  || ( (theta1[0]>-43) && ((theta1[0]-5)<-43) ) || (theta1[0]==-43)  )
				{
					if (  ( (phi2[0]<-45) && ((phi2[0]+5)>-45) )  || ( (phi2[0]>-45) && ((phi2[0]-5)<-45) ) || (phi2[0]==-45))
					{
						phi2[2]=-45;
					}
					else if (phi2[0]<-45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>-45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];
				}
				else
				{
					if (  ( (phi2[0]<-45) && ((phi2[0]+5)>-45) )  || ( (phi2[0]>-45) && ((phi2[0]-5)<-45) ) || (phi2[0]==-45))
					{
						phi2[2]=-45;
					}
					else if (phi2[0]<-45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>-45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					if (theta1[0]<-43)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}

					theta1[0]=theta1[2];
				}

			}

			else if ( (Py[0]<Py[1]) && (Pz[0]>Pz[1]) )
			{

				if (  ( (theta1[0]<-43) && ((theta1[0]+5)>-43) )  || ( (theta1[0]>-43) && ((theta1[0]-5)<-43) ) || (theta1[0]==-43)  )
				{
					if (  ( (phi2[0]<45) && ((phi2[0]+5)>45) )  || ( (phi2[0]>45) && ((phi2[0]-5)<45) ) || (phi2[0]==45))
					{
						phi2[2]=45;
					}
					else if (phi2[0]<45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					theta1[2]=-43;
					theta1[0]=theta1[2];
				}
				else
				{
					if (  ( (phi2[0]<45) && ((phi2[0]+5)>45) )  || ( (phi2[0]>45) && ((phi2[0]-5)<45) ) || (phi2[0]==45))
					{
						phi2[2]=45;
					}
					else if (phi2[0]<45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					if (theta1[0]<-43)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}

					theta1[0]=theta1[2];
				}

			}

		}
		else if ( (Px[0]<Px[1]) && (abs(Px[1])<=420) )
		{
			if ( (Py[0]==Py[1]) && (Pz[0]==Pz[1]) )
			{

				if (  ( (theta1[0]<180) && ((theta1[0]+5)>180) )  || ( (theta1[0]>180) && ((theta1[0]-5)<180) ) || (theta1[0]==180)  )
				{
					if (  ( (phi2[0]<0) && ((phi2[0]+5)>0) )  || ( (phi2[0]>0) && ((phi2[0]-5)<0) ) || (phi2[0]==0))
					{
						phi2[2]=0;
					}
					else
					{
						phi2[2]=(abs(phi2[0])-5)*phi2Sign;
					}

					phi2[0]=phi2[2];

					theta1[2]=180;
					theta1[0]=theta1[2];
				}
				else
				{
					if (  ( (phi2[0]<0) && ((phi2[0]+5)>0) )  || ( (phi2[0]>0) && ((phi2[0]-5)<0) ) || (phi2[0]==0))
					{
						phi2[2]=0;
					}
					else
					{
						phi2[2]=(abs(phi2[0])-5)*phi2Sign;
					}
					phi2[0]=phi2[2];

					if (theta1[0]<180)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}

					theta1[0]=theta1[2];
				}
			}

			else if ( (Py[0]>Py[1]) && (Pz[0]==Pz[1]) )
			{

				if (  ( (theta1[0]<135) && ((theta1[0]+5)>135) )  || ( (theta1[0]>135) && ((theta1[0]-5)<135) ) || (theta1[0]==135)  )
				{
					if (  ( (phi2[0]<0) && ((phi2[0]+5)>0) )  || ( (phi2[0]>0) && ((phi2[0]-5)<0) ) || (phi2[0]==0))
					{
						phi2[2]=0;
					}
					else
					{
						phi2[2]=(abs(phi2[0])-5)*phi2Sign;
					}

					phi2[0]=phi2[2];

					theta1[2]=135;
					theta1[0]=theta1[2];
				}
				else
				{
					if (  ( (phi2[0]<0) && ((phi2[0]+5)>0) )  || ( (phi2[0]>0) && ((phi2[0]-5)<0) ) || (phi2[0]==0))
					{
						phi2[2]=0;
					}
					else
					{
						phi2[2]=(abs(phi2[0])-5)*phi2Sign;
					}
					phi2[0]=phi2[2];

					if (theta1[0]<135)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}

					theta1[0]=theta1[2];
				}

			}
			else if ( (Py[0]<Py[1]) && (Pz[0]==Pz[1]) )
			{

				if (  ( (theta1[0]<223) && ((theta1[0]+5)>223) )  || ( (theta1[0]>313) && ((theta1[0]-5)<223) ) || (theta1[0]==223)  )
				{
					if (  ( (phi2[0]<0) && ((phi2[0]+5)>0) )  || ( (phi2[0]>0) && ((phi2[0]-5)<0) ) || (phi2[0]==0))
					{
						phi2[2]=0;
					}
					else
					{
						phi2[2]=(abs(phi2[0])-5)*phi2Sign;
					}

					phi2[0]=phi2[2];

					theta1[2]=223;
					theta1[0]=theta1[2];
				}
				else
				{
					if (  ( (phi2[0]<0) && ((phi2[0]+5)>0) )  || ( (phi2[0]>0) && ((phi2[0]-5)<0) ) || (phi2[0]==0))
					{
						phi2[2]=0;
					}
					else
					{
						phi2[2]=(abs(phi2[0])-5)*phi2Sign;
					}
					phi2[0]=phi2[2];

					if (theta1[0]<223)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}

					theta1[0]=theta1[2];
				}

			}

			else if ( (Py[0]==Py[1]) && (Pz[0]>Pz[1]) )
			{

				if (  ( (theta1[0]<180) && ((theta1[0]+5)>180) )  || ( (theta1[0]>180) && ((theta1[0]-5)<180) ) || (theta1[0]==180)  )
				{
					if (  ( (phi2[0]<45) && ((phi2[0]+5)>45) )  || ( (phi2[0]>45) && ((phi2[0]-5)<45) ) || (phi2[0]==45))
					{
						phi2[2]=45;
					}
					else if (phi2[0]<45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					theta1[2]=180;
					theta1[0]=theta1[2];
				}
				else
				{
					if (  ( (phi2[0]<45) && ((phi2[0]+5)>45) )  || ( (phi2[0]>45) && ((phi2[0]-5)<45) ) || (phi2[0]==45))
					{
						phi2[2]=45;
					}
					else if (phi2[0]<45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					if (theta1[0]<180)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}

					theta1[0]=theta1[2];
				}

			}
			else if ( (Py[0]==Py[1]) && (Pz[0]<Pz[1]) )
			{

				if (  ( (theta1[0]<180) && ((theta1[0]+5)>180) )  || ( (theta1[0]>180) && ((theta1[0]-5)<180) ) || (theta1[0]==180)  )
				{
					if (  ( (phi2[0]<-45) && ((phi2[0]+5)>-45) )  || ( (phi2[0]>-45) && ((phi2[0]-5)<-45) ) || (phi2[0]==-45))
					{
						phi2[2]=-45;
					}
					else if (phi2[0]<-45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>-45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					theta1[2]=180;
					theta1[0]=theta1[2];
				}
				else
				{
					if (  ( (phi2[0]<-45) && ((phi2[0]+5)>-45) )  || ( (phi2[0]>-45) && ((phi2[0]-5)<-45) ) || (phi2[0]==-45))
					{
						phi2[2]=-45;
					}
					else if (phi2[0]<-45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>-45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					if (theta1[0]<180)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}

					theta1[0]=theta1[2];
				}

			}
			else if ( (Py[0]>Py[1]) && (Pz[0]>Pz[1]) )
			{

				if (  ( (theta1[0]<135) && ((theta1[0]+5)>135) )  || ( (theta1[0]>135) && ((theta1[0]-5)<135) ) || (theta1[0]==135)  )
				{
					if (  ( (phi2[0]<45) && ((phi2[0]+5)>45) )  || ( (phi2[0]>45) && ((phi2[0]-5)<45) ) || (phi2[0]==45))
					{
						phi2[2]=45;
					}
					else if (phi2[0]<45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					theta1[2]=135;
					theta1[0]=theta1[2];
				}
				else
				{
					if (  ( (phi2[0]<45) && ((phi2[0]+5)>45) )  || ( (phi2[0]>45) && ((phi2[0]-5)<45) ) || (phi2[0]==45))
					{
						phi2[2]=45;
					}
					else if (phi2[0]<45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					if (theta1[0]<135)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}

					theta1[0]=theta1[2];
				}

			}

			else if ( (Py[0]>Py[1]) && (Pz[0]<Pz[1]) )
			{

				if (  ( (theta1[0]<135) && ((theta1[0]+5)>135) )  || ( (theta1[0]>135) && ((theta1[0]-5)<135) ) || (theta1[0]==135)  )
				{
					if (  ( (phi2[0]<-45) && ((phi2[0]+5)>-45) )  || ( (phi2[0]>-45) && ((phi2[0]-5)<-45) ) || (phi2[0]==-45))
					{
						phi2[2]=-45;
					}
					else if (phi2[0]<-45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>-45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					theta1[2]=135;
					theta1[0]=theta1[2];
				}
				else
				{
					if (  ( (phi2[0]<-45) && ((phi2[0]+5)>-45) )  || ( (phi2[0]>-45) && ((phi2[0]-5)<-45) ) || (phi2[0]==-45))
					{
						phi2[2]=-45;
					}
					else if (phi2[0]<-45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>-45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					if (theta1[0]<135)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}

					theta1[0]=theta1[2];
				}

			}


			else if ( (Py[0]<Py[1]) && (Pz[0]<Pz[1]) )
			{

				if (  ( (theta1[0]<223) && ((theta1[0]+5)>223) )  || ( (theta1[0]>223) && ((theta1[0]-5)<223) ) || (theta1[0]==223)  )
				{
					if (  ( (phi2[0]<-45) && ((phi2[0]+5)>-45) )  || ( (phi2[0]>-45) && ((phi2[0]-5)<-45) ) || (phi2[0]==-45))
					{
						phi2[2]=-45;
					}
					else if (phi2[0]<-45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>-45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					theta1[2]=223;
					theta1[0]=theta1[2];
				}
				else
				{
					if (  ( (phi2[0]<-45) && ((phi2[0]+5)>-45) )  || ( (phi2[0]>-45) && ((phi2[0]-5)<-45) ) || (phi2[0]==-45))
					{
						phi2[2]=-45;
					}
					else if (phi2[0]<-45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>-45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					if (theta1[0]<223)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}

					theta1[0]=theta1[2];

				}

			}

			else if ( (Py[0]<Py[1]) && (Pz[0]>Pz[1]) )
			{

				if (  ( (theta1[0]<223) && ((theta1[0]+5)>223) )  || ( (theta1[0]>223) && ((theta1[0]-5)<223) ) || (theta1[0]==223)  )
				{
					if (  ( (phi2[0]<45) && ((phi2[0]+5)>45) )  || ( (phi2[0]>45) && ((phi2[0]-5)<45) ) || (phi2[0]==45))
					{
						phi2[2]=45;
					}
					else if (phi2[0]<45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					theta1[2]=223;
					theta1[0]=theta1[2];

				}
				else
				{
					if (  ( (phi2[0]<45) && ((phi2[0]+5)>45) )  || ( (phi2[0]>45) && ((phi2[0]-5)<45) ) || (phi2[0]==45))
					{
						phi2[2]=45;
					}
					else if (phi2[0]<45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					if (theta1[0]<223)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}

					theta1[0]=theta1[2];
				}

			}


		}

		else if ( (Px[0]==Px[1]) && (Py[0]>Py[1]) && (Py[1]<=420)  )
		{
			if (Pz[0]==Pz[1])
			{
				if (  ( (theta1[0]<90) && ((theta1[0]+5)>90) )  || ( (theta1[0]>90) && ((theta1[0]-5)<90) ) || (theta1[0]==90))
				{
					if (  ( (phi2[0]<0) && ((phi2[0]+5)>0) )  || ( (phi2[0]>0) && ((phi2[0]-5)<0) ) || (phi2[0]==0))
					{
						phi2[2]=0;
					}
					else
					{
						phi2[2]=(abs(phi2[0])-5)*phi2Sign;
					}

					phi2[0]=phi2[2];
					theta1[2]=90;
					theta1[0]=theta1[2];
				}
				else
				{
					if (  ( (phi2[0]<0) && ((phi2[0]+5)>0) )  || ( (phi2[0]>0) && ((phi2[0]-5)<0) ) || (phi2[0]==0))
					{
						phi2[2]=0;
					}
					else
					{
						phi2[2]=(abs(phi2[0])-5)*phi2Sign;
					}

					phi2[0]=phi2[2];

					if (theta1[0]<90)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}
					theta1[0]=theta1[2];
				}
			}
			else if (Pz[0]>Pz[1])
			{
				if (  ( (theta1[0]<90) && ((theta1[0]+5)>90) )  || ( (theta1[0]>90) && ((theta1[0]-5)<90) ) || (theta1[0]==90)  )
				{
					if (  ( (phi2[0]<45) && ((phi2[0]+5)>45) )  || ( (phi2[0]>45) && ((phi2[0]-5)<45) ) || (phi2[0]==45))
					{
						phi2[2]=45;
					}
					else if (phi2[0]<45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					theta1[2]=90;
					theta1[0]=theta1[2];

				}
				else
				{
					if (  ( (phi2[0]<45) && ((phi2[0]+5)>45) )  || ( (phi2[0]>45) && ((phi2[0]-5)<45) ) || (phi2[0]==45))
					{
						phi2[2]=45;
					}
					else if (phi2[0]<45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					if (theta1[0]<90)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}

					theta1[0]=theta1[2];

				}

			}
			else if (Pz[0]<Pz[1])
			{
				if (  ( (theta1[0]<90) && ((theta1[0]+5)>90) )  || ( (theta1[0]>90) && ((theta1[0]-5)<90) ) || (theta1[0]==90)  )
				{
					if (  ( (phi2[0]<-45) && ((phi2[0]+5)>-45) )  || ( (phi2[0]>-45) && ((phi2[0]-5)<-45) ) || (phi2[0]==-45))
					{
						phi2[2]=-45;
					}
					else if (phi2[0]<-45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>-45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					theta1[2]=90;
					theta1[0]=theta1[2];

				}
				else
				{
					if (  ( (phi2[0]<-45) && ((phi2[0]+5)>-45) )  || ( (phi2[0]>-45) && ((phi2[0]-5)<-45) ) || (phi2[0]==-45))
					{
						phi2[2]=-45;
					}
					else if (phi2[0]<-45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>-45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					if (theta1[0]<90)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}

					theta1[0]=theta1[2];

				}

			}

		}

		else if ( (Px[0]==Px[1]) && (Py[0]<Py[1]) )
		{
			if (Pz[0]==Pz[1])
			{
				if (  ( (theta1[0]>-43) && ((theta1[0]-5)<-43) ) || (theta1[0]==-43))
				{
					if (  ( (phi2[0]<0) && ((phi2[0]+5)>0) )  || ( (phi2[0]>0) && ((phi2[0]-5)<0) ) || (phi2[0]==0))
					{
						phi2[2]=0;
					}
					else
					{
						phi2[2]=(abs(phi2[0])-5)*phi2Sign;
					}

					phi2[0]=phi2[2];
					theta1[2]=-43;
					theta1[0]=theta1[2];

				}
				else if (  ( (theta1[0]<223) && ((theta1[0]+5)>223) ) || (theta1[0]==223))
				{
					if (  ( (phi2[0]<0) && ((phi2[0]+5)>0) )  || ( (phi2[0]>0) && ((phi2[0]-5)<0) ) || (phi2[0]==0))
					{
						phi2[2]=0;
					}
					else
					{
						phi2[2]=(abs(phi2[0])-5)*phi2Sign;
					}

					phi2[0]=phi2[2];
					theta1[2]=223;
					theta1[0]=theta1[2];

				}
				else if (Px[1]>0)
				{
					if (  ( (phi2[0]<0) && ((phi2[0]+5)>0) )  || ( (phi2[0]>0) && ((phi2[0]-5)<0) ) || (phi2[0]==0))
					{
						phi2[2]=0;
					}
					else
					{
						phi2[2]=(abs(phi2[0])-5)*phi2Sign;
					}

					phi2[0]=phi2[2];


					theta1[2]=theta1[0]-5;
					theta1[0]=theta1[2];

				}
				else if (Px[1]<0)
				{
					if (  ( (phi2[0]<0) && ((phi2[0]+5)>0) )  || ( (phi2[0]>0) && ((phi2[0]-5)<0) ) || (phi2[0]==0))
					{
						phi2[2]=0;
					}
					else
					{
						phi2[2]=(abs(phi2[0])-5)*phi2Sign;
					}

					phi2[0]=phi2[2];

					theta1[2]=theta1[0]+5;
					theta1[0]=theta1[2];

				}
			}

			else if (Pz[0]>Pz[1])
			{
				if (  ( (theta1[0]<-43) && ((theta1[0]+5)>-43) )  || ( (theta1[0]>-43) && ((theta1[0]-5)<-43) ) || (theta1[0]==-43))
				{
					if (  ( (phi2[0]<45) && ((phi2[0]+5)>45) )  || ( (phi2[0]>45) && ((phi2[0]-5)<45) ) || (phi2[0]==45))
					{
						phi2[2]=45;
					}
					else if (phi2[0]<45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];
					theta1[2]=-43;
					theta1[0]=theta1[2];

				}
				else if (  ( (theta1[0]<223) && ((theta1[0]+5)>223) )  || ( (theta1[0]>223) && ((theta1[0]-5)<223) ) || (theta1[0]==223))
				{
					if (  ( (phi2[0]<45) && ((phi2[0]+5)>45) )  || ( (phi2[0]>45) && ((phi2[0]-5)<45) ) || (phi2[0]==45))
					{
						phi2[2]=45;
					}
					else if (phi2[0]<45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];
					theta1[2]=223;
					theta1[0]=theta1[2];

				}
				else if (Px[1]>0)
				{
					if (  ( (phi2[0]<45) && ((phi2[0]+5)>45) )  || ( (phi2[0]>45) && ((phi2[0]-5)<45) ) || (phi2[0]==45))
					{
						phi2[2]=45;
					}
					else if (phi2[0]<45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					if (theta1[0]<-43)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}
					theta1[0]=theta1[2];

				}
				else if (Px[1]<0)
				{
					if (  ( (phi2[0]<45) && ((phi2[0]+5)>45) )  || ( (phi2[0]>45) && ((phi2[0]-5)<45) ) || (phi2[0]==45))
					{
						phi2[2]=45;
					}
					else if (phi2[0]<45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					if (theta1[0]<223)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}
					theta1[0]=theta1[2];

				}
			}

			else if (Pz[0]<Pz[1])
			{
				if (  ( (theta1[0]<-43) && ((theta1[0]+5)>-43) )  || ( (theta1[0]>-43) && ((theta1[0]-5)<-43) ) || (theta1[0]==-43))
				{
					if (  ( (phi2[0]<-45) && ((phi2[0]+5)>-45) )  || ( (phi2[0]>-45) && ((phi2[0]-5)<-45) ) || (phi2[0]==-45))
					{
						phi2[2]=-45;
					}
					else if (phi2[0]<-45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>-45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];
					theta1[2]=-43;
					theta1[0]=theta1[2];

				}
				else if (  ( (theta1[0]<223) && ((theta1[0]+5)>223) )  || ( (theta1[0]>223) && ((theta1[0]-5)<223) ) || (theta1[0]==223))
				{
					if (  ( (phi2[0]<-45) && ((phi2[0]+5)>-45) )  || ( (phi2[0]>-45) && ((phi2[0]-5)<-45) ) || (phi2[0]==-45))
					{
						phi2[2]=-45;
					}
					else if (phi2[0]<-45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>-45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];
					theta1[2]=223;
					theta1[0]=theta1[2];

				}
				else if (Px[1]>0)
				{
					if (  ( (phi2[0]<-45) && ((phi2[0]+5)>-45) )  || ( (phi2[0]>-45) && ((phi2[0]-5)<-45) ) || (phi2[0]==-45))
					{
						phi2[2]=-45;
					}
					else if (phi2[0]<-45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>-45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					if (theta1[0]<-43)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}
					theta1[0]=theta1[2];

				}
				else if (Px[1]<0)
				{
					if (  ( (phi2[0]<-45) && ((phi2[0]+5)>-45) )  || ( (phi2[0]>-45) && ((phi2[0]-5)<-45) ) || (phi2[0]==-45))
					{
						phi2[2]=-45;
					}
					else if (phi2[0]<-45)
					{
						phi2[2]=phi2[0]+5;
					}
					else if (phi2[0]>-45)
					{
						phi2[2]=phi2[0]-5;
					}

					phi2[0]=phi2[2];

					if (theta1[0]<223)
					{
						theta1[2]=theta1[0]+5;
					}
					else
					{
						theta1[2]=theta1[0]-5;
					}
					theta1[0]=theta1[2];

				}
			}


		}

		else if ( (Px[0]==Px[1]) && (Py[0]==Py[1]) && (Pz[0]>Pz[1])  && (Pz[1]<=420) )
		{
			if (  ( (phi2[0]<90) && ((phi2[0]+5)>90) )  ||  (phi2[0]==90) )
			{
				phi2[2]=90;
				phi2[0]=phi2[2];

			}
			else
			{
				phi2[2]=phi2[2]+5;
				phi2[0]=phi2[2];

			}

		}

		else if ( (Px[0]==Px[1]) && (Py[0]==Py[1]) && (Pz[0]<Pz[1]) )
		{
			if  ( ( (phi2[0]>-55) && ((phi2[0]-5)<-55) ) || (phi2[0]==-55) )
			{
				phi2[2]=-55;
				phi2[0]=phi2[2];

			}
			else
			{
				phi2[2]=phi2[2]-5;
				phi2[0]=phi2[2];


			}

		}

		else
		{
			Px[0]=Px[1];
			Py[0]=Py[1];
			Pz[0]=Pz[1];

			XTest= (cos(theta1[0]*PI/180)*((d1*cos(phi2[0]*PI/180))+(d2*cos((phi2[0]+phi3[0])*PI/180))));
			YTest= (sin(theta1[0]*PI/180)*((d1*cos(phi2[0]*PI/180))+(d2*cos((phi2[0]+phi3[0])*PI/180))));
			ZTest= ((d1*sin(phi2[0]*PI/180)+(d2*sin((phi2[0]+phi3[0])*PI/180))));

			return;
		}

		if (phi2[0]!=0)
		{
			P2_1[0]=d1*cos(theta1[0]*PI/180)*(d1*cos(phi2[0]*PI/180));
			P2_1[1]=d1*sin(theta1[0]*PI/180)*(d1*cos(phi2[0]*PI/180));
			P2_1[2]=d1*sin(phi2[0]*PI/180);

			P3_1[0]= (cos(theta1[0]*PI/180)*((d1*cos(phi2[0]*PI/180))+(d2*cos((phi2[0]+phi3[0])*PI/180))));
			P3_1[1]= (sin(theta1[0]*PI/180)*((d1*cos(phi2[0]*PI/180))+(d2*cos((phi2[0]+phi3[0])*PI/180))));
			P3_1[2]= ((d1*sin(phi2[0]*PI/180)+(d2*sin((phi2[0]+phi3[0])*PI/180))));
		}
		else
		{
			P2_1[0]=d1*cos(theta1[0]*PI/180);
			P2_1[1]=d1*sin(theta1[0]*PI/180);
			P2_1[2]=0;

			P3_1[0]= (d1+d2)*cos(theta1[0]*PI/180);
			P3_1[1]= (d1+d2)*sin(theta1[0]*PI/180);
			P3_1[2]= 0;
		}

		Px[0]=P3_1[0];
		Py[0]=P3_1[1];
		Pz[0]=P3_1[2];

		Px[1]=Px[0];
		Py[1]=Py[0];
		Pz[1]=Pz[0];

		XTest= (cos(theta1[0]*PI/180)*((d1*cos(phi2[0]*PI/180))+(d2*cos((phi2[0]+phi3[0])*PI/180))));
		YTest= (sin(theta1[0]*PI/180)*((d1*cos(phi2[0]*PI/180))+(d2*cos((phi2[0]+phi3[0])*PI/180))));
		ZTest= ((d1*sin(phi2[0]*PI/180)+(d2*sin((phi2[0]+phi3[0])*PI/180))));

		ROM_TimerEnable(TIMER3_BASE, TIMER_A);

		return;
	}


	if ( (sqrt( (pow(Px[0],2)) + (pow(Py[0],2) ) ) < 180.0) && (Pz[0]<=0) )
	{
		if ( ( sqrt( (pow(Px[0],2)) + (pow(Py[0],2) ) ) < 180.0 ) && ( Px[0]!=Px[1]) )
		{
			Px[0]=-Px[1];
		}
		else if ( ( sqrt( (pow(Px[0],2)) + (pow(Py[0],2) ) ) < 180.0 ) &&  (Px[0]>=0)  && (Px[0]==Px[1]) && (Py[0]!=Py[1] ) )
		{
			while ( sqrt( (pow(Px[0],2)) + (pow(Py[0],2) ) + (pow(Pz[0],2) )  ) < 180.0000000 )
			{
				Px[0]=Px[0]+10;
			}
		}
		else if ( ( sqrt( (pow(Px[0],2)) + (pow(Py[0],2) ) ) < 180.0 ) &&  (Px[0]<0)  && (Px[0]==Px[1]) && (Py[0]!=Py[1] ) )
		{
			while ( sqrt( (pow(Px[0],2)) + (pow(Py[0],2) ) + (pow(Pz[0],2) )  ) < 180.0000000 )
			{
				Px[0]=Px[0]-10;
			}
		}
	}
	else if ( (sqrt((pow(Px[0],2)) + (pow(Py[0],2)) + (pow(Pz[0],2))) < 180.0) && (Pz[0]>0) )
	{
		if ( ( Px[0]!=Px[1]) && (Py[0]==Py[1]) )
		{
			Px[0]=-Px[1];
		}
		else if ( (Px[0]>=0) && (Py[0]!=Py[1] ) )
		{
			while ( sqrt( (pow(Px[0],2)) + (pow(Py[0],2) ) + (pow(Pz[0],2) ) ) < 180.0 )
			{
				Px[0]=Px[0]+10;
			}

		}
		else if ( (Px[0]<0) && (Py[0]!=Py[1] ) )
		{
			while ( sqrt( (pow(Px[0],2)) + (pow(Py[0],2) ) + (pow(Pz[0],2) ) ) < 180.0 )
			{
				Px[0]=Px[0]-10;
			}

		}
		else if ( (Px[0]>=0) && (Px[0]==Px[1]) && (Py[0]==Py[1] ) && (Pz[0]<Pz[1]) )
		{
			while ( sqrt( (pow(Px[0],2)) + (pow(Py[0],2) ) + (pow(Pz[0],2) ) ) < 180.0 )
			{
				Px[0]=Px[0]+10;
			}

		}
		else if ( (Px[0]<0)  && (Px[0]==Px[1]) && (Py[0]==Py[1] ) && (Pz[0]<Pz[1]) )
		{
			while ( sqrt( (pow(Px[0],2)) + (pow(Py[0],2) ) + (pow(Pz[0],2) ) ) < 180.0 )
			{
				Px[0]=Px[0]-10;
			}

		}
		else if ( (Px[0]!=Px[1]) && (Py[0]!=Py[1]) )
		{
			Px[0]=-Px[1];
		}

	}

	// Calculate theta for target and motor 1
	thetaTarget = atan2(Py[0],Px[0]);

	theta1[2]= thetaTarget*180/PI;

	if ( (theta1[2]<(-43)) && (theta1[2]>(-137)) )
	{
		if ( (Px[0]!=Px[1]) && (Py[0]==Py[1]) )
		{
			Px[0]=-Px[1];
		}
		else if ( (Px[0]>=0) && (Py[0]<Py[1]) )
		{
			while ( (theta1[2]<(-43)) && (theta1[2]>(-137)) )
			{
				Px[0]=Px[0]+10;
				thetaTarget = atan2(Py[0],Px[0]);
				theta1[2]= thetaTarget*180/PI;
			}

		}
		else if ( (Px[0]<0) && (Py[0]<Py[1]) )
		{
			while ( (theta1[2]<(-43)) && (theta1[2]>(-137)) )
			{
				Px[0]=Px[0]-10;
				thetaTarget = atan2(Py[0],Px[0]);
				theta1[2]= thetaTarget*180/PI;
			}
		}
		else
		{
			Px[0]=Px[1];
			Py[0]=Py[1];
		}

	}

	thetaTarget = atan2(Py[0],Px[0]);
	theta1[2]= thetaTarget*180/PI;

	if (theta1[2]<-90)
	{
		theta1[2]=  360 + theta1[2];
	}

	t[0] = Px[0];
	t[1] = Py[0];
	t[2] = Pz[0];

	// Calculate R in the (x, y) plane
	R2=sqrt((pow(P2_1[0],2))+(pow(P2_1[1],2)));
	R3=sqrt((pow(P3_1[0],2))+(pow(P3_1[1],2)));
	R=sqrt((pow(t[0],2))+(pow(t[1],2)));

	// Rotate point and calculate new X values
	P2_1[0]=R2;
	P3_1[0]=R3;
	t[0]=R;

	// Rotate point and Calculate new Y values (should always be 0)
	P2_1[1]=0;
	P3_1[1]=0;
	t[1]=0;

	// Calculate phi for all moving points
	phiTarget = atan2(t[2],t[0]);
	phi2[2] = phi2[0];
	phi3[2] = phi3[0];


	// Code takes long when trying to fully strtetch arm
	// create condition to shorcut this

	if ( ((sqrt( (pow(t[0],2)) + (pow(t[1],2)) + (pow(t[2],2)) )) >= (d1+d2-11)) && ( (abs(Px[0])>abs(Px[1])) || (abs(Py[0])>abs(Py[1])) || (abs(Pz[0])>abs(Pz[1])) ) )
	{
		phi2[2]=(phiTarget*180)/PI;
		phi2[0]=phi2[2];

		phi3[2]=0;
		phi3[0]=phi3[2];

		P3_1[0]=Px[0];
		P3_1[1]=Py[0];
		P3_1[2]=Pz[0];

		Px[1]=Px[0];
		Py[1]=Py[0];
		Pz[1]=Pz[0];

		XTest= (cos(theta1[0]*PI/180)*((d1*cos(phi2[0]*PI/180))+(d2*cos((phi2[0]+phi3[0])*PI/180))));
		YTest= (sin(theta1[0]*PI/180)*((d1*cos(phi2[0]*PI/180))+(d2*cos((phi2[0]+phi3[0])*PI/180))));
		ZTest= ((d1*sin(phi2[0]*PI/180)+(d2*sin((phi2[0]+phi3[0])*PI/180))));

		ROM_TimerEnable(TIMER3_BASE, TIMER_A);

		return;
	}
	else
	{
		// Add kink between links
		if (phi2[2]!=-15)
		{phi2[2]=phi2[2]+15;
		}
		if (phi3[2]!=30)
		{
			phi3[2]=phi3[2]-30;
		}

		// Calculate new values for P2 and P3
		if (phi2[2]!=0)
		{
			P2_1[0]=d1*cos(theta1[2]*PI/180)*(d1*cos(phi2[2]*PI/180));
			P2_1[1]=d1*sin(theta1[2]*PI/180)*(d1*cos(phi2[2]*PI/180));
			P2_1[2]=d1*sin(phi2[2]*PI/180);


			P3_1[0]= (cos(theta1[2]*PI/180)*((d1*cos(phi2[2]*PI/180))+(d2*cos((phi2[2]+phi3[2])*PI/180))));
			P3_1[1]= (sin(theta1[2]*PI/180)*((d1*cos(phi2[2]*PI/180))+(d2*cos((phi2[2]+phi3[2])*PI/180))));
			P3_1[2]= ((d1*sin(phi2[2]*PI/180)+(d2*sin((phi2[2]+phi3[2])*PI/180))));
		}
		else
		{
			P2_1[0]=d1*cos(theta1[2]*PI/180);
			P2_1[1]=d1*sin(theta1[2]*PI/180);
			P2_1[2]=0;

			P3_1[0]= (d1+d2)*cos(theta1[2]*PI/180);
			P3_1[1]= (d1+d2)*sin(theta1[2]*PI/180);
			P3_1[2]= 0;
		}


		// Rotate target back to original position and calculate new X and Y values
		t[0]=R*cos(thetaTarget);
		t[1]=R*sin(thetaTarget);

		// The target is reachable; thus, set origin joint as b
		b[0] = P1_1[0];
		b[1] = P1_1[1];
		b[2] = P1_1[2];

		difA = sqrt( (pow((P3_1[0]-t[0]),2)) + (pow((P3_1[1]-t[1]),2)) + (pow((P3_1[2]-t[2]),2)) );

		while (difA > tol )

		{
			//STAGE 1 FORWARD REACHING

			P3_1[0]=t[0];
			P3_1[1]=t[1];
			P3_1[2]=t[2];

			Move2[0] = P3_1[0]-P2_1[0];
			Move2[1] = P3_1[1]-P2_1[1];
			Move2[2] = P3_1[2]-P2_1[2];

			Delta2 = d2/(sqrt((pow(Move2[0],2))+(pow(Move2[1],2))+(pow(Move2[2],2))));

			P2_1[0] = ((1 - Delta2)*Move2[0])+P2_1[0];
			P2_1[1] = ((1 - Delta2)*Move2[1])+P2_1[1];
			P2_1[2] = ((1 - Delta2)*Move2[2])+P2_1[2];


			Move1[0] = P2_1[0]-P1_1[0];
			Move1[1] = P2_1[1]-P1_1[1];
			Move1[2] = P2_1[2]-P1_1[2];

			Delta1 = d1/ (sqrt( (pow(Move1[0],2)) + (pow(Move1[1],2)) + (pow(Move1[2],2)) ));

			P1_1[0] = ((1 - Delta1)*Move1[0])+P1_1[0];
			P1_1[1] = ((1 - Delta1)*Move1[1])+P1_1[1];
			P1_1[2] = ((1 - Delta1)*Move1[2])+P1_1[2];

			//STAGE 2 BACKWARD REACHING

			// Set P1 to origin
			P1_1[0] = b[0];
			P1_1[1] = b[1];
			P1_1[2] = b[2];

			Move1[0] = P1_1[0]-P2_1[0];
			Move1[1] = P1_1[1]-P2_1[1];
			Move1[2] = P1_1[2]-P2_1[2];

			Delta1 = d1/ (sqrt( (pow(Move1[0],2)) + (pow(Move1[1],2)) + (pow(Move1[2],2)) ));

			P2_1[0] = ((1 - Delta1)*Move1[0])+P2_1[0];
			P2_1[1] = ((1 - Delta1)*Move1[1])+P2_1[1];
			P2_1[2] = ((1 - Delta1)*Move1[2])+P2_1[2];


			Move2[0] = P2_1[0]-P3_1[0];
			Move2[1] = P2_1[1]-P3_1[1];
			Move2[2] = P2_1[2]-P3_1[2];

			Delta2 = d2/(sqrt( (pow(Move2[0],2)) + (pow(Move2[1],2)) + (pow(Move2[2],2)) ));

			P3_1[0] = ((1 - Delta2)*Move2[0])+P3_1[0];
			P3_1[1] = ((1 - Delta2)*Move2[1])+P3_1[1];
			P3_1[2] = ((1 - Delta2)*Move2[2])+P3_1[2];

			difA = (sqrt( (pow((P3_1[0]-t[0]),2)) + (pow((P3_1[1]-t[1]),2)) + (pow((P3_1[2]-t[2]),2)) ));

		}

		// Rotate points back to (x,z) plane and calculate angles

		// Calculate R in the (x, y) plane
		R2=sqrt( (pow(P2_1[0],2)) + (pow(P2_1[1],2)) );
		R3=sqrt( (pow(P3_1[0],2)) + (pow(P3_1[1],2)) );

		P2_1_rotate[0]=R2;

		P3_1_rotate[0]=R3;

		// Rotate point and Calculate new Y values
		P2_1_rotate[1]=0;
		P3_1_rotate[1]=0;

		//Store Pn Z values into Pn_rotate
		P2_1_rotate[2]=P2_1[2];
		P3_1_rotate[2]=P3_1[2];

		// Calculate joint (motor) angles
		phi2[2] = atan2(P2_1_rotate[2],P2_1_rotate[0]);
		phi2[2] = (phi2[2]*180)/PI;

		// Add -P2 to P3  then calculate required motor angle for motor 3
		P3_1_rotate[0] = P3_1_rotate[0] - P2_1_rotate[0];
		P3_1_rotate[1] = P3_1_rotate[1] - P2_1_rotate[1];
		P3_1_rotate[2] = P3_1_rotate[2] - P2_1_rotate[2];

		// Motor angle 3 = phi3 + (-phi2)
		phi3[2] = atan2(P3_1_rotate[2],P3_1_rotate[0]) ;
		phi3[2] = ((phi3[2]*180)/PI) - phi2[2];

		if (phi3[2]<=0)
		{
			ELBOW=1;
		}
		else
		{
			ELBOW=0;
		}

		if  ( (P2_1[2]>0) && (t[2]>0) && (ELBOW==1) && ((sqrt( (pow(P3_1[0],2)) + (pow(P3_1[1],2)) + (pow(P3_1[2],2)) )) !=(d1+d2))  )
		{
			phi2[2]=(2*phiTarget*180/PI)-phi2[2];
			phi3[2]=-phi3[2];

			P2_1[0]=cos(theta1[2]*PI/180) * (d1*cos(phi2[2]*PI/180));
			P2_1[1]=sin(theta1[2]*PI/180) * (d1*cos(phi2[2]*PI/180));
			P2_1[2]=d1*sin(phi2[2]*PI/180);

			P3_1[0]=(cos(theta1[2]*PI/180)*((d1*cos(phi2[2]*PI/180))+(d2*cos((phi2[2]+phi3[2])*PI/180))));
			P3_1[1]= (sin(theta1[2]*PI/180)*((d1*cos(phi2[2]*PI/180))+(d2*cos((phi2[2]+phi3[2])*PI/180))));
			P3_1[2]=(d1*sin(phi2[2]*PI/180)+(d2*sin((phi2[2]+phi3[2])*PI/180)));

			ELBOW=0;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			Position(Px, Py, Pz, RotateE);
		}

		else if ( (P2_1[2]<0) && (t[2]<=0) && (ELBOW==0) && ((sqrt( (pow(P3_1[0],2)) + (pow(P3_1[1],2)) + (pow(P3_1[2],2)) )) !=(d1+d2)) )
		{
			phi2[2]=(2*phiTarget*180/PI)-phi2[2];
			phi3[2]=-phi3[2];

			P2_1[0]=cos(theta1[2]*PI/180) * (d1*cos(phi2[2]*PI/180));
			P2_1[1]=sin(theta1[2]*PI/180) * (d1*cos(phi2[2]*PI/180));
			P2_1[2]=d1*sin(phi2[2]*PI/180);

			P3_1[0]=(cos(theta1[2]*PI/180)*((d1*cos(phi2[2]*PI/180))+(d2*cos((phi2[2]+phi3[2])*PI/180))));
			P3_1[1]= (sin(theta1[2]*PI/180)*((d1*cos(phi2[2]*PI/180))+(d2*cos((phi2[2]+phi3[2])*PI/180))));
			P3_1[2]=(d1*sin(phi2[2]*PI/180)+(d2*sin((phi2[2]+phi3[2])*PI/180)));

			ELBOW=1;

			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////			Position(Px, Py, Pz, RotateE);
		}


		if (phi3[2]>180)
		{
			phi3[2]=phi3[2]-360;

		}
		else if (phi3[2]< -180)
		{
			phi3[2]=phi3[2]+360;
		}

		if (phi2[2]>90)
		{
			phi2[2]=180 - phi2[2];

		}


		// Multiply calculated angles by steps per angle, also add centre for all motors
		// Add phase shift for required motors and multiply motor 1 clock ticks by pulley ratio (48/32)
		// Add 0.25 degree to all motors then back off 0.25 degree after movement
		// to ease stress on motors and reduce motor noise

	}

	if ( (phi3[2]<=130) && (phi3[2]>=-130) && (phi2[2]>=-55) && (phi2[2]<=90) )
	{
		theta1[0]=theta1[2];
		phi2[0]=phi2[2];
		phi3[0]=phi3[2];

		Px[1]=Px[0];
		Py[1]=Py[0];
		Pz[1]=Pz[0];

		XTest= (cos(theta1[0]*PI/180)*((d1*cos(phi2[0]*PI/180))+(d2*cos((phi2[0]+phi3[0])*PI/180))));
		YTest= (sin(theta1[0]*PI/180)*((d1*cos(phi2[0]*PI/180))+(d2*cos((phi2[0]+phi3[0])*PI/180))));
		ZTest= ((d1*sin(phi2[0]*PI/180)+(d2*sin((phi2[0]+phi3[0])*PI/180))));

		ROM_TimerEnable(TIMER3_BASE, TIMER_A);

		return;
	}
	else if ( ( (phi2[2]<-55) || (phi3[2]<-130) || (phi3[2]>130) ) && (Pz[0]>Pz[1]) )
	{
		Pz[0]=Pz[0]+10;

		XTest= (cos(theta1[0]*PI/180)*((d1*cos(phi2[0]*PI/180))+(d2*cos((phi2[0]+phi3[0])*PI/180))));
		YTest= (sin(theta1[0]*PI/180)*((d1*cos(phi2[0]*PI/180))+(d2*cos((phi2[0]+phi3[0])*PI/180))));
		ZTest= ((d1*sin(phi2[0]*PI/180)+(d2*sin((phi2[0]+phi3[0])*PI/180))));

		Position(Px, Py, Pz, RotateE);
	}
	else if ( ( (phi2[2]<-55) || (phi3[2]<-130) || (phi3[2]>130) ) && (Pz[0]<Pz[1]) )
	{
		Pz[0]=Pz[0]-10;

		XTest= (cos(theta1[0]*PI/180)*((d1*cos(phi2[0]*PI/180))+(d2*cos((phi2[0]+phi3[0])*PI/180))));
		YTest= (sin(theta1[0]*PI/180)*((d1*cos(phi2[0]*PI/180))+(d2*cos((phi2[0]+phi3[0])*PI/180))));
		ZTest= ((d1*sin(phi2[0]*PI/180)+(d2*sin((phi2[0]+phi3[0])*PI/180))));

		Position(Px, Py, Pz, RotateE);
	}
	else if ( ( (phi2[2]<-55) || (phi3[2]<-130) || (phi3[2]>130) ) && (Px[0]!=Px[1]) )
	{
		Px[0]=-Px[1];

		XTest= (cos(theta1[0]*PI/180)*((d1*cos(phi2[0]*PI/180))+(d2*cos((phi2[0]+phi3[0])*PI/180))));
		YTest= (sin(theta1[0]*PI/180)*((d1*cos(phi2[0]*PI/180))+(d2*cos((phi2[0]+phi3[0])*PI/180))));
		ZTest= ((d1*sin(phi2[0]*PI/180)+(d2*sin((phi2[0]+phi3[0])*PI/180))));

		Position(Px, Py, Pz, RotateE);
	}
	else
	{
		Px[0]=Px[1];
		Py[0]=Py[1];
		Pz[0]=Pz[1];

	}

}

void DEMO( double *DEMO_PosX, double *DEMO_PosY, double *DEMO_PosZ ,double *DEMO_Rotate)
{
	while(1)
	{
		if ( ROM_TimerValueGet(TIMER5_BASE, TIMER_A)!=((ROM_SysCtlClockGet() / 1000)*60000) )
		{
			return;
		}
		ROM_SysCtlDelay(5000000);
		DEMO_PosX[0]=400;
		DEMO_PosY[0]=0;
		DEMO_PosZ[0]=0;
		DEMO_Rotate[1]=0;
		DEMO_Rotate[0]=DEMO_Rotate[1];
		Position(DEMO_PosX, DEMO_PosY, DEMO_PosZ, DEMO_Rotate);
		while ( (abs(theta1[1]-theta1[0])>1) || (abs(phi2[1]-phi2[0])>1) || (abs(phi3[1]-phi3[0])>1) || (abs(RotateEND[1]-RotateEND[0])>1) )
		{

		}
		if ( ROM_TimerValueGet(TIMER5_BASE, TIMER_A)!=((ROM_SysCtlClockGet() / 1000)*60000) )
		{
			return;
		}
		ROM_SysCtlDelay(5000000);
		DEMO_PosX[0]=180;
		DEMO_PosY[0]=0;
		DEMO_PosZ[0]=0;
		DEMO_Rotate[1]=0;
		DEMO_Rotate[0]=DEMO_Rotate[1];
		Position(DEMO_PosX, DEMO_PosY, DEMO_PosZ, DEMO_Rotate);
		while ( (abs(theta1[1]-theta1[0])>1) || (abs(phi2[1]-phi2[0])>1) || (abs(phi3[1]-phi3[0])>1) || (abs(RotateEND[1]-RotateEND[0])>1) )
		{

		}
		if ( ROM_TimerValueGet(TIMER5_BASE, TIMER_A)!=((ROM_SysCtlClockGet() / 1000)*60000) )
		{
			return;
		}
		ROM_SysCtlDelay(5000000);
		DEMO_PosX[0]=0;
		DEMO_PosY[0]=180;
		DEMO_PosZ[0]=0;
		DEMO_Rotate[1]=180;
		DEMO_Rotate[0]=DEMO_Rotate[1];
		Position(DEMO_PosX, DEMO_PosY, DEMO_PosZ, DEMO_Rotate);
		while ( (abs(theta1[1]-theta1[0])>1) || (abs(phi2[1]-phi2[0])>1) || (abs(phi3[1]-phi3[0])>1) || (abs(RotateEND[1]-RotateEND[0])>1) )
		{

		}
		if ( ROM_TimerValueGet(TIMER5_BASE, TIMER_A)!=((ROM_SysCtlClockGet() / 1000)*60000) )
		{
			return;
		}
		ROM_SysCtlDelay(5000000);
		DEMO_PosX[0]=-180;
		DEMO_PosY[0]=0;
		DEMO_PosZ[0]=0;
		DEMO_Rotate[1]=0;
		DEMO_Rotate[0]=DEMO_Rotate[1];
		Position(DEMO_PosX, DEMO_PosY, DEMO_PosZ, DEMO_Rotate);
		while ( (abs(theta1[1]-theta1[0])>1) || (abs(phi2[1]-phi2[0])>1) || (abs(phi3[1]-phi3[0])>1) || (abs(RotateEND[1]-RotateEND[0])>1) )
		{

		}
		if ( ROM_TimerValueGet(TIMER5_BASE, TIMER_A)!=((ROM_SysCtlClockGet() / 1000)*60000) )
		{
			return;
		}
		ROM_SysCtlDelay(5000000);
		DEMO_PosX[0]=-420;
		DEMO_PosY[0]=0;
		DEMO_PosZ[0]=0;
		DEMO_Rotate[1]=0;
		DEMO_Rotate[0]=DEMO_Rotate[1];
		// Gripper Close
		ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0);
		// Ensure gripper PWM is enabled
		ROM_PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
		// Start Timer4_A to disable
		ROM_TimerEnable(TIMER4_BASE, TIMER_A);
		Position(DEMO_PosX, DEMO_PosY, DEMO_PosZ, DEMO_Rotate);
		while ( (abs(theta1[1]-theta1[0])>1) || (abs(phi2[1]-phi2[0])>1) || (abs(phi3[1]-phi3[0])>1) || (abs(RotateEND[1]-RotateEND[0])>1) )
		{

		}
		if ( ROM_TimerValueGet(TIMER5_BASE, TIMER_A)!=((ROM_SysCtlClockGet() / 1000)*60000) )
		{
			return;
		}
		ROM_SysCtlDelay(5000000);
		DEMO_PosX[0]=400;
		DEMO_PosY[0]=0;
		DEMO_PosZ[0]=0;
		DEMO_Rotate[1]=0;
		DEMO_Rotate[0]=DEMO_Rotate[1];
		Position(DEMO_PosX, DEMO_PosY, DEMO_PosZ, DEMO_Rotate);
		while ( (abs(theta1[1]-theta1[0])>1) || (abs(phi2[1]-phi2[0])>1) || (abs(phi3[1]-phi3[0])>1) || (abs(RotateEND[1]-RotateEND[0])>1) )
		{

		}
		if ( ROM_TimerValueGet(TIMER5_BASE, TIMER_A)!=((ROM_SysCtlClockGet() / 1000)*60000) )
		{
			return;
		}
		ROM_SysCtlDelay(5000000);
		DEMO_PosX[0]=0;
		DEMO_PosY[0]=0;
		DEMO_PosZ[0]=420;
		DEMO_Rotate[1]=0;
		DEMO_Rotate[0]=DEMO_Rotate[1];
		Position(DEMO_PosX, DEMO_PosY, DEMO_PosZ, DEMO_Rotate);
		while ( (abs(theta1[1]-theta1[0])>1) || (abs(phi2[1]-phi2[0])>1) || (abs(phi3[1]-phi3[0])>1) || (abs(RotateEND[1]-RotateEND[0])>1) )
		{

		}
		if ( ROM_TimerValueGet(TIMER5_BASE, TIMER_A)!=((ROM_SysCtlClockGet() / 1000)*60000) )
		{
			return;
		}
		ROM_SysCtlDelay(5000000);
		DEMO_PosX[0]=0;
		DEMO_PosY[0]=0;
		DEMO_PosZ[0]=420;
		DEMO_Rotate[1]=180;
		DEMO_Rotate[0]=DEMO_Rotate[1];
		// Gripper Close
		ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0);
		// Ensure gripper PWM is enabled
		ROM_PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
		// Start Timer4_A to disable
		ROM_TimerEnable(TIMER4_BASE, TIMER_A);
		Position(DEMO_PosX, DEMO_PosY, DEMO_PosZ, DEMO_Rotate);
		while ( (abs(theta1[1]-theta1[0])>1) || (abs(phi2[1]-phi2[0])>1) || (abs(phi3[1]-phi3[0])>1) || (abs(RotateEND[1]-RotateEND[0])>1) )
		{

		}
		if ( ROM_TimerValueGet(TIMER5_BASE, TIMER_A)!=((ROM_SysCtlClockGet() / 1000)*60000) )
		{
			return;
		}
		ROM_SysCtlDelay(5000000);
		DEMO_PosX[0]=0;
		DEMO_PosY[0]=0;
		DEMO_PosZ[0]=420;
		DEMO_Rotate[1]=0;
		DEMO_Rotate[0]=DEMO_Rotate[1];
		// Gripper Open
		ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);
		// Ensure gripper PWM is enabled
		ROM_PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
		// Start Timer4_A to disable
		ROM_TimerEnable(TIMER4_BASE, TIMER_A);
		Position(DEMO_PosX, DEMO_PosY, DEMO_PosZ, DEMO_Rotate);
		while ( (abs(theta1[1]-theta1[0])>1) || (abs(phi2[1]-phi2[0])>1) || (abs(phi3[1]-phi3[0])>1) || (abs(RotateEND[1]-RotateEND[0])>1) )
		{

		}
		if ( ROM_TimerValueGet(TIMER5_BASE, TIMER_A)!=((ROM_SysCtlClockGet() / 1000)*60000) )
		{
			return;
		}
		ROM_SysCtlDelay(5000000);
		DEMO_PosX[0]=0;
		DEMO_PosY[0]=0;
		DEMO_PosZ[0]=180;
		DEMO_Rotate[1]=180;
		DEMO_Rotate[0]=DEMO_Rotate[1];
		// Gripper Open
		ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);
		// Ensure gripper PWM is enabled
		ROM_PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
		// Start Timer4_A to disable
		ROM_TimerEnable(TIMER4_BASE, TIMER_A);
		Position(DEMO_PosX, DEMO_PosY, DEMO_PosZ, DEMO_Rotate);
		while ( (abs(theta1[1]-theta1[0])>1) || (abs(phi2[1]-phi2[0])>1) || (abs(phi3[1]-phi3[0])>1) || (abs(RotateEND[1]-RotateEND[0])>1) )
		{

		}
		if ( ROM_TimerValueGet(TIMER5_BASE, TIMER_A)!=((ROM_SysCtlClockGet() / 1000)*60000) )
		{
			return;
		}
		ROM_SysCtlDelay(5000000);
		DEMO_PosX[0]=0;
		DEMO_PosY[0]=10;
		DEMO_PosZ[0]=180;
		DEMO_Rotate[1]=0;
		DEMO_Rotate[0]=DEMO_Rotate[1];
		Position(DEMO_PosX, DEMO_PosY, DEMO_PosZ, DEMO_Rotate);
		while ( (abs(theta1[1]-theta1[0])>1) || (abs(phi2[1]-phi2[0])>1) || (abs(phi3[1]-phi3[0])>1) || (abs(RotateEND[1]-RotateEND[0])>1) )
		{

		}
		if ( ROM_TimerValueGet(TIMER5_BASE, TIMER_A)!=((ROM_SysCtlClockGet() / 1000)*60000) )
		{
			return;
		}
		ROM_SysCtlDelay(5000000);
		DEMO_PosX[0]=-10;
		DEMO_PosY[0]=0;
		DEMO_PosZ[0]=180;
		DEMO_Rotate[1]=180;
		DEMO_Rotate[0]=DEMO_Rotate[1];
		// Gripper Open
		ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);
		// Ensure gripper PWM is enabled
		ROM_PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
		// Start Timer4_A to disable
		ROM_TimerEnable(TIMER4_BASE, TIMER_A);
		Position(DEMO_PosX, DEMO_PosY, DEMO_PosZ, DEMO_Rotate);
		while ( (abs(theta1[1]-theta1[0])>1) || (abs(phi2[1]-phi2[0])>1) || (abs(phi3[1]-phi3[0])>1) || (abs(RotateEND[1]-RotateEND[0])>1) )
		{

		}
		if ( ROM_TimerValueGet(TIMER5_BASE, TIMER_A)!=((ROM_SysCtlClockGet() / 1000)*60000) )
		{
			return;
		}
		ROM_SysCtlDelay(5000000);
		DEMO_PosX[0]=-420;
		DEMO_PosY[0]=0;
		DEMO_PosZ[0]=0;
		DEMO_Rotate[1]=0;
		DEMO_Rotate[0]=DEMO_Rotate[1];
		Position(DEMO_PosX, DEMO_PosY, DEMO_PosZ, DEMO_Rotate);
		while ( (abs(theta1[1]-theta1[0])>1) || (abs(phi2[1]-phi2[0])>1) || (abs(phi3[1]-phi3[0])>1) || (abs(RotateEND[1]-RotateEND[0])>1) )
		{

		}

	}

}


void Timer3_A_INT(void)
{
	// Clear the timer interrupt
	ROM_TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);


	if (  ( (theta1[1]<theta1[0]) && ((theta1[1]+Vel1)>theta1[0]) )  || ( (theta1[1]>theta1[0]) && ((theta1[1]+Vel1)<theta1[0]) ) || (theta1[1]==theta1[0]) )
	{
		theta1[1]=theta1[0];
		Vel1=0;
		MotorAngles[0]  = (-theta1[1] * 18.59941447) + 7620 + (18.59941447*(90)) - 1 ;
		ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, MotorAngles[0]);
	}
	else if (theta1[0]>theta1[1])
	{

		Vel1= 0.04* ((theta1[0]-theta1[1])/40) + 0.08;

//		if (Vel1>0.2)
//		{
//			Vel1=0.2;
//		}
		if  ((theta1[1]+Vel1)<theta1[0])
		{
			theta1[1] = theta1[1] + Vel1;
			MotorAngles[0]  = (-theta1[1] * 18.59941447) + 7620 + (18.59941447*(90)) - 1 ;
			ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, MotorAngles[0]);
			ROM_TimerEnable(TIMER3_BASE, TIMER_A);
		}

	}

	else if (theta1[0]<theta1[1])
	{
		Vel1= -0.04* ((theta1[1]-theta1[0])/40) - 0.08;
//		if (Vel1<-0.2)
//		{
//			Vel1=-0.2;
//		}
		if ((theta1[1]-Vel1)>theta1[0])
		{
			theta1[1] = theta1[1] + Vel1;
			MotorAngles[0]  = (-theta1[1] * 18.59941447) + 7620 + (18.59941447*(90)) - 1;
			ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, MotorAngles[0]);
			ROM_TimerEnable(TIMER3_BASE, TIMER_A);
		}
	}


	if (  ( (phi2[1]<phi2[0]) && ((phi2[1]+Vel2)>phi2[0]) )  || ( (phi2[1]>phi2[0]) && ((phi2[1]+Vel2)<phi2[0]) ) || (phi2[1]==phi2[0]) )
	{
		phi2[1]=phi2[0];
		Vel2=0;
		MotorAngles[1] = ( (phi2[1]*-17.6)  *48/32)  + 7492.5   ;
		ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, MotorAngles[1]);
	}
	else if (phi2[0]>phi2[1])
	{
		Vel2= 0.06* ((phi2[0]-phi2[1])/20)+0.02;

		if ((phi2[1]+Vel2)<phi2[0])
		{
			phi2[1] = phi2[1] + Vel2;
			MotorAngles[1] = ( (phi2[1]*-17.6)  *48/32)  + 7492.5   ;
			ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, MotorAngles[1]);
			ROM_TimerEnable(TIMER3_BASE, TIMER_A);
		}

	}
	else if (phi2[0]<phi2[1])
	{
		Vel2= -0.06* ((phi2[1]-phi2[0])/20)-0.02;

		if ((phi2[1]+Vel2)>phi2[0])
		{
			phi2[1] = phi2[1] + Vel2;
			MotorAngles[1]  = ( (phi2[1]*-17.6)  *48/32)  + 7492.5   ;
			ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, MotorAngles[1]);
			ROM_TimerEnable(TIMER3_BASE, TIMER_A);
		}
	}

	if (  ( (phi3[1]<phi3[0]) && ((phi3[1]+Vel3)>phi3[0]) )  || ( (phi3[1]>phi3[0]) && ((phi3[1]+Vel3)<phi3[0]) ) || (phi3[1]==phi3[0]) )
	{
		phi3[1]=phi3[0];
		Vel3=0;
		MotorAngles[2]  = (phi3[1]*17.6) + 7440 ;
		ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, MotorAngles[2]);
	}
	else if (phi3[0]>phi3[1])
	{
		Vel3= 0.12* ((phi3[0]-phi3[1])/30) + 0.03;

		if  ((phi3[1]+Vel3)<phi3[0])
		{
			phi3[1] = phi3[1] + Vel3;
			MotorAngles[2]  = (phi3[1]*17.6) + 7440 ;
			ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, MotorAngles[2]);
			ROM_TimerEnable(TIMER3_BASE, TIMER_A);
		}

	}
	else if (phi3[0]<phi3[1])
	{
		Vel3= -0.12* ((phi3[1]-phi3[0])/30) - 0.03;

		if ((phi3[1]+Vel3)>phi3[0])
		{
			phi3[1] = phi3[1] + Vel3;
			MotorAngles[2]  = (phi3[1]*17.6) + 7440 ;
			ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, MotorAngles[2]);
			ROM_TimerEnable(TIMER3_BASE, TIMER_A);
		}

	}

	if (  ( (RotateEND[1]<RotateEND[0]) && ((RotateEND[1]+Vel4)>RotateEND[0]) )  || ( (RotateEND[1]>RotateEND[0]) && ((RotateEND[1]+Vel4)<RotateEND[0]) ) || (RotateEND[1]==RotateEND[0]) )
	{
		RotateEND[1]=RotateEND[0];
		Vel4=0;
		RotateTicks  = (RotateEND[1]*25.363) + 5217.33;
		ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,  RotateTicks);
	}
	else if (RotateEND[0]>RotateEND[1])
	{
		Vel4= 0.35* ((RotateEND[0]-RotateEND[1])/30) + 0.03;

		if  ((RotateEND[1]+Vel4)<RotateEND[0])
		{
			RotateEND[1] = RotateEND[1] + Vel4;
			RotateTicks  = (RotateEND[1]*25.363) + 5217.33;
			ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,  RotateTicks);
			ROM_TimerEnable(TIMER3_BASE, TIMER_A);
		}

	}

	else if (RotateEND[0]<RotateEND[1])
	{
		Vel4= -0.35* ((RotateEND[1]-RotateEND[0])/30) - 0.03;

		if ((RotateEND[1]+Vel4)>RotateEND[0])
		{
			RotateEND[1] = RotateEND[1] + Vel4;
			RotateTicks = (RotateEND[1]*25.363) + 5217.33 ;
			ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,  RotateTicks);
			ROM_TimerEnable(TIMER3_BASE, TIMER_A);
		}

	}

}

void Timer4_A_INT(void)
{
	// Clear the timer interrupt
	ROM_TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
	// Disable Gripper
	ROM_PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, false);
}
