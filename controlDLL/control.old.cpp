// controlDLL.cpp : Defines the entry point for the DLL application.
//
#ifdef WIN32
#include "stdafx.h"
BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved
					 )
{
    return TRUE;
}
#include <stdio.h>

#else //#ifdef WIN32
#include "servo.h"

#endif //#ifdef WIN32

#include "param.h"
#include "control.h"
//#include "UiAgent.h"
#include "PrVector.h"
#include "PrMatrix.h"
#include "Utils.h" // misc. utility functions, such as toRad, toDeg, etc.
#include <math.h>
#include <algorithm>
using std::min;
using std::max;


void FindCubicSpline(GlobalVariables& gv);
void FindOperationalCubicSpline(GlobalVariables& gv);
void EvaluateCubicSpline( PrVector& pos, PrVector& vel, PrVector& acc, GlobalVariables& gv );
bool inv_kin( const PrVector3& pos, const PrMatrix3& rot, int elbow,
              PrVector& qOut, GlobalVariables& gv );
void getSaturatedFprime( PrVector& fPrime, GlobalVariables& gv );
void OpDynamics( const PrVector& fPrime, GlobalVariables& gv );

static PrVector splineParam0;  // params used for track splines
static PrVector splineParam2;
static PrVector splineParam3;
static Float splineStartTime, splineDuration;








// *******************************************************************
// Initialization functions
// *******************************************************************

void InitControl(GlobalVariables& gv) 
{
   // This code runs before the first servo loop
}

void PreprocessControl(GlobalVariables& gv)
{
   // This code runs on every servo loop, just before the control law
}

void PostprocessControl(GlobalVariables& gv) 
{
   // This code runs on every servo loop, just after the control law
}

void initFloatControl(GlobalVariables& gv) 
{
    // Control Initialization Code Here
}

void initOpenControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjholdControl(GlobalVariables& gv) 
{
	//set desired position to original 
	gv.qd = gv.q;
	//set desired velocity to zero
	gv.dqd.zero();
}

void initJholdControl(GlobalVariables& gv) 
{
	//set desired position to original 
	gv.qd = gv.q;
	//set desired velocity to zero
	gv.dqd.zero();
}

void initNjmoveControl(GlobalVariables& gv) 
{
	//set desired position
	//gv.qd = gv.qd;
	//set desired velocity to zero
	gv.dqd.zero();
}

void initJmoveControl(GlobalVariables& gv) 
{
	//set desired position
	//gv.qd = gv.qd;
	//set desired velocity to zero
	gv.dqd.zero();
}


void initNjgotoControl(GlobalVariables& gv) 
{
	// set desired position
	bool flag = false;
	for(int i = 0; i < gv.qd.size(); i++)
	{
		if( (gv.qd[i] > gv.qmax[i]) || (gv.qd[i] < gv.qmin[i]))
			flag = true;
	}

	if(flag)
		gv.qd = gv.q;

//	else gv.qd = gv.qd

	gv.dqd.zero();
} 

void initJgotoControl(GlobalVariables& gv) 
{
	// set desired position
	bool flag = false;
	for(int i = 0; i < gv.qd.size(); i++)
	{
		if( (gv.qd[i] > gv.qmax[i]) || (gv.qd[i] < gv.qmin[i]))
			flag = true;
	}

	if(flag)
		gv.qd = gv.q;
	//else gv.qd = gv.qd

	gv.dqd.zero();
}

void initNjtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initJtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNxtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initXtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initNholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initHoldControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initGotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initNtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initTrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initPfmoveControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initLineControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initProj1Control(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initProj2Control(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initProj3Control(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}


// *******************************************************************
// Control laws
// *******************************************************************

void noControl(GlobalVariables& gv)
{
}

void floatControl(GlobalVariables& gv)
{   
	gv.tau = gv.G;
}

void openControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void njholdControl(GlobalVariables& gv) 
{
   gv.tau = -gv.kp*(gv.q-gv.qd) - gv.kv*(gv.dq-gv.dqd);
}

void jholdControl(GlobalVariables& gv) 
{
  gv.tau = gv.A*(-gv.kp*(gv.q-gv.qd)-gv.kv*(gv.dq-gv.dqd)) + gv.B + gv.G;
}

void njmoveControl(GlobalVariables& gv)
{
   gv.tau = -gv.kp*(gv.q-gv.qd) - gv.kv*(gv.dq-gv.dqd)+gv.G;
}

void jmoveControl(GlobalVariables& gv)
{
   gv.tau = gv.A*(-gv.kp*(gv.q-gv.qd)-gv.kv*(gv.dq-gv.dqd)) + gv.B + gv.G;
}


void njgotoControl(GlobalVariables& gv) 
{	
  PrVector dqd_temp = gv.dqmax;

  for(int i = 0; i<dqd_temp.size(); i++)
  {
	  if(gv.kv[i] < 0.0001)
		gv.kv[i] = 0.0001;
	  dqd_temp[i] = -((gv.kp[i]/gv.kv[i])*(gv.q[i]-gv.qd[i])+gv.dqd[i]);
  }
  
  for(int i=0; i<dqd_temp.size(); i++)
  {
	  if(dqd_temp[i] > gv.dqmax[i]) 
	  {
		  dqd_temp[i] = gv.dqmax[i]; 
	  }
	  if(dqd_temp[i] < -gv.dqmax[i])
	  {
		  dqd_temp[i] = -gv.dqmax[i];
	  }
  }
	
  gv.tau = -gv.kv*(gv.dq-dqd_temp) + gv.G;
}

void jgotoControl(GlobalVariables& gv) 
{
  PrVector dqd_temp = gv.dqmax;

  for(int i = 0; i<dqd_temp.size(); i++)
  {
	  if(gv.kv[i] < 0.0001)
		gv.kv[i] = 0.0001;
	  dqd_temp[i] = -((gv.kp[i]/gv.kv[i])*(gv.q[i]-gv.qd[i])+gv.dqd[i]);
  }
  
  for(int i=0; i<dqd_temp.size(); i++)
  {
	if(dqd_temp[i] > gv.dqmax[i]) 
	{
		dqd_temp[i] = gv.dqmax[i]; 
	}
	if(dqd_temp[i] < -gv.dqmax[i])
	{
		dqd_temp[i] = -gv.dqmax[i];
	}
  }
	
  gv.tau = -gv.kv*(gv.dq-dqd_temp) + gv.B + gv.G;
}

void njtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void jtrackControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void nxtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void xtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void nholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void holdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void ngotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void gotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void ntrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void trackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void pfmoveControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void lineControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement openControl
}

void proj1Control(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement proj1Control
}

void proj2Control(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement proj2Control
}

void proj3Control(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement proj3Control
}

// *******************************************************************
// Debug function
// *******************************************************************

void PrintDebug(GlobalVariables& gv)
{
   // Replace this code with any debug information you'd like to get
   // when you type "pdebug" at the prompt.
   printf( "This sample code prints the torque and mass\n" );
   gv.tau.display( "tau" );
   gv.A.display( "A" );
}

#ifdef WIN32
// *******************************************************************
// XPrintf(): Replacement for printf() which calls ui->VDisplay()
// whenever the ui object is available.  See utility/XPrintf.h.
// *******************************************************************

int XPrintf( const char* fmt, ... )
{
  int returnValue;
  va_list argptr;
  va_start( argptr, fmt );

  returnValue = vprintf( fmt, argptr );

  va_end( argptr );
  return returnValue;
}
#endif //#ifdef WIN32

/********************************************************

END OF DEFAULT STUDENT FILE 

ADD HERE ALL STUDENT DEFINED AND AUX FUNCTIONS 

*******************************************************/
