// controlDLL.cpp : Defines the entry point for the DLL application.
//
#ifdef WIN32
#include "stdafx.h"
BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved )
{
    return TRUE;
}
#include <stdio.h>

#else //#ifdef WIN32
#include "servo.h"

#endif //#ifdef WIN32

#include "param.h"
#include "control.h"
#include "PrVector.h"
#include "PrMatrix.h"
#include "Utils.h" // misc. utility functions, such as toRad, toDeg, etc.
#include <math.h>
#include <algorithm>
using std::min;
using std::max;


// *******************************************************************
// Initialization functions
// *******************************************************************

void InitControl(GlobalVariables& gv) 
{
}

void PreprocessControl(GlobalVariables& gv)
{
   // This code runs on every servo loop, just before the control law
}


void PostprocessControl(GlobalVariables& gv) 
{
}

void initFloatControl(GlobalVariables& gv) 
{
    gv.tau =gv.G;
}

void initOpenControl(GlobalVariables& gv) 
{
}

void initNjholdControl(GlobalVariables& gv) 
{
   initFloatControl(gv);
   gv.qd = gv.q; //The desired position equals the current position
   gv.dqd.zero(); // Set the desired velocities to zero
}

void initJholdControl(GlobalVariables& gv) 
{
   initFloatControl(gv);
   gv.qd = gv.q; //The desired position equals the current position
   gv.dqd.zero(); // Set the desired velocities to zero
}

void initNjmoveControl(GlobalVariables& gv) 
{
	initFloatControl(gv);
	gv.dqd.zero(); // Set the desired velocities to zero
}

void initJmoveControl(GlobalVariables& gv) 
{
	initFloatControl(gv);
	gv.dqd.zero(); // Set the desired velocities to zero
}

void initNjgotoControl(GlobalVariables& gv) 
{
   initFloatControl(gv);
} 

void initJgotoControl(GlobalVariables& gv) 
{
   initFloatControl(gv);
}

void initNjtrackControl(GlobalVariables& gv) 
{
   initFloatControl(gv);
}

void initJtrackControl(GlobalVariables& gv) 
{
   initFloatControl(gv);
}

void initNxtrackControl(GlobalVariables& gv) 
{
   initFloatControl(gv);
}

void initXtrackControl(GlobalVariables& gv) 
{
   initFloatControl(gv);
} 

void initNholdControl(GlobalVariables& gv) 
{
   initFloatControl(gv);
}

void initHoldControl(GlobalVariables& gv) 
{
   initFloatControl(gv);
}

void initNgotoControl(GlobalVariables& gv) 
{
	initFloatControl(gv);
} 

void initGotoControl(GlobalVariables& gv) 
{
	initFloatControl(gv);
} 

void initNtrackControl(GlobalVariables& gv) 
{
   initFloatControl(gv);
}

void initTrackControl(GlobalVariables& gv) 
{
   initFloatControl(gv);
} 

void initPfmoveControl(GlobalVariables& gv) 
{
	initFloatControl(gv);
} 

void initLineControl(GlobalVariables& gv) 
{
	initFloatControl(gv);
}

void initProj1Control(GlobalVariables& gv) 
{
	initFloatControl(gv);
}

void initProj2Control(GlobalVariables& gv) 
{
	initFloatControl(gv);
}

void initProj3Control(GlobalVariables& gv) 
{
	initFloatControl(gv);
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
   floatControl(gv);
}

void njholdControl(GlobalVariables& gv) 
{
   floatControl(gv);
   gv.tau = -gv.kp*(gv.q-gv.qd)-gv.kv*(gv.dq-gv.dqd); 
}

void jholdControl(GlobalVariables& gv) 
{
   floatControl(gv);
   PrVector tau_prime = -gv.kp*(gv.q-gv.qd)-gv.kv*(gv.dq-gv.dqd);
   gv.tau = gv.A * tau_prime + gv.B + gv.G ;
}

void njmoveControl(GlobalVariables& gv)
{
   floatControl(gv);
   gv.tau = -gv.kp*(gv.q-gv.qd)-gv.kv*(gv.dq-gv.dqd)+gv.G; //same as njhold but with gravity

}

void jmoveControl(GlobalVariables& gv)
{
   floatControl(gv);
   jholdControl(gv);
}

void njgotoControl(GlobalVariables& gv) 
{
   floatControl(gv);
}

void jgotoControl(GlobalVariables& gv) 
{
   floatControl(gv);
}

void njtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);
}

void jtrackControl(GlobalVariables& gv)
{
   floatControl(gv);
}

void nxtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);
}

void xtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);
}

void nholdControl(GlobalVariables& gv) 
{
   floatControl(gv);
}

void holdControl(GlobalVariables& gv) 
{
   floatControl(gv);
}

void ngotoControl(GlobalVariables& gv) 
{
   floatControl(gv);
}

void gotoControl(GlobalVariables& gv) 
{
   floatControl(gv);
}

void ntrackControl(GlobalVariables& gv) 
{
   floatControl(gv);
}

void trackControl(GlobalVariables& gv) 
{
   floatControl(gv);
}

void pfmoveControl(GlobalVariables& gv) 
{
   floatControl(gv);
}

void lineControl(GlobalVariables& gv)
{
	floatControl(gv);
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

/********************************************************

END OF DEFAULT STUDENT FILE 

ADD HERE ALL STUDENT DEFINED AND AUX FUNCTIONS 

*******************************************************/


#ifdef WIN32
// *******************************************************************
// XPrintf(): Replacement for printf() which calls ui->VDisplay()
// whenever the ui object is available.  See utility/XPrintf.h.
// *******************************************************************

#ifdef USING_XPRINTF
int XPrintf( const char* fmt, ... )
{
  int returnValue;
  va_list argptr;
  va_start( argptr, fmt );

  returnValue = vprintf( fmt, argptr );

  va_end( argptr );
  return returnValue;
}
#endif
#endif //#ifdef WIN32