#ifndef CONTROL_DLL_H
#define CONTROL_DLL_H
  
#include "../GlobalVariables.h"

#ifdef WIN32

void InitControl(GlobalVariables& gv);
void jmoveControl(GlobalVariables& gv);
void ngotoControl(GlobalVariables& gv);
void gotoControl(GlobalVariables& gv);



#else //#ifdef WIN32

// Initialize your variables here.  InitControl() is called just once,
// before the first servo loop.  PreprocessControl() is called on
// every servo loop just before the control law function, and
// PostprocessControl() is called just after the control law function.

extern void InitControl(GlobalVariables& gv) ;
extern void PreprocessControl(GlobalVariables& gv) ;
extern void PostprocessControl(GlobalVariables& gv) ;

// Initialization functions for each control law.
// Called once when a new control mode is set.

extern void initFloatControl(GlobalVariables& gv) ;
extern void initNjholdControl(GlobalVariables& gv) ;
extern void initJholdControl(GlobalVariables& gv) ;
extern void initOpenControl(GlobalVariables& gv) ;
extern void initNjmoveControl(GlobalVariables& gv) ;
extern void initJmoveControl(GlobalVariables& gv) ;
extern void initNjgotoControl(GlobalVariables& gv) ;
extern void initJgotoControl(GlobalVariables& gv) ;
extern void initNjtrackControl(GlobalVariables& gv) ;
extern void initJtrackControl(GlobalVariables& gv) ;
extern void initNxtrackControl(GlobalVariables& gv) ;
extern void initXtrackControl(GlobalVariables& gv) ;
extern void initNholdControl(GlobalVariables& gv) ;
extern void initHoldControl(GlobalVariables& gv) ;
extern void initNgotoControl(GlobalVariables& gv) ;
extern void initGotoControl(GlobalVariables& gv) ;
extern void initNtrackControl(GlobalVariables& gv) ;
extern void initTrackControl(GlobalVariables& gv) ;
extern void initPfmoveControl(GlobalVariables& gv) ;
extern void initLineControl(GlobalVariables& gv) ;
extern void initProj1Control(GlobalVariables& gv) ;
extern void initProj2Control(GlobalVariables& gv) ;
extern void initProj3Control(GlobalVariables& gv) ;

// Functions containing the actual control laws.

extern void noControl(GlobalVariables& gv) ;
extern void floatControl(GlobalVariables& gv) ;
extern void njholdControl(GlobalVariables& gv) ;
extern void jholdControl(GlobalVariables& gv) ;
extern void openControl(GlobalVariables& gv) ;
extern void njmoveControl(GlobalVariables& gv) ;
extern void jmoveControl(GlobalVariables& gv) ;
extern void njgotoControl(GlobalVariables& gv) ;
extern void jgotoControl(GlobalVariables& gv) ;
extern void njtrackControl(GlobalVariables& gv) ;
extern void jtrackControl(GlobalVariables& gv) ;
extern void nxtrackControl(GlobalVariables& gv) ;
extern void xtrackControl(GlobalVariables& gv) ;
extern void nholdControl(GlobalVariables& gv) ;
extern void holdControl(GlobalVariables& gv) ;
extern void ngotoControl(GlobalVariables& gv) ;
extern void gotoControl(GlobalVariables& gv) ;
extern void ntrackControl(GlobalVariables& gv) ;
extern void trackControl(GlobalVariables& gv) ;
extern void pfmoveControl(GlobalVariables& gv) ;
extern void lineControl(GlobalVariables& gv) ;
extern void proj1Control(GlobalVariables& gv) ;
extern void proj2Control(GlobalVariables& gv) ;
extern void proj3Control(GlobalVariables& gv) ;

// PrintDebug is called when you type pdebug at the prompt
extern void PrintDebug(GlobalVariables& gv) ;


#endif //#ifdef WIN32


#endif