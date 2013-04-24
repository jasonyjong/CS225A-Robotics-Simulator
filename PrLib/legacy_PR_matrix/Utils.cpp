// *******************************************************************
// Utils.cpp
//
// This file implements miscellaneous functions used throughout the
// code.
// *******************************************************************

#include "Utils.h"
#include <stdlib.h>
using std::vector;

// This variable is set when the program is shutting down.  It tells
// the various threads to quit.

bool g_appQuit = false;

// ===================================================================
// toRad():  Convert from degrees to radians.
// ===================================================================

Float toRad( Float val )
{
  return (Float) (val * ( M_PI / 180.0 ));
}

void toRad( PrVector& vals )
{
  for( int ii = 0; ii < vals.size(); ii++ )
  {
    vals[ii] *= ( M_PI / 180.0 );
  }
}

// ===================================================================
// toDeg():  Convert from radians to degrees
// ===================================================================

Float toDeg( Float radval )
{
  return (Float) (radval * ( 180.0 / M_PI ));
}

void toDeg( PrVector& vals )
{
  for( int ii = 0; ii < vals.size(); ii++ )
  {
    vals[ii] = (Float) (vals[ii]*( 180.0 / M_PI ));
  }
}

// ===================================================================
// CheckJointLimits(): This function checks if a given joint position
// vector lies in the allowed boundaries.  Returns true if yes, false
// otherwise.
// ===================================================================
bool CheckJointLimits( const PrVector& pos,
                      const PrVector& qmin, const PrVector& qmax )
{
  SAIAssert( pos.size() == qmin.size() && pos.size() == qmax.size() );
  for( int ii = 0; ii < pos.size(); ii++ ) 
  {
    if ( pos[ii] > qmax[ii] || pos[ii] < qmin[ii] )
    {
      printf( "joint %d pos = %f\n", ii, pos[ii] );
      return false;
    }
  }
  return true;
}
