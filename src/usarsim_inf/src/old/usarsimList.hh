/*****************************************************************************
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*****************************************************************************/
/*!
  \file   usarsimMisc.hh
  \brief  Provides several utility classes for the usarsim interface

  More information on USARSim may be found at www.usarsim.sourceforge.net.
  USARSim provides a physics based 3D simulation of various robots, sensors,
  humans, and automation equipment. This interface allows full access to
  the simulation. This current factoring of the code was performed by
  Stephen Balakirsky and is based on code from Fred Proctor.

  \code CVS Status:
  $Author: dr_steveb $
  $Revision: $
  $Date: $
  \endcode

  \author Stephen Balakirsky
  \date   October 19, 2011
*/
#ifndef __usarsimList__
#define __usarsimList__
#include "simware.hh"

//! Flipper definitions
typedef enum FLIPPER_TYPE {
  FR_FLIPPER_TYPE, //front right
  FL_FLIPPER_TYPE, //front left
  RR_FLIPPER_TYPE, //rear right
  RL_FLIPPER_TYPE  //rear left
};

////////////////////////////////////////////////////////////////////////
// UsarsimList
////////////////////////////////////////////////////////////////////////
class UsarsimList
{
public:
  UsarsimList(int typeIn=SW_TYPE_UNINITIALIZED);
  void setName( const char *name);
  sw_struct *getSW(){return &sw;}
  UsarsimList *classFind( std::string name );
  int didConf(){return didConfMsg;}
  int didGeo(){return didGeoMsg;}
  void setDidConf(int value){ didConfMsg=value;}
  void setDidGeo(int value){ didGeoMsg=value;}
  UsarsimList *getNext(){return next;}

private:
  sw_struct sw;
  int didConfMsg;
  int didGeoMsg;
  UsarsimList *next;
};

#endif
