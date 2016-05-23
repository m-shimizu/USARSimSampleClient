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
  \file   genericInf.hh
  \brief  Provides a generic interface class for the simulation interface.

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
  \date   October 20, 2011
*/
#ifndef __genericInf__
#define __genericInf__
#include "ros/ros.h"
#include "simware.hh"

class GenericInf
{
public:
  GenericInf * sibling;
  GenericInf ();
  ros::NodeHandle * getNH ();
  int init (GenericInf * siblingIn);
  int msgOut ();
  int msgIn (sw_struct * sw);
  virtual int peerMsg (sw_struct * sw);
protected:
    ros::NodeHandle * nh;
};
#endif
