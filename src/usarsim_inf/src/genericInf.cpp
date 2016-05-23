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
  \file   genericInf.cpp
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
#include "genericInf.hh"

GenericInf::GenericInf ()
{
  nh = new ros::NodeHandle ();
}

ros::NodeHandle * GenericInf::getNH ()
{
  return nh;
}

int
GenericInf::init (GenericInf * siblingIn)
{
  //  sleep (1);			// allows the logging facility to catch up and log stuff

  sibling = siblingIn;
  ROS_INFO ("GenericInf sibling set");
  return 1;
}

int
GenericInf::peerMsg (sw_struct * sw)
{
  ROS_ERROR ("peerMsg should be overwritten");
  return -1;
}

int
GenericInf::msgIn (sw_struct * sw)
{
  ROS_ERROR ("msgIn should be overwritten");
  return -1;
}

int
GenericInf::msgOut ()
{
  ROS_ERROR ("msgOut should be overwritten");
  return -1;
}
