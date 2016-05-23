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
  \file   usarsim.cpp
  \brief  Provides a ROS interface to the USARSim simulation system.

  More information on USARSim may be found at www.usarsim.sourceforge.net.
  USARSim provides a physics based 3D simulation of various robots, sensors,
  humans, and automation equipment. This interface allows full access to
  the simulation. This current factoring of the code was performed by
  Stephen Balakirsky and is based on code from Fred Proctor.

  \code CVS Status:
  $Author: dr_steveb $
  $Revision: 1.6 $
  $Date: 2007/05/16 20:57:14 $
  \endcode

  \author Stephen Balakirsky
  \date   October 19, 2011
*/
#include "ros/ros.h"
#include "ulapi.hh"
#include "servoInf.hh"
#include "usarsimInf.hh"

void
rosThread (void *arg)
{
  ServoInf *servo = reinterpret_cast < ServoInf * >(arg);

  //  servo->this = ((RosThreadArgs*)arg)->thisPtr;
  servo->msgIn ();
  ROS_WARN ("Servo thread exited");
}

int
main (int argc, char **argv)
{
  ServoInf *servo;		// servo level interface
  UsarsimInf *usarsim;		// usarsim interface
  void *rosTask = NULL;
  // init ros
  ros::init (argc, argv, "usarsim");
  //  ros::Rate r(60);

  servo = new ServoInf ();
  usarsim = new UsarsimInf ();

  // this code uses the ULAPI library to provide portability
  // between different operating systems and architectures
  if (ULAPI_OK != ulapi_init (UL_USE_DEFAULT))
    {
      ROS_FATAL ("can't initialize ulapi");
      return 1;
    }

  // initialize the ROS interface wrapper
  servo->init (usarsim);

  // initialize the USARSim interface wrapper
  usarsim->init (servo);

  rosTask = ulapi_task_new ();

  ulapi_task_start (rosTask, rosThread, (void *) servo, ulapi_prio_lowest (),
		    1);

  // main loop
  while ((usarsim->getNH ())->ok ())
    {
      if (usarsim->msgIn () != 1)
	{
	  ROS_ERROR ("Error from usarsimInf, exiting");
	  break;
	}
    }
  ulapi_exit ();
}
