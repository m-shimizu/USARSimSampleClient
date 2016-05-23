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
  \file   usarsimList.cpp
  \brief  Provides the list class for maintaining lists of a particular type of sensor.

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
#include "usarsimList.hh"

UsarsimList::UsarsimList(int typeIn)
{
  sw.time = 0;
  sw.op = SW_NONE;
  sw.type = typeIn;
  sw.name = "";
  didConfMsg = 0;
  didGeoMsg = 0;
}

void UsarsimList::setName( const char *name)
{
  sw.name = name;
}

UsarsimList *UsarsimList::classFind (std::string name )
{
  UsarsimList *ptr;

  ptr = this;
  while (ptr->sw.name != "")
    {
      if (ptr->sw.name == name )
	{
	  /* found it */
	  return ptr;
	}
      ptr = ptr->next;
    }

  /* a new one-- fill in the terminal empty structure... */
  ptr->sw.name = name;
  ptr->didConfMsg = 0;
  ptr->didGeoMsg = 0;

  /* ...and get a new terminal empty structure */
  ptr->next = new UsarsimList(ptr->sw.type);
  return ptr;
}
