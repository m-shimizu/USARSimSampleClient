/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

/*!
  \file simware.c

  \brief Definitions of some Simware utility functions.
*/

#include <stdio.h>
#include "simware.h"

char *
sw_type_to_string (sw_type type)
{
  static char retstr[3 * sizeof (type)];	/* not reentrant, just for debugging */

  if (type == SW_SEN_ENCODER)
    return "SW_SEN_ENCODER";
  if (type == SW_SEN_SONAR)
    return "SW_SEN_SONAR";
  if (type == SW_SEN_RANGESCANNER)
    return "SW_SEN_RANGESCANNER";
  if (type == SW_SEN_RANGEIMAGER)
    return "SW_SEN_RANGEIMAGER";
  if (type == SW_SEN_TOUCH)
    return "SW_SEN_TOUCH";
  if (type == SW_SEN_CO2)
    return "SW_SEN_CO2";
  if (type == SW_SEN_GROUNDTRUTH)
    return "SW_SEN_GROUNDTRUTH";
  if (type == SW_SEN_INS)
    return "SW_SEN_INS";
  if (type == SW_SEN_GPS)
    return "SW_SEN_GPS";
  if (type == SW_SEN_ODOMETER)
    return "SW_SEN_ODOMETER";
  if (type == SW_SEN_VICTIM)
    return "SW_SEN_VICTIM";
  if (type == SW_SEN_TACHOMETER)
    return "SW_SEN_TACHOMETER";
  if (type == SW_SEN_ACOUSTIC)
    return "SW_SEN_ACOUSTIC";
  if (type == SW_EFF_GRIPPER)
    return "SW_EFF_GRIPPER";
  if (type == SW_ACT)
    return "SW_ACT";
  if (type == SW_ROBOT_FIXED)
    return "SW_ROBOT_FIXED";
  if (type == SW_ROBOT_GROUNDVEHICLE)
    return "SW_ROBOT_GROUNDVEHICLE";
  if (type == SW_ROBOT_AIRBOT)
    return "SW_ROBOT_AIRBOT";
  if (type == SW_DEVICE_BOXCHUTE)
    return "SW_DEVICE_BOXCHUTE";
  if (type == SW_DEVICE_CONVEYOR)
    return "SW_DEVICE_CONVEYOR";
  if (type == SW_OBJECT_CARGO)
    return "SW_OBJECT_CARGO";

  sprintf (retstr, "%d", (int) type);

  return retstr;
}
