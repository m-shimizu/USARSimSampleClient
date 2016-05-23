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
  \file   usarsimInf.hh
  \brief  Provides the class that will read and write from the ROS system.

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
#ifndef __usarsimInf__
#define __usarsimInf__
#include <ros/ros.h>
#include "simware.hh"
#include "usarsimMisc.hh"
#include "genericInf.hh"
#include "ulapi.hh"

#define SOCKET_MUTEX_KEY 1
#define DELIMITER 10
#define MAX_MSG_LEN 1024
#define MAX_TOKEN_LEN 1024
/* only works with arrays, not heap */
#define NULLTERM(s) (s)[sizeof(s)-1]=0
#define BUFFERLEN 8

//////////////////////////////////////////////
// structures
//////////////////////////////////////////////
typedef struct
{
  char token[MAX_TOKEN_LEN];
  char *ptr;
  char *nextptr;
  int sawname;
  int count;
  double time;
  int op;
  UsarsimList def;
  UsarsimList *where;
} componentInfo;

//////////////////////////////////////////////
// class
//////////////////////////////////////////////
class UsarsimInf:public GenericInf
{
public:
  UsarsimInf ();
  int init (GenericInf * siblingIn);
  int tell (sw_struct * sw, componentInfo info);
  int ask ();
  char *getKey (char *msg, char *key);
  char *getValue (char *msg, char *value);
  int expect (componentInfo * info, const char *token);
  int getName (UsarsimList * list, componentInfo * info, int op);
  int getInteger (componentInfo * info);
  double getReal (componentInfo * info);
  void getTime (componentInfo * info);
  int msgIn ();
  int msgout (sw_struct * sw, componentInfo info);
  int peerMsg (sw_struct * sw);

private:
  int waitingForConf;
  int waitingForGeo;
  int socket_fd;
  void *socket_mutex;
  int buildlen;
  char *build;
  char *build_ptr;
  char *build_end;
  char str[MAX_MSG_LEN];
  /* list to hold all of the sensors */
  UsarsimList *encoders;
  UsarsimList *sonars;
  UsarsimList *rangescanners;
  UsarsimList *rangeimagers;
  UsarsimList *touches;
  UsarsimList *co2sensors;
  UsarsimList *inses;
  UsarsimList *groundtruths;
  UsarsimList *gpses;
  UsarsimList *odometers;
  UsarsimList *victims;
  UsarsimList *tachometers;
  UsarsimList *acoustics;
  UsarsimList *objectsensors;

  UsarsimList *misstas;

  UsarsimList *grippers;
  UsarsimList *toolchangers;

  UsarsimList *robot;

  void setComponentInfo (char *msg, componentInfo * info);
  ulapi_integer usarsim_socket_write (ulapi_integer id, char *buf,
				      ulapi_integer len);
  int doSenConfs (UsarsimList * where, char *type);
  int doEffConfs (UsarsimList * where, char *type);
  int doRobotConfs (UsarsimList * where);

  int handleConf (char *msg);
  int handleConfEncoder (char *msg);
  int handleConfSonar (char *msg);
  int handleConfRangeimager (char *msg);
  int handleConfRangescanner (char *msg);
  int handleConfTouch (char *msg);
  int handleConfCo2sensor (char *msg);
  //  int handleConfGroundtruth (char *msg);
  int handleConfGps (char *msg);
  int handleConfIns (char *msg, const char *sensorType);
  int handleConfOdometry (char *msg);
  int handleConfTachometer (char *msg);
  int handleConfAcoustic (char *msg);
  int handleConfVictim (char *msg);
  int handleConfGripper (char *msg);
  int handleConfToolchanger (char *msg);
  int handleConfActuator (char *msg);
  int handleConfGroundvehicle (char *msg);
  int handleConfBasemachine (char *msg);
  int handleConfStaticplatform (char *msg);
  int handleConfObjectsensor (char *msg);

  int handleGeo (char *msg);
  int handleGeoEncoder (char *msg);
  int handleGeoSonar (char *msg);
  int handleGeoTouch (char *msg);
  int handleGeoCo2sensor (char *msg);
  //  int handleGeoGroundtruth (char *msg);
  int handleGeoGps (char *msg);
  int handleGeoIns (char *msg, const char *sensorType);
  int handleGeoOdometry (char *msg);
  int handleGeoTachometer (char *msg);
  int handleGeoAcoustic (char *msg);
  int handleGeoVictim (char *msg);
  int handleGeoActuator (char *msg);
  int handleGeoGroundvehicle (char *msg);
  int handleGeoBasemachine (char *msg);
  int handleGeoStaticplatform (char *msg);
  int handleGeoGripper (char *msg);
  
  int handleGeoComponent(const char* componentName, char *msg, sw_pose &mount, UsarsimList *list, int opcode);

  int handleAsta (char *msg);
  int handleMsg (char *msg);
  
  int handleSen (char *msg);
  int handleSenEncoder (char *msg);
  int handleSenSonar (char *msg);
  int handleSenRangeimager (char *msg);
  int handleSenRangescanner (char *msg);
  int handleSenTouch (char *msg);
  int handleSenCo2sensor (char *msg);
  //  int handleSenGroundtruth (char *msg);
  int handleSenGps (char *msg);
  int handleSenIns (char *msg, const char *sensorType);
  int handleSenOdometry (char *msg);
  int handleSenTachometer (char *msg);
  int handleSenAcoustic (char *msg);
  int handleSenVictim (char *msg);
  int handleSenGripper (char *msg);
  int handleSenMispkg (char *msg);
  int handleSenGroundvehicle (char *msg);
  int handleSenBasemachine (char *msg);
  int handleSenStaticplatform (char *msg);
  int handleSenObjectSensor(char *msg);
  int handleSenKinect(char *msg);
  
  int handleEff (char *msg);
  int handleEffGripper(char *msg);
  int handleEffToolchanger(char *msg);

  int handleSta (char *msg);
  int handleStaGroundvehicle (char *msg);
  int handleStaBasemachine (char *msg);
  int handleStaStaticplatform (char *msg);

  int handleEm (char *msg);
  int handleNfo (char *msg);
};
#endif
