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
#ifndef __usarsimMisc__
#define __usarsimMisc__
#include <deque>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <usarsim_inf/SenseObject.h>
#include <usarsim_inf/EffectorStatus.h>
#include <usarsim_inf/EffectorCommand.h>
#include <usarsim_inf/ToolchangerStatus.h>
#include <usarsim_inf/RangeImageScan.h>
#include "simware.hh"
#include "genericInf.hh"

//using namespace std;

////////////////////////////////////////////////////////////////////////
//TrajectoryPoint
////////////////////////////////////////////////////////////////////////
class TrajectoryPoint
{
public:
  TrajectoryPoint();
  unsigned int numJoints;
  double jointGoals[SW_ACT_LINK_MAX];
  double tolerances[SW_ACT_LINK_MAX];
  ros::Time time;
};

////////////////////////////////////////////////////////////////////////
//Trajectory
////////////////////////////////////////////////////////////////////////
class Trajectory
{
public:
  std::deque<TrajectoryPoint> goals;
  TrajectoryPoint finalGoal;
};

////////////////////////////////////////////////////////////////////////
//Cycle timer
////////////////////////////////////////////////////////////////////////
typedef struct CycleTimer
{
  ros::Time lastTime;
  double cycleTime;
  std::deque <double> cycleDeque;
}CycleTimer;

////////////////////////////////////////////////////////////////////////
// UsarsimList
////////////////////////////////////////////////////////////////////////
class UsarsimList
{
public:
  UsarsimList (int typeIn = SW_TYPE_UNINITIALIZED);
  void setName (const char *name);
  sw_struct *getSW ()
  {
    return &sw;
  }
  UsarsimList *classFind (std::string name);
  int didConf ()
  {
    return didConfMsg;
  }
  int didGeo ()
  {
    return didGeoMsg;
  }
  void setDidConf (int value)
  {
    didConfMsg = value;
  }
  void setDidGeo (int value)
  {
    didGeoMsg = value;
  }
  UsarsimList *getNext ()
  {
    return next;
  }

private:
  sw_struct sw;
  int didConfMsg;
  int didGeoMsg;
  UsarsimList *next;
};


////////////////////////////////////////////////////////////////////////
// Flippers
////////////////////////////////////////////////////////////////////////
enum FLIPPER_TYPE
{
  FLIPPER_NONE_TYPE = 0,	// no flipper set
  FR_FLIPPER_TYPE,		//front right
  FL_FLIPPER_TYPE,		//front left
  RR_FLIPPER_TYPE,		//rear right
  RL_FLIPPER_TYPE		//rear left
};

////////////////////////////////////////////////////////////////////////
// UsarsimFlippers: Provides FlipperSettings
////////////////////////////////////////////////////////////////////////
class UsarsimFlippers
{
public:
  UsarsimFlippers ();
  geometry_msgs::Transform flipperTrans;
  FLIPPER_TYPE fType;
  double minAngle;
  double maxAngle;
  double length;
  double width;
};

////////////////////////////////////////////////////////////////////////
// UsarsimPlatform
////////////////////////////////////////////////////////////////////////
// ROS does not seem to maintain some vital information about the
// robot. So, we maintain it here
class UsarsimPlatform
{
public:
  UsarsimPlatform ();
  geometry_msgs::TransformStamped tf;	// transform for platform
  double cycleTime;		//!< cycle time, in seconds, of mobility servo
  //! Name of the platform 
    std::string platformName;
  /*! length, width and height are the bounding box around the vehicle,
     origin is at center. Units are [m]. x forward/back, y left/right
     z up/down */
    geometry_msgs::Vector3 platformSize;
  //! mass in [g]
  double mass;
  //! center of gravity wrt origin, in [m], same convention as above
    geometry_msgs::Vector3 cg;
  sw_steer_type steerType;
  bool groundTruthSet;
  /* not yet used
     //! Number of effecters mounted on platform
     // If effCount==0, servo echelon will not open the effecter nml channels
     int effCount;
     //! Number of sensors mounted on platform
     int senCount;
     //! Number of mission packages mounted on platform
     // Indicates which mission package nml channels are valid
     int misCount;
   */
};

////////////////////////////////////////////////////////////////////////
// UsarsimGrdVeh
////////////////////////////////////////////////////////////////////////
class UsarsimGrdVeh:public UsarsimPlatform
{
public:
  UsarsimGrdVeh ();
  double maxWheelRot;
  double maxTorque;
  double wheelSeparation;
  double wheelRadius;
  double wheelBase;
  double maxSteerAngle;
  double maxCrabAngle;
  double minTurningRadius;
  UsarsimFlippers *flippers;
  int numFlippers;
};

////////////////////////////////////////////////////////////////////////
// UsarsimSensor
////////////////////////////////////////////////////////////////////////
class UsarsimSensor
{
public:
  UsarsimSensor ();
  std::string name;		// name of sensor from usarsim
  double time;			// usarsim time of last report
  ros::Publisher pub;		// publisher for data
  geometry_msgs::TransformStamped tf;	// transform for sensor
  int linkOffset; //which link this component is mounted on. -1 if not parented to a link.  
};

////////////////////////////////////////////////////////////////////////
// OdomSensor
////////////////////////////////////////////////////////////////////////
class UsarsimOdomSensor:public UsarsimSensor
{
public:
  UsarsimOdomSensor ();
  geometry_msgs::Twist lastPosition;
  nav_msgs::Odometry odom;
  
};

////////////////////////////////////////////////////////////////////////
// Range scanner
////////////////////////////////////////////////////////////////////////
class UsarsimRngScnSensor:public UsarsimSensor
{
public:
  UsarsimRngScnSensor ();
  sensor_msgs::LaserScan scan;
};
////////////////////////////////////////////////////////////////////////
// Range Imager
////////////////////////////////////////////////////////////////////////
class UsarsimRngImgSensor:public UsarsimSensor
{
public:
  UsarsimRngImgSensor(GenericInf *parentInf);
  GenericInf *infHandle;
  sensor_msgs::Image depthImage;
  int totalFrames;
  ros::Publisher cameraInfoPub;
  ros::Subscriber command;
  sensor_msgs::CameraInfo camInfo;
  geometry_msgs::TransformStamped opticalTransform;
  bool isReady();
  void sentFrame(int frame);
  void commandCallback(const usarsim_inf::RangeImageScanConstPtr &msg);
private:
  int lastFrameReceived;
  bool ready;
};
////////////////////////////////////////////////////////////////////////
// Object sensor
////////////////////////////////////////////////////////////////////////
class UsarsimObjectSensor:public UsarsimSensor
{
public:
  UsarsimObjectSensor ();
  usarsim_inf::SenseObject objSense;
};

////////////////////////////////////////////////////////////////////////
// Gripper
////////////////////////////////////////////////////////////////////////
class UsarsimGripperEffector:public UsarsimSensor
{
public:
  UsarsimGripperEffector (GenericInf *parentInf);
  GenericInf *infHandle;
  
  //status and commands could all be done with an action server.
  //it can be changed later if I have time.
  void commandCallback(const usarsim_inf::EffectorCommandConstPtr &msg);
  ros::Subscriber command;
  usarsim_inf::EffectorCommand goal;
  usarsim_inf::EffectorStatus status;
  bool isActive(){return commandActive;}
  void clearActive(){commandActive = false;}
  bool isDone();
private:
  bool commandActive;
};
////////////////////////////////////////////////////////////////////////
// Toolchanger
////////////////////////////////////////////////////////////////////////
class UsarsimToolchanger:public UsarsimSensor
{
public:
  void commandCallback(const usarsim_inf::EffectorCommandConstPtr &msg);
  ros::Subscriber command;
  UsarsimToolchanger (GenericInf *parentInf);
  GenericInf *infHandle;
  usarsim_inf::ToolchangerStatus status;
  usarsim_inf::EffectorCommand goal;
};
////////////////////////////////////////////////////////////////////////
// Actuators
////////////////////////////////////////////////////////////////////////
class UsarsimActuator:public UsarsimSensor
{
public:
  UsarsimActuator (GenericInf *parentInf);
  ~UsarsimActuator();
  std::vector<float> minValues;
  std::vector<float> maxValues; 
  std::vector<float> maxTorques; //config data needed for URDF generation
  std::vector <geometry_msgs::TransformStamped> jointTf; // transforms for links
  
  sensor_msgs::JointState jstate;
  GenericInf *infHandle;
  CycleTimer cycleTimer;
  Trajectory currentTrajectory;
  int numJoints;
  
  void setUpTrajectory();
  void trajectoryCallback();
  void preemptCallback();
  bool isTrajectoryActive();
  bool preempted();
  void setTrajectoryResult(control_msgs::FollowJointTrajectoryResult result);
private:
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> *trajectoryServer;
};

////////////////////////////////////////////////////////////////////////
// UsarsimConverter
////////////////////////////////////////////////////////////////////////
class UsarsimConverter
{
public:
  static geometry_msgs::Vector3 PointToVector(geometry_msgs::Point pointIn);
  static geometry_msgs::Point VectorToPoint(geometry_msgs::Vector3 pointIn);
};

#endif
