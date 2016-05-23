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
  \file   servoInf.cpp
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
#include "servoInf.hh"
#include <sensor_msgs/image_encodings.h>
#include "ulapi.hh"

void
ServoInf::VelCmdCallback (const geometry_msgs::TwistConstPtr & msg)
{
  sw_struct sw;

  /*
  ROS_INFO ("servoInf received: <%f %f %f> <%f %f %f>",
	    msg->linear.x,
	    msg->linear.y,
	    msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
  */
  sw.type = SW_ROS_CMD_VEL;
  sw.data.roscmdvel.linearx = msg->linear.x;
  sw.data.roscmdvel.lineary = msg->linear.y;
  sw.data.roscmdvel.linearz = msg->linear.z;
  sw.data.roscmdvel.angularx = msg->angular.x;
  sw.data.roscmdvel.angulary = msg->angular.y;
  sw.data.roscmdvel.angularz = msg->angular.z;
  sibling->peerMsg (&sw);
  return;
}

ServoInf::ServoInf ():GenericInf ()
{
  botType = SW_ROBOT_UNKNOWN;
  // set platform pointer to something to avoid core dumps
  basePlatform = &grdVehSettings;
  buildTFTree = false;
}

/*const UsarsimActuator*
ServoInf::getActuator( unsigned int num)
{
  if( num > actuators.size() )
    return NULL;
  return &actuators[num];
}*/
std::list<UsarsimActuator>::iterator ServoInf::getActuatorBegin()
{
	return actuators.begin();
}
std::list<UsarsimActuator>::iterator ServoInf::getActuatorEnd()
{
	return actuators.end();
}
//unsigned int
//ServoInf::getNumActuators()
//{
//  return actuators.size();
//}
unsigned int ServoInf::getNumExtras()
{
	return grippers.size() + toolchangers.size(); //should include any components that are specified in the robot URDF file
}
const UsarsimSensor* ServoInf::getComponent(unsigned int num)
{
	if(num >= grippers.size() || grippers.empty())
	{
		if(num - grippers.size() >= toolchangers.size())
			return NULL;
		return &toolchangers[num - grippers.size()];
	}
	return &grippers[num];
}
const std::string
ServoInf::getPlatformName()
{
  return basePlatform->platformName;
}

const geometry_msgs::Vector3 
ServoInf::getPlatformSize()
{
  return basePlatform->platformSize;
}

/* The interface must be initialized prior to use.
   Still more to do in this routine!
*/
int
ServoInf::init (GenericInf * usarsimIn)
{
  if (!nh->getParam ("/usarsim/odomSensor", odomName))
    {
      odomName = std::string("");
      ROS_DEBUG ("Parameter /usarsim/odomSensor not set");
    }
  else
    ROS_DEBUG ("Parameter /usarsim/odomSensor: %s", odomName.c_str ());
  buildTFTree = false;
  
  //initialize joint publisher
  jointPublisher = n.advertise <sensor_msgs::JointState> ("joint_states", 2);
  //add the world joint
  addJoint("world_joint", 0.0);
	  
  sibling = usarsimIn;
  servoSetMutex = ulapi_mutex_new (SERVO_SET_KEY);
  if (servoSetMutex == NULL)
    {
      ROS_ERROR ("Unable to create servoSetMutex");
      return -1;
    }
  ROS_INFO ("servoInf initialized");
  return 1;
}
void ServoInf::setBuildingTFTree()
{
	buildTFTree = true;
}
int
ServoInf::peerMsg (sw_struct * sw)
{
  int num;
  UsarsimActuator *actPtr;
  static double previousTime = 0;
  ros::Time currentTime;
  currentTime = ros::Time::now();
  if( sw->time <= 0. )
    {
      sw->time = previousTime;
      ROS_WARN ("Sensor msg class %s with operand %d without time",
		swTypeToString (sw->type), sw->op);
    }
  switch (sw->type)
    {
    case SW_ACT:
      switch (sw->op)
	{
	case SW_ACT_STAT:
	actPtr = actuatorIn(actuators, sw->name);
	  //num = actuatorIndex (actuators, sw->name);
	  if( copyActuator( actPtr, sw ) )
	  {
	    //actuators[num].pub.publish (actuators[num].jstate);
	    updateActuatorTF(actPtr, sw, buildTFTree);
	    if(!buildTFTree)
	    	publishJoints();
	    updateActuatorCycle(actPtr);
	    if(actPtr->isTrajectoryActive() && updateTrajectory(actPtr, sw))
	    {
	    	control_msgs::FollowJointTrajectoryResult result;
	    	if(checkTrajectoryGoal(actPtr, sw))
	    	{
	    		ROS_INFO("Trajectory succeeded");
	    		result.error_code = result.SUCCESSFUL;
	    	}
	    	else
	    	{
	    		ROS_ERROR("Trajectory aborted: arm position not at goal");
	    		result.error_code = result.GOAL_TOLERANCE_VIOLATED;
	    	}
	    	actPtr->setTrajectoryResult(result);
	    }
	  }
	  
	  break;

	case SW_ACT_SET:
	  actPtr = actuatorIn (actuators, sw->name);
	  if(copyActuator( actPtr, sw ))
	  {
	  	publishJoints();
	  	updateActuatorTF(actPtr, sw, true);
	  }
	  //	  rosTfBroadcaster.sendTransform (actuators[num].tf);
	  //	  ROS_INFO ( "Act setting %d joints", actuators[num].jointTf.size() );
	  //	  for( unsigned int count=0; count<actuators[num].jointTf.size(); 
	  //	       count++ )
	  //	    rosTfBroadcaster.sendTransform (actuators[num].jointTf[count]);
	  break;

	default:
	  break;
	}
      break;

    case SW_SEN_INS:
      switch (sw->op)
	{
	case SW_SEN_INS_STAT:
	  ROS_DEBUG
	    ("Ins status for %s at time %f: %f,%f,%f %f,%f,%f",
	     sw->name.c_str (), sw->time, sw->data.ins.position.x,
	     sw->data.ins.position.y, sw->data.ins.position.z,
	     sw->data.ins.position.roll,
	     sw->data.ins.position.pitch,
	     sw->data.ins.position.yaw);

	  num = odomSensorIndex (odometers, sw->name);
	  if (copyIns (&odometers[num], sw) == 1)
	    {
	      rosTfBroadcaster.sendTransform (odometers[num].tf);
	      if(odometers[num].name == odomName)
	      	rosTfBroadcaster.sendTransform(basePlatform->tf);
	      /*
	      ROS_INFO("Sending transform frame: %s child: %s",
		       odometers[num].tf.header.frame_id.c_str(),
		       odometers[num].tf.child_frame_id.c_str());
	      */
	    }
	  /*
	  ROS_INFO("Sending odometer message for %s <%f %f>", 
		   odometers[num].odom.header.frame_id.c_str (),
		   odometers[num].odom.pose.pose.position.x,
		   odometers[num].odom.pose.pose.position.y);
	  */
	  odometers[num].pub.publish (odometers[num].odom);
	  break;
	case SW_SEN_INS_SET:
	  ROS_DEBUG ("Ins settings for %s: %f %f,%f,%f %f,%f,%f",
		     sw->name.c_str (),
		     sw->data.ins.period,
		     sw->data.ins.mount.x,
		     sw->data.ins.mount.y,
		     sw->data.ins.mount.z,
		     sw->data.ins.mount.roll,
		     sw->data.ins.mount.pitch,
		     sw->data.ins.mount.yaw);
	  num = odomSensorIndex (odometers, sw->name);
	  if (copyIns (&odometers[num], sw) == 1)
	    {
	      rosTfBroadcaster.sendTransform (odometers[num].tf);
	      if(odometers[num].name == odomName)
	      {
	      	rosTfBroadcaster.sendTransform (basePlatform->tf);
	      	if(!basePlatform->groundTruthSet)
	      		ROS_INFO("Ground truth set.");
	      	basePlatform->groundTruthSet = true;
	      }
	      else if(!basePlatform->groundTruthSet)
	      {
	      	ROS_ERROR("Got status for INS sensor \"%s\" but have not yet received status for expected ground truth sensor \"%s\"",
	      	odometers[num].name.c_str(), odomName.c_str());
	      }
	      /*
	      ROS_INFO("Sending transform frame: %s child: %s",
		       odometers[num].tf.header.frame_id.c_str(),
		       odometers[num].tf.child_frame_id.c_str());
	      */
	    }
	  break;
	default:
	  break;
	}
      break;

    case SW_ROBOT_FIXED:
      switch (sw->op)
	{
	case SW_DEVICE_STAT:
	publishJoints();
	  if (botType == SW_ROBOT_UNKNOWN)
	    {
	      botType = SW_ROBOT_STATIC_VEH;
	     
	      // first time we know about the robot type
	      copyStaticVehSettings(basePlatform, sw);
	       if(!basePlatform->groundTruthSet)
		  	ROS_INFO("Waiting for ground truth...");
	    }
	  else if (botType != SW_ROBOT_STATIC_VEH)
	    {
	      // it was already set to something else
	      ROS_WARN ("Fixed Robot.stat overriding %s\n",
			swRobotType (botType));
	    }
	  else
	    {
	     if(!basePlatform->groundTruthSet)
		  	ROS_INFO("Waiting for ground truth...");
	    //rosTfBroadcaster.sendTransform (basePlatform->tf);
	    /*
	    ROS_INFO("Sending vehicle transform frame: %s child: %s <%f %f>",
		     basePlatform->tf.header.frame_id.c_str(),
		     basePlatform->tf.child_frame_id.c_str(),
		     basePlatform->tf.transform.translation.x,
		     basePlatform->tf.transform.translation.y);
	    */
	    }
	  break;

	case SW_DEVICE_SET:
	  if (botType == SW_ROBOT_UNKNOWN)
	    {
	      // first time we know about the robot type
	      botType = SW_ROBOT_STATIC_VEH;
	      if (copyStaticVehSettings (basePlatform, sw) == 1)
		{
		  
		  //rosTfBroadcaster.sendTransform (basePlatform->tf);
		  /*
		  ROS_INFO("Sending vehicle transform frame: %s child: %s <%f %f>",
			   basePlatform->tf.header.frame_id.c_str(),
			   basePlatform->tf.child_frame_id.c_str(),
			   basePlatform->tf.transform.translation.x,
			   basePlatform->tf.transform.translation.y);
		  */
		}
	      else
		{
		  ROS_ERROR ("Error copying robot status");
		}
	    }
	  else if (botType != SW_ROBOT_STATIC_VEH)
	    {
	      // it was already set to something else
	      ROS_WARN ("StaticVehicle.set overriding %s\n",
			swRobotType (botType));
	    }
	  else
	    {
	      //rosTfBroadcaster.sendTransform (basePlatform->tf);
	      /*
		  ROS_INFO("Sending vehicle transform frame: %s child: %s <%f %f>",
			   basePlatform->tf.header.frame_id.c_str(),
			   basePlatform->tf.child_frame_id.c_str(),
			   basePlatform->tf.transform.translation.x,
			   basePlatform->tf.transform.translation.y);
	      */
	    }
	  break;
	default:
	  ROS_WARN ("unknown sw operand for class %s with operand %d",
		    swTypeToString (sw->type), sw->op);
	  break;
	}
      break;

    case SW_ROBOT_GROUNDVEHICLE:
      switch (sw->op)
	{
	case SW_ROBOT_STAT:
	  if (botType == SW_ROBOT_UNKNOWN)
	    {
	      botType = SW_ROBOT_GRD_VEH;
	      // first time we know about the robot type
	      if (copyGrdVehSettings (&grdVehSettings, sw) == 1)
		{
		  rosTfBroadcaster.sendTransform (grdVehSettings.tf);
		  /*
		  ROS_INFO("Sending vehicle transform frame: %s child: %s <%f %f>",
			   grdVehSettings.tf.header.frame_id.c_str(),
			   grdVehSettings.tf.child_frame_id.c_str(),
			   grdVehSettings.tf.transform.translation.x,
			   grdVehSettings.tf.transform.translation.y);
		  */
		}
	    }
	  else if (botType != SW_ROBOT_GRD_VEH)
	    {
	      // it was already set to something else
	      ROS_WARN ("GroundVehicle.stat overriding %s\n",
			swRobotType (botType));
	    }
	  else
	    {
	    rosTfBroadcaster.sendTransform (grdVehSettings.tf);
	    /*
	    ROS_INFO("Sending vehicle transform frame: %s child: %s <%f %f>",
		     grdVehSettings.tf.header.frame_id.c_str(),
		     grdVehSettings.tf.child_frame_id.c_str(),
		     grdVehSettings.tf.transform.translation.x,
		     grdVehSettings.tf.transform.translation.y);
	    */
	    }
	  break;

	case SW_ROBOT_SET:
	  if (botType == SW_ROBOT_UNKNOWN)
	    {
	      // first time we know about the robot type
	      botType = SW_ROBOT_GRD_VEH;
	      if (copyGrdVehSettings (&grdVehSettings, sw) == 1)
		{
		  rosTfBroadcaster.sendTransform (grdVehSettings.tf);
		  /*
		  ROS_INFO("Sending vehicle transform frame: %s child: %s <%f %f>",
			   grdVehSettings.tf.header.frame_id.c_str(),
			   grdVehSettings.tf.child_frame_id.c_str(),
			   grdVehSettings.tf.transform.translation.x,
			   grdVehSettings.tf.transform.translation.y);
		  */
		}
	      else
		{
		  ROS_ERROR ("Error copying robot status");
		}
	    }
	  else if (botType != SW_ROBOT_GRD_VEH)
	    {
	      // it was already set to something else
	      ROS_WARN ("GroundVehicle.set overriding %s\n",
			swRobotType (botType));
	    }
	  else
	    {
	      rosTfBroadcaster.sendTransform (grdVehSettings.tf);
	      /*
		  ROS_INFO("Sending vehicle transform frame: %s child: %s <%f %f>",
			   grdVehSettings.tf.header.frame_id.c_str(),
			   grdVehSettings.tf.child_frame_id.c_str(),
			   grdVehSettings.tf.transform.translation.x,
			   grdVehSettings.tf.transform.translation.y);
	      */
	    }
	  break;
	}
      break;

    case SW_SEN_RANGESCANNER:
      switch (sw->op)
	{
	case SW_SEN_RANGESCANNER_STAT:
	  ROS_DEBUG ("RangeScanner status for %s at time %f: ",
		     sw->name.c_str (), sw->time);
	  num = rangeSensorIndex (rangeScanners, sw->name);
	  if (copyRangeScanner (&rangeScanners[num], sw) == 1)
	    {
	      rosTfBroadcaster.sendTransform (rangeScanners[num].tf);
	      /*
	      ROS_INFO("Sending transform frame: %s child: %s",
		       rangeScanners[num].tf.header.frame_id.c_str(),
		       rangeScanners[num].tf.child_frame_id.c_str());
	      ROS_INFO("Sending rangescanner message for %s", sw->name.c_str ());
	      */
	      rangeScanners[num].pub.publish (rangeScanners[num].scan);
	    }
	  else
	    {
	      ROS_ERROR
		("RangeScanner error for %s: can't copy it.",
		 sw->name.c_str ());
	      return -1;
	    }
	  break;

	case SW_SEN_RANGESCANNER_SET:
	  ROS_DEBUG ("RangeScanner settings for %s mount: %s ",
		     sw->name.c_str (),
		     sw->data.rangescanner.mount.offsetFrom);
	  /*
	     ROS_DEBUG ("%f %f %f %f %f,%f,%f %f,%f,%f",
	     sw->data.rangescanner.minrange,
	     sw->data.rangescanner.maxrange,
	     sw->data.rangescanner.resolution,
	     sw->data.rangescanner.fov,
	     sw->data.rangescanner.mount.x,
	     sw->data.rangescanner.mount.y,
	     sw->data.rangescanner.mount.z,
	     sw->data.rangescanner.mount.roll,
	     sw->data.rangescanner.mount.pitch,
	     sw->data.rangescanner.mount.yaw);
	   */
	  num = rangeSensorIndex (rangeScanners, sw->name);
	  if (copyRangeScanner (&rangeScanners[num], sw) == 1)
	    {
	      rosTfBroadcaster.sendTransform (rangeScanners[num].tf);
	      /*
	      ROS_INFO("Sending transform frame: %s child: %s",
		       rangeScanners[num].tf.header.frame_id.c_str(),
		       rangeScanners[num].tf.child_frame_id.c_str());
	      */
	    }
	  else
	    {
	      ROS_ERROR
		("RangeScanner error for %s: can't copy it.",
		 sw->name.c_str ());
	      return -1;
	    }
	  break;
	default:
	  ROS_ERROR ("invalid operation: %d\n", sw->op);
	  return -1;
	  break;
	}
      break;
	case SW_SEN_OBJECTSENSOR:
	switch(sw->op)
	{
		case SW_SEN_OBJECTSENSOR_STAT:
			num = objectSensorIndex(objectSensors, sw->name);
			if(copyObjectSensor(&objectSensors[num], sw) == 1)
			{
				rosTfBroadcaster.sendTransform (objectSensors[num].tf);
				objectSensors[num].pub.publish(objectSensors[num].objSense);
			}
			else
				ROS_ERROR("Object sensor error for %s: can't copy it.",
				sw->name.c_str());
			break;
		case SW_SEN_OBJECTSENSOR_SET:
			num = objectSensorIndex(objectSensors, sw->name);
			if(copyObjectSensor(&objectSensors[num], sw) == 1)
				rosTfBroadcaster.sendTransform (objectSensors[num].tf);
			else
				ROS_ERROR("Object sensor error for %s: can't copy it.",
				sw->name.c_str());
			break;
		default:
			ROS_ERROR("invalid operation: %d\n", sw->op);
			return -1;
		break;
	}
	break;
	case SW_EFF_GRIPPER:
		switch(sw->op)
		{
		case SW_EFF_GRIPPER_STAT:
			num = gripperEffectorIndex(grippers, sw->name);
			if(copyGripperEffector(&grippers[num], sw) == 1)
			{
				//if we aren't building an URDF file, but this item is mounted on an actuator link, publish it as a joint
				//otherwise publish its transformation directly.
				if(!buildTFTree && grippers[num].linkOffset >= 0)
					publishJoints();
				else
					rosTfBroadcaster.sendTransform (grippers[num].tf);
				grippers[num].pub.publish(grippers[num].status);
				if(grippers[num].isActive() && grippers[num].isDone())
					grippers[num].clearActive();
			
			}else
			{
				ROS_ERROR("Gripper effector error for %s: couldn't copy",sw->name.c_str());
			}
			break;
		case SW_EFF_GRIPPER_SET:
			num = gripperEffectorIndex(grippers, sw->name);
			if(copyGripperEffector(&grippers[num], sw) == 1)
			{
				rosTfBroadcaster.sendTransform (grippers[num].tf);
			}else
			{
				ROS_ERROR("Gripper effector error for %s: couldn't copy",sw->name.c_str());
			}
			break;
		default:
			ROS_ERROR("invalid operation: %d\n", sw->op);
			return -1;
			break;
		}
	break;
	case SW_EFF_TOOLCHANGER:
		switch(sw->op)
		{
			case SW_EFF_TOOLCHANGER_STAT:
			num = toolchangerIndex(toolchangers, sw->name);
			if(copyToolchanger(&toolchangers[num], sw) == 1)
			{
				//if we aren't building an URDF file, but this item is mounted on an actuator link, publish it as a joint
				//otherwise publish its transformation directly.
				if(!buildTFTree && toolchangers[num].linkOffset >= 0) 
					publishJoints();
				else
					rosTfBroadcaster.sendTransform(toolchangers[num].tf);
				toolchangers[num].pub.publish(toolchangers[num].status);
			}else
			{
				ROS_ERROR("Toolchanger error for %s: couldn't copy",sw->name.c_str());
			}
			break;
			case SW_EFF_TOOLCHANGER_SET:
			num = toolchangerIndex(toolchangers, sw->name);
			if(copyToolchanger(&toolchangers[num], sw) == 1)
			{
				if(!buildTFTree && toolchangers[num].linkOffset >= 0)
					publishJoints();
				else
					rosTfBroadcaster.sendTransform(toolchangers[num].tf);
			}else
			{
				ROS_ERROR("Toolchanger error for %s: couldn't copy",sw->name.c_str());
			}
			break;
			default:
			ROS_ERROR("Invalid operation: %d\n",sw->op);
			break;
		}
		break;
	case SW_SEN_RANGEIMAGER:
	switch(sw->op)
	{
	case SW_SEN_RANGEIMAGER_STAT:
		num = rangeImagerIndex(rangeImagers, sw->name);
		if(copyRangeImager(&rangeImagers[num], sw) == 1)
		{
			
			rosTfBroadcaster.sendTransform(rangeImagers[num].tf);
			rosTfBroadcaster.sendTransform(rangeImagers[num].opticalTransform);
			//since virtual range imaging is slow, wait for a full scan before publishing the camera info and depth image
			if(rangeImagers[num].isReady())
			{
				rangeImagers[num].depthImage.header.stamp = currentTime;
				rangeImagers[num].camInfo.header.stamp = currentTime;
				//camera info and depth image need to be published in sync
				rangeImagers[num].pub.publish(rangeImagers[num].depthImage);
				rangeImagers[num].cameraInfoPub.publish(rangeImagers[num].camInfo);
				
			}
		}else
		{
			ROS_ERROR("Range imager error for %s, couldn't copy.", sw->name.c_str());
		}
	break;
	case SW_SEN_RANGEIMAGER_SET:
		num = rangeImagerIndex(rangeImagers, sw->name);
		if(copyRangeImager(&rangeImagers[num], sw) == 1)
		{
			rosTfBroadcaster.sendTransform(rangeImagers[num].tf);
			rosTfBroadcaster.sendTransform(rangeImagers[num].opticalTransform);
		}else
		{
			ROS_ERROR("Range imager error for %s: couldn't copy",sw->name.c_str());
		}
	break;
	default:
	ROS_ERROR("Invalid operation: %d",sw->op);
	break;
	}
	break;
    default:
      	ROS_WARN ("unknown sw class %s with operand %d",
		swTypeToString (sw->type), sw->op);
      break;
    }

  previousTime = sw->time;
  return 1;
}

int
ServoInf::msgIn ()
{
  ROS_INFO ("In servoInf msgIn");
  static ros::NodeHandle n;	// need new nodehandle for receivingmessages

  // manage subscriptions
  static ros::Subscriber sub =
    n.subscribe ("cmd_vel", 10, &ServoInf::VelCmdCallback, this); //vehicle velocity subscriber
  //static ros::Subscriber opSub = 
  //  n.subscribe ("cmd_op", 10, &ServoInf::OpCmdCallback, this); //opcode subscriber
  
  ROS_INFO ("servoInf going to spin");
  
  ros::spin ();
  return 1;
}

int
ServoInf::msgOut (void)
{
  ROS_INFO ("In servoInf msgOut");
  return 1;
}

ServoInf::~ServoInf ()
{
  if (servoSetMutex != NULL)
    {
      ulapi_mutex_delete (servoSetMutex);
      servoSetMutex = NULL;
    }
}

int
ServoInf::copyActuator (UsarsimActuator * act, const sw_struct * sw)
{
  ros::Time currentTime;

  std::stringstream tempSS;

  currentTime = ros::Time::now ();
  act->numJoints = sw->data.actuator.number;

  //define the mounting joint for this actuator
  addJoint(act->name + "_mount", 0.0);
  act->minValues.clear();
  act->maxValues.clear();
  act->maxTorques.clear();

  //create actuator joints
  for( int i=0; i<sw->data.actuator.number; i++ )
    {
      tempSS.str("");
      tempSS << i+1;
      
      addJoint(act->name + std::string("_joint_") + tempSS.str (), sw->data.actuator.link[i].position);
      
      act->minValues.push_back(sw->data.actuator.link[i].minvalue);
      act->maxValues.push_back(sw->data.actuator.link[i].maxvalue);
      act->maxTorques.push_back(sw->data.actuator.link[i].maxtorque);
    }
  //  ROS_ERROR( "CopyAct success!!" );
  return 1;
}
int ServoInf::updateActuatorTF(UsarsimActuator *act, const sw_struct *sw, bool broadcastTF)
{
  ros::Time currentTime;
  tf::Quaternion quat;
  geometry_msgs::Quaternion quatMsg;
  geometry_msgs::TransformStamped currentJointTf;  
  std::stringstream tempSS;
  tf::Vector3 currentTipPosition; //relative to actuator base
  tf::Transform lastTipTransform; //relative to actuator base
  tf::Transform absoluteTransform;//relative to actuator base

  currentTime = ros::Time::now ();
  act->jointTf.clear();
  currentJointTf.header.stamp = currentTime;
  
  setTransform(act, sw->data.actuator.mount, currentTime);
  act->tf.child_frame_id = act->name + "_link0";
  if(broadcastTF)
  	rosTfBroadcaster.sendTransform (act->tf);
  
  lastTipTransform.setOrigin(tf::Vector3(0,0,0));
  lastTipTransform.setRotation(tf::Quaternion(0,0,0,1));
  currentTipPosition = tf::Vector3(0,0,0);
  absoluteTransform.setRotation(tf::Quaternion(0,0,0,1)); 
  //  ROS_ERROR( "sent transform from \"%s\" to \"%s\"", act->tf.header.frame_id.c_str(), act->tf.child_frame_id.c_str() );
  for(int i = 0;i<act->numJoints;i++)
  {
      tempSS.str("");
      tempSS << i+1;
      currentJointTf.child_frame_id = act->name + std::string("_link") + tempSS.str ();
	  tempSS.str("");
	  tempSS << sw->data.actuator.link[i].parent;
	  currentJointTf.header.frame_id = act->name + std::string("_link") + tempSS.str ();
	  
	  //USARSim specifies link offsets in actuator coordinates and link rotations in link coordinates,
	  //so we need to treat rotations and positions seperately when calculating link transforms.
      quat = tf::createQuaternionFromRPY (sw->data.actuator.link[i].mount.roll,
					  sw->data.actuator.link[i].mount.pitch,
					  sw->data.actuator.link[i].mount.yaw);
	  
	  //find the position of the next link tip in the actuator coordinate frame
	  currentTipPosition += tf::Vector3(sw->data.actuator.link[i].mount.x,sw->data.actuator.link[i].mount.y,sw->data.actuator.link[i].mount.z);
	  absoluteTransform.setOrigin(currentTipPosition);
	  
	  //find the transformation from the tip of the last link to the tip of the next link
	  tf::Transform relativeTransform = lastTipTransform.inverseTimes(absoluteTransform);
	  
	  //set the rotation to the local one specified by Usarsim
	  relativeTransform.setRotation(quat);
	  
	  //update the transform for the tip of the last link
	  lastTipTransform *= relativeTransform;
	  
	  tf::transformTFToMsg(relativeTransform, currentJointTf.transform);
      act->jointTf.push_back(currentJointTf);
      if(broadcastTF)
      	rosTfBroadcaster.sendTransform(currentJointTf);
  }
  //add transformation for arm tip
  if(!act->jointTf.empty())
  {
  	currentJointTf.header.frame_id = act->jointTf.back().child_frame_id;
  }else
  {
    currentJointTf.header.frame_id = act->name + "_link0";
  }
  currentJointTf.child_frame_id = act->name + "_tip";
  //find the position of the next link tip in the global coordinate frame
  tf::Vector3 tipOffset(sw->data.actuator.tip.x,sw->data.actuator.tip.y,sw->data.actuator.tip.z);
  currentTipPosition += tipOffset;
  
  //tip z-axis points along the tip offset, so get the rotation between actuator z-axis and tip offset
  if(tipOffset.length2() == 0)
  	quat = tf::Quaternion(0, 0, 0, 1);
  else
  {	
	  tf::Vector3 tipZAxis = tipOffset.normalized();
	  tf::Vector3 rotationAxis = tf::Vector3(-1* tipZAxis.getY(), tipZAxis.getX(), 0.0); //rotation axis is k cross tipOffset
	  if(rotationAxis.length2() == 0)
	  {
	  	rotationAxis = tf::Vector3(0, 1, 0);
	  }
	  double angle = acos(tipZAxis.getZ());
	  quat = tf::Quaternion(rotationAxis, angle);
  }
  absoluteTransform.setRotation(quat);
  absoluteTransform.setOrigin(currentTipPosition);
  //find the transformation from the tip of the last link to the tip of the arm
  tf::Transform relativeTransform = lastTipTransform.inverseTimes(absoluteTransform);
  
  tf::transformTFToMsg(relativeTransform, currentJointTf.transform);
  
  //tip transformation has no joint or link, so always publish it
  rosTfBroadcaster.sendTransform (currentJointTf); 
  act->jointTf.push_back(currentJointTf);
   
  return 1;
}
int
ServoInf::copyGrdVehSettings (UsarsimGrdVeh * settings, const sw_struct * sw)
{
  settings->steerType = sw->data.groundvehicle.steertype;
  // the platform name needs to be set to "base_link"
  // and we will need to change any sensor mounted to the 
  // platform to point to the "base_link"
  settings->platformName = std::string (sw->name);
  //  settings->platformName = std::string ("base_link");
  settings->platformSize.x = sw->data.groundvehicle.length;
  settings->platformSize.y = sw->data.groundvehicle.width;
  settings->platformSize.z = sw->data.groundvehicle.height;
  settings->mass = sw->data.groundvehicle.mass;
  settings->cg.x = sw->data.groundvehicle.cg.x;
  settings->cg.y = sw->data.groundvehicle.cg.y;
  settings->cg.z = sw->data.groundvehicle.cg.z;
  settings->maxWheelRot = sw->data.groundvehicle.max_speed;
  settings->maxTorque = sw->data.groundvehicle.max_torque;
  settings->wheelSeparation = sw->data.groundvehicle.wheel_separation;
  settings->wheelRadius = sw->data.groundvehicle.wheel_radius;
  settings->wheelBase = sw->data.groundvehicle.wheel_base;
  settings->maxSteerAngle = sw->data.groundvehicle.max_steer_angle;
  settings->minTurningRadius = sw->data.groundvehicle.min_turning_radius;
  return 1;
}

int
ServoInf::copyStaticVehSettings (UsarsimPlatform * settings, const sw_struct * sw)
{
  // the platform name needs to be set to "base_link"
  // and we will need to change any sensor mounted to the 
  // platform to point to the "base_link"
  settings->platformName = std::string (sw->name);
  return 1;
}

/* returns 1 if we need to send out a sensor tf message
   returns 0 if no sensor tf message needed
*/
int
ServoInf::copyIns (UsarsimOdomSensor * sen, const sw_struct * sw)
{
  int retValue;
  ros::Time currentTime;
  tf::Quaternion quat;
  geometry_msgs::Quaternion quatMsg;
  std::string sen_frame_id, sen_child_id;
  currentTime = ros::Time::now ();


  setTransform(sen, sw->data.ins.mount, currentTime);
  quat = tf::createQuaternionFromRPY (sw->data.ins.mount.roll,
				      sw->data.ins.mount.pitch,
				      sw->data.ins.mount.yaw);
  tf::quaternionTFToMsg (quat, quatMsg);
  // set how sensor is mounted to vehicle
  retValue = 1;
  if( sen->name == odomName )
    {
      basePlatform->tf.transform.translation.x = sw->data.ins.mount.x;
      basePlatform->tf.transform.translation.y = sw->data.ins.mount.y;
      basePlatform->tf.transform.translation.z = sw->data.ins.mount.z;
      basePlatform->tf.transform.rotation = quatMsg;
      //      basePlatform->tf.header.frame_id = sen->name.c_str ();
      basePlatform->tf.header.frame_id = "base_footprint";
      basePlatform->tf.header.stamp = currentTime;
      ROS_DEBUG( "servoInf.cpp:: rosTime: %f sensorTime: %f",
		 currentTime.toSec(), sw->time );
      basePlatform->tf.child_frame_id = "base_link";
      //  basePlatform->tf.child_frame_id = basePlatform->platformName.c_str ();
      sen_child_id = std::string("base_footprint");
      sen_frame_id = std::string("odom");
    }
  else
    {
      sen_child_id = std::string("base_") + sen->name;
      sen_frame_id = sen->name;
      //sen_frame_id = std::string("odom");
      //sen_child_id = sen->name;
    }
  // now set up the sensor
  sen->tf.transform.translation.x = sw->data.ins.position.x;
  sen->tf.transform.translation.y = sw->data.ins.position.y;
  sen->tf.transform.translation.z = sw->data.ins.position.z;
  quat = tf::createQuaternionFromRPY (sw->data.ins.position.roll,
				      sw->data.ins.position.pitch,
				      sw->data.ins.position.yaw);
  tf::quaternionTFToMsg (quat, quatMsg);
  sen->tf.transform.rotation = quatMsg;
  sen->tf.child_frame_id = sen_child_id;
  sen->tf.header.frame_id = sen_frame_id;
  sen->tf.header.stamp = currentTime;

  // odom message
  sen->odom.header.stamp = currentTime;
  sen->odom.header.frame_id = sen->tf.header.frame_id;
  sen->odom.child_frame_id = sen->tf.child_frame_id;

  // set the position
  sen->odom.pose.pose.position.x = sw->data.ins.position.x;
  sen->odom.pose.pose.position.y = sw->data.ins.position.y;
  sen->odom.pose.pose.position.z = sw->data.ins.position.z;
  sen->odom.pose.pose.orientation = quatMsg;

  // set the velocity
  double dt = sw->time - sen->time;
  geometry_msgs::Vector3 currentAngular;
  currentAngular.x = sw->data.ins.mount.roll;
  currentAngular.y = sw->data.ins.mount.pitch;
  currentAngular.z = sw->data.ins.mount.yaw;
  sen->odom.twist.twist.linear.x =
    (sw->data.ins.position.x - sen->lastPosition.linear.x) / dt;
  sen->odom.twist.twist.linear.y =
    (sw->data.ins.position.y - sen->lastPosition.linear.y) / dt;
  sen->odom.twist.twist.linear.z =
    (sw->data.ins.position.z - sen->lastPosition.linear.z) / dt;
  sen->odom.twist.twist.angular.x =
    (currentAngular.x - sen->lastPosition.angular.x) / dt;
  sen->odom.twist.twist.angular.y =
    (currentAngular.y - sen->lastPosition.angular.y) / dt;
  sen->odom.twist.twist.angular.z =
    (currentAngular.z - sen->lastPosition.angular.z) / dt;
  /*
  ROS_ERROR( "Time: %f (%f %f) Linear: %f %f %f Angular: %f %f %f Current: %f %f %f",
	     dt, sw->time, sen->time,
	     sen->odom.twist.twist.linear.x,
	     sen->odom.twist.twist.linear.y,
	     sen->odom.twist.twist.linear.z,
	     sen->odom.twist.twist.angular.x,
	     sen->odom.twist.twist.angular.y,
	     sen->odom.twist.twist.angular.z,
	     currentAngular.x,
	     currentAngular.y,
	     currentAngular.z);
  */

  // set last position and time

  sen->lastPosition.linear.x = sen->odom.pose.pose.position.x;
  sen->lastPosition.linear.y = sen->odom.pose.pose.position.y;
  sen->lastPosition.linear.z = sen->odom.pose.pose.position.z;
  sen->lastPosition.angular = currentAngular;
  sen->time = sw->time;
  return retValue;
}

int
ServoInf::copyRangeScanner (UsarsimRngScnSensor * sen, const sw_struct * sw)
{
  int flipScanner = 1; // if set to 1, flip direction of range scanner pan
  ros::Time currentTime;
  tf::Quaternion quat;
  
  geometry_msgs::Quaternion quatMsg;
  currentTime = ros::Time::now ();
  
  setTransform(sen, sw->data.rangescanner.mount, currentTime);
  
  sen->scan.header.stamp = currentTime;
  //  sen->scan.header.frame_id = sen->tf.header.frame_id;
  sen->scan.header.frame_id = sen->name;
  sen->scan.angle_min =  -sw->data.rangescanner.fov / 2.;
  sen->scan.angle_max =  sw->data.rangescanner.fov / 2.;
  sen->scan.angle_increment = sw->data.rangescanner.resolution;
  sen->scan.time_increment = 0;	// (1 / laser_frequency) / (num_readings);
  sen->scan.range_min = sw->data.rangescanner.minrange;
  sen->scan.range_max = sw->data.rangescanner.maxrange;

  //  sen->scan.set_ranges_size((unsigned int)sw->data.rangescanner.number);
  //  sen->scan.set_intensities_size((unsigned int)0);
  sen->scan.ranges.clear();
  sen->scan.intensities.clear();
  if( flipScanner )
    {
      for (int i = sw->data.rangescanner.number-1; i>= 0; i--)
	{
	  sen->scan.ranges.push_back (sw->data.rangescanner.range[i]);
	}
    }
  else
    {
      for (int i = 0; i < sw->data.rangescanner.number; i++)
	{
	  sen->scan.ranges.push_back (sw->data.rangescanner.range[i]);
	}
    }
  return 1;
}
int ServoInf::copyObjectSensor (UsarsimObjectSensor *sen, const sw_struct *sw)
{
  ros::Time currentTime;
  tf::Quaternion quat;
  geometry_msgs::Quaternion quatMsg;
  currentTime = ros::Time::now ();
  setTransform(sen, sw->data.objectsensor.mount, currentTime);
  
  sen->objSense.header.stamp = currentTime;
  sen->objSense.header.frame_id = sen->name;
  sen->objSense.fov = sw->data.objectsensor.fov;
  sen->objSense.object_names.clear();
  sen->objSense.material_names.clear();
  sen->objSense.object_poses.clear();
  sen->objSense.object_hit_locations.clear();
  for(int i = 0;i<sw->data.objectsensor.number;i++)
  {
  	sen->objSense.object_names.push_back(std::string(sw->data.objectsensor.objects[i].tag));
  	sen->objSense.material_names.push_back(std::string(sw->data.objectsensor.objects[i].material_name));
  	geometry_msgs::Pose objectPose;
  	geometry_msgs::Pose objectHitLocation;
  	
  	objectPose.position.x = sw->data.objectsensor.objects[i].position.x;
  	objectPose.position.y = sw->data.objectsensor.objects[i].position.y;
  	objectPose.position.z = sw->data.objectsensor.objects[i].position.z;
  	
  	objectHitLocation.position.x = sw->data.objectsensor.objects[i].hit_location.x;
  	objectHitLocation.position.y = sw->data.objectsensor.objects[i].hit_location.y;
  	objectHitLocation.position.z = sw->data.objectsensor.objects[i].hit_location.z;
  	
  	quat = tf::createQuaternionFromRPY(sw->data.objectsensor.objects[i].position.roll,
  						sw->data.objectsensor.objects[i].position.pitch,
  						sw->data.objectsensor.objects[i].position.yaw);
  						
  	tf::quaternionTFToMsg(quat, quatMsg);
  	objectPose.orientation = quatMsg;
  	
  	sen->objSense.object_hit_locations.push_back(objectHitLocation);
  	sen->objSense.object_poses.push_back(objectPose);
  	
  }
  
  return 1;
}
int ServoInf::copyRangeImager(UsarsimRngImgSensor *sen, const sw_struct *sw)
{
	ros::Time currentTime = ros::Time::now();
	setTransform(sen, sw->data.rangeimager.mount, currentTime);
	sen->opticalTransform.header.stamp = currentTime;
	sen->depthImage.header.stamp = currentTime;
	sen->depthImage.header.frame_id = sen->name + "_optical";
	
	sen->totalFrames = sw->data.rangeimager.totalframes;
	sen->depthImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
	if(sen->totalFrames != 0)
	{
		//insert the current frame into a depth image
		sen->sentFrame(sw->data.rangeimager.frame);
		sen->depthImage.height = sw->data.rangeimager.resolutiony;
		sen->depthImage.width = sw->data.rangeimager.resolutionx;
		sen->depthImage.step = sizeof(float)*sw->data.rangeimager.resolutionx;
		sen->depthImage.data.reserve(sen->depthImage.step * sen->depthImage.height);
		sen->depthImage.data.insert(sen->depthImage.data.begin() + (sw->data.rangeimager.frame * sw->data.rangeimager.numberperframe * sizeof(float)), 
		    reinterpret_cast<const uint8_t*>(sw->data.rangeimager.range),
		 	reinterpret_cast<const uint8_t*>(sw->data.rangeimager.range + sw->data.rangeimager.numberperframe));
	}
	//camera calibration data from the Kinect. 
	//This will be scaled incorrectly if the camera's FOV is not the same as the Kinect's! (58x45 degrees)
	sen->camInfo.header.frame_id = "/"+sen->name+"_optical";
	sen->camInfo.height = sw->data.rangeimager.resolutiony;
    sen->camInfo.width = sw->data.rangeimager.resolutionx;
    float xScale = (float)sen->camInfo.width/640.0;
    float yScale = (float)sen->camInfo.height/480.0;
    sen->camInfo.distortion_model = "plumb_bob";
    double dArray[] = {0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000};
    double kArray[] = {585.05108211 * xScale, 0.00000000, 315.83800193 * xScale, 0.00000000, 585.05108211 * yScale, 242.94140713 * yScale, 0.00000000, 0.00000000, 1.00000000};
    double rArray[] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    double pArray[] = {585.05108211 * xScale, 0.00000000, 315.83800193 * xScale, 0.0, 0.00000000, 585.05108211 * yScale, 242.94140713 * yScale, 0.0, 0.00000000, 0.00000000, 1.00000000, 0.0};
    
    
    sen->camInfo.D.assign(dArray, dArray+5);
    std::copy(kArray, kArray+9, sen->camInfo.K.begin());
    std::copy(rArray, rArray+9, sen->camInfo.R.begin());
    std::copy(pArray, pArray+12, sen->camInfo.P.begin());
    sen->camInfo.binning_x = 0;
    sen->camInfo.binning_y = 0;
    sen->camInfo.roi.x_offset = 0;
    sen->camInfo.roi.y_offset = 0;
    sen->camInfo.roi.height = 0;
    sen->camInfo.roi.width = 0;
    sen->camInfo.roi.do_rectify = false;
	return 1;
}
int ServoInf::copyGripperEffector(UsarsimGripperEffector * effector, const sw_struct *sw)
{
	ros::Time currentTime = ros::Time::now();
	tf::Transform baseTransform;
	tf::Transform tipTransform;
	
	setTransform(effector, sw->data.gripper.mount, currentTime);

	//adjust transform for gripper by tip offset
	tipTransform.setRotation(tf::Quaternion(0,0,0,1));
	tipTransform.setOrigin(tf::Vector3(sw->data.gripper.tip.x, sw->data.gripper.tip.y, sw->data.gripper.tip.z));
	tf::transformMsgToTF(effector->tf.transform, baseTransform);
	tf::transformTFToMsg(baseTransform * tipTransform, effector->tf.transform);

	effector->status.header.stamp = currentTime;
	effector->status.header.frame_id = effector->name;
	
	if(sw->data.gripper.status == SW_EFF_OPEN)
		effector->status.state = usarsim_inf::EffectorStatus::OPEN;
	else if(sw->data.gripper.status == SW_EFF_CLOSE)
		effector->status.state = usarsim_inf::EffectorStatus::CLOSE;
	else
	{
		ROS_ERROR("Unrecognized gripper state: %d", sw->data.gripper.status);
		return -1;
	}
	return 1;
}
int ServoInf::copyToolchanger(UsarsimToolchanger * effector, const sw_struct *sw)
{
	ros::Time currentTime = ros::Time::now();
	effector->status.header.stamp = currentTime;
	effector->status.header.frame_id = effector->name;
	setTransform(effector, sw->data.toolchanger.mount, currentTime);
	
	switch(sw->data.toolchanger.tooltype)
	{
		case SW_EFF_TOOLCHANGER_GRIPPER:
			effector->status.tool_type.type = usarsim_inf::ToolType::GRIPPER;
			break;
		case SW_EFF_TOOLCHANGER_VACUUM:
			effector->status.tool_type.type = usarsim_inf::ToolType::VACUUM;
			break;
		case SW_EFF_TOOLCHANGER_TOOLCHANGER:
			effector->status.tool_type.type = usarsim_inf::ToolType::TOOLCHANGER;
			break;
		case SW_EFF_TOOLCHANGER_UNKNOWN_TYPE:
		default:
			effector->status.tool_type.type = usarsim_inf::ToolType::UNKNOWN;
			break;
	}
	if(sw->data.toolchanger.status == SW_EFF_OPEN)
		effector->status.effector_status.state = usarsim_inf::EffectorStatus::OPEN;
	else if(sw->data.toolchanger.status == SW_EFF_CLOSE)
		effector->status.effector_status.state = usarsim_inf::EffectorStatus::CLOSE;
	else
	{
		ROS_ERROR("Unrecognized toolchanger state: %d", sw->data.gripper.status);
		return -1;
	}
	return 1;
}
/*
Set up the tf transformation for a component with the given pose
*/
void ServoInf::setTransform(UsarsimSensor *sen, const sw_pose &pose, ros::Time currentTime)
{
  std::stringstream tempSS;
  tf::Quaternion quat;
  tf::StampedTransform parentTransform;
  tf::Transform absoluteTransform, relativeTransform;
  geometry_msgs::Quaternion quatMsg;
  quat = tf::createQuaternionFromRPY (pose.roll,
				      pose.pitch,
				      pose.yaw);
  tf::quaternionTFToMsg (quat, quatMsg);

  sen->tf.transform.translation.x = pose.x;
  sen->tf.transform.translation.y = pose.y;
  sen->tf.transform.translation.z = pose.z;
  sen->tf.transform.rotation = quatMsg;
  sen->tf.header.stamp = currentTime;
  sen->tf.child_frame_id = sen->name;
  //if the object is mounted on the robot, mount it on base_link
  if(!ulapi_strcasecmp(pose.offsetFrom, basePlatform->platformName.c_str()))
  {
  	sen->tf.header.frame_id = "base_link";
  }else if(!ulapi_strcasecmp (pose.offsetFrom, "HARD"))
  {
  	sen->tf.header.frame_id = "base_link";
  }
  else
  {
  	if(pose.linkOffset < 0) //if no link is specified, mount the object directly on to its parent frame
    	sen->tf.header.frame_id = pose.offsetFrom;
    else
    {
    	//mount the object on its parent link frame and create a joint to publish it.
    	tempSS.str("");
    	tempSS<<pose.offsetFrom;
    	tempSS<<"_link";
    	tempSS<<pose.linkOffset;
    	sen->tf.header.frame_id = tempSS.str();
    	addJoint(sen->name + "_mount", 0.0);
    }
	bool success = false;
	//get the transformation from the robot frame to this item's direct parent
	try
	{
		tfListener.lookupTransform("base_link", sen->tf.header.frame_id, ros::Time(0), parentTransform);
		success = true;
	}catch(tf::LookupException e)
	{
		ROS_DEBUG("%s: No transform for frame %s, skipping transform until one is available.", 
		sen->name.c_str(), sen->tf.header.frame_id.c_str());
	}
	catch(tf::ConnectivityException e)
	{
		ROS_DEBUG("%s: No transform for frame %s, skipping transform until one is available.", 
		sen->name.c_str(), sen->tf.header.frame_id.c_str());
	}
	catch(tf::ExtrapolationException e)
	{
		ROS_DEBUG("%s: No transform for frame %s, skipping transform until one is available.", 
		sen->name.c_str(), sen->tf.header.frame_id.c_str());
	}
	if(success)
	{
		//find the relative transformation from the link frame to the object frame
		absoluteTransform.setOrigin(parentTransform.getOrigin() + tf::Vector3(pose.x, pose.y, pose.z));
		absoluteTransform.setRotation(quat);
		
		relativeTransform = parentTransform.inverseTimes(absoluteTransform);	    
		tf::transformTFToMsg(relativeTransform, sen->tf.transform);
	}
  }
  sen->linkOffset = pose.linkOffset;  
}

/*
  Each general data array sensor (tachometer, odometer, etc.) uses one
  of the several SensorData structures.  These data and functions
  determine which index in the array of SensorData structures is
  assigned to which name.
*/
/*
Returns a pointer to the actuator with the given name. If the actuator with this name 
does not exist, create and return it.
This behavior is DISTINCT from that of the other *Index functions.
*/
UsarsimActuator* 
ServoInf::actuatorIn (std::list < UsarsimActuator > &actuatorsIn,
			   std::string name)
{
  std::list<UsarsimActuator>::iterator it;
  std::string pubName;
  UsarsimActuator newActuator(this);
  UsarsimActuator *actPtr;
  
  for(it = actuatorsIn.begin();it != actuatorsIn.end();it++)
  {
  	if(it->name == name)
  		return (UsarsimActuator*)(&*it);
  }
  
  //unable to find the actuator, so must create it.
  actuatorsIn.push_back (newActuator);
  actPtr = &actuatorsIn.back();
  actPtr->name = name;
  actPtr->time = 0;
  actPtr->setUpTrajectory();
  return actPtr;
}

int
ServoInf::odomSensorIndex (std::vector < UsarsimOdomSensor > &sensors,
			   std::string name)
{
  unsigned int t;
  UsarsimOdomSensor newSensor;
  std::string pubName;

  for (t = 0; t < sensors.size (); t++)
    {
      if (name == sensors[t].name)
	return t;		// found it
    }

  ROS_INFO ("Adding sensor: %s", name.c_str ());
  if( odomName == std::string(""))
    odomName = name;

  //unable to find the sensor, so must create it.
  newSensor.name = name;

  newSensor.time = 0;

  if( name == odomName )
    pubName = "odom";
  else
    pubName = newSensor.name;

  newSensor.pub = nh->advertise < nav_msgs::Odometry > (pubName.c_str (), 2);
  newSensor.tf.header.frame_id = "base_link";
  newSensor.tf.child_frame_id = newSensor.name.c_str ();


  sensors.push_back (newSensor);
  return sensors.size () - 1;
}

int
ServoInf::rangeSensorIndex (std::vector < UsarsimRngScnSensor > &sensors,
			    std::string name)
{
  unsigned int t;
  UsarsimRngScnSensor newSensor;

  for (t = 0; t < sensors.size (); t++)
    {
      if (name == sensors[t].name)
	return t;		// found it
    }

  ROS_INFO ("Adding sensor: %s", name.c_str ());

  //unable to find the sensor, so must create it.
  newSensor.name = name;
  newSensor.time = 0;
  newSensor.pub = nh->advertise < sensor_msgs::LaserScan > (name.c_str (), 2);
  newSensor.tf.header.frame_id = "base_link";
  newSensor.tf.child_frame_id = name.c_str ();

  sensors.push_back (newSensor);
  return sensors.size () - 1;
}
int ServoInf::objectSensorIndex (std::vector < UsarsimObjectSensor > &sensors, 
  	std::string name)
{
	unsigned int t;
	UsarsimObjectSensor newSensor;
	for (t = 0; t < sensors.size (); t++)
    {
      if (name == sensors[t].name)
	return t;		// found it
    }
    ROS_INFO ("Adding sensor: %s", name.c_str ());

  //unable to find the sensor, so must create it.
  newSensor.name = name;
  newSensor.time = 0;
  newSensor.pub = nh->advertise < usarsim_inf::SenseObject > (name.c_str (), 2);
  newSensor.tf.header.frame_id = "base_link";
  newSensor.tf.child_frame_id = name.c_str ();

  sensors.push_back (newSensor);
  return sensors.size () - 1;
}
int ServoInf::rangeImagerIndex(std::vector < UsarsimRngImgSensor> &sensors, std::string name)
{
	unsigned int t;
	for (t = 0; t < sensors.size (); t++)
    {
      if (name == sensors[t].name)
	return t;		// found it
    }
    ROS_INFO("Adding sensor: %s",name.c_str());
    UsarsimRngImgSensor newSensor(this);
    sensors.push_back(newSensor);
    UsarsimRngImgSensor *sensePtr = &(sensors.back());
    sensePtr->name = name;
    sensePtr->time = 0;
    sensePtr->pub = nh->advertise <sensor_msgs::Image > ("image_mono", 2);
    ROS_INFO("subscribing to topic %s",(sensePtr->name+"/command").c_str());
    sensePtr->command = nh->subscribe(sensePtr->name+"/command", 10, &UsarsimRngImgSensor::commandCallback, sensePtr);
    sensePtr->cameraInfoPub = nh->advertise<sensor_msgs::CameraInfo >("camera_info",2);
    sensePtr->tf.header.frame_id = "base_link";
    sensePtr->tf.child_frame_id = ("/"+name).c_str ();
    sensePtr->opticalTransform.header.frame_id = "/"+name;
    sensePtr->opticalTransform.child_frame_id = "/"+name+"_optical";
    //create a transformation from the camera frame to the optical frame (image coordinates)
    tf::Quaternion quat;
    quat.setEuler(1.5707, 0, 1.5707);//yaw, pitch, roll 
    tf::quaternionTFToMsg(quat, sensePtr->opticalTransform.transform.rotation);
    return sensors.size() - 1;
}

int ServoInf::gripperEffectorIndex(std::vector < UsarsimGripperEffector> &effectors, std::string name)
{
	unsigned int t;
	
	for (t = 0; t < effectors.size (); t++)
    {
      if (name == effectors[t].name)
	return t;		// found it
    }
    ROS_INFO ("Adding effector: %s", name.c_str ());
  UsarsimGripperEffector newEffector(this);
  effectors.push_back (newEffector);
  UsarsimGripperEffector *effectPtr = &(effectors.back());
  //unable to find the effector, so must create it.
  effectPtr->name = name;
  effectPtr->time = 0;
  effectPtr->pub = nh->advertise < usarsim_inf::EffectorStatus > (name + "/status", 2);
  effectPtr->command = nh->subscribe(name+"/command",10,&UsarsimGripperEffector::commandCallback, effectPtr);
  effectPtr->tf.header.frame_id = "base_link"; // Mount this on the base_link until we get a geo message
  effectPtr->tf.child_frame_id = name.c_str ();

  return effectors.size () - 1;
}
int ServoInf::toolchangerIndex(std::vector < UsarsimToolchanger> &effectors, std::string name)
{
	unsigned int t;
	
	for (t = 0; t < effectors.size (); t++)
    {
      if (name == effectors[t].name)
	return t;		// found it
    }
    ROS_INFO ("Adding effector: %s", name.c_str ());
  UsarsimToolchanger newEffector(this);
  effectors.push_back (newEffector);
  UsarsimToolchanger *effectPtr = &(effectors.back());
  //unable to find the effector, so must create it.
  effectPtr->name = name;
  effectPtr->time = 0;
  effectPtr->pub = nh->advertise < usarsim_inf::ToolchangerStatus > (name + "/status", 2);
  effectPtr->command = nh->subscribe(name+"/command",10,&UsarsimToolchanger::commandCallback, effectPtr);
  effectPtr->tf.header.frame_id = "base_link"; // Mount this on the base_link until we get a geo message
  effectPtr->tf.child_frame_id = name.c_str ();

  return effectors.size () - 1;
}
/*
Update the cycle time estimate for this actuator (used to time joint trajectory messages)
*/
void ServoInf::updateActuatorCycle(UsarsimActuator *act)
{
  ros::Time currentTime = ros::Time::now();
  double currentCycle;
  int dequeLength = act->cycleTimer.cycleDeque.size();
  
	// update cycle time with ctNow = ctOld - p0/n + pn/n
  currentCycle = ros::Duration(currentTime - act->cycleTimer.lastTime).toSec();
  act->cycleTimer.cycleTime = act->cycleTimer.cycleTime - act->cycleTimer.cycleDeque.front()/dequeLength + currentCycle/dequeLength;
  act->cycleTimer.cycleDeque.push_back(currentCycle);
  act->cycleTimer.cycleDeque.pop_front();
  act->cycleTimer.lastTime = currentTime;
}

/*
returns true if the trajectory has been completed, false 
if it is still in progress
*/
bool ServoInf::updateTrajectory(UsarsimActuator *act, const sw_struct *sw)
{
	TrajectoryPoint currentPoint;
  ros::Time currentTime = ros::Time::now();
  
  if( act->currentTrajectory.goals.size() == 0 ) // check if done
      return true;
  currentPoint = act->currentTrajectory.goals.front();
  // find next point to execute by looking at where we are likely to be the next time that this routine is called
  currentTime += ros::Duration(act->cycleTimer.cycleTime);
  while( currentPoint.time < currentTime )
    {
      act->currentTrajectory.goals.pop_front();
      if( act->currentTrajectory.goals.size() == 0 )
	  	break;
      currentPoint = act->currentTrajectory.goals.front();
    }
    
    sw_struct newSw;
    newSw.type = SW_ROS_CMD_TRAJ;
    newSw.name = sw->name;
    newSw.data.roscmdtraj.number = currentPoint.numJoints;
    for(unsigned int i = 0; i<currentPoint.numJoints; i++)
    {
	  // send next goal point to the robotic arm
	  newSw.data.roscmdtraj.goal[i] = currentPoint.jointGoals[i];
    }
    sibling->peerMsg(&newSw);
    
    return false;
  
}
/*
returns true if the actuator is within the bounds of its goal
*/
bool ServoInf::checkTrajectoryGoal(UsarsimActuator *act, const sw_struct *sw)
{
	for(int i = 0;i<sw->data.actuator.number;i++)
	  {
	  if(abs(sw->data.actuator.link[i].position - act->currentTrajectory.finalGoal.jointGoals[i]) > 
	     act->currentTrajectory.finalGoal.tolerances[i])
	     {
	    	return false;
	     }
	  }
	  return true;
}
/*
Add a joint to the joints array if it hasn't already been added
*/
void ServoInf::addJoint(std::string jointName, double jointValue)
{
	for(unsigned int i = 0;i<joints.name.size();i++)
	{
		if(joints.name[i] == jointName)
		{
			joints.position[i] = jointValue;
			return;
		}
	}
	joints.name.push_back(jointName);
	joints.position.push_back(jointValue);
}
/*
Publish all of the joint angles
*/
void ServoInf::publishJoints()
{
	ros::Time currentTime = ros::Time::now();
	joints.header.frame_id = "base_link";
	joints.header.stamp = currentTime;
	jointPublisher.publish(joints);
}
void *
  ServoInf::servoSetMutex = NULL;
