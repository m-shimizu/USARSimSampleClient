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
  \file   usarsimInf.cpp
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
#include "usarsimInf.hh"
#include <XmlRpcValue.h>

UsarsimInf::UsarsimInf ():GenericInf ()
{
  socket_fd = -1;
  socket_mutex = NULL;
  buildlen = BUFFERLEN;
  build = NULL;
  waitingForConf = 0;
  waitingForGeo = 0;
}

int
UsarsimInf::init (GenericInf * siblingIn)
{
  std::string hostname, startPosition, robotName, robotType;
  std::stringstream tempSS;
  int port;

  GenericInf::init (siblingIn);
  /* get all of the parameters for starting usarsim we need:
     startPosition
     robotType
     hostname
     robotName
     port
   */
  if (!nh->getParam ("/usarsim/startPosition", startPosition))
    {
      ROS_ERROR ("Must provide robot start position");
      return -1;
    }
  ROS_DEBUG ("Parameter /usarsim/startPosition: %s", startPosition.c_str ());

  nh->param < std::string > ("/usarsim/robotType", robotType, "P3AT");
  ROS_DEBUG ("Parameter /usarsim/hostname: %s", robotType.c_str ());

  ros::Time myTime = ros::Time::now ();
  tempSS << myTime.sec;
  robotName = "ROS" + tempSS.str ();
  nh->param < std::string > ("/usarsim/robotName", robotName,
			     robotName.c_str ());
  ROS_DEBUG ("Parameter /usarsim/robotName: %s", robotName.c_str ());

  nh->param < std::string > ("/usarsim/hostname", hostname, "localhost");
  ROS_DEBUG ("Parameter /usarsim/hostname: %s", hostname.c_str ());

  nh->param < int >("/usarsim/port", port, 3000);
  ROS_DEBUG ("parameter /usarsim/port: %d", port);

  socket_fd = ulapi_socket_get_client_id (port, hostname.c_str ());
  if (socket_fd < 0)
    {
      ROS_ERROR ("can't open socket to %s port %d", hostname.c_str (), port);
      return -1;
    }

  socket_mutex = ulapi_mutex_new (SOCKET_MUTEX_KEY);
  if (NULL == socket_mutex)
    {
      ulapi_socket_close (socket_fd);
      socket_fd = -1;
      return -1;
    }

  ulapi_snprintf (str, sizeof (str),
		  "GETSTARTPOSES\r\nINIT {Classname USARBot.%s} {Name %s} {Start %s}\r\n",
		  robotType.c_str (), robotName.c_str (),
		  startPosition.c_str ());
  /*
     else
     {
     ulapi_snprintf (str, sizeof (str),
     "GETSTARTPOSES\r\nINIT {Classname USARBot.%s} {Name %s} {Location %f,%f,%f} {Rotation %f,%f,%f}\r\n",
     type, name, x, y, z, roll, pitch, yaw);
     }
   */
  NULLTERM (str);
  ulapi_mutex_take (socket_mutex);
  usarsim_socket_write (socket_fd, str, strlen (str));
  ulapi_mutex_give (socket_mutex);

  /*
     Note:

     [Some of the USARsim resources don't announce themselves.  Sensors
     do, but effectors don't. For the ones that don't, like effectors,
     we need to force a request for geo and conf information. It may
     come back empty.] <--  THIS IS NO LONGER TRUE!
   */
  /*ulapi_snprintf (str, sizeof (str), "GETCONF {Type Actuator}\r\n");
  NULLTERM (str);
  ulapi_mutex_take (socket_mutex);
  usarsim_socket_write (socket_fd, str, strlen (str));
  ulapi_mutex_give (socket_mutex);

  ulapi_snprintf (str, sizeof (str), "GETGEO {Type Actuator}\r\n");
  NULLTERM (str);
  ulapi_mutex_take (socket_mutex);
  usarsim_socket_write (socket_fd, str, strlen (str));
  ulapi_mutex_give (socket_mutex);*/

  /*
  ulapi_snprintf (str, sizeof (str), "GETCONF {Type MisPkg}\r\n");
  NULLTERM (str);
  ulapi_mutex_take (socket_mutex);
  usarsim_socket_write (socket_fd, str, strlen (str));
  ulapi_mutex_give (socket_mutex);

  ulapi_snprintf (str, sizeof (str), "GETGEO {Type MisPkg}\r\n");
  NULLTERM (str);
  ulapi_mutex_take (socket_mutex);
  usarsim_socket_write (socket_fd, str, strlen (str));
  ulapi_mutex_give (socket_mutex);
  */

  //  printf( "usarsiminf: Getting gripper(2) conf\n");
  /*ulapi_snprintf (str, sizeof (str), "GETCONF {Type Gripper}\r\n");
  NULLTERM (str);
  ulapi_mutex_take (socket_mutex);
  usarsim_socket_write (socket_fd, str, strlen (str));
  ulapi_mutex_give (socket_mutex);

  ulapi_snprintf (str, sizeof (str), "GETGEO {Type Gripper}\r\n");
  NULLTERM (str);
  ulapi_mutex_take (socket_mutex);
  usarsim_socket_write (socket_fd, str, strlen (str));
  ulapi_mutex_give (socket_mutex);*/

  //  printf( "usarsiminf: Getting rfid conf\n");
  ulapi_snprintf (str, sizeof (str), "GETCONF {Type RFID}\r\n");
  NULLTERM (str);
  ulapi_mutex_take (socket_mutex);
  usarsim_socket_write (socket_fd, str, strlen (str));
  ulapi_mutex_give (socket_mutex);

  ulapi_snprintf (str, sizeof (str), "GETGEO {Type RFID}\r\n");
  NULLTERM (str);
  ulapi_mutex_take (socket_mutex);
  usarsim_socket_write (socket_fd, str, strlen (str));
  ulapi_mutex_give (socket_mutex);

  build = (char *) realloc (build, buildlen * sizeof (char));
  build_ptr = build;
  build_end = build + buildlen;

  encoders = new UsarsimList (SW_SEN_ENCODER);
  sonars = new UsarsimList (SW_SEN_SONAR);
  rangescanners = new UsarsimList (SW_SEN_RANGESCANNER);
  rangeimagers = new UsarsimList (SW_SEN_RANGEIMAGER);
  touches = new UsarsimList (SW_SEN_TOUCH);
  co2sensors = new UsarsimList (SW_SEN_CO2);
  groundtruths = new UsarsimList (SW_SEN_INS);
  inses = new UsarsimList (SW_SEN_INS);
  gpses = new UsarsimList (SW_SEN_GPS);
  odometers = new UsarsimList (SW_SEN_ODOMETER);
  victims = new UsarsimList (SW_SEN_VICTIM);
  tachometers = new UsarsimList (SW_SEN_TACHOMETER);
  acoustics = new UsarsimList (SW_SEN_ACOUSTIC);
  objectsensors = new UsarsimList (SW_SEN_OBJECTSENSOR);
  

  misstas = new UsarsimList (SW_ACT);

  grippers = new UsarsimList (SW_EFF_GRIPPER);
  toolchangers = new UsarsimList(SW_EFF_TOOLCHANGER);

  /*
     Note: the name of a robot appears to be ignored, and USARSim will
     set the name to be the type. It's OK to pass the name in geo and
     conf requests; since there can be only one robot, the name is
     ignored. Use SW_TYPE_UNITIALIZED since  I
     don't know or care what type, since this will be set explicitly in
     the handleSta_xxx functions
   */
  robot = new UsarsimList (SW_TYPE_UNINITIALIZED);
  robot->setName (robotName.c_str ());

  ROS_INFO ("usarsim interface initialized");
  //sleep (20);
  return 1;
}

int
UsarsimInf::msgout (sw_struct * sw, componentInfo info)
{
  if (sw->name == "")
    {
      ROS_ERROR( "Peer message must contain a name" );
      return -1;
    }
  else
    {
      sw->time = info.time;
      //      ROS_ERROR( "time: %f swtime: %f", info.time, sw->time );
      sw->op = info.op;
      sibling->peerMsg (sw);
    }
  return 1;
}

int
UsarsimInf::peerMsg (sw_struct * swIn)
{
  char str[MAX_MSG_LEN];
  sw_struct *sw;
  double turnRadius;
  double leftVel, rightVel;
  double steerAngle, vehVel;
  double scale;
  std::string command;
  std::stringstream tempSS;
  /*
     char cmp[MAX_MSG_LEN];
     int t;
   */

  //  ROS_INFO ("In usarsimInf peerMsg");

  switch (swIn->type)
    {
    case SW_ROS_CMD_VEL:
      sw = robot->getSW ();
      if (sw->type == SW_ROBOT_GROUNDVEHICLE)
	{
	  /* equations of motion:
	     SL = rTh
	     SR = (r + b)Th
	     SM = (r +b/2)Th
	   */
	  if (sw->data.groundvehicle.steertype == SW_STEER_SKID)
	    {
	      if (swIn->data.roscmdvel.lineary != 0
		  || swIn->data.roscmdvel.linearz != 0)
		{
		  ROS_WARN ("Invalid skid steering message <%f %f %f>",
			    swIn->data.roscmdvel.linearx,
			    swIn->data.roscmdvel.lineary,
			    swIn->data.roscmdvel.linearz);
		}
	      if (swIn->data.roscmdvel.angularz != 0)
		{
		  turnRadius =
		    swIn->data.roscmdvel.linearx /
		    swIn->data.roscmdvel.angularz -
		    sw->data.groundvehicle.wheel_separation / 2.;
		  leftVel = turnRadius * swIn->data.roscmdvel.angularz;
		  rightVel = (turnRadius +
			      sw->data.groundvehicle.wheel_separation) *
		    swIn->data.roscmdvel.angularz;
		}
	      else
		{
		  leftVel = swIn->data.roscmdvel.linearx;
		  rightVel = swIn->data.roscmdvel.linearx;
		}
	      leftVel /= sw->data.groundvehicle.wheel_radius;
	      rightVel /= sw->data.groundvehicle.wheel_radius;
	      if (leftVel > sw->data.groundvehicle.max_speed)
		{
		  scale = sw->data.groundvehicle.max_speed / leftVel;
		  ROS_WARN ("Left wheel spin speed too high! Scaling by %f%%",	// note that %% prints %
			    100 * scale);
		  leftVel = sw->data.groundvehicle.max_speed;
		  rightVel *= scale;
		}
	      if (rightVel > sw->data.groundvehicle.max_speed)
		{
		  scale = sw->data.groundvehicle.max_speed / rightVel;
		  ROS_WARN
		    ("Right wheel spin speed too high! Scaling by %f%%",
		     100. * scale);
		  rightVel = sw->data.groundvehicle.max_speed;
		  leftVel *= scale;
		}
	      /*
	      ROS_INFO
		("Sent skid steering with sep: %f radius: %f and vel <%f %f>",
		 sw->data.groundvehicle.wheel_separation,
		 sw->data.groundvehicle.wheel_radius, leftVel, rightVel);
	      */
	      ulapi_snprintf (str, sizeof (str),
			      "Drive {Left %f} {Right %f}\r\n", leftVel,
			      rightVel);
	      NULLTERM (str);
	      ulapi_mutex_take (socket_mutex);
	      usarsim_socket_write (socket_fd, str, strlen (str));
	      ulapi_mutex_give (socket_mutex);
	    }
	  else if (sw->data.groundvehicle.steertype == SW_STEER_ACKERMAN)
	    {
	      if (swIn->data.roscmdvel.lineary != 0
		  || swIn->data.roscmdvel.linearz != 0
		  || swIn->data.roscmdvel.angularx != 0
		  || swIn->data.roscmdvel.angulary != 0)
		{
		  ROS_WARN ("Invalid skid steering message <%f %f %f> <%f %f %f>",
			    swIn->data.roscmdvel.linearx,
			    swIn->data.roscmdvel.lineary,
			    swIn->data.roscmdvel.linearz,
			    swIn->data.roscmdvel.angularx,
			    swIn->data.roscmdvel.angulary,
			    swIn->data.roscmdvel.angularz);
		}
	      if( swIn->data.roscmdvel.linearx == 0 )
		{
		  steerAngle = swIn->data.roscmdvel.angularz;
		  vehVel = 0.;
		}
	      else
		{
		  steerAngle = atan2(swIn->data.roscmdvel.angularz *
				     sw->data.groundvehicle.wheel_base,
				     swIn->data.roscmdvel.linearx);
		  vehVel = swIn->data.roscmdvel.linearx/cos(steerAngle);
		}
	      // fixeme! How do I know if it is front or rear steer?
	      ulapi_snprintf (str, sizeof (str),
			      "Drive {Speed %f} {FrontSteer %f} {RearSteer %f}\r\n", 
			      vehVel, steerAngle, steerAngle );
	      NULLTERM (str);
	      ulapi_mutex_take (socket_mutex);
	      usarsim_socket_write (socket_fd, str, strlen (str));
	      ulapi_mutex_give (socket_mutex);
	      ROS_ERROR ("Wrote %s", str );
	    }
	  else
	    {
	      ROS_ERROR ("Currently only support skid steered and Ackerman steeredrobots");
	      break;
	    }
	}
      else
	{
	  ROS_ERROR ("Currently only support ground robot");
	  break;
	}

      /*
         case SW_ROBOT_ACKERMAN_MOVE:
         // FIXME -- how do we know if the vehicle is front steered or rear steered? 
         ulapi_snprintf (str, sizeof (str),
         "Drive {Speed %f} {FrontSteer %f} {RearSteer %f} {Normalized False} {Light False} {Flip False}\r\n",
         sw->data.groundvehicle.speed,
         sw->data.groundvehicle.heading,
         -sw->data.groundvehicle.heading);
         NULLTERM (str);
         ulapi_mutex_take (socket_mutex);
         usarsim_socket_write (socket_fd, str, strlen (str));
         ulapi_mutex_give (socket_mutex);
         break;
       */
      break;
    case SW_ROS_CMD_TRAJ:
	command = "ACT {Name " + swIn->name + "}";
	for(int i = 0;i<swIn->data.roscmdtraj.number;i++)
	{
	  tempSS.str("");
	  tempSS<<i;
	  command += " {Link " + tempSS.str() + "}";
	  tempSS.str("");
	  tempSS << swIn->data.roscmdtraj.goal[i];
	  command += " {Value "+tempSS.str() + "}";
	}
        ulapi_snprintf(str, sizeof(str), "%s\r\n",command.c_str());
        NULLTERM (str);
        ulapi_mutex_take (socket_mutex);
	usarsim_socket_write (socket_fd, str, strlen (str));
	ulapi_mutex_give (socket_mutex);
        break;
      case SW_ROS_CMD_GRIP:
        if(swIn->data.roscmdeff.goal == SW_EFF_OPEN)
        	command = "OPEN";
        else
        	command = "CLOSE";
      	ulapi_snprintf(str, sizeof(str), "SET {Type Gripper} {Name %s} {Opcode %s}\r\n", 
      	swIn->name.c_str(), command.c_str());
      	NULLTERM(str);
      	usarsim_socket_write (socket_fd, str, strlen (str));
		ulapi_mutex_give (socket_mutex);
      break;
      case SW_ROS_CMD_TOOLCHANGE:
        if(swIn->data.roscmdeff.goal == SW_EFF_OPEN)
        	command = "OPEN";
        else
        	command = "CLOSE";
      	ulapi_snprintf(str, sizeof(str), "SET {Type ToolChanger} {Name %s} {Opcode %s}\r\n", 
      	swIn->name.c_str(), command.c_str());
      	NULLTERM(str);
      	usarsim_socket_write (socket_fd, str, strlen (str));
		ulapi_mutex_give (socket_mutex);
      break; 
      case SW_ROS_CMD_SCAN:
      ulapi_snprintf(str, sizeof(str), "SET {Type RangeImager} {Name %s} {Opcode SCAN}\r\n",
      swIn->name.c_str());
      NULLTERM(str);
      usarsim_socket_write(socket_fd, str, strlen (str));
		ulapi_mutex_give (socket_mutex);
      break;
    default:
      ROS_ERROR ("usarsimInf::peerMsg: not handling type %s",
		 swTypeToString (swIn->type));
      break;
      

    }

  return 1;
}

char *
UsarsimInf::getKey (char *msg, char *key)
{
  char *ptr = msg;

  while ((*ptr != 0) && (*ptr != '{'))
    ptr++;			/* find the {  */
  if (*ptr == 0)
    return msg;			/* didn't find the {  */
  ptr++;			/* skip over the {  */
  while (isspace (*ptr))
    ptr++;			/* skip over any space  */

  /* copy everything until we hit space */
  while (1)
    {
      if ((*ptr == 0) || (*ptr == ',') || (*ptr == '{') || (*ptr == '}'))
	{
	  /* unexpected delimiter -- return the original pointer */
	  return msg;
	}
      if (isspace (*ptr))
	{
	  /* expected delimiter -- break to return current pointer */
	  break;
	}
      /* else copy it */
      *key++ = *ptr++;
    }
  *key = 0;			/* null terminate  */

  return ptr;
}

char *
UsarsimInf::getValue (char *msg, char *token)
{
  char *ptr = msg;
  char *tokenPtr = token;

  while ((isspace (*ptr)) || (*ptr == ',') || (*ptr == '}') || (*ptr == '{'))
    ptr++;			/* skip over delimiters  */

  /* copy everything until we hit a delimiter  */
  while (!isspace (*ptr))
    {
      if (*ptr == 0)
	return msg;		/* didn't finish cleanly */
      if ((*ptr == ',') || (*ptr == '}'))
	{
	  break;
	}
      /* ok to copy  */
      *tokenPtr++ = *ptr++;
    }
  *tokenPtr = 0;		/* null terminate  */
  // printf( "get_value: value: %s ptr: %s\n", value, ptr );
  return ptr;
}

int
UsarsimInf::expect (componentInfo * info, const char *token)
{
  info->nextptr = getValue (info->ptr, info->token);
  if (info->nextptr == info->ptr)
    {
      ROS_ERROR ("EXPECT: nextptr = ptr for token %s and pointer %s",
		 info->token, info->ptr);
      return -1;
    }
  if (strncmp (info->token, token, strlen (token)) != 0)
    {
      ROS_DEBUG ("info->token(%d):%s token(%d):%s result:%d",
		 (int) strlen (info->token), info->token,
		 (int) strlen (token), token, strcmp (info->token, token));
      ROS_ERROR ("EXPECT: found %s wanted %s", info->token, token);
      return -1;
    }
  info->ptr = info->nextptr;
  return 1;
}

int
UsarsimInf::getInteger (componentInfo * info)
{
  int i;

  info->nextptr = getValue (info->ptr, info->token);
  if (info->nextptr == info->ptr)
    {
      ROS_ERROR ("Unable to find integer");
      return -1;
    }
  if (sscanf (info->token, "%i", &i) != 1)
    {
      ROS_ERROR ("Unable to format integer");
      return -1;
    }
  info->count++;
  info->ptr = info->nextptr;
  return i;
}

double
UsarsimInf::getReal (componentInfo * info)
{
  double d;

  info->nextptr = getValue (info->ptr, info->token);
  if (info->nextptr == info->ptr)
    {
      ROS_ERROR ("GetReal unable to find real");
      return -1;
    }
  if (sscanf (info->token, "%lf", &d) != 1)
    {
      ROS_ERROR ("Unable to format double for GetReal");
      return -1;
    }
  info->count++;
  info->ptr = info->nextptr;
  return d;
}

void
UsarsimInf::getTime (componentInfo * info)
{
  info->nextptr = getValue (info->ptr, info->token);
  if (info->nextptr == info->ptr)
    {
      ROS_ERROR ("GetTime unable to find time");
      return;
    }
  if (sscanf (info->token, "%lf", &info->time) != 1)
    {
      ROS_ERROR ("Unable to format double for gettime");
      return;
    }
  info->ptr = info->nextptr;
}

int
UsarsimInf::getName (UsarsimList * list, componentInfo * info, int op)
{
  /* why?
  if (info->sawname)
    {
      info->op = op;
      msgout (list->getSW (), *info);
    }
  */
  info->sawname = 1;
  info->nextptr = getValue (info->ptr, info->token);
  if (info->nextptr == info->ptr)
    return -1;
  info->where = list->classFind (info->token);
  if (info->where == NULL)
    {
      ROS_ERROR ("error from getName");
      return -1;
    }
  info->ptr = info->nextptr;
  return 1;
}

/* wraps socket writing with a verbose printf, for debugging */
ulapi_integer
  UsarsimInf::usarsim_socket_write (ulapi_integer id, char *buf,
				    ulapi_integer len)
{
  ROS_DEBUG ("Sending: %s", buf);
  return ulapi_socket_write (id, buf, len);
}

int
UsarsimInf::msgIn ()
{
  char buffer[BUFFERLEN];
  char *buffer_ptr;
  char *buffer_end;
  ptrdiff_t offset;
  int nchars;
  int err;

  nchars = ulapi_socket_read (socket_fd, buffer, BUFFERLEN);
  if (nchars == -1)
    {				/* bad read */
      return -1;
    }
  if (nchars == 0)
    {				/* end of file */
      return -1;
    }
  buffer_ptr = buffer;
  buffer_end = buffer + nchars;

  while (buffer_ptr != buffer_end)
    {
      if (build_ptr == build_end)
	{
	  offset = build_ptr - build;
	  buildlen *= 2;
	  build = (char *) realloc (build, buildlen * sizeof (char));
	  build_ptr = build + offset;
	  build_end = build + buildlen;
	}
      *build_ptr++ = *buffer_ptr;
      if (*buffer_ptr++ == DELIMITER)
	{
	  offset = build_ptr - build;
	  build_ptr = build;
	  build[offset] = 0;
	  if ((err = handleMsg (build)) < 0)
	    {
	      ROS_ERROR ("msgIn: error(%d) handling %s", err, build);
	    }
	}
    }
  return 1;
}

/*!
  \return Returns -1 on error, otherwise returns a number indicating how
  many elements were handled.
*/
int
UsarsimInf::handleMsg (char *msg)
{
  char head[MAX_TOKEN_LEN];
  char *ptr = msg;
  unsigned int headindex = 0;
  int count;

  //ROS_ERROR ("msg: %s", msg);

  while (isspace (*ptr))
    ptr++;			/* skip over space  */
  if (*ptr == 0)
    return 0;			/* blank message -- ignore */

  /* copy everything until we hit space or {  */
  for (headindex = 0; headindex < sizeof (head); headindex++)
    {
      if ((*ptr == 0) || (*ptr == ',') || (*ptr == '}'))
	{
	  /* unexpected delimiter -- error in the head */
	  ROS_ERROR ("no head on ``%s''", msg);
	  return -1;
	}
      if (isspace (*ptr) || (*ptr == '{'))
	{
	  /* here's the delimiter -- term the string and we're done */
	  head[headindex] = 0;
	  break;
	}
      /* else copy it */
      head[headindex] = *ptr++;
    }
  //  ROS_DEBUG( "usarsimInf.cpp::handleMsg: socket message received: %s", msg );
  if (!strcmp (head, "SEN"))
    {
      count = handleSen (msg);
    }
  else if (!strcmp (head, "NFO"))
    {
      count = handleNfo (msg);
    }
  
     else if (!strcmp (head, "EFF"))
     {
     count = handleEff (msg);
     }
   
  else if (!strcmp (head, "STA") )
    {
      count = handleSta (msg);
    }
  else if (!strcmp (head, "MISSTA") || !strcmp (head, "ASTA"))
    {
      count = handleAsta (msg);
    }
  /*
     else if (!strcmp (head, "RES"))
     {
     count = handleRes (msg);
     }
   */
  else if (!strcmp (head, "CONF"))
    {
      ROS_INFO ("CONF: %s", msg );
      count = handleConf (msg);
    }
  else if (!strcmp (head, "GEO"))
    {
      ROS_INFO ("GEO: %s", msg );

      count = handleGeo (msg);
    }
  else
    {
      ROS_ERROR ("unknown head: ``%s''", msg);
      count = handleEm (msg);
    }

  doSenConfs (encoders, (char *) "Encoder");
  doSenConfs (sonars, (char *) "Sonar");
  doSenConfs (rangescanners, (char *) "RangeScanner");
  doSenConfs (rangeimagers, (char *) "RangeImager");
  doSenConfs (touches, (char *) "Touch");
  doSenConfs (co2sensors, (char *) "CO2Sensor");
  doSenConfs (inses, (char *) "INS");
  doSenConfs (groundtruths, (char *) "GroundTruth");
  doSenConfs (gpses, (char *) "GPS");
  doSenConfs (odometers, (char *) "Odometry");
  doSenConfs (victims, (char *) "VictSensor");
  doSenConfs (tachometers, (char *) "Tachometer");
  doSenConfs (acoustics, (char *) "Acoustic");
  doSenConfs (objectsensors, (char *) "ObjectSensor");
  //  doSenConfs (misstas, (char *) "MisPkg");
  doSenConfs (misstas, (char *) "Actuator");
  doEffConfs (grippers, (char *) "Gripper");
  doEffConfs (toolchangers, (char *) "ToolChanger");

  doRobotConfs (robot);

  return count;
}

int
UsarsimInf::handleEm (char *msg)
{
  char token[MAX_TOKEN_LEN];
  char *ptr = msg;
  char *nextptr = ptr;
  int count = 0;
  std::string valueStr;

  while (1)
    {
      nextptr = getKey (ptr, &token[0]);
      if (nextptr == ptr)
	break;
      ROS_WARN ("key %s", token);
      ptr = nextptr;
      nextptr = getValue (ptr, token);
      if (nextptr == ptr)
	break;
      ROS_WARN ("value %s", token);
      ptr = nextptr;
      count++;
      while (1)
	{
	  nextptr = getValue (ptr, token);
	  if (nextptr == ptr)
	    break;
	  ROS_WARN ("value %s", token);
	  ptr = nextptr;
	}
    }

  return count;
}

int
UsarsimInf::doEffConfs (UsarsimList * where, char *type)
{
  char str[MAX_MSG_LEN];
  sw_struct *sw;

  if (NULL == where)
    return -1;
  sw = where->getSW ();
  while (sw->name != "")
    {
      if (!where->didConf () && !waitingForConf)
	{
	  ulapi_snprintf (str, sizeof (str),
			  "GETCONF {Type %s} {Name %s}\r\n", type,
			  sw->name.c_str ());
	  NULLTERM (str);
	  ulapi_mutex_take (socket_mutex);
	  usarsim_socket_write (socket_fd, str, strlen (str));
	  ulapi_mutex_give (socket_mutex);
	  waitingForConf = 1;
	  ROS_DEBUG ("usarsiminf: (2) waitingForConf set \n");
	}
      else if (!where->didGeo () && !waitingForGeo)
	{
	  ulapi_snprintf (str, sizeof (str), "GETGEO {Type %s} {Name %s}\r\n",
			  type, sw->name.c_str ());
	  NULLTERM (str);
	  ulapi_mutex_take (socket_mutex);
	  usarsim_socket_write (socket_fd, str, strlen (str));
	  ulapi_mutex_give (socket_mutex);
	  ROS_DEBUG ("waitingForGeo set (1)");
	  waitingForGeo = 1;
	}
      where = where->getNext ();
      sw = where->getSW();
    }

  return 0;
}

int
UsarsimInf::doRobotConfs (UsarsimList * where)
{
  char str[MAX_MSG_LEN];
  sw_struct *sw;

  if (where == NULL)
    return -1;
  sw = where->getSW ();
  if (!where->didConf () && !waitingForConf)
    {
      ulapi_snprintf (str, sizeof (str), "GETCONF {Type Robot} {Name %s}\r\n",
		      sw->name.c_str ());
      NULLTERM (str);
      ulapi_mutex_take (socket_mutex);
      usarsim_socket_write (socket_fd, str, strlen (str));
      ulapi_mutex_give (socket_mutex);
      waitingForConf = 1;
      ROS_DEBUG ("usarsiminf: (3) waitingForConf set \n");
    }
  else if (!where->didGeo () && !waitingForGeo)
    {
      ulapi_snprintf (str, sizeof (str), "GETGEO {Type Robot} {Name %s}\r\n",
		      sw->name.c_str ());
      NULLTERM (str);
      ulapi_mutex_take (socket_mutex);
      usarsim_socket_write (socket_fd, str, strlen (str));
      ulapi_mutex_give (socket_mutex);
      ROS_DEBUG ("waitingForGeo set (2)");
      waitingForGeo = 1;
    }

  return 0;
}

int
UsarsimInf::doSenConfs (UsarsimList * where, char *type)
{
  char str[MAX_MSG_LEN];
  sw_struct *sw;
  if (where == NULL)
    return -1;

  sw = where->getSW ();
  while (sw->name != "")
    {
      if (!where->didConf () && !waitingForConf)
	{
	  ulapi_snprintf (str, sizeof (str),
			  "GETCONF {Type %s} {Name %s}\r\n", type,
			  sw->name.c_str ());
	  NULLTERM (str);
	  ulapi_mutex_take (socket_mutex);
	  usarsim_socket_write (socket_fd, str, strlen (str));
	  ulapi_mutex_give (socket_mutex);
	  waitingForConf = 1;
	  ROS_DEBUG ("usarsiminf: (1) waitingForConf set \n");
	}
      else if (!where->didGeo () && !waitingForGeo)
	{
	  ulapi_snprintf (str, sizeof (str), "GETGEO {Type %s} {Name %s}\r\n",
			  type, sw->name.c_str ());
	  NULLTERM (str);
	  ulapi_mutex_take (socket_mutex);
	  usarsim_socket_write (socket_fd, str, strlen (str));
	  ulapi_mutex_give (socket_mutex);
	  ROS_DEBUG ("waitingForGeo set (3)");
	  waitingForGeo = 1;
	}
      where = where->getNext ();
      sw = where->getSW ();
    }
  return 0;
}

void
UsarsimInf::setComponentInfo (char *msg, componentInfo * info)
{
  info->ptr = msg;
  info->sawname = 0;
  info->count = 0;
  info->time = 0;
  info->where = &info->def;
}

/*
  STA {Type GroundVehicle}

  {Time 5609.45} {FrontSteer 0.0000} {RearSteer 0.0000} {LightToggle False} {LightIntensity 0} {Battery 1199} {View -1}

  {Time 48.22} {FrontSteer 0.0000} {RearSteer 0.0000} {FRFlipper -0.0006} {FLFlipper -0.0005} {RRFlipper 0.0005} {RLFlipper 0.0006} {LightToggle False} {LightIntensity 0} {Battery 1196} {View -1}

  FIXME -- there's no name here, consequently we can only have one
  vehicle. The name of the vehicle is irrelevant.
*/
int
UsarsimInf::handleStaGroundvehicle (char *msg)
{
  componentInfo info;
  sw_struct *sw;
  setComponentInfo (msg, &info);
  static int didError = 0;

  /* since we only have one robot, we just set it explicitly here
     without going through 'usarsim_class_find' and needing a name */
  info.where = robot;
  sw = info.where->getSW ();
  sw->type = SW_ROBOT_GROUNDVEHICLE;

  if (!info.where->didConf ())
    {
      if (!didError )
	{
	  ROS_WARN ("usarsimInf: Received groundvehicle sta before conf");
	  didError = 1;
	}
      return 1;
    }
  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "GroundVehicle");
	}
      else if (!strcmp (info.token, "Time"))
	{
	  getTime (&info);
	}
      else
	{
	  // skip unknown entry  
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.op = SW_ROBOT_STAT;
  msgout (sw, info);
  return info.count;
}

/*
  STA {Type BaseMachine}

  {Time 5609.45}

  FIXME -- there's no name here.
*/
int
UsarsimInf::handleStaBasemachine (char *msg)
{
  componentInfo info;
  sw_struct *sw;
  setComponentInfo (msg, &info);

  info.where = robot;
  sw = info.where->getSW ();
  sw->type = SW_ROBOT_FIXED;

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "BaseMachine");
	}
      else if (!strcmp (info.token, "Time"))
	{
	  getTime (&info);
	}
      else
	{
	  // skip unknown entry 
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }
  info.op = SW_DEVICE_STAT;
  msgout (sw, info);

  /*
     FIXME-- there is some weirdness with SW_ROBOT_FIXED,GROUNDVEHICLE,
     devices, basemachines, etc. Rather than using a fixed robot, which
     suggests an industrial robot arm, try something like
     SW_ENVIRONMENT, which could be a factory, or a collapsed building.
   */

  return info.count;
}

/*
  STA {Type staticplatform}

  {Time 5609.45}

  FIXME -- there's no name here.
*/
int
UsarsimInf::handleStaStaticplatform (char *msg)
{
  componentInfo info;
  sw_struct *sw = robot->getSW ();
  setComponentInfo (msg, &info);

  info.where = robot;
  sw->type = SW_ROBOT_FIXED;

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "StaticPlatform");
	}
      else if (!strcmp (info.token, "Time"))
	{
	  getTime (&info);
	}
      else
	{
	  // skip unknown entry  
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }
  info.op = SW_DEVICE_STAT;
  msgout (sw, info);
  return info.count;
}


int
UsarsimInf::handleSta (char *msg)
{
  char token[MAX_TOKEN_LEN];
  char *ptr = msg;
  char *nextptr;
  int count = 0;

  while (1)
    {
      nextptr = getKey (ptr, &token[0]);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      // look for {Type <name>}, and pass the whole msg to the STA 
      if (!strcmp (token, "Type"))
	{
	  nextptr = getValue (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if (!strcmp (token, "GroundVehicle"))
	    {
	      return handleStaGroundvehicle (msg);
	    }
	  else if (!strcmp (token, "BaseMachine"))
	    {
	      return handleStaBasemachine (msg);
	    }
	  else if (!strcmp (token, "StaticPlatform"))
	    {
	      return handleStaStaticplatform (msg);
	    }
	  else
	    {
	      ROS_ERROR ("Unknown STA type %s", token);
	      // skip it and keep going 
	    }
	}
      // else something else, probably {Time #} 
    }
  ROS_ERROR ("Unknown STA type(2) %s", msg);
  return count;
}

/*
  NFO {Gametype BotDeathMatch} {Level DM-Factory_250} {TimeLimit 0}

  NFO {StartPoses 3} {Name UnitLoader1 Location 10.24,3.54,10.05 Orientation 0.00,0.00,1.60} {Name UnitLoader2 Location 10.24,3.54,10.05 Orientation 0.00,0.00,1.60} {Name UnitLoader3 Location 10.24,3.54,10.05 Orientation 0.00,0.00,1.60}

*/
int
UsarsimInf::handleNfo (char *msg)
{
  ROS_DEBUG ("handleNfo received: %s", msg);
  return 1;
}

/*
  SEN {Time 5608.85} {Type RangeImager} 

  {Name Scanner1} {Frames 10} {Resolution 0.0174} {FOV 3.1415} {Range 11.3059,11.3014,11.3002,...}
*/

int
UsarsimInf::handleSenRangeimager (char *msg)
{
  
  componentInfo info;;
  int number;
  sw_struct *sw = rangeimagers->getSW ();
  float f;

  setComponentInfo (msg, &info);

  number = 0;
  
  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "RangeImager");
	}
      else if (!strcmp (info.token, "Frame"))
	{
	  sw->data.rangeimager.frame = getInteger (&info);
	}
      else if (!strcmp (info.token, "Frames"))
	{
	  sw->data.rangeimager.totalframes = getInteger (&info);
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (rangeimagers, &info, SW_SEN_RANGEIMAGER_STAT);
	  sw = info.where->getSW ();
	}
      else if (!strcmp (info.token, "Time"))
	{
	  getTime (&info);
	}
      else if (!strcmp (info.token, "Resolution"))
	{
	  sw->data.rangeimager.resolutionx = getReal (&info);
	  sw->data.rangeimager.resolutiony = getReal (&info);
	}
      else if (!strcmp (info.token, "FOV"))
	{
	  sw->data.rangeimager.fovx = getReal (&info);
	  sw->data.rangeimager.fovy = getReal (&info);
	}
      else if (!strcmp (info.token, "Range"))
	{
	  /*
	     We won't use the usual GET_REAL macro to get range values,
	     since there are a bunch separated by commas, and we'll also
	     want to keep track of the cumulative number.
	   */
	  while (1)
	    {
	      info.nextptr = getValue (info.ptr, info.token);
	      if (info.nextptr == info.ptr)
		{
		  if (number == 0)
		    return -1;	// need at least one range value
		  else
		    break;
		}
	      if (sscanf (info.token, "%f", &f) != 1)
		return -1;
	      if (number >= SW_SEN_RANGEIMAGER_MAX)
		{
		  // drop it 
		  ROS_WARN ("rangeimager warning, dropping data");
		}
	      else
		{
		  sw->data.rangeimager.range[number] = f;
		  number++;
		}
	      info.ptr = info.nextptr;
	    }
	  // all ok, so credit the count
	  info.count++;
	}
      else
	{
	  // skip unknown entry 
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }
  sw->data.rangeimager.numberperframe = number;
  info.op = SW_SEN_RANGEIMAGER_STAT;
  msgout (sw, info);

  return info.count;
}

/*
  SEN {Type Touch}

  {Name Touch Touch False}
*/

int
UsarsimInf::handleSenTouch (char *msg)
{
  componentInfo info;;
  sw_struct *sw = touches->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Touch");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (touches, &info, SW_SEN_TOUCH_STAT);
	  sw = info.where->getSW ();
	  /* FIXME -- conf and geo are not handled inside usarsim */
	  info.where->setDidConf (1);
	  info.where->setDidGeo (1);
	  expect (&info, "Touch");
	  /* expecting "True" or "False" */
	  info.nextptr = getValue (info.ptr, info.token);
	  if (info.nextptr == info.ptr)
	    return -1;
	  if (!strcmp (info.token, "False"))
	    sw->data.touch.touched = 0;
	  else if (!strcmp (info.token, "True"))
	    sw->data.touch.touched = 1;
	  else
	    return -1;
	  info.ptr = info.nextptr;
	  /* all ok, so credit the count */
	  info.count++;
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.op = SW_SEN_TOUCH_STAT;
  msgout (sw, info);

  return info.count;
}

/*
  SEN {Type CO2Sensor}

  {Name CO2} {Gas CO2} {Density 0.00}
*/
int
UsarsimInf::handleSenCo2sensor (char *msg)
{
  componentInfo info;;
  sw_struct *sw = co2sensors->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "CO2Sensor");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (co2sensors, &info, SW_SEN_CO2_STAT);
	  sw = info.where->getSW ();
	  /* FIXME -- conf and geo are not handled inside usarsim */
	  info.where->setDidGeo (1);
	  info.where->setDidConf (1);
	}
      else if (!strcmp (info.token, "Gas"))
	{
	  expect (&info, "CO2");
	}
      else if (!strcmp (info.token, "Density"))
	{
	  sw->data.co2sensor.density = getReal (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.op = SW_SEN_CO2_STAT;
  msgout (sw, info);

  return info.count;
}

/*
  SEN {Time 5608.8500} {Type GroundTruth} {Name GroundTruth} {Location 12.61,-2.68,1.64} {Orientation 0.00,6.23,0.00}
*/
/*
int
UsarsimInf::handleSenGroundtruth (char *msg)
{
  componentInfo info;;
  sw_struct *sw = inses->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "GroundTruth");
	}
      else if (!strcmp (info.token, "Time"))
	{
	  getTime (&info);
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (groundtruths, &info, SW_SEN_GROUNDTRUTH_STAT);
	  sw = info.where->getSW ();
	}
      else if (!strcmp (info.token, "Location"))
	{
	  sw->data.groundtruth.position.x = getReal (&info);
	  sw->data.groundtruth.position.y = getReal (&info);
	  sw->data.groundtruth.position.z = getReal (&info);
	}
      else if (!strcmp (info.token, "Orientation"))
	{
	  sw->data.groundtruth.position.roll = getReal (&info);
	  sw->data.groundtruth.position.pitch = getReal (&info);
	  sw->data.groundtruth.position.yaw = getReal (&info);
	}
      else
	{
	// skip unknown entry 
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.op = SW_SEN_GROUNDTRUTH_STAT;
  msgout (sw, info);

  return info.count;
}
*/
/*
  SEN {Type GPS}

  {Name GPS} {Latitude 39,8.0341,N} {Longitude 77,13.0001,W} {Fix 1} {Satellites 11}
*/

int
UsarsimInf::handleSenGps (char *msg)
{
  componentInfo info;;
  double latdeg;
  double latmin;
  double londeg;
  double lonmin;
  int south = 0;
  int west = 0;
  sw_struct *sw = gpses->getSW ();

  setComponentInfo (msg, &info);
  latdeg = londeg = latmin = lonmin = 0;

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "GPS");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (gpses, &info, SW_SEN_GPS_STAT);
	  sw = info.where->getSW ();
	}
      else if (!strcmp (info.token, "Latitude"))
	{
	  latdeg = getReal (&info);
	  latmin = getReal (&info);
	  /* this is too specific to define convenience macro */
	  info.nextptr = getValue (info.ptr, info.token);
	  if (info.nextptr == info.ptr)
	    return -1;
	  if ('N' == info.token[0])
	    south = 0;
	  else if ('S' == info.token[0])
	    south = 1;
	  else
	    return -1;
	  info.count++;
	  info.ptr = info.nextptr;
	}
      else if (!strcmp (info.token, "Longitude"))
	{
	  londeg = getReal (&info);
	  lonmin = getReal (&info);
	  info.nextptr = getValue (info.ptr, info.token);
	  if (info.nextptr == info.ptr)
	    return -1;
	  if ('E' == info.token[0])
	    west = 0;
	  else if ('W' == info.token[0])
	    west = 1;
	  else
	    return -1;
	  info.count++;
	  info.ptr = info.nextptr;
	}
      else if (!strcmp (info.token, "Fix"))
	{
	  sw->data.gps.fix = getInteger (&info);
	}
      else if (!strcmp (info.token, "Satellites"))
	{
	  sw->data.gps.satellites = getInteger (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  sw->data.gps.latitude = latdeg + (latmin / 60.0);
  if (south)
    sw->data.gps.latitude = -sw->data.gps.latitude;
  sw->data.gps.longitude = londeg + (lonmin / 60.0);
  if (west)
    sw->data.gps.longitude = -sw->data.gps.longitude;

  info.op = SW_SEN_GPS_STAT;
  msgout (sw, info);

  return info.count;
}

/*
  SEN {Type INS}

  {Name INS} {Location 12.61,-2.68,1.84} {Orientation 0.00,0.00,0.00}
*/

int
UsarsimInf::handleSenIns (char *msg, const char *sensorType)
{
  componentInfo info;;
  sw_struct *sw;
  UsarsimList *myList;

  setComponentInfo (msg, &info);

  if( !strcmp(sensorType, "INS" ))
    myList = inses;
  else if ( !strcmp(sensorType, "GroundTruth" ) )
    myList = groundtruths;
  else
    {
      ROS_ERROR ("Unknown Ins type: %s", sensorType );
      return info.count;
    }

  sw = myList->getSW ();
  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, sensorType);
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (myList, &info, SW_SEN_INS_STAT);
	  sw = info.where->getSW ();
	}
      else if (!strcmp (info.token, "Location"))
	{
	  sw->data.ins.position.x = getReal (&info);
	  sw->data.ins.position.y = getReal (&info);
	  sw->data.ins.position.z = getReal (&info);
	}
      else if (!strcmp (info.token, "Orientation"))
	{
	  sw->data.ins.position.roll = getReal (&info);
	  sw->data.ins.position.pitch = getReal (&info);
	  sw->data.ins.position.yaw = getReal (&info);
	}
      else if (!strcmp (info.token, "Time"))
	{
	  getTime (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.op = SW_SEN_INS_STAT;
  msgout (sw, info);

  return info.count;
}

/*
  SEN {Time 44.5467} {Type Odometry} 
  {Name Odometer} {Pose  0.0000,0.0000,0.0000}

  FIXME -- this needs to have full x y z r p w info
*/

int
UsarsimInf::handleSenOdometry (char *msg)
{
  componentInfo info;;
  sw_struct *sw = odometers->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Odometry");
	}
      else if (!strcmp (info.token, "Time"))
	{
	  getTime (&info);
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (odometers, &info, SW_SEN_ODOMETER_STAT);
	  sw = info.where->getSW ();
	}
      else if (!strcmp (info.token, "Pose"))
	{
	  sw->data.odometer.position.x = getReal (&info);
	  sw->data.odometer.position.y = getReal (&info);
	  sw->data.odometer.position.yaw = getReal (&info);
	  sw->data.odometer.position.roll = 0;
	  sw->data.odometer.position.pitch = 0;
	  sw->data.odometer.position.z = 0;
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.op = SW_SEN_ODOMETER_STAT;
  msgout (sw, info);

  return info.count;
}

/*
  SEN {Time 5609.4467} {Type VictSensor} {Status NoVictims}

  FIXME -- the name is required to be VictSensor in the .uc code.
*/

int
UsarsimInf::handleSenVictim (char *msg)
{
  componentInfo info;;
  //  sw_struct *sw = victims->getSW();

  setComponentInfo (msg, &info);

  ROS_WARN ("Victim sensor not working");
  return 0;
  /*
     // FIXME per comment above 
     if (0 != usarsim_class_find (victims, "VictSensor", &where))
     return -1;
     sw_set_name (&where->sw, "VictSensor");

     while (1)
     {
     info.nextptr = getKey (info.ptr, info.token);
     if (info.nextptr == info.ptr)
     break;
     info.ptr = info.nextptr;

     if (!strcmp (info.token, "Type"))
     {
     expect (&info, "VictSensor");
     }
     else if (!strcmp (info.token, "Time"))
     {
     getTime (&info);
     }
     else if (!strcmp (info.token, "Status"))
     {
     info.nextptr = getValue (info.ptr, info.token);
     if (info.nextptr == info.ptr)
     return -1;
     if (!strcmp (info.token, "Victims"))
     sw->data.victim.victims = 1;
     else if (!strcmp (info.token, "NoVictims"))
     sw->data.victim.victims = 0;
     else
     return -1;
     info.ptr = info.nextptr;
     // all ok, so credit the count
     info.count++;
     }
     else
     {
     // skip unknown entry 
     info.nextptr = getValue (info.ptr, info.token);
     }
     }

     info.op =SW_SEN_VICTIM_STAT;
     msgout (sw, info );

     return info.count;
   */
}

/*
  SEN {Type Tachometer} {Name TachTest} {Vel 0.0000,0.0000,0.0000,0.0000} {Pos 6.2832,6.2832,6.2832,6.2832}
*/

int
UsarsimInf::handleSenTachometer (char *msg)
{
  componentInfo info;;
  int vel_number = 0;
  int pos_number = 0;
  sw_struct *sw = tachometers->getSW ();
  double d;

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Tachometer");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (tachometers, &info, SW_SEN_TACHOMETER_STAT);
	  sw = info.where->getSW ();
	}
      else if (!strcmp (info.token, "Vel"))
	{
	  /*
	     We won't use the usual GET_REAL macro to get range values,
	     since there are a bunch separated by commas, and we'll also
	     want to keep track of the cumulative number.
	   */
	  while (1)
	    {
	      info.nextptr = getValue (info.ptr, info.token);
	      if (info.nextptr == info.ptr)
		{
		  if (vel_number == 0)
		    {
		      ROS_ERROR ("Tach needs at least one speed value");
		      return -1;	/* need at least one speed value */
		    }
		  else
		    break;
		}
	      if (sscanf (info.token, "%lf", &d) != 1)
		{
		  // this should only happen if we are done reading positions
		  if (vel_number == 0)
		    {
		      ROS_ERROR ("Tach needs at least one speed value(2)");
		      return -1;	/* need at least one speed value */
		    }
		  else
		    break;
		}
	      if (vel_number >= SW_SEN_TACHOMETER_MAX)
		{
		  /* drop it */
		}
	      else
		{
		  sw->data.tachometer.speed[vel_number] = d;
		  vel_number++;
		}
	      info.ptr = info.nextptr;
	    }
	}
      else if (!strcmp (info.token, "Pos"))
	{
	  while (1)
	    {
	      info.nextptr = getValue (info.ptr, info.token);
	      if (info.nextptr == info.ptr)
		{
		  if (pos_number == 0)
		    {
		      ROS_ERROR ("Tach needs at least one pos value");
		      return -1;	/* need at least one speed value */
		    }
		  else
		    break;
		}
	      if (sscanf (info.token, "%lf", &d) != 1)
		{
		  // this should only happen if we are done reading positions
		  if (pos_number == 0)
		    {
		      ROS_ERROR ("Tach needs at least one pos value(2)");
		      return -1;	/* need at least one speed value */
		    }
		  else
		    break;

		}
	      if (pos_number >= SW_SEN_TACHOMETER_MAX)
		{
		  /* drop it */
		}
	      else
		{
		  sw->data.tachometer.position[pos_number] = d;
		  pos_number++;
		}
	      info.ptr = info.nextptr;
	    }
	  /* credit the count here, after the last expected data type 'Pos' */
	  info.count++;
	}
      else if (!strcmp (info.token, "Time"))
	{
	  getTime (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	  ROS_WARN ("Unknown key in tach: %s value: %s",
		    info.token, info.nextptr);
	}
    }

  sw->data.tachometer.number =
    pos_number > vel_number ? pos_number : vel_number;
  info.op = SW_SEN_TACHOMETER_STAT;
  msgout (sw, info);

  return info.count;
}

/*
  SEN {Time 390.5659} {Type Acoustic} {Name Test}
  {None}
  or
  {Far}
  or
  {Direction 1,0,0} {Volume 0.5} {Duration 1.4} {Delay 0.1}
*/

int
UsarsimInf::handleSenAcoustic (char *msg)
{
  componentInfo info;;
  double x, y, z;
  sw_struct *sw = acoustics->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Acoustic");
	}
      else if (!strcmp (info.token, "Time"))
	{
	  getTime (&info);
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (acoustics, &info, SW_SEN_ACOUSTIC_STAT);
	  sw = info.where->getSW ();
	}
      else if (!strcmp (info.token, "Direction"))
	{
	  x = getReal (&info);
	  y = getReal (&info);
	  z = getReal (&info);
	  sw->data.acoustic.azimuth = atan2 (y, x);
	  sw->data.acoustic.altitude = atan2 (z, sqrt (x * x + y * y));
	}
      else if (!strcmp (info.token, "Volume"))
	{
	  sw->data.acoustic.volume = getReal (&info);
	}
      else if (!strcmp (info.token, "Duration"))
	{
	  sw->data.acoustic.duration = getReal (&info);
	  /* ignore "Delay" since that's not obtainable by a real sensor */
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.op = SW_SEN_ACOUSTIC_STAT;
  msgout (sw, info);

  return info.count;
}


/*
  SEN {Type Encoder} 

  {Name EncTest1} {Tick 0}
*/
int
UsarsimInf::handleSenEncoder (char *msg)
{
  componentInfo info;;
  sw_struct *sw = encoders->getSW ();

  setComponentInfo (msg, &info);

  /*
     NOTE -- the 'where' pointer doesn't get set until "Name" is seen,
     but where->sw may be accessed earlier by "Time". USARsim needs to
     be fixed so that all messages begin with "Name". Since in all
     cases, the only time data precedes "Name" is with "Time", we'll
     save time locally and fill it in when 'tell' is called. Just in
     case, we start off 'where' to point to a dummy 'def' struct. That
     way, any rogue USARsim messages that don't set "Name" until later
     won't cause a null pointer dereference. Any data that precedes
     "Name" (except for "Time") will be lost.
   */
  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;
      sw = info.where->getSW ();

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Encoder");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (encoders, &info, SW_SEN_ENCODER_STAT);
	}
      else if (!strcmp (info.token, "Tick"))
	{
	  sw->data.encoder.tick = getInteger (&info);
	}
      else if (!strcmp (info.token, "Time"))
	{
	  getTime (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }
  info.op = SW_SEN_ENCODER_STAT;
  msgout (sw, info);
  return info.count;
}

/*
  SEN {Time 5608.8500} {Type Sonar}

  {Name F1 Range 2.5098} {Name F2 Range 3.3153} {Name F3 Range 4.9950} {Name F4 Range 2.0774} {Name F5 Range 2.0780} {Name F6 Range 5.0000} {Name F7 Range 5.0000} {Name F8 Range 4.9978} {Name R1 Range 4.9978} {Name R2 Range 4.0775} {Name R3 Range 2.9782} {Name R4 Range 2.6137} {Name R5 Range 2.6148} {Name R6 Range 2.9942} {Name R7 Range 3.3052} {Name R8 Range 2.5098}
*/

int
UsarsimInf::handleSenSonar (char *msg)
{
  componentInfo info;;
  sw_struct *sw = sonars->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Sonar");
	}
      else if (!strcmp (info.token, "Time"))
	{
	  getTime (&info);
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (sonars, &info, SW_SEN_SONAR_STAT);
	  sw = info.where->getSW ();
	  expect (&info, "Range");
	  sw->data.sonar.range = getReal (&info);
	}
      else
	{
	  // skip unknown entry
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.op = SW_SEN_SONAR_STAT;
  msgout (sw, info);

  return info.count;
}

/*
  SEN {Time 5608.85} {Type RangeScanner} 

  {Name Scanner1} {Resolution 0.0174} {FOV 3.1415} {Range 11.3059,11.3014,11.3002,...}
*/

int
UsarsimInf::handleSenRangescanner (char *msg)
{
  componentInfo info;
  sw_struct *sw = rangescanners->getSW ();
  double d;

  int number = 0;
  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "RangeScanner");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (rangescanners, &info, SW_SEN_RANGESCANNER_STAT);
	  sw = info.where->getSW ();
	}
      else if (!strcmp (info.token, "Time"))
	{
	  getTime (&info);
	}
      else if (!strcmp (info.token, "Resolution"))
	{
	  sw->data.rangescanner.resolution = getReal (&info);
	}
      else if (!strcmp (info.token, "FOV"))
	{
	  sw->data.rangescanner.fov = getReal (&info);
	}
      else if (!strcmp (info.token, "Range"))
	{
	  /*
	     We won't use the usual GET_REAL macro to get range values,
	     since there are a bunch separated by commas, and we'll also
	     want to keep track of the cumulative number.
	   */
	  while (1)
	    {
	      info.nextptr = getValue (info.ptr, info.token);
	      if (info.nextptr == info.ptr)
		{
		  if (number == 0)
		    return -1;	// need at least one range value 
		  else
		    break;
		}
	      if (sscanf (info.token, "%lf", &d) != 1)
		return -1;
	      if (number >= SW_SEN_RANGESCANNER_MAX)
		{
		  // drop it 
		}
	      else
		{
		  sw->data.rangescanner.range[number] = d;
		  number++;
		}
	      info.ptr = info.nextptr;
	    }
	  // all ok, so credit the count
	  info.count++;
	}
      else
	{
	  // skip unknown entry  
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  sw->data.rangescanner.number = number;
  info.op = SW_SEN_RANGESCANNER_STAT;
  msgout (sw, info);

  return info.count;
}

int UsarsimInf::handleSenObjectSensor(char *msg)
{
    componentInfo info;
    sw_struct *sw = objectsensors->getSW();
    int objectIndex = -1;
    setComponentInfo(msg, &info);
    
	while(1)
	{
		info.nextptr = getKey(info.ptr, info.token);
		if(info.nextptr == info.ptr)
			break;
		info.ptr = info.nextptr;
		if(!strcmp(info.token, "Type"))
		{
			expect(&info, "ObjectSensor");
		}else if(!strcmp(info.token, "Name"))
		{
			getName (objectsensors, &info, SW_SEN_OBJECTSENSOR_STAT);
			sw = info.where->getSW();
			objectIndex = -1;
		}
		else if (!strcmp (info.token, "Time"))
		{
			getTime (&info);
		}else if(!strcmp(info.token, "Object"))
		{
			objectIndex++;
			if(objectIndex >= SW_SEN_RANGESCANNER_MAX)
			{
			}
			else
			{
				info.nextptr = getValue (info.ptr, info.token);
		  		if (info.nextptr == info.ptr)
					return -1;
				strcpy(sw->data.objectsensor.objects[objectIndex].tag, info.token);
	    	}
		}
		else if (!strcmp(info.token, "Location"))
		{
			if(objectIndex < 0)
				return -1;
			sw->data.objectsensor.objects[objectIndex].position.x = getReal (&info);
	  		sw->data.objectsensor.objects[objectIndex].position.y = getReal (&info);
	  		sw->data.objectsensor.objects[objectIndex].position.z = getReal (&info);
		}
		else if (!strcmp(info.token, "Orientation"))
		{
			if(objectIndex < 0)
				return -1;
			sw->data.objectsensor.objects[objectIndex].position.roll = getReal (&info);
	  		sw->data.objectsensor.objects[objectIndex].position.pitch = getReal (&info);
	  		sw->data.objectsensor.objects[objectIndex].position.yaw = getReal (&info);
		}
		else if (!strcmp(info.token, "HitLoc"))
		{
			if(objectIndex < 0)
				return -1;
			sw->data.objectsensor.objects[objectIndex].hit_location.x = getReal (&info);
	  		sw->data.objectsensor.objects[objectIndex].hit_location.y = getReal (&info);
	  		sw->data.objectsensor.objects[objectIndex].hit_location.z = getReal (&info);
		}
		else if (!strcmp(info.token, "Material"))
		{
			info.nextptr = getValue (info.ptr, info.token);
			if (info.nextptr == info.ptr)
				return -1;
			strcpy(sw->data.objectsensor.objects[objectIndex].material_name, info.token);
		}
		else
		{
			
			info.nextptr = getValue (info.ptr, info.token);
		}
	}
	sw->data.objectsensor.number = objectIndex + 1;
	info.op = SW_SEN_OBJECTSENSOR_STAT;
	
	msgout(sw, info);
	return 0;
}
int
UsarsimInf::handleSen (char *msg)
{
  char token[MAX_TOKEN_LEN];
  char *ptr = msg;
  char *nextptr;
  int count = 0;

  while (1)
    {
      nextptr = getKey (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      /* look for {Type <name>}, and pass the whole msg to the sensor */
      if (!strcmp (token, "Type"))
	{
	  nextptr = getValue (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if (!strcmp (token, "Sonar"))
	    {
	      return handleSenSonar (msg);
	    }
	  else if (!strcmp (token, "RangeScanner"))
	    {
	      return handleSenRangescanner (msg);
	    }
	  else if (!strcmp (token, "RangeImager"))
	    {
	      return handleSenRangeimager (msg);
	    }
	  if (!strcmp (token, "Encoder"))
	    {
	      return handleSenEncoder (msg);
	    }
	  else if (!strcmp (token, "Touch"))
	    {
	      return handleSenTouch (msg);
	    }
	  else if (!strcmp (token, "CO2Sensor"))
	    {
	      return handleSenCo2sensor (msg);
	    }
	  else if (!strcmp (token, "GroundTruth"))
	    {
	      return handleSenIns (msg, "GroundTruth");
	    }
	  else if (!strcmp (token, "GPS"))
	    {
	      return handleSenGps (msg);
	    }
	  else if (!strcmp (token, "INS"))
	    {
	      return handleSenIns (msg, "INS");
	    }
	  else if (!strcmp (token, "Odometry"))
	    {
	      return handleSenOdometry (msg);
	    }
	  else if (!strcmp (token, "VictSensor"))
	    {
	      return handleSenVictim (msg);
	    }
	  else if (!strcmp (token, "Tachometer"))
	    {
	      return handleSenTachometer (msg);
	    }
	  else if (!strcmp (token, "Acoustic"))
	    {
	      return handleSenAcoustic (msg);
	    }
	  else if (!strcmp(token, "ObjectSensor"))
	    {
	      return handleSenObjectSensor (msg);
	    }
	  else if (!strcmp (token, "Camera"))
	    {
	      return count;
	    }
	  else
	    {
	      ROS_ERROR ("Unknown sensor type %s", token);
	      /* skip it and keep going */
	    }
	}
      /* else something else to be handled by sensor, probably {Time #} */
    }

  return count;
}

/*
  CONF {Type Encoder} {Name ECLeft} {Resolution 0.0174}
*/
int
UsarsimInf::handleConfEncoder (char *msg)
{
  componentInfo info;
  sw_struct *sw = encoders->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Encoder");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (encoders, &info, SW_SEN_ENCODER_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);

	}
      else if (!strcmp (info.token, "Resolution"))
	{
	  sw->data.encoder.resolution = getReal (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }
  info.where->setDidConf (1);
  info.op = SW_SEN_ENCODER_SET;
  msgout (sw, info);
  return info.count;
}

/*
  CONF {Type Sonar} {Name R8} {MaxRange 5.0000} {MinRange 0.1000} {BeamAngle 0.3491}
*/

int
UsarsimInf::handleConfSonar (char *msg)
{
  componentInfo info;
  sw_struct *sw = sonars->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Sonar");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (sonars, &info, SW_SEN_SONAR_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	}
      else if (!strcmp (info.token, "MaxRange"))
	{
	  sw->data.sonar.maxrange = getReal (&info);
	}
      else if (!strcmp (info.token, "MinRange"))
	{
	  sw->data.sonar.minrange = getReal (&info);
	}
      else if (!strcmp (info.token, "BeamAngle"))
	{
	  sw->data.sonar.beamangle = getReal (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidConf (1);
  info.op = SW_SEN_SONAR_SET;
  msgout (sw, info);
  return info.count;
}

/*
  CONF {Type RangeImager} {Name Scanner1} {MaxRange 20.0000} {MinRange 0.1000} {Resolution 0.0174} {Fov 3.1415} {Paning True} {Tilting False}
*/
int
UsarsimInf::handleConfRangeimager (char *msg)
{
  componentInfo info;
  sw_struct *sw = rangeimagers->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "RangeImager");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (rangeimagers, &info, SW_SEN_RANGEIMAGER_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	}
      else if (!strcmp (info.token, "MaxRange"))
	{
	  sw->data.rangeimager.maxrange = getReal (&info);
	}
      else if (!strcmp (info.token, "MinRange"))
	{
	  sw->data.rangeimager.minrange = getReal (&info);
	}
      else if (!strcmp (info.token, "Resolution"))
	{
	  sw->data.rangeimager.resolutionx = getReal (&info);
	  sw->data.rangeimager.resolutiony = getReal (&info);
	}
      else if (!strcmp (info.token, "Fov"))
	{
	  sw->data.rangeimager.fovx = getReal (&info);
	  sw->data.rangeimager.fovy = getReal (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.op = SW_SEN_RANGEIMAGER_SET;
  info.where->setDidConf (1);
  msgout (sw, info);
  return info.count;
}


/*
  CONF {Type RangeScanner} {Name Scanner1} {MaxRange 20.0000} {MinRange 0.1000} {Resolution 0.0174} {Fov 3.1415} {Paning True} {Tilting False}
*/
int
UsarsimInf::handleConfRangescanner (char *msg)
{
  componentInfo info;
  sw_struct *sw = rangescanners->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "RangeScanner");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (rangescanners, &info, SW_SEN_RANGESCANNER_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	}
      else if (!strcmp (info.token, "MaxRange"))
	{
	  sw->data.rangescanner.maxrange = getReal (&info);
	}
      else if (!strcmp (info.token, "MinRange"))
	{
	  sw->data.rangescanner.minrange = getReal (&info);
	}
      else if (!strcmp (info.token, "Resolution"))
	{
	  sw->data.rangescanner.resolution = getReal (&info);
	}
      else if (!strcmp (info.token, "Fov"))
	{
	  sw->data.rangescanner.fov = getReal (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidConf (1);
  info.op = SW_SEN_RANGESCANNER_SET;
  msgout (sw, info);
  return info.count;
}

int
UsarsimInf::handleConfTouch (char *msg)
{
  static int didit = 0;
  if (!didit)
    {
      ROS_WARN ("not handling %s", msg);
      didit = 1;
    }
  return 0;
}

int
UsarsimInf::handleConfCo2sensor (char *msg)
{
  static int didit = 0;

  if (!didit)
    {
      ROS_WARN ("not handling %s\n", msg);
      didit = 1;
    }
  return 0;
}

/*
  CONF {Type GroundTruth} {Name GroundTruth} {ScanInterval 0.05}
*/
/*
int
UsarsimInf::handleConfGroundtruth (char *msg)
{
  componentInfo info;
  sw_struct *sw = groundtruths->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "GroundTruth");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (groundtruths, &info, SW_SEN_GROUNDTRUTH_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	}
      else if (!strcmp (info.token, "ScanInterval"))
	{
	  sw->data.groundtruth.period = getReal (&info);
	}
      else
	{
	  // skip unknown entry 
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidConf (1);
  info.op = SW_SEN_GROUNDTRUTH_SET;
  msgout (sw, info);
  return info.count;
}
*/
/*
  CONF {Type GPS} {Name GPS} {ScanInterval 0.20}
*/
int
UsarsimInf::handleConfGps (char *msg)
{
  componentInfo info;
  sw_struct *sw = gpses->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "GPS");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (gpses, &info, SW_SEN_GPS_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	}
      else if (!strcmp (info.token, "ScanInterval"))
	{
	  sw->data.gps.period = getReal (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidConf (1);
  info.op = SW_SEN_GPS_SET;
  msgout (sw, info);
  return info.count;
}

/*
  CONF {Type INS} {Name INS} {ScanInterval 0.20}
*/
int
UsarsimInf::handleConfIns (char *msg, const char *sensorType)
{
  componentInfo info;
  sw_struct *sw;
  UsarsimList *myList;

  setComponentInfo (msg, &info);
  if( !strcmp(sensorType, "INS" ))
    myList = inses;
  else if ( !strcmp(sensorType, "GroundTruth" ) )
    myList = groundtruths;
  else
    {
      ROS_ERROR ("Unknown Ins type: %s", sensorType );
      return info.count;
    }

  sw = myList->getSW ();
  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, sensorType);
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (myList, &info, SW_SEN_INS_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	}
      else if (!strcmp (info.token, "ScanInterval"))
	{
	  sw->data.ins.period = getReal (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidConf (1);
  info.op = SW_SEN_INS_SET;
  msgout (sw, info);
  return info.count;
}

/*
  CONF {Type Odometry} {Name Odometry} {ScanInterval 0.2000} {EncoderResolution 0.0099}
*/
int
UsarsimInf::handleConfOdometry (char *msg)
{
  componentInfo info;
  sw_struct *sw = odometers->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Odometry");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (odometers, &info, SW_SEN_ODOMETER_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	}
      else if (!strcmp (info.token, "ScanInterval"))
	{
	  sw->data.odometer.period = getReal (&info);
	}
      else if (!strcmp (info.token, "EncoderResolution"))
	{
	  sw->data.odometer.resolution = getReal (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidConf (1);
  info.op = SW_SEN_ODOMETER_SET;
  msgout (sw, info);
  return info.count;
}

/*
  CONF {Type Tachometer} {Name TachTest} {ScanInterval 0.2000}
*/
int
UsarsimInf::handleConfTachometer (char *msg)
{
  componentInfo info;
  sw_struct *sw = tachometers->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Tachometer");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (tachometers, &info, SW_SEN_TACHOMETER_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	}
      else if (!strcmp (info.token, "ScanInterval"))
	{
	  sw->data.odometer.period = getReal (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidConf (1);
  info.op = SW_SEN_TACHOMETER_SET;
  msgout (sw, info);
  return info.count;
}

/*
  CONF {Type Acoustic} {Name Test}
*/
int
UsarsimInf::handleConfAcoustic (char *msg)
{
  componentInfo info;
  sw_struct *sw = acoustics->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Acoustic");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (acoustics, &info, SW_SEN_ACOUSTIC_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	}
      else if (!strcmp (info.token, "ScanInterval"))
	{
	  sw->data.odometer.period = getReal (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidConf (1);
  info.op = SW_SEN_ACOUSTIC_SET;
  msgout (sw, info);
  return info.count;
}

/*
  CONF {Type VictSensor} {Name VictSensor} {MaxRange 6.0000} {HorizontalFOV 0.6981} {VerticalFOV 0.6981}
*/
int
UsarsimInf::handleConfVictim (char *msg)
{
  componentInfo info;
  sw_struct *sw = victims->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "VictSensor");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (victims, &info, SW_SEN_VICTIM_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	}
      else if (!strcmp (info.token, "MaxRange"))
	{
	  sw->data.victim.maxrange = getReal (&info);
	}
      else if (!strcmp (info.token, "HorizontalFOV"))
	{
	  sw->data.victim.hfov = getReal (&info);
	}
      else if (!strcmp (info.token, "VerticalFOV"))
	{
	  sw->data.victim.vfov = getReal (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidConf (1);
  info.op = SW_SEN_VICTIM_SET;
  msgout (sw, info);
  return info.count;
}

/*
  CONF {Type Gripper} {Name Gripper1} {Opcode GRIP} {MaxVal 1} {MinVal 0}
*/
int
UsarsimInf::handleConfGripper (char *msg)
{
  componentInfo info;
  sw_struct *sw = grippers->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Gripper");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (grippers, &info, SW_EFF_GRIPPER_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	}
	else if(!strcmp (info.token, "Opcode"))
	{
		info.nextptr = getValue (info.ptr, info.token);
		//add this to the list of available opcodes for this gripper
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidConf (1);
  info.op = SW_EFF_GRIPPER_SET;
  msgout (sw, info);
  return info.count;
}

int
UsarsimInf::handleConfToolchanger (char *msg)
{
  componentInfo info;
  sw_struct *sw = toolchangers->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "ToolChanger");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (toolchangers, &info, SW_EFF_TOOLCHANGER_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	}
	else if(!strcmp (info.token, "Opcode"))
	{
		info.nextptr = getValue (info.ptr, info.token);
		//add this to the list of available opcodes for this gripper
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidConf (1);
  info.op = SW_EFF_TOOLCHANGER_SET;
  msgout (sw, info);
  return info.count;
}

/*
  CONF {Type Actuator} {Name TeleMaxArm} {Link 1} {JointType Revolute} {MaxSpeed 0.17} {MaxTorque 300.00} {MinValue 1.00} {MaxValue 0.00} ...
*/
int
UsarsimInf::handleConfActuator (char *msg)
{
  componentInfo info;
  int i;
  int linkindex;
  sw_struct *sw = misstas->getSW ();

  setComponentInfo (msg, &info);
  linkindex = 0;
  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Actuator");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (misstas, &info, SW_ACT_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	  sw->data.actuator.number = 0;
	}
      else if (!strcmp (info.token, "Link"))
	{
	  /* expecting number */
	  info.nextptr = getValue (info.ptr, info.token);
	  if (info.nextptr == info.ptr)
	    return -1;
	  if (1 != sscanf (info.token, "%i", &i))
	    return -1;
	  if (i < 1)
	    {
	      ROS_ERROR ("invalid link number for %s: %d", sw->name.c_str (),
			 i);
	      i = 1;
	    }
	  else if (i > SW_ACT_LINK_MAX)
	    {
	      ROS_ERROR ("invalid link number for %s: %d", sw->name.c_str (),
			 i);
	      i = SW_ACT_LINK_MAX;
	    }
	  linkindex = i - 1;
	  info.ptr = info.nextptr;
	  if (i > sw->data.actuator.number)
	    sw->data.actuator.number = i;
	  info.count++;
	}
      else if (!strcmp (info.token, "JointType"))
	{
	  /* expecting "Revolute" or "Prismatic" */
	  info.nextptr = getValue (info.ptr, info.token);
	  if (info.nextptr == info.ptr)
	    return -1;
	  if (!strcmp (info.token, "Prismatic"))
	    {
	      sw->data.actuator.link[linkindex].type = SW_LINK_PRISMATIC;
	    }
	  else if (!strcmp (info.token, "Revolute"))
	    {
	      sw->data.actuator.link[linkindex].type = SW_LINK_REVOLUTE;
	    }
	  else if (!strcmp (info.token, "Scissor"))
	    {
	      sw->data.actuator.link[linkindex].type = SW_LINK_SCISSOR;
	    }
	  else
	    {
	      ROS_ERROR ("bad value for %s JointType: %s", sw->name.c_str (),
			 info.token);
	      sw->data.actuator.link[linkindex].type = SW_NONE;
	    }
	  info.ptr = info.nextptr;
	  /* all ok, so credit the count */
	  info.count++;
	}
      else if (!strcmp (info.token, "MaxSpeed"))
	{
	  sw->data.actuator.link[linkindex].maxspeed = getReal (&info);
	}
      else if (!strcmp (info.token, "MaxTorque"))
	{
	  sw->data.actuator.link[linkindex].maxtorque = getReal (&info);
	}
      else if (!strcmp (info.token, "MinValue"))
	{
	  sw->data.actuator.link[linkindex].minvalue = getReal (&info);
	}
      else if (!strcmp (info.token, "MaxValue"))
	{
	  sw->data.actuator.link[linkindex].maxvalue = getReal (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  /*
     Some mission packages don't report any link information. For these,
     set the number of links to be the max.
   */

  if (0 == sw->data.actuator.number)
    {
      sw->data.actuator.number = SW_ACT_LINK_MAX;
    }

  /*
     Only do the 'tell' operation if we got data -- this will ignore
     empty "GEO {Type Actuator}" strings that may be sent out after a
     "GETGEO {Type Actuator}" if there are no mission packages.

     FIXME -- should do this for all messages
   */
  if (info.count > 0)
    {
      info.op = SW_ACT_SET;
      msgout (sw, info);
    }

  info.where->setDidConf (1);
  return info.count;
}

/*
  CONF {Type GroundVehicle}

  {Name P2AT} {SteeringType SkidSteered} {Mass 14.0000} {MaxSpeed 5.3850} {MaxTorque 60.0000} {MaxFrontSteer 0.0000} {MaxRearSteer 0.0000}
*/
int
UsarsimInf::handleConfGroundvehicle (char *msg)
{
  componentInfo info;
  sw_struct *sw = robot->getSW ();

  setComponentInfo (msg, &info);
  sw->type = SW_ROBOT_GROUNDVEHICLE;

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "GroundVehicle");
	  robot->setDidConf (1);
	  info.ptr = info.nextptr;
	}
      else if (!strcmp (info.token, "Name"))
	{
	  info.nextptr = getValue (info.ptr, info.token);
	  if (info.nextptr == info.ptr)
	    return -1;
	  sw->name = std::string (info.token);
	  info.ptr = info.nextptr;
	}
      else if (!strcmp (info.token, "SteeringType"))
	{
	  /* expecting "SkidSteered", "AckermanSteered" or "OmniDrive" */
	  info.nextptr = getValue (info.ptr, info.token);
	  if (info.nextptr == info.ptr)
	    return -1;
	  if (!strcmp (info.token, "SkidSteered"))
	    {
	      sw->data.groundvehicle.steertype = SW_STEER_SKID;
	    }
	  else if (!strcmp (info.token, "AckermanSteered"))
	    {
	      sw->data.groundvehicle.steertype = SW_STEER_ACKERMAN;
	    }
	  else if (!strcmp (info.token, "OmniDrive"))
	    {
	      sw->data.groundvehicle.steertype = SW_STEER_OMNI;
	    }
	  else
	    {
	      sw->data.groundvehicle.steertype = SW_STEER_UNKNOWN;
	      ROS_ERROR ("bad value for SteeringType: %s", info.token);
	    }
	  info.ptr = info.nextptr;
	  info.count++;
	}
      else if (!strcmp (info.token, "Mass"))
	{
	  sw->data.groundvehicle.mass = getReal (&info);
	}
      else if (!strcmp (info.token, "MaxSpeed"))
	{
	  sw->data.groundvehicle.max_speed = getReal (&info);
	}
      else if (!strcmp (info.token, "MaxTorque"))
	{
	  sw->data.groundvehicle.max_torque = getReal (&info);
	}
      else if (!strcmp (info.token, "MaxFrontSteer"))
	{
	  sw->data.groundvehicle.max_steer_angle = getReal (&info);
	}
      else
	{
	  /* skip MaxRearSteer, other unknown entries  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidConf (1);
  info.op = SW_ROBOT_SET;
  msgout (sw, info);
  return info.count;
}

/*
  CONF {Type BaseMachine} {Name FactoryControlBot}
*/
int
UsarsimInf::handleConfBasemachine (char *msg)
{
  componentInfo info;
  sw_struct *sw = robot->getSW ();

  setComponentInfo (msg, &info);
  sw->type = SW_ROBOT_FIXED;

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "BaseMachine");
	  robot->setDidConf (1);
	  info.ptr = info.nextptr;
	}
      else if (!strcmp (info.token, "Name"))
	{
	  info.nextptr = getValue (info.ptr, info.token);
	  if (info.nextptr == info.ptr)
	    return -1;
	  sw->name = std::string (info.token);
	  info.ptr = info.nextptr;
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidConf (1);
  info.op = SW_DEVICE_SET;
  msgout (sw, info);
  return info.count;
}

/*
  CONF {Type StaticPlatform} {Name FactoryControlBot}
*/
int
UsarsimInf::handleConfStaticplatform (char *msg)
{
  componentInfo info;
  sw_struct *sw = robot->getSW ();

  setComponentInfo (msg, &info);
  sw->type = SW_ROBOT_FIXED;

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "StaticPlatform");
	  robot->setDidConf (1);
	  info.ptr = info.nextptr;
	}
      else if (!strcmp (info.token, "Name"))
	{
	  info.nextptr = getValue (info.ptr, info.token);
	  if (info.nextptr == info.ptr)
	    return -1;
	  sw->name = std::string (info.token);
	  info.ptr = info.nextptr;
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidConf (1);
  info.op = SW_DEVICE_SET;
  msgout (sw, info);
  return info.count;
}

int UsarsimInf::handleConfObjectsensor(char *msg)
{
	componentInfo info;
  sw_struct *sw = objectsensors->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "ObjectSensor");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (objectsensors, &info, SW_SEN_OBJECTSENSOR_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	}
	  else if (!strcmp (info.token, "Fov"))
	{
	  sw->data.objectsensor.fov = getReal (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidConf (1);
  info.op = SW_SEN_OBJECTSENSOR_SET;
  msgout (sw, info);
  return info.count;
}

int
UsarsimInf::handleConf (char *msg)
{
  char token[MAX_TOKEN_LEN];
  char *ptr = msg;
  char *nextptr;
  int count = 0;

  waitingForConf = 0;
  ROS_DEBUG ("waitingForConf cleared");
  while (1)
    {
      nextptr = getKey (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      /* look for {Type <name>}, and pass the whole msg to the sensor */
      if (!strcmp (token, "Type"))
	{
	  nextptr = getValue (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if (!strcmp (token, "Sonar"))
	    {
	      return handleConfSonar (msg);
	    }
	  else if (!strcmp (token, "RangeScanner"))
	    {
	      return handleConfRangescanner (msg);
	    }
	  else if (!strcmp (token, "RangeImager"))
	    {
	      return handleConfRangeimager (msg);
	    }
	  else if (!strcmp (token, "Encoder"))
	    {
	      return handleConfEncoder (msg);
	    }
	  else if (!strcmp (token, "Touch"))
	    {
	      return handleConfTouch (msg);
	    }
	  else if (!strcmp (token, "CO2Sensor"))
	    {
	      return handleConfCo2sensor (msg);
	    }
	  else if (!strcmp (token, "GroundTruth"))
	    {
	      return handleConfIns (msg, "GroundTruth");
	    }
	  else if (!strcmp (token, "GPS"))
	    {
	      return handleConfGps (msg);
	    }
	  else if (!strcmp (token, "INS"))
	    {
	      return handleConfIns (msg, "INS");
	    }
	  else if (!strcmp (token, "Odometry"))
	    {
	      return handleConfOdometry (msg);
	    }
	  else if (!strcmp (token, "Tachometer"))
	    {
	      return handleConfTachometer (msg);
	    }
	  else if (!strcmp (token, "Acoustic"))
	    {
	      return handleConfAcoustic (msg);
	    }
	  else if (!strcmp (token, "VictSensor"))
	    {
	      return handleConfVictim (msg);
	    }
	  else if (!strcmp (token, "Gripper"))
	    {
	      return handleConfGripper (msg);
	    }
	  else if (!strcmp (token, "ToolChanger"))
	  {
	  	return handleConfToolchanger (msg);
	  }
	  else if (!strcmp (token, "Actuator"))
	    {
	      return handleConfActuator (msg);
	    }
	  else if (!strcmp (token, "GroundVehicle"))
	    {
	      return handleConfGroundvehicle (msg);
	    }
	  else if (!strcmp (token, "BaseMachine"))
	    {
	      return handleConfBasemachine (msg);
	    }
	  else if (!strcmp (token, "StaticPlatform"))
	    {
	      return handleConfStaticplatform (msg);
	    }
	  else if (!strcmp(token, "ObjectSensor"))
	    {
	      return handleConfObjectsensor(msg);
	    }
	  else
	    {
	      ROS_ERROR ("Unknown conf type %s", token);
	      /* skip it and keep going */
	    }
	}
    }

  return count;
}

/*
  GEO {Type StaticPlatform} {Name FactoryControlBot} {Dimensions 0.0000,0.0000,0.0000}
*/
int
UsarsimInf::handleGeoStaticplatform (char *msg)
{
  componentInfo info;
  sw_struct *sw = robot->getSW ();

  setComponentInfo (msg, &info);
  sw->type = SW_ROBOT_FIXED;
  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "StaticPlatform");
	  robot->setDidGeo (1);
	  info.ptr = info.nextptr;
	}
      else if (!strcmp (info.token, "Name"))
	{
	  info.nextptr = getValue (info.ptr, info.token);
	  if (info.nextptr == info.ptr)
	    return -1;
	  sw->name = std::string (info.token);
	  info.ptr = info.nextptr;
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidGeo (1);
  info.op = SW_DEVICE_SET;
  msgout (sw, info);
  return info.count;
}

/*
  GEO {Type Encoder} {Name ECRight Location 0.0000,0.0000,0.0000 Orientation 0.0000,0.0000,1.5707 Mount RightFWheel} < {Name ...} >
*/
int
UsarsimInf::handleGeoEncoder (char *msg)
{
  componentInfo info;
  sw_struct *sw = encoders->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Encoder");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (encoders, &info, SW_SEN_ENCODER_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	  expect (&info, "Location");
	  sw->data.encoder.mount.x = getReal (&info);
	  sw->data.encoder.mount.y = getReal (&info);
	  sw->data.encoder.mount.z = getReal (&info);
	  expect (&info, "Orientation");
	  sw->data.encoder.mount.roll = getReal (&info);
	  sw->data.encoder.mount.pitch = getReal (&info);
	  sw->data.encoder.mount.yaw = getReal (&info);
	  expect (&info, "Mount");
	  info.nextptr = getValue (info.ptr, info.token);
	  ulapi_strncpy (sw->data.encoder.mount.offsetFrom, info.token,
			 SW_NAME_MAX);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidGeo (1);
  info.op = SW_SEN_ENCODER_SET;
  msgout (sw, info);
  return info.count;
}

/*
  GEO {Type Sonar} {Name F2 Location 0.1850,-0.1150,0.0000 Orientation 0.0000,0.0000,-0.8727 Mount HARD}

  Why didn't the original USARSim designers use the same tag convention
  as for the CONF message?
*/

int
UsarsimInf::handleGeoSonar (char *msg)
{
  componentInfo info;
  sw_struct *sw = sonars->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Sonar");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (sonars, &info, SW_SEN_SONAR_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	  expect (&info, "Location");
	  sw->data.sonar.mount.x = getReal (&info);
	  sw->data.sonar.mount.y = getReal (&info);
	  sw->data.sonar.mount.z = getReal (&info);
	  expect (&info, "Orientation");
	  sw->data.sonar.mount.roll = getReal (&info);
	  sw->data.sonar.mount.pitch = getReal (&info);
	  sw->data.sonar.mount.yaw = getReal (&info);
	  expect (&info, "Mount");
	  info.nextptr = getValue (info.ptr, info.token);
	  ulapi_strncpy (sw->data.sonar.mount.offsetFrom, info.token,
			 SW_NAME_MAX);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidGeo (1);
  info.op = SW_SEN_SONAR_SET;
  msgout (sw, info);
  return info.count;
}

int
UsarsimInf::handleGeoTouch (char *msg)
{
  static int didit = 0;

  if (!didit)
    {
      ROS_WARN ("not handling %s", msg);
      didit = 1;
    }
  return 0;
}

int
UsarsimInf::handleGeoCo2sensor (char *msg)
{
  static int didit = 0;

  if (!didit)
    {
      ROS_WARN ("not handling %s", msg);
      didit = 1;
    }
  return 0;
}

/*
  GEO {Type GroundTruth} {Name GroundTruth Location 0.0000,0.0000,0.0000 Orientation 0.0000,0.0000,0.0000 Mount HARD}
*/
/*
int
UsarsimInf::handleGeoGroundtruth (char *msg)
{
  componentInfo info;
  sw_struct *sw = groundtruths->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "GroundTruth");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (groundtruths, &info, SW_SEN_GROUNDTRUTH_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	  expect (&info, "Location");
	  sw->data.groundtruth.mount.x = getReal (&info);
	  sw->data.groundtruth.mount.y = getReal (&info);
	  sw->data.groundtruth.mount.z = getReal (&info);
	  expect (&info, "Orientation");
	  sw->data.groundtruth.mount.roll = getReal (&info);
	  sw->data.groundtruth.mount.pitch = getReal (&info);
	  sw->data.groundtruth.mount.yaw = getReal (&info);
	  expect (&info, "Mount");
	  info.nextptr = getValue (info.ptr, info.token);
	  ulapi_strncpy (sw->data.groundtruth.mount.offsetFrom, info.token,
			 SW_NAME_MAX);
	}
      else
	{
	  // skip unknown entry 
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.op = SW_SEN_GROUNDTRUTH_SET;
  info.where->setDidGeo (1);
  msgout (sw, info);
  return info.count;
}
*/
/*
  GEO {Type GPS} {Name GPS Location 0.0000,0.0000,0.0000 Orientation 0.0000,-0.0000,0.0000 Mount HARD}
*/
int
UsarsimInf::handleGeoGps (char *msg)
{
  componentInfo info;
  sw_struct *sw = gpses->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "GPS");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (gpses, &info, SW_SEN_GPS_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	  expect (&info, "Location");
	  sw->data.gps.mount.x = getReal (&info);
	  sw->data.gps.mount.y = getReal (&info);
	  sw->data.gps.mount.z = getReal (&info);
	  expect (&info, "Orientation");
	  sw->data.gps.mount.roll = getReal (&info);
	  sw->data.gps.mount.pitch = getReal (&info);
	  sw->data.gps.mount.yaw = getReal (&info);
	  expect (&info, "Mount");
	  info.nextptr = getValue (info.ptr, info.token);
	  ulapi_strncpy (sw->data.gps.mount.offsetFrom, info.token,
			 SW_NAME_MAX);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidGeo (1);
  info.op = SW_SEN_GPS_SET;
  msgout (sw, info);
  return info.count;
}

/*
  GEO {Type INS} {Name INS Location 0.0000,0.0000,0.0000 Orientation 0.0000,-0.0000,0.0000 Mount HARD}
*/
int
UsarsimInf::handleGeoIns (char *msg, const char *sensorType)
{
  componentInfo info;
  sw_struct *sw;
  UsarsimList *myList;

  setComponentInfo (msg, &info);
  if( !strcmp(sensorType, "INS" ))
    myList = inses;
  else if ( !strcmp(sensorType, "GroundTruth" ) )
    myList = groundtruths;
  else
    {
      ROS_ERROR ("Unknown Ins type: %s", sensorType );
      return info.count;
    }

  sw = myList->getSW ();
  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, sensorType);
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (myList, &info, SW_SEN_INS_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	  expect (&info, "Location");
	  sw->data.ins.mount.x = getReal (&info);
	  sw->data.ins.mount.y = getReal (&info);
	  sw->data.ins.mount.z = getReal (&info);
	  expect (&info, "Orientation");
	  sw->data.ins.mount.roll = getReal (&info);
	  sw->data.ins.mount.pitch = getReal (&info);
	  sw->data.ins.mount.yaw = getReal (&info);
	  expect (&info, "Mount");
	  info.nextptr = getValue (info.ptr, info.token);
	  ulapi_strncpy (sw->data.ins.mount.offsetFrom, info.token,
			 SW_NAME_MAX);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidGeo (1);
  info.op = SW_SEN_INS_SET;
  msgout (sw, info);
  return info.count;
}

/*
  GEO {Type Odometry} {Name Odometry 
  Location 0.0000,0.0000,0.0000 Orientation 0.0000,-0.0000,0.0000 Mount HARD}
*/
int
UsarsimInf::handleGeoOdometry (char *msg)
{
  componentInfo info;
  sw_struct *sw = odometers->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Odometry");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (odometers, &info, SW_SEN_ODOMETER_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	  expect (&info, "Location");
	  sw->data.odometer.mount.x = getReal (&info);
	  sw->data.odometer.mount.y = getReal (&info);
	  sw->data.odometer.mount.z = getReal (&info);
	  expect (&info, "Orientation");
	  sw->data.odometer.mount.roll = getReal (&info);
	  sw->data.odometer.mount.pitch = getReal (&info);
	  sw->data.odometer.mount.yaw = getReal (&info);
	  expect (&info, "Mount");
	  info.nextptr = getValue (info.ptr, info.token);
	  ulapi_strncpy (sw->data.odometer.mount.offsetFrom, info.token,
			 SW_NAME_MAX);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidGeo (1);
  info.op = SW_SEN_ODOMETER_SET;
  msgout (sw, info);
  return info.count;
}

/*
  GEO {Type Tachometer} {Name TachTest Location 0.0000,0.0000,0.0000 Orientation 0.0000,0.0000,0.0000 Mount USARbot.P3AT}
*/
int
UsarsimInf::handleGeoTachometer (char *msg)
{
  componentInfo info;
  sw_struct *sw = tachometers->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Tachometer");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (tachometers, &info, SW_SEN_TACHOMETER_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	  expect (&info, "Location");
	  sw->data.tachometer.mount.x = getReal (&info);
	  sw->data.tachometer.mount.y = getReal (&info);
	  sw->data.tachometer.mount.z = getReal (&info);
	  expect (&info, "Orientation");
	  sw->data.tachometer.mount.roll = getReal (&info);
	  sw->data.tachometer.mount.pitch = getReal (&info);
	  sw->data.tachometer.mount.yaw = getReal (&info);
	  expect (&info, "Mount");
	  info.nextptr = getValue (info.ptr, info.token);
	  ulapi_strncpy (sw->data.tachometer.mount.offsetFrom, info.token,
			 SW_NAME_MAX);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidGeo (1);
  info.op = SW_SEN_TACHOMETER_SET;
  msgout (sw, info);
  return info.count;
}

/*
  GEO {Type Acoustic} {Name Test Location 0.0000,0.0000,0.0000 Orientation 0.0000,0.0000,0.0000 Mount USARbot.P3AT}
*/
int
UsarsimInf::handleGeoAcoustic (char *msg)
{
  componentInfo info;
  sw_struct *sw = acoustics->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Acoustic");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (acoustics, &info, SW_SEN_ACOUSTIC_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	  expect (&info, "Location");
	  sw->data.acoustic.mount.x = getReal (&info);
	  sw->data.acoustic.mount.y = getReal (&info);
	  sw->data.acoustic.mount.z = getReal (&info);
	  expect (&info, "Orientation");
	  sw->data.acoustic.mount.roll = getReal (&info);
	  sw->data.acoustic.mount.pitch = getReal (&info);
	  sw->data.acoustic.mount.yaw = getReal (&info);
	  expect (&info, "Mount");
	  info.nextptr = getValue (info.ptr, info.token);
	  ulapi_strncpy (sw->data.acoustic.mount.offsetFrom, info.token,
			 SW_NAME_MAX);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidGeo (1);
  info.op = SW_SEN_ACOUSTIC_SET;
  msgout (sw, info);
  return info.count;
}

/*
  GEO {Type VictSensor} {Name VictSensor Location 0.0600,0.0000,-0.0087 Orientation 0.0000,0.0000,0.0000 Mount CameraPanTilt_Link2}
*/
int
UsarsimInf::handleGeoVictim (char *msg)
{
  componentInfo info;
  sw_struct *sw = victims->getSW ();

  setComponentInfo (msg, &info);

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "VictSensor");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (victims, &info, SW_SEN_VICTIM_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	  expect (&info, "Location");
	  sw->data.victim.mount.x = getReal (&info);
	  sw->data.victim.mount.y = getReal (&info);
	  sw->data.victim.mount.z = getReal (&info);
	  expect (&info, "Orientation");
	  sw->data.victim.mount.roll = getReal (&info);
	  sw->data.victim.mount.pitch = getReal (&info);
	  sw->data.victim.mount.yaw = getReal (&info);
	  expect (&info, "Mount");
	  info.nextptr = getValue (info.ptr, info.token);
	  ulapi_strncpy (sw->data.victim.mount.offsetFrom, info.token,
			 SW_NAME_MAX);

	  if (info.nextptr == info.ptr)
	    return -1;
	  strncpy (sw->data.victim.parent, info.token,
		   sizeof (sw->data.victim.parent));
	  NULLTERM (sw->data.victim.parent);
	  info.count++;
	  info.ptr = info.nextptr;
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.where->setDidGeo (1);
  info.op = SW_SEN_VICTIM_SET;
  msgout (sw, info);
  return info.count;
}

//method to handle many different GEO messages (with some exceptions)
/*
GEO {Type componentName} {Name <name>} {Location 0.0600,0.0000,-0.0087} {Orientation 0.0000,0.0000,0.0000} {Mount Parent} {Link 5}
*/
int UsarsimInf::handleGeoComponent(const char* componentName, char* msg, sw_pose &mount, UsarsimList *list, int opcode)
{
	componentInfo info;
    setComponentInfo (msg, &info);
    sw_struct *sw = list->getSW();
  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, componentName);
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (list, &info, opcode);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	  mount.linkOffset = -1;
	  expect (&info, "Location");
	  mount.x = getReal (&info);
	  mount.y = getReal (&info);
	  mount.z = getReal (&info);
	  expect (&info, "Orientation");
	  mount.roll = getReal (&info);
	  mount.pitch = getReal (&info);
	  mount.yaw = getReal (&info);
	  expect (&info, "Mount");
	  info.nextptr = getValue (info.ptr, info.token);
	  ulapi_strncpy (mount.offsetFrom, info.token,
			 SW_NAME_MAX);
	  info.count++;
	  info.ptr = info.nextptr;
	}
	  else if(!strcmp(info.token, "MountLink"))
	{
	   info.nextptr = getValue (info.ptr, info.token);
	   if (info.nextptr == info.ptr)
	    return -1;
	   mount.linkOffset = getReal(&info); 
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }
  info.where->setDidGeo (1);
  info.op = opcode;
  msgout (sw, info);
  return info.count;
}

/*
  GEO {Type Actuator} {Name TeleMaxArm} {Link 1} {Parent -1} {Location 0.1789,-0.0014,-0.0905} {Orientation 3.1415,0.0000,0.0000} {Link 2} {ParentLink 1} {Location 0.0258,-0.0720,0.1566} {Orientation 1.5707,0.0000,0.0000} ...

  where the location and orientation of each link are with respect to
  the base frame of the mission package at the current location. This
  means that when the arm moves, these values will change. However, we
  compute the mounting as the transform from the link to its parent,
  which won't change.
*/
int
UsarsimInf::handleGeoActuator (char *msg)
{
  componentInfo info;
  int i;
  int linkindex;
  sw_struct *sw = misstas->getSW();

  setComponentInfo( msg, &info );
  linkindex = 0;
  
  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Actuator");
	}
      else if (!strcmp (info.token, "Name"))
	{ 
	  getName (misstas, &info, SW_ACT_SET);
	  sw = info.where->getSW();
	  info.where->setDidConf(1);
	  sw->data.actuator.number = 0;
	  sw->data.actuator.mount.linkOffset = -1;
	  expect (&info, "Location");
	  sw->data.actuator.mount.x = getReal (&info);
	  sw->data.actuator.mount.y = getReal (&info);
	  sw->data.actuator.mount.z = getReal (&info);

	  expect (&info, "Orientation");
	  sw->data.actuator.mount.roll = getReal (&info);
	  sw->data.actuator.mount.pitch = getReal (&info);
	  sw->data.actuator.mount.yaw = getReal (&info);
	  expect (&info, "Mount");
	  info.nextptr = getValue (info.ptr, info.token);
	  ulapi_strncpy (sw->data.actuator.mount.offsetFrom, info.token,
			 SW_NAME_MAX);
	}
      else if (!strcmp (info.token, "Link"))
	{
	  // expecting number 
	  info.nextptr = getValue (info.ptr, info.token);
	  if (info.nextptr == info.ptr)
	    {
	      ROS_ERROR( "Missing link number for mispackage message" );
	      return -1;
	    }
	  if (sscanf (info.token, "%i", &i) != 1)
	    {
	      ROS_ERROR( "Missing link number for mispackage message" );
	      return -1;
	    }
	  if (i < 1)
	    {
	      ROS_ERROR ("invalid link number for %s: %d", sw->name.c_str(),
			 i);
	      return -1;
	    }
	  else if (i > SW_ACT_LINK_MAX)
	    {
	      ROS_ERROR ("invalid link number for %s: %d (greater than max %d)", sw->name.c_str(),
			 i, SW_ACT_LINK_MAX);
	      return -1;
	    }
	  linkindex = i - 1; // links start at 1, arrays at 0
	  if (i > sw->data.actuator.number) //more links!
	    sw->data.actuator.number = i;
	  info.count++;
	  info.ptr = info.nextptr;
	}
      else if (!strcmp (info.token, "Parent"))
	{
	  // expecting number 
	  info.nextptr = getValue (info.ptr, info.token);
	  if (info.nextptr == info.ptr)
	    {
	      ROS_ERROR( "Missing parent link number for mispackage message" );
	      return -1;
	    }
	  if (sscanf (info.token, "%i", &i)!= 1)
	    {
	      ROS_ERROR( "Missing parent link number for mispackage message" );
	      return -1;
	    }
	  if (i < -1)
	    {
	      ROS_ERROR ("invalid parent link number for %s: %d", sw->name.c_str(),
			 i);
	      return -1;
	    }
	  else if (i > SW_ACT_LINK_MAX)
	    {
	      ROS_ERROR ("invalid parent link number for %s: %d (greater than max %d)" , sw->name.c_str(),
			 i, SW_ACT_LINK_MAX);
	      return -1;
	    }
	  // change convention of base at -1 to base at 0 
	  //	  if (i < 0)
	  //	    i = 0;
	  sw->data.actuator.link[linkindex].parent = i;
	  info.count++;
	  info.ptr = info.nextptr;
	}
      else if (!strcmp (info.token, "Location"))
	{
	  sw->data.actuator.link[linkindex].mount.x = getReal(&info); 
	  sw->data.actuator.link[linkindex].mount.y = getReal(&info); 
	  sw->data.actuator.link[linkindex].mount.z = getReal(&info); 
	}
      else if (!strcmp (info.token, "Orientation"))
	{
	  sw->data.actuator.link[linkindex].mount.roll = getReal(&info); 
	  sw->data.actuator.link[linkindex].mount.pitch = getReal(&info); 
	  sw->data.actuator.link[linkindex].mount.yaw = getReal(&info); 
	}
	else if (!strcmp (info.token, "MountLink"))
	{
	   sw->data.actuator.mount.linkOffset = getReal(&info);
	   ROS_ERROR("Mountlink for %s is %d", sw->name.c_str(), sw->data.actuator.mount.linkOffset);
	}
	else if (!strcmp (info.token, "Tip"))
	{
	  sw->data.actuator.tip.x = getReal(&info); 
	  sw->data.actuator.tip.y = getReal(&info); 
	  sw->data.actuator.tip.z = getReal(&info); 
	}
      else
	{
	  // skip unknown entry 
	  ROS_WARN ("Unknown entry in ACTUATOR: %s", info.ptr );
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  // as with handle_conf_actuator, we inhibit telling of blank messages 
  if (info.count > 0)
    {
      info.op = SW_ACT_SET;
      msgout (sw, info );
    }
  info.where->setDidGeo (1);
  return info.count;
}

int UsarsimInf::handleGeoGripper (char *msg)
{
	componentInfo info;
    setComponentInfo (msg, &info);
    sw_struct *sw = grippers->getSW();
  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "Gripper");
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (grippers, &info, SW_EFF_GRIPPER_SET);
	  sw = info.where->getSW ();
	  info.where->setDidConf (1);
	  sw->data.gripper.mount.linkOffset = -1;
	  expect (&info, "Location");
	  sw->data.gripper.mount.x = getReal (&info);
	  sw->data.gripper.mount.y = getReal (&info);
	  sw->data.gripper.mount.z = getReal (&info);
	  expect (&info, "Orientation");
	  sw->data.gripper.mount.roll = getReal (&info);
	  sw->data.gripper.mount.pitch = getReal (&info);
	  sw->data.gripper.mount.yaw = getReal (&info);
	  expect (&info, "Mount");
	  info.nextptr = getValue (info.ptr, info.token);
	  ulapi_strncpy (sw->data.gripper.mount.offsetFrom, info.token,
			 SW_NAME_MAX);
	  info.count++;
	  info.ptr = info.nextptr;
	}
	  else if(!strcmp(info.token, "MountLink"))
	{
	   info.nextptr = getValue (info.ptr, info.token);
	   if (info.nextptr == info.ptr)
	    return -1;
	   sw->data.gripper.mount.linkOffset = getReal(&info); 
	}
	  else if(!strcmp(info.token, "Tip"))
	{
	  //adjust position to be at the tip of the effector instead of the base
	  sw->data.gripper.tip.x = getReal(&info);
	  sw->data.gripper.tip.y = getReal(&info);
	  sw->data.gripper.tip.z = getReal(&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }
  info.where->setDidGeo (1);
  info.op = SW_EFF_GRIPPER_SET;
  msgout (sw, info);
  return info.count;
}


/*
  GEO {Type GroundVehicle}

  {Name P2AT} {Dimensions 0.5238,0.4968,0.2913} {COG 0.0000,0.0000,0.0000} {WheelRadius 0.1300} {WheelSeparation 0.4712} {WheelBase 0.2884}
*/
int
UsarsimInf::handleGeoGroundvehicle (char *msg)
{
  componentInfo info;
  sw_struct *sw = robot->getSW ();

  setComponentInfo (msg, &info);

  sw->type = SW_ROBOT_GROUNDVEHICLE;

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "GroundVehicle");
	  robot->setDidGeo (1);
	  info.ptr = info.nextptr;
	}
      else if (!strcmp (info.token, "Name"))
	{
	  info.nextptr = getValue (info.ptr, info.token);
	  if (info.nextptr == info.ptr)
	    return -1;
	  sw->name = std::string (info.token);
	  info.ptr = info.nextptr;
	}
      else if (!strcmp (info.token, "Dimensions"))
	{
	  sw->data.groundvehicle.length = getReal (&info);
	  sw->data.groundvehicle.width = getReal (&info);
	  sw->data.groundvehicle.height = getReal (&info);
	}
      else if (!strcmp (info.token, "COG"))
	{
	  sw->data.groundvehicle.cg.roll = 0;
	  sw->data.groundvehicle.cg.pitch = 0;
	  sw->data.groundvehicle.cg.yaw = 0;
	  sw->data.groundvehicle.cg.x = getReal (&info);
	  sw->data.groundvehicle.cg.y = getReal (&info);
	  sw->data.groundvehicle.cg.z = getReal (&info);
	}
      else if (!strcmp (info.token, "WheelRadius"))
	{
	  sw->data.groundvehicle.wheel_radius = getReal (&info);
	}
      else if (!strcmp (info.token, "WheelSeparation"))
	{
	  sw->data.groundvehicle.wheel_separation = getReal (&info);
	}
      else if (!strcmp (info.token, "WheelBase"))
	{
	  sw->data.groundvehicle.wheel_base = getReal (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.op = SW_ROBOT_SET;
  info.where->setDidGeo (1);
  msgout (sw, info);
  return info.count;
}

/*
  GEO {Type BaseMachine} {Name FactoryControlBot} {Dimensions 0.0000,0.0000,0.0000}
*/
int
UsarsimInf::handleGeoBasemachine (char *msg)
{
  componentInfo info;
  sw_struct *sw = robot->getSW ();

  setComponentInfo (msg, &info);

  sw->type = SW_ROBOT_FIXED;

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Type"))
	{
	  expect (&info, "BaseMachine");
	  robot->setDidGeo (1);
	  info.ptr = info.nextptr;
	}
      else if (!strcmp (info.token, "Name"))
	{
	  info.nextptr = getValue (info.ptr, info.token);
	  if (info.nextptr == info.ptr)
	    return -1;
	  sw->name = std::string (info.token);
	  info.ptr = info.nextptr;
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }

  info.op = SW_DEVICE_SET;
  info.where->setDidGeo (1);
  msgout (sw, info);
  return info.count;
}

int
UsarsimInf::handleGeo (char *msg)
{
  char token[MAX_TOKEN_LEN];
  char *ptr = msg;
  char *nextptr;
  sw_struct *sw;
  waitingForGeo = 0;
  ROS_DEBUG ("waitingForGeo cleared");
  while (1)
    {
      nextptr = getKey (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      /* look for {Type <name>}, and pass the whole msg to the sensor */
      if (!strcmp (token, "Type"))
	{
	  nextptr = getValue (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if (!strcmp (token, "Sonar"))
	    {
	      return handleGeoSonar (msg);
	    }
	  else if (!strcmp (token, "RangeScanner"))
	    {
	      sw = rangescanners->getSW();
	      return handleGeoComponent("RangeScanner", msg, sw->data.rangescanner.mount, rangescanners, SW_SEN_RANGESCANNER_SET);
	    }
	  else if (!strcmp (token, "RangeImager"))
	    {
	      sw = rangeimagers->getSW();
	      return handleGeoComponent("RangeImager", msg, sw->data.rangeimager.mount, rangeimagers, SW_SEN_RANGEIMAGER_SET);
	    }
	  else if (!strcmp (token, "Encoder"))
	    {
	      return handleGeoEncoder (msg);
	    }
	  else if (!strcmp (token, "Touch"))
	    {
	      return handleGeoTouch (msg);
	    }
	  else if (!strcmp (token, "CO2Sensor"))
	    {
	      return handleGeoCo2sensor (msg);
	    }
	  else if (!strcmp (token, "GroundTruth"))
	    {
	      return handleGeoIns (msg, "GroundTruth");
	    }
	  else if (!strcmp (token, "GPS"))
	    {
	      return handleGeoGps (msg);
	    }
	  else if (!strcmp (token, "INS"))
	    {
	      return handleGeoIns (msg, "INS");
	    }
	  else if (!strcmp (token, "Odometry"))
	    {
	      return handleGeoOdometry (msg);
	    }
	  else if (!strcmp (token, "Tachometer"))
	    {
	      return handleGeoTachometer (msg);
	    }
	  else if (!strcmp (token, "Acoustic"))
	    {
	      return handleGeoAcoustic (msg);
	    }
	  else if (!strcmp (token, "VictSensor"))
	    {
	      return handleGeoVictim (msg);
	    }
	  else if (!strcmp (token, "Gripper"))
	    {
	      return handleGeoGripper(msg);
	    }
	  else if (!strcmp (token, "ToolChanger"))
	    {
	      sw = toolchangers->getSW();
	      return handleGeoComponent("ToolChanger", msg, sw->data.toolchanger.mount, toolchangers, SW_EFF_TOOLCHANGER_SET);
	    }
	  else if (!strcmp (token, "Actuator"))
	    {
	      return handleGeoActuator (msg);
	    }
	  else if (!strcmp (token, "GroundVehicle"))
	    {
	      return handleGeoGroundvehicle (msg);
	    }
	  else if (!strcmp (token, "BaseMachine"))
	    {
	      return handleGeoBasemachine (msg);
	    }
	  else if (!strcmp (token, "StaticPlatform"))
	    {
	      return handleGeoStaticplatform (msg);
	    }
	  else if(!strcmp(token, "ObjectSensor"))
	    {
	      sw = objectsensors->getSW();
	      return handleGeoComponent("ObjectSensor", msg, sw->data.objectsensor.mount, objectsensors, SW_SEN_OBJECTSENSOR_SET);
	    }
	  else
	    {
	      ROS_ERROR ("Unknown geo type %s", token);
	      /* skip it and keep going */
	    }
	}
    }

  return 0;
}

/*
  ASTA {Time 5609.45} {Name CameraPanTilt} {Link 1} {Value 0.0000} {Torque -20.00} {Link 2} {Value 0.0000} {Torque -20.00}

  ASTA {Time 48.22} {Name TeleMaxArm} {Link 1} {Value 0.0000} {Torque -300.00} {Link 2} {Value -0.0000} {Torque -300.00} {Link 3} {Value 0.0002} {Torque -300.00} {Link 4} {Value 0.0000} {Torque -300.00} {Link 5} {Value 0.0000} {Torque -300.00} {Link 6} {Value 0.0000} {Torque -300.00} {Link 7} {Value 0.0000} {Torque -300.00} {Link 8} {Value 0.0000} {Torque -300.00} {Link 9} {Value 0.0000} {Torque -300.00} {Link 10} {Value 0.0000} {Torque -300.00} {Link 11} {Value 0.0000} {Torque -300.00}
*/

int
UsarsimInf::handleAsta (char *msg)
{
  componentInfo info;
  int i;
  int linkindex = 0;

  sw_struct *sw = misstas->getSW();
  setComponentInfo( msg, &info );
  //  ROS_ERROR( "%s", msg );

  while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
	break;
      info.ptr = info.nextptr;

      if (!strcmp (info.token, "Time"))
	{
	  getTime(&info);
	}
      else if (!strcmp (info.token, "Name"))
	{
	  getName (misstas, &info, SW_ACT_STAT);
	  sw = info.where->getSW ();
	  //	  info.where->setDidConf (1);
	  sw->data.actuator.number = 0;
	}
      else if (!strcmp (info.token, "Link"))
	{
	  /* expecting number */
	  info.nextptr = getValue (info.ptr, info.token);
	  if (info.nextptr == info.ptr)
	    return -1;
	  if (sscanf (info.token, "%i", &i) != 1)
	    return -1;
	  if (i < 1)
	    {
	      ROS_ERROR ("invalid link number for %s: %d\n", sw->name.c_str(),
			 i);
	      i = 1;
	    }
	  else if (i > SW_ACT_LINK_MAX)
	    {
	      ROS_ERROR ("invalid link number for %s: %d\n", sw->name.c_str(),
			 i);
	      i = SW_ACT_LINK_MAX;
	    }
	  linkindex = i - 1;
	  sw->data.actuator.link[linkindex].torque = 0;
	  if (i > sw->data.actuator.number)
	    sw->data.actuator.number = i;
	  info.count++;
	  info.ptr = info.nextptr;
	}
      else if (!strcmp (info.token, "Value"))
	{
	  sw->data.actuator.link[linkindex].position = getReal (&info);
	}
      else if (!strcmp (info.token, "Torque"))
	{
	  sw->data.actuator.link[linkindex].torque = getReal (&info);
	}
      else
	{
	  /* skip unknown entry  */
	  info.nextptr = getValue (info.ptr, info.token);
	}
    }
  info.op = SW_ACT_STAT;
  msgout (sw, info);
  return info.count;
}
int UsarsimInf::handleEff(char *msg)
{
  char token[MAX_TOKEN_LEN];
  char *ptr = msg;
  char *nextptr;
  while (1)
    {
      nextptr = getKey (ptr, token);
      if (nextptr == ptr)
		break;
      ptr = nextptr;
      /* look for {Type <name>}, and pass the whole msg to the effector */
		if (!strcmp (token, "Type"))
		{
		  nextptr = getValue (ptr, token);
		  if (nextptr == ptr)
			return -1;
		  if (!strcmp (token, "Gripper"))
			{
			  return handleEffGripper (msg);
			}
		  else if (!strcmp (token, "ToolChanger"))
		    {
		      return handleEffToolchanger (msg);
		    }
		  else
			{
			  ROS_ERROR ("Unknown effector type %s", token);
			  /* skip it and keep going */
			}
		}
      /* else something else to be handled, probably {Time #} */
    }

  return 0;
}
int UsarsimInf::handleEffGripper(char *msg)
{
	componentInfo info;
	sw_struct *sw = grippers->getSW();

  	setComponentInfo (msg, &info);
  
  	while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
		break;
      info.ptr = info.nextptr;
      if (!strcmp (info.token, "Type"))
	  {
	  	expect (&info, "Gripper");
	  }
      else if (!strcmp (info.token, "Name"))
	  {
	  	getName (grippers, &info, SW_EFF_GRIPPER_STAT);
	  	sw = info.where->getSW ();
	  }
      else if (!strcmp (info.token, "Status"))
	  {
	  	info.nextptr = getValue (info.ptr, info.token);
	  	if (info.nextptr == info.ptr)
	    	return -1;
	    if(!ulapi_strcasecmp(info.token,"OPEN"))
	    	sw->data.gripper.status = SW_EFF_OPEN;
	    else if(!ulapi_strcasecmp(info.token, "CLOSED"))
	    	sw->data.gripper.status = SW_EFF_CLOSE;
	    else
	    {
	    	ROS_ERROR("Bad gripper status %s",info.token);
	    }
	  }
      else
	  {
		  // skip unknown entry  
		  info.nextptr = getValue (info.ptr, info.token);
	  }
    }
  info.op = SW_EFF_GRIPPER_STAT;
  msgout (sw, info);

  return 0;
}
int UsarsimInf::handleEffToolchanger(char *msg)
{
	componentInfo info;
	sw_struct *sw = toolchangers->getSW();
  	setComponentInfo (msg, &info);
  	while (1)
    {
      info.nextptr = getKey (info.ptr, info.token);
      if (info.nextptr == info.ptr)
		break;
      info.ptr = info.nextptr;
      if (!strcmp (info.token, "Type"))
	  {
	  	expect (&info, "ToolChanger");
	  }
      else if (!strcmp (info.token, "Name"))
	  {
	  	getName (toolchangers, &info, SW_EFF_TOOLCHANGER_STAT);
	  	sw = info.where->getSW ();
	  }
      else if (!strcmp (info.token, "Status"))
	  {
	  	info.nextptr = getValue (info.ptr, info.token);
	  	if (info.nextptr == info.ptr)
	    	return -1;
	    if(!ulapi_strcasecmp(info.token,"OPEN"))
	    	sw->data.toolchanger.status = SW_EFF_OPEN;
	    else if(!ulapi_strcasecmp(info.token, "CLOSED"))
	    	sw->data.toolchanger.status = SW_EFF_CLOSE;
	    else
	    {
	    	ROS_ERROR("Bad toolchanger status %s",info.token);
	    }
	  }
	  else if (!strcmp(info.token, "Tool"))
	  {
	  	info.nextptr = getValue (info.ptr, info.token);
	  	if (info.nextptr == info.ptr)
	    	return -1;
	    if(!strcmp(info.token, "Gripper"))
	    	sw->data.toolchanger.tooltype = SW_EFF_TOOLCHANGER_GRIPPER;
	    else if(!strcmp(info.token, "Vacuum"))
	    	sw->data.toolchanger.tooltype = SW_EFF_TOOLCHANGER_VACUUM;
	    else if(!strcmp(info.token, "ToolChanger"))
	    	sw->data.toolchanger.tooltype = SW_EFF_TOOLCHANGER_TOOLCHANGER;
	    else
	    	sw->data.toolchanger.tooltype = SW_EFF_TOOLCHANGER_UNKNOWN_TYPE;
	  }
      else
	  {
		  // skip unknown entry  
		  info.nextptr = getValue (info.ptr, info.token);
	  }
    }
  info.op = SW_EFF_TOOLCHANGER_STAT;
  msgout (sw, info);

  return 0;
}
