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
  \file servoMisJANode.cc

  \brief Mission package controller
*/

#include <stddef.h>
#include <rcs.hh>
#include "ulapi.h"
#include "skin.h"
#include "servoMisJA.hh"
#include "servoMisJANode.h"

static int
no_inf_tell (sw_struct * sw)
{
  return -1;
}

static void
cmdPassThr (void *arg)
{
  ServoMisJANode *node = reinterpret_cast < ServoMisJANode * >(arg);

  while (true)
    {
      node->cmdPass ();
      node->sleep ();
    }
}

static void
cmdReadThr (void *arg)
{
  ServoMisJANode *node = reinterpret_cast < ServoMisJANode * >(arg);

  while (true)
    {
      if (0 != node->cmdRead ())
	{
	  node->sleep ();
	}
    }
}

static void
cfgPassThr (void *arg)
{
  ServoMisJANode *node = reinterpret_cast < ServoMisJANode * >(arg);

  while (true)
    {
      node->cfgPass ();
      node->sleep ();
    }
}

static void
cfgReadThr (void *arg)
{
  ServoMisJANode *node = reinterpret_cast < ServoMisJANode * >(arg);

  while (true)
    {
      if (0 != node->cfgRead ())
	{
	  node->sleep ();
	}
    }
}

ServoMisJANode::ServoMisJANode (void)
{
  cmdBuf = NULL;
  statBuf = NULL;
  cfgBuf = NULL;
  setBuf = NULL;
  statMutex = NULL;
  setMutex = NULL;
  cmdReadTask = NULL;
  cmdPassTask = NULL;
  cfgReadTask = NULL;
  cfgPassTask = NULL;
  cycleTime = 0.1;

  number = 0;
  name[0] = 0;
  inf_tell = no_inf_tell;

  reset_time_tracker (&stat.tt);
}

ServoMisJANode::~ServoMisJANode (void)
{
  ulapi_task_stop (cmdReadTask);
  ulapi_task_stop (cmdPassTask);
  ulapi_task_stop (cfgReadTask);
  ulapi_task_stop (cfgPassTask);

  ulapi_task_delete (cmdReadTask);
  ulapi_task_delete (cmdPassTask);
  ulapi_task_delete (cfgReadTask);
  ulapi_task_delete (cfgPassTask);

  ulapi_mutex_delete (statMutex);
  ulapi_mutex_delete (setMutex);

  delete cmdBuf;
  delete statBuf;
  delete cfgBuf;
  delete setBuf;
}

int
ServoMisJANode::init (char *processName, char *nmlFile, int packageNumber,
		      int argMission, char *argName,
		      int (*itell) (sw_struct * sw))
{
  char bufferName[MOAST_NML_BUFFER_NAME_LEN];

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoMisJACmd%d_%d",
		  packageNumber, argMission);
  bufferName[sizeof (bufferName) - 1] = 0;
  cmdBuf =
    new RCS_CMD_CHANNEL (servoMisJA_format, bufferName, processName, nmlFile);
  if (NULL == cmdBuf || !cmdBuf->valid ())
    {
      return -1;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoMisJAStat%d_%d",
		  packageNumber, argMission);
  bufferName[sizeof (bufferName) - 1] = 0;
  statBuf =
    new RCS_STAT_CHANNEL (servoMisJA_format, bufferName, processName,
			  nmlFile);
  if (NULL == statBuf || !statBuf->valid ())
    {
      return -1;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoMisJACfg%d_%d",
		  packageNumber, argMission);
  bufferName[sizeof (bufferName) - 1] = 0;
  cfgBuf =
    new RCS_CMD_CHANNEL (servoMisJA_format, bufferName, processName, nmlFile);
  if (NULL == cfgBuf || !cfgBuf->valid ())
    {
      return -1;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoMisJASet%d_%d",
		  packageNumber, argMission);
  bufferName[sizeof (bufferName) - 1] = 0;
  setBuf =
    new RCS_STAT_CHANNEL (servoMisJA_format, bufferName, processName,
			  nmlFile);
  if (NULL == setBuf || !setBuf->valid ())
    {
      return -1;
    }

  statMutex = ulapi_mutex_new (101);	// FIXME
  setMutex = ulapi_mutex_new (102);

  strncpy (name, argName, sizeof (name));
  name[sizeof (name) - 1] = 0;
  number = packageNumber;
  inf_tell = itell;

#define START_THREAD(TASK,THREAD)				\
  if (NULL == TASK) {						\
    TASK = ulapi_task_new();					\
    if (NULL != TASK) {						\
      if (ULAPI_OK == ulapi_task_start(TASK,			\
				       THREAD,			\
				       this,			\
				       ulapi_prio_lowest(),	\
				       1)) {			\
      } else {							\
	ulapi_task_delete(TASK);				\
	TASK = NULL;						\
	fprintf(stderr, "can't start one of the superior threads\n");	\
      }								\
    } else {							\
      fprintf(stderr, "can't create one of the superior tasks\n");	\
    }								\
  }

  START_THREAD (cmdReadTask, cmdReadThr);
  START_THREAD (cmdPassTask, cmdPassThr);
  START_THREAD (cfgReadTask, cfgReadThr);
  START_THREAD (cfgPassTask, cfgPassThr);

  return 0;
}

char *
ServoMisJANode::getName (void)
{
  return name;
}


void
ServoMisJANode::takeStatMutex (void)
{
  ulapi_mutex_take (statMutex);
}

void
ServoMisJANode::giveStatMutex (void)
{
  ulapi_mutex_give (statMutex);
}

void
ServoMisJANode::takeSetMutex (void)
{
  ulapi_mutex_take (setMutex);
}

void
ServoMisJANode::giveSetMutex (void)
{
  ulapi_mutex_give (setMutex);
}

void
ServoMisJANode::sleep (void)
{
  esleep (cycleTime);
}

int
ServoMisJANode::cmdPass (void)
{
  ulapi_mutex_take (statMutex);

  switch (stat.command_type)
    {
    case 0:
      // no new command
      break;
    case -1:
      // comm error
      break;

    case SERVO_MIS_JA_CMD_INIT_TYPE:
      doCmdInit ();
      break;

    case SERVO_MIS_JA_CMD_MOVE_TYPE:
      doCmdMove ();
      break;
    }

  cycle_time_tracker (&stat.tt);
  statBuf->write (stat);

  ulapi_mutex_give (statMutex);

  return 0;
}

int
ServoMisJANode::cmdRead (void)
{
  NMLTYPE cmdType;
  int cmdSerialNumber;
  int retval = 0;

  if (NULL == cmdBuf)
    return -1;
  if (NULL == statBuf)
    return -1;

  cmdType = cmdBuf->blocking_read (-1);
  cmdSerialNumber = cmdBuf->get_address ()->serial_number;

  ulapi_mutex_take (statMutex);

  switch (cmdType)
    {
    case 0:
      // no new command
      break;
    case -1:
      // comm error
      retval = -1;
      break;

    case SERVO_MIS_JA_CMD_INIT_TYPE:
    case SERVO_MIS_JA_CMD_MOVE_TYPE:
      stat.command_type = cmdType;
      if (cmdSerialNumber != stat.echo_serial_number)
	{
	  stat.echo_serial_number = cmdSerialNumber;
	  stat.state = NEW_COMMAND;
	}
      break;

    default:
      fprintf (stderr, "unknown mis command %s\n",
	       servoMisJA_symbol_lookup (cmdType));
      break;
    }				// switch (cmdType)

  ulapi_mutex_give (statMutex);

  return cmdPass ();
}

int
ServoMisJANode::cfgPass (void)
{
  ulapi_mutex_take (setMutex);

  switch (set.command_type)
    {
    case SERVO_MIS_JA_CFG_CYCLE_TIME_TYPE:
      doCfgCycleTime ();
      break;
    }
  setBuf->write (set);

  ulapi_mutex_give (setMutex);

  return 0;
}

int
ServoMisJANode::cfgRead (void)
{
  NMLTYPE cfgType;
  int cfgSerialNumber;
  int retval = 0;

  if (NULL == cfgBuf)
    return -1;
  if (NULL == setBuf)
    return -1;

  cfgType = cfgBuf->blocking_read (-1);
  cfgSerialNumber = cfgBuf->get_address ()->serial_number;

  ulapi_mutex_take (setMutex);

  switch (cfgType)
    {
    case 0:
      // no new command
      break;
    case -1:
      // comm error
      retval = -1;
      break;

    case SERVO_MIS_JA_CFG_CYCLE_TIME_TYPE:
      set.command_type = cfgType;
      if (cfgSerialNumber != set.echo_serial_number)
	{
	  set.echo_serial_number = cfgSerialNumber;
	  set.state = NEW_COMMAND;
	}
      break;

    default:
      fprintf (stderr, "unknown mis config %s\n",
	       servoMisJA_symbol_lookup (cfgType));
      break;
    }				// switch (cfgType)

  ulapi_mutex_give (setMutex);

  return cfgPass ();
}

void
ServoMisJANode::doCmdInit (void)
{
  if (state_match (&stat, NEW_COMMAND))
    {
      state_new (&stat);
      stat.admin_state = ADMIN_INITIALIZED;
      status_next (&stat, RCS_DONE);
      state_next (&stat, S0);
    }
  else
    {				// S0
      state_default (&stat);
    }
}

void
ServoMisJANode::doCfgCycleTime (void)
{
  ServoMisJACfgCycleTime *msg;

  msg = reinterpret_cast < ServoMisJACfgCycleTime * >(cfgBuf->get_address ());

  if (state_match (&set, NEW_COMMAND))
    {
      set.cycleTime = msg->cycleTime;
      state_next (&set, S0);
      status_next (&set, RCS_DONE);
    }
  else
    {				// S0
      state_default (&set);
    }
}

/*
  The MOAST mission package move command can in principle specify a
  mix of position, speed and force/torque moves among all the links.
  Simware only allows for one type across all links. Here we'll just
  set the type to be the last one encountered.
*/
void
ServoMisJANode::doCmdMove (void)
{
  ServoMisJACmdMove *msg;
  sw_struct sw;
  int num;
  int t;
  bool done = true;
  double myTwoPi, myValue;

  msg = reinterpret_cast < ServoMisJACmdMove * >(cmdBuf->get_address ());

  if (state_match (&stat, NEW_COMMAND))
    {
      status_next (&stat, RCS_EXEC);
      state_new (&stat);
      stat.admin_state = ADMIN_INITIALIZED;
      sw_init (&sw);
      sw_set_type (&sw, SW_ACT);
      num = msg->linkCmd_length;
      if (num > SERVO_MIS_JA_LINK_MAX)
	num = SERVO_MIS_JA_LINK_MAX;
      if (num > SW_ACT_LINK_MAX)
	num = SW_ACT_LINK_MAX;
      if (num > 0)
	{
	  //      fprintf( stderr, "servoMisJANode move to joint:" );
	  for (t = 0; t < num; t++)
	    {
	      if (SERVO_MIS_JA_LINK_CMD_ABS_VALUE_TYPE ==
		  msg->linkCmd[t].type)
		{
		  //              printf( "Link %d is value type\n", t );
		  done = false;
		  sw_set_op (&sw, SW_ACT_POSITION);
		  // if max < min then no limits on joint. Else, check limits
		  if (set.linkSet[t].minRange < set.linkSet[t].maxRange &&
		      (msg->linkCmd[t].value < set.linkSet[t].minRange ||
		       msg->linkCmd[t].value > set.linkSet[t].maxRange))
		    {
		      //  printf( "servoMisJANode: Joint %d set point: %.2f out of limits <%.2f %.2f>\n",
		      //      t, msg->linkCmd[t].value, set.linkSet[t].minRange, set.linkSet[t].maxRange);
		      status_next (&stat, RCS_ERROR);
		      state_next (&stat, S0);
		      return;
		    }
		  sw.data.mispkg.link[t].position = msg->linkCmd[t].value;
		}
	      else if (SERVO_MIS_JA_LINK_CMD_VELOCITY_TYPE ==
		       msg->linkCmd[t].type)
		{
		  sw_set_op (&sw, SW_ACT_SPEED);
		  sw.data.mispkg.link[t].speed = msg->linkCmd[t].value;
		}
	      else if (SERVO_MIS_JA_LINK_CMD_TORQUE_TYPE ==
		       msg->linkCmd[t].type)
		{
		  sw_set_op (&sw, SW_ACT_TORQUE);
		  sw.data.mispkg.link[t].torque = msg->linkCmd[t].value;
		}
	      else
		{
		  status_next (&stat, RCS_ERROR);
		  state_next (&stat, S0);
		  return;
		}
	      //      fprintf(stderr, " %f", sw.data.mispkg.link[t].position);
	    }
	  //      fprintf(stderr, "\n" );
	  sw_set_name (&sw, name);
	  sw.data.mispkg.number = num;
	  inf_tell (&sw);
	}
      else
	{
	  fprintf (stderr,
		   "doCmdMove with nothing to move (%d joints, with max of %d)\n",
		   num, SERVO_MIS_JA_LINK_MAX);
	}
      if (!done)
	state_next (&stat, S1);
      else
	{
	  status_next (&stat, RCS_DONE);
	  state_next (&stat, S0);
	}
    }
  else if (state_match (&stat, S1))
    {
      myTwoPi = 2. * M_PI;
      num = msg->linkCmd_length;
      //      printf( "servoMisJANode status(m) for %d: ", num );
      for (t = 0; t < num; t++)
	{
	  if (SERVO_MIS_JA_LINK_CMD_ABS_VALUE_TYPE == msg->linkCmd[t].type)
	    {
	      //              printf( "%f ", stat.linkStat[t].jointVal );
	      /*
	         // need to set values to be 0-2PI
	         msg->linkCmd[t].value = fmod(msg->linkCmd[t].value, myTwoPi);
	         if( msg->linkCmd[t].value < 0 )
	         {
	         printf( "fixing the value of linkCmd for joint %d\n", t );
	         msg->linkCmd[t].value += myTwoPi;
	         }
	       */
	      //              stat.linkStat[t].jointVal = fmod(stat.linkStat[t].jointVal, myTwoPi);
	      /*
	         if( stat.linkStat[t].jointVal < 0 )
	         {
	         printf( "fixing the value of linkStat for joint %d\n", t );
	         stat.linkStat[t].jointVal += myTwoPi;
	         }
	       */
	      myValue =
		fabs (msg->linkCmd[t].value - stat.linkStat[t].jointVal);
	      //
	      // fixme for non-rotatry joints!/
	      //
	      if (myValue > msg->linkCmd[t].tolerance &&
		  fabs (myValue - myTwoPi) > msg->linkCmd[t].tolerance)
		{
		  /*
		     printf( "servoMis: link %d at %f want %f tol %f\n",
		     t, stat.linkStat[t].jointVal,
		     msg->linkCmd[t].value, 
		     msg->linkCmd[t].tolerance);
		   */
		  done = false;
		}
	    }
	}
      //      printf( "\n" );
      if (done)
	{
	  status_next (&stat, RCS_DONE);
	  state_next (&stat, S0);
	}
    }
  else
    {				// S0
      state_default (&stat);
    }
}

int
ServoMisJANode::setStat (sw_struct * sw)
{
  int t;
  int num;

  num = sw->data.mispkg.number;
  if (num >= SERVO_MIS_JA_LINK_MAX)
    num = SERVO_MIS_JA_LINK_MAX;
  if (num >= SW_ACT_LINK_MAX)
    num = SW_ACT_LINK_MAX;

  //  printf( "servoMisJANode status: " );
  for (t = 0; t < num; t++)
    {
      stat.linkStat[t].linkID = t + 1;
      stat.linkStat[t].jointVal = sw->data.mispkg.link[t].position;
      stat.linkStat[t].torque = sw->data.mispkg.link[t].torque;
      //      printf( "%f ", stat.linkStat[t].jointVal );
    }
  //  printf( "\n" );
  stat.linkStat_length = num;
  return 0;
}

int
ServoMisJANode::setSet (sw_struct * sw)
{
  int t;
  int num;

  num = sw->data.mispkg.number;
  if (num >= SERVO_MIS_JA_LINK_MAX)
    num = SERVO_MIS_JA_LINK_MAX;
  if (num >= SW_ACT_LINK_MAX)
    num = SW_ACT_LINK_MAX;

  strncpy (set.name, sw->name, sizeof (set.name));
  set.name[sizeof (set.name) - 1] = 0;

  for (t = 0; t < num; t++)
    {
      set.linkSet[t].linkID = t + 1;
      set.linkSet[t].parentLinkID = sw->data.mispkg.link[t].parent;
      if (set.linkSet[t].parentLinkID < 0)
	set.linkSet[t].parentLinkID = 0;
      set.linkSet[t].jType =
	sw->data.mispkg.link[t].type ==
	SW_LINK_REVOLUTE ? SERVO_MIS_JA_REVOLUTE_JOINT_TYPE : sw->data.mispkg.
	link[t].type ==
	SW_LINK_PRISMATIC ? SERVO_MIS_JA_PRISMATIC_JOINT_TYPE : sw->data.
	mispkg.link[t].type ==
	SW_LINK_SCISSOR ? SERVO_MIS_JA_SCISSOR_JOINT_TYPE :
	SERVO_MIS_JA_INVALID_JOINT_TYPE;
      set.linkSet[t].maxVelocity = sw->data.mispkg.link[t].maxspeed;
      set.linkSet[t].maxTorque = sw->data.mispkg.link[t].maxtorque;
      set.linkSet[t].maxRange = sw->data.mispkg.link[t].maxrange;
      set.linkSet[t].minRange = sw->data.mispkg.link[t].minrange;
      set.linkSet[t].loc.x = sw->data.mispkg.link[t].mount.x;
      set.linkSet[t].loc.y = sw->data.mispkg.link[t].mount.y;
      set.linkSet[t].loc.z = sw->data.mispkg.link[t].mount.z;
      set.linkSet[t].rot.r = sw->data.mispkg.link[t].mount.roll;
      set.linkSet[t].rot.p = sw->data.mispkg.link[t].mount.pitch;
      set.linkSet[t].rot.y = sw->data.mispkg.link[t].mount.yaw;
    }
  set.linkSet_length = num;

  return 0;
}

int
ServoMisJANode::writeStat (void)
{
  return statBuf->write (stat);
}

int
ServoMisJANode::writeSet (void)
{
  return setBuf->write (set);
}
