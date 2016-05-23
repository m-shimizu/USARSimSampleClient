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
  \file servoEffJANode.cc

  \brief Effector control code
*/

#include <stddef.h>
#include <float.h>
#include <rcs.hh>
#include "ulapi.h"
#include "skin.h"
#include "servoEffJA.hh"
#include "servoEffJANode.h"

static int
no_inf_tell (sw_struct * sw)
{
  return -1;
}

static void
cmdPassThr (void *arg)
{
  ServoEffJANode *node = reinterpret_cast < ServoEffJANode * >(arg);

  while (true)
    {
      node->cmdPass ();
      node->sleep ();
    }
}

static void
cmdReadThr (void *arg)
{
  ServoEffJANode *node = reinterpret_cast < ServoEffJANode * >(arg);

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
  ServoEffJANode *node = reinterpret_cast < ServoEffJANode * >(arg);

  while (true)
    {
      node->cfgPass ();
      node->sleep ();
    }
}

static void
cfgReadThr (void *arg)
{
  ServoEffJANode *node = reinterpret_cast < ServoEffJANode * >(arg);

  while (true)
    {
      if (0 != node->cfgRead ())
	{
	  node->sleep ();
	}
    }
}

ServoEffJANode::ServoEffJANode (void)
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

ServoEffJANode::~ServoEffJANode (void)
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
ServoEffJANode::init (char *processName, char *nmlFile, int packageNumber,
		      int argMission, char *argName,
		      int (*itell) (sw_struct * sw))
{
  char bufferName[MOAST_NML_BUFFER_NAME_LEN];

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoEffJACmd%d_%d",
		  packageNumber, argMission);
  bufferName[sizeof (bufferName) - 1] = 0;
  cmdBuf =
    new RCS_CMD_CHANNEL (servoEffJA_format, bufferName, processName, nmlFile);
  if (NULL == cmdBuf || !cmdBuf->valid ())
    {
      return -1;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoEffJAStat%d_%d",
		  packageNumber, argMission);
  bufferName[sizeof (bufferName) - 1] = 0;
  statBuf =
    new RCS_STAT_CHANNEL (servoEffJA_format, bufferName, processName,
			  nmlFile);
  if (NULL == statBuf || !statBuf->valid ())
    {
      return -1;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoEffJACfg%d_%d",
		  packageNumber, argMission);
  bufferName[sizeof (bufferName) - 1] = 0;
  cfgBuf =
    new RCS_CMD_CHANNEL (servoEffJA_format, bufferName, processName, nmlFile);
  if (NULL == cfgBuf || !cfgBuf->valid ())
    {
      return -1;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoEffJASet%d_%d",
		  packageNumber, argMission);
  bufferName[sizeof (bufferName) - 1] = 0;
  setBuf =
    new RCS_STAT_CHANNEL (servoEffJA_format, bufferName, processName,
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
ServoEffJANode::getName (void)
{
  return name;
}


void
ServoEffJANode::takeStatMutex (void)
{
  ulapi_mutex_take (statMutex);
}

void
ServoEffJANode::giveStatMutex (void)
{
  ulapi_mutex_give (statMutex);
}

void
ServoEffJANode::takeSetMutex (void)
{
  ulapi_mutex_take (setMutex);
}

void
ServoEffJANode::giveSetMutex (void)
{
  ulapi_mutex_give (setMutex);
}

void
ServoEffJANode::sleep (void)
{
  esleep (cycleTime);
}

int
ServoEffJANode::cmdPass (void)
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

    case SERVO_EFF_JA_CMD_INIT_TYPE:
      doCmdInit ();
      break;

    case SERVO_EFF_JA_CMD_OPCODE_TYPE:
      doCmdOpcode ();
      break;

    default:
      break;
    }

  cycle_time_tracker (&stat.tt);
  statBuf->write (stat);

  ulapi_mutex_give (statMutex);

  return 0;
}

int
ServoEffJANode::cmdRead (void)
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

    case SERVO_EFF_JA_CMD_INIT_TYPE:
    case SERVO_EFF_JA_CMD_OPCODE_TYPE:
      stat.command_type = cmdType;
      if (cmdSerialNumber != stat.echo_serial_number)
	{
	  stat.echo_serial_number = cmdSerialNumber;
	  stat.state = NEW_COMMAND;
	}
      break;

    default:
      fprintf (stderr, "unknown eff command %s\n",
	       servoEffJA_symbol_lookup (cmdType));
      break;
    }				// switch (cmdType)

  ulapi_mutex_give (statMutex);

  return cmdPass ();
}

int
ServoEffJANode::cfgPass (void)
{
  ulapi_mutex_take (setMutex);

  switch (set.command_type)
    {
    case SERVO_EFF_JA_CFG_CYCLE_TIME_TYPE:
      doCfgCycleTime ();
      break;
    }
  setBuf->write (set);

  ulapi_mutex_give (setMutex);

  return 0;
}

int
ServoEffJANode::cfgRead (void)
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

    case SERVO_EFF_JA_CFG_CYCLE_TIME_TYPE:
      set.command_type = cfgType;
      if (cfgSerialNumber != set.echo_serial_number)
	{
	  set.echo_serial_number = cfgSerialNumber;
	  set.state = NEW_COMMAND;
	}
      break;

    default:
      fprintf (stderr, "unknown eff config %s\n",
	       servoEffJA_symbol_lookup (cfgType));
      break;
    }				// switch (cfgType)

  ulapi_mutex_give (setMutex);

  return cfgPass ();
}

void
ServoEffJANode::doCmdInit (void)
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
ServoEffJANode::doCmdOpcode (void)
{
  ServoEffJACmdOpcode *msg;
  sw_struct sw;

  msg = reinterpret_cast < ServoEffJACmdOpcode * >(cmdBuf->get_address ());

  if (state_match (&stat, NEW_COMMAND))
    {
      state_new (&stat);
      stat.admin_state = ADMIN_INITIALIZED;
      sw_init (&sw);
      // FIXME -- hard-coding a gripper as our only effector
      sw_set_type (&sw, SW_EFF_GRIPPER);
      if (msg->param > DBL_EPSILON)
	{
	  sw_set_op (&sw, SW_EFF_GRIPPER_CLOSE);
	}
      else
	{
	  sw_set_op (&sw, SW_EFF_GRIPPER_OPEN);
	}
      sw_set_name (&sw, name);
      inf_tell (&sw);
      status_next (&stat, RCS_DONE);
      state_next (&stat, S0);
    }
  else
    {				// S0
      state_default (&stat);
    }
}

void
ServoEffJANode::doCfgCycleTime (void)
{
  ServoEffJACfgCycleTime *msg;

  msg = reinterpret_cast < ServoEffJACfgCycleTime * >(cfgBuf->get_address ());

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

int
ServoEffJANode::setStat (sw_struct * sw)
{
  return 0;
}

int
ServoEffJANode::setSet (sw_struct * sw)
{
  return 0;
}

int
ServoEffJANode::writeStat (void)
{
  return statBuf->write (stat);
}

int
ServoEffJANode::writeSet (void)
{
  return setBuf->write (set);
}
