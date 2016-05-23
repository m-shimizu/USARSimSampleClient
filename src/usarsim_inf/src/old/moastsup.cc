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
  \file moastsup.cc

  \brief The superior skin to MOAST.
*/

/*
  To Do:

  Many of the sensors don't go into NML yet. We are doing this as we
  need them since we're lazy. Sonars, rangescanners, groundtruth,
  encoders, odometers, tachometers, and acoustic sensors all come out.

  Consider moving the servoMobJA and servoSP code out of here
  and into what you would newly create as servoMobJANode and
  servoSPNode, respectively, for consistency with servoMisJA and
  servoEffJA.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdarg.h>
#include "ulapi.h"
#include "simware.h"
#include "skin.h"

#include <rcs.hh>
#include "moastTypes.hh"
#include "sensorData.hh"
#include "servoMobJA.hh"
#include "servoMisJA.hh"
#include "servoEffJA.hh"
#include "servoFactJA.hh"
#include "servoSP.hh"
#include "moastbufs.h"
#include "servoMisJANode.h"
#include "servoEffJANode.h"
#include "servoFactJANode.h"

static char thisProcess[] = "simware";

static bool debug = false;

static void
dbprintf (const char *fmt, ...)
{
  va_list ap;

  if (debug)
    {
      va_start (ap, fmt);
      fprintf (stderr, "moastsup: ");
      vfprintf (stderr, fmt, ap);
      va_end (ap);
    }
}

static bool verbose = false;


static void
writeRangeImageData (char *fileName, SensorData2D * rangeimagerData,
		     double *data)
{
  FILE *fp;
  int xCount, yCount;

  fp = fopen (fileName, "w");
  fprintf (fp, "# Dimensions: %d %d CollectTime: %f Duration: %f\n",
	   rangeimagerData->x, rangeimagerData->y,
	   rangeimagerData->collectTimeStart,
	   rangeimagerData->collectTimeDuration);
  fprintf (fp, "# Res: %f %f\n", rangeimagerData->incrementX,
	   rangeimagerData->incrementY);
  fprintf (fp, "x y z\n");
  printf ("moastsup: going to write %d elements from array sized %d\n",
	  rangeimagerData->x * rangeimagerData->y, MAX_SENSOR_DATA_2D);
  for (yCount = 0; yCount < rangeimagerData->y; yCount++)
    {
      for (xCount = 0; xCount < rangeimagerData->x; xCount++)
	{
	  //      fprintf( fp, "%d %d %f\n", xCount, yCount, data[yCount*rangeimagerData->y+xCount] );
	  fprintf (fp, "%f ", data[yCount * rangeimagerData->y + xCount]);
	}
      fprintf (fp, "\n");
    }
  printf ("moastsup: wrote range %dx%d image data to file\n",
	  rangeimagerData->x, rangeimagerData->y);
  fclose (fp);
}




static void
vbprintf (const char *fmt, ...)
{
  va_list ap;

  if (verbose)
    {
      va_start (ap, fmt);
      vfprintf (stderr, fmt, ap);
      va_end (ap);
    }
}

static void
errprintf (const char *fmt, ...)
{
  va_list ap;

  fprintf (stderr, "moastsup: ");
  va_start (ap, fmt);
  vfprintf (stderr, fmt, ap);
  va_end (ap);
}

static int
no_inf_tell (sw_struct * sw)
{
  errprintf ("no inf_tell function defined\n");

  return -1;
}

static int (*inf_tell) (sw_struct * sw) = no_inf_tell;

enum
{
  SERVO_MOB_JA_STAT_KEY = 101,
  SERVO_MOB_JA_SET_KEY,
  SERVO_SP_STAT_KEY,
  SERVO_SP_SET_KEY
};

#define DEFAULT_CYCLE_TIME 0.010

#define DEFAULT_NML_FILE "moast.nml"
static char *nmlFile = NULL;
static bool freeNmlFile = false;

#define DEFAULT_INI_FILE "moast.ini"
static char *iniFile = NULL;
static bool freeIniFile = false;

static void *servoMobJAStatMutex = NULL;
// no 'servoMobJACmdReadTask' - this is part of 'sup_ask'
static void *servoMobJACmdPassTask = NULL;

static void *servoMobJASetMutex = NULL;
static void *servoMobJACfgReadTask = NULL;
static void *servoMobJACfgPassTask = NULL;

static ServoMisJANode *servoMisJANodes[SERVO_MIS_JA_MIS_PKG_MAX];
static char servoMisJANames[SERVO_MIS_JA_MIS_PKG_MAX][SW_NAME_MAX];
static int
servoMisJAIndex (char *name)
{
  int t;

  for (t = 0; t < SERVO_MIS_JA_MIS_PKG_MAX; t++)
    {
      if (!strcmp (servoMisJANames[t], name))
	return t;
      if (0 == servoMisJANames[t][0])
	{
	  strncpy (servoMisJANames[t], name, SW_NAME_MAX);
	  servoMisJANames[t][SW_NAME_MAX - 1] = 0;
	  servoMobJASet.misCount++;
	  servoMobJASetGrdVeh.misCount++;
	  servoMobJASetAirBot.misCount++;
	  return t;
	}
    }

  // didn't find it, and no spot for it
  return -1;
}

static ServoEffJANode *servoEffJANodes[SERVO_EFF_JA_EFFECTOR_MAX];
static char servoEffJANames[SERVO_EFF_JA_EFFECTOR_MAX][SW_NAME_MAX];
static int
servoEffJAIndex (char *name)
{
  int t;

  for (t = 0; t < SERVO_EFF_JA_EFFECTOR_MAX; t++)
    {
      if (!strcmp (servoEffJANames[t], name))
	return t;
      if (0 == servoEffJANames[t][0])
	{
	  strncpy (servoEffJANames[t], name, SW_NAME_MAX);
	  servoEffJANames[t][SW_NAME_MAX - 1] = 0;
	  servoMobJASet.effCount++;
	  servoMobJASetGrdVeh.effCount++;
	  servoMobJASetAirBot.effCount++;
	  return t;
	}
    }

  // didn't find it, and no spot for it
  return -1;
}

static ServoFactJANode *servoFactJANode;

static void *servoSPStatMutex = NULL;
static void *servoSPCmdReadTask = NULL;
static void *servoSPCmdPassTask = NULL;

static void *servoSPSetMutex = NULL;
static void *servoSPCfgReadTask = NULL;
static void *servoSPCfgPassTask = NULL;

static int missionNumber = 1;

/*
  Each of the threads sleeps each cycle by the current 'cycleTime'
  value in the associated settings structure. Since the settings
  structures are shared, the cycle time needs to be mutexed. Rather
  than suffer a mutex take/give each cycle, the values are copied out
  into globals that are sized to guarantee atomic access, so no mutex
  is required.

  At init, these are set to be what's in the settings structures.
  Upon a new config message, these are updated to be consistent.

  We use 'float' which is typically a 32-bit format with atomic access.
*/
typedef float atomic_real;
static atomic_real servoMobJACycleTime =
  static_cast < atomic_real > (DEFAULT_CYCLE_TIME);
static atomic_real servoSPCycleTime =
  static_cast < atomic_real > (DEFAULT_CYCLE_TIME);

/*
  These will be set to either ground or air vehicle types once
  that's known from 'sup_tell'. These defaults inhibit anything
  from happening before the bot type is known.

  Since they're shared between threads, they are sized for atomic data
  access, per comment above.
*/
typedef int atomic_integer;
static atomic_integer botType = SERVO_MOB_JA_STAT_TYPE;

/*
  Each sonar value is sent separately, but they are aggregated into a
  single SensorData3D structure. These data and functions determine
  which index in the SensorData3D structure is assigned to which name.
*/

static char sonarNames[MAX_SENSOR_DATA_3D][SW_NAME_MAX];
static int sonarNumber = 0;
static int
sonarIndex (char *name)
{
  int t;

  for (t = 0; t < MAX_SENSOR_DATA_3D; t++)
    {
      if (!strcmp (sonarNames[t], name))
	return t;		// found it
      if (0 == sonarNames[t][0])
	{			// empty spot for it
	  strncpy (sonarNames[t], name, SW_NAME_MAX);
	  sonarNames[t][SW_NAME_MAX - 1] = 0;
	  sonarNumber = t + 1;
	  servoMobJASet.senCount++;
	  servoMobJASetGrdVeh.senCount++;
	  servoMobJASetAirBot.senCount++;
	  return t;
	}
    }

  // didn't find it, and no spot for it
  return -1;
}

/*
  Each range imager uses one of the several SensorData2D structures.
  These data and functions determine which index in the array of
  SensorData2D structures is assigned to which name.
*/
static char rangeimagerNames[MAX_SENSOR_DATA_2D_BUFFERS][SW_NAME_MAX];
static int
rangeimagerIndex (char *name)
{
  int t;

  for (t = 0; t < MAX_SENSOR_DATA_2D_BUFFERS; t++)
    {
      if (!strcmp (rangeimagerNames[t], name))
	return t;		// found it
      if (0 == rangeimagerNames[t][0])
	{			// empty spot for it
	  //      printf ("moastsup: found rangeimager named %s\n", name);
	  strncpy (rangeimagerNames[t], name, SW_NAME_MAX);
	  rangeimagerNames[t][SW_NAME_MAX - 1] = 0;
	  servoMobJASet.senCount++;
	  servoMobJASetGrdVeh.senCount++;
	  servoMobJASetAirBot.senCount++;
	  return t;
	}
    }

  // didn't find it, and no spot for it
  return -1;
}


/*
  Each range scanner uses one of the several SensorData1D structures.
  These data and functions determine which index in the array of
  SensorData1D structures is assigned to which name.
*/

static char rangescannerNames[MAX_SENSOR_DATA_1D_BUFFERS][SW_NAME_MAX];
static int
rangescannerIndex (char *name)
{
  int t;

  for (t = 0; t < MAX_SENSOR_DATA_1D_BUFFERS; t++)
    {
      if (!strcmp (rangescannerNames[t], name))
	return t;		// found it
      if (0 == rangescannerNames[t][0])
	{			// empty spot for it
	  strncpy (rangescannerNames[t], name, SW_NAME_MAX);
	  rangescannerNames[t][SW_NAME_MAX - 1] = 0;
	  servoMobJASet.senCount++;
	  servoMobJASetGrdVeh.senCount++;
	  servoMobJASetAirBot.senCount++;
	  return t;
	}
    }

  // didn't find it, and no spot for it
  return -1;
}

/*
  Each general data array sensor (tachometer, odometer, etc.) uses one
  of the several SensorData structures.  These data and functions
  determine which index in the array of SensorData structures is
  assigned to which name.
*/

static char sensorDataNames[MAX_SENSOR_DATA_BUFFERS][SW_NAME_MAX];
static int
sensorDataIndex (char *name)
{
  int t;

  for (t = 0; t < MAX_SENSOR_DATA_BUFFERS; t++)
    {
      if (!strcmp (sensorDataNames[t], name))
	return t;		// found it
      if (0 == sensorDataNames[t][0])
	{			// empty spot for it
	  strncpy (sensorDataNames[t], name, SW_NAME_MAX);
	  sensorDataNames[t][SW_NAME_MAX - 1] = 0;
	  servoMobJASet.senCount++;
	  servoMobJASetGrdVeh.senCount++;
	  servoMobJASetAirBot.senCount++;
	  return t;
	}
    }

  // didn't find it, and no spot for it
  return -1;
}

static void
doServoMobJACmdInitGrdVeh (ServoMobJAStatGrdVeh * stat)
{
#define INITCODE				\
  if (state_match(stat, NEW_COMMAND)) {		\
    state_new(stat);				\
    stat->admin_state = ADMIN_INITIALIZED;	\
    status_next(stat, RCS_DONE);		\
    state_next(stat, S0);			\
  } else {					\
    state_default(stat);			\
  }
  INITCODE;
}

static void
doServoMobJACmdInitAirBot (ServoMobJAStatAirBot * stat)
{
  INITCODE;
#undef INITCODE
}

static void
doServoMobJACmdSkid (ServoMobJACmdSkid * msg, ServoMobJAStatGrdVeh * stat,
		     ServoMobJASetGrdVeh * set)
{
  sw_struct sw;

  if (state_match (stat, NEW_COMMAND))
    {
      dbprintf ("doServoMobJACmdSkid %e %e\n", msg->wLeft, msg->wRight);
      state_new (stat);
      if (stat->admin_state != ADMIN_INITIALIZED)
	{
	  status_next (stat, RCS_ERROR);
	  state_next (stat, S0);
	}
      else if (MOAST_SKID_STEER_TYPE != set->steerType)
	{
	  status_next (stat, RCS_ERROR);
	  state_next (stat, S0);
	}
      else
	{
	  state_next (stat, S1);
	  status_next (stat, RCS_EXEC);
	}
    }
  else if (state_match (stat, S1))
    {
      sw_init (&sw);
      sw_set_type (&sw, SW_ROBOT_GROUNDVEHICLE);
      sw_set_op (&sw, SW_ROBOT_SKID_MOVE);
      sw.data.groundvehicle.left_speed = msg->wLeft;
      sw.data.groundvehicle.right_speed = msg->wRight;
      inf_tell (&sw);
      state_next (stat, S0);
      status_next (stat, RCS_DONE);
    }
  else
    {				// S0
      state_default (stat);
    }
}

static void
doServoMobJACmdAckerman (ServoMobJACmdAckerman * msg,
			 ServoMobJAStatGrdVeh * stat,
			 ServoMobJASetGrdVeh * set)
{
  sw_struct sw;

  if (state_match (stat, NEW_COMMAND))
    {
      dbprintf ("doServoMobJACmdAckerman\n");
      state_new (stat);
      if (stat->admin_state != ADMIN_INITIALIZED)
	{
	  status_next (stat, RCS_ERROR);
	  state_next (stat, S0);
	}
      else if (MOAST_ACKERMAN_STEER_TYPE != set->steerType)
	{
	  status_next (stat, RCS_ERROR);
	  state_next (stat, S0);
	}
      else
	{
	  state_next (stat, S1);
	  status_next (stat, RCS_EXEC);
	}
    }
  else if (state_match (stat, S1))
    {
      sw_init (&sw);
      sw_set_type (&sw, SW_ROBOT_GROUNDVEHICLE);
      sw_set_op (&sw, SW_ROBOT_ACKERMAN_MOVE);
      sw.data.groundvehicle.speed = msg->velocity;
      sw.data.groundvehicle.heading = msg->steerAngle;
      inf_tell (&sw);
      state_next (stat, S0);
      status_next (stat, RCS_DONE);
    }
  else
    {				// S0
      state_default (stat);
    }
}

static void
doServoMobJACfgCycleTimeGrdVeh (ServoMobJACfgCycleTime * msg,
				ServoMobJASetGrdVeh * set)
{
#define CTCODE								\
  if (state_match(set, NEW_COMMAND)) {					\
    dbprintf("doServoMobJACfgCycleTime\n");				\
    servoMobJACycleTime = static_cast<atomic_real>(msg->cycleTime);	\
    set->cycleTime = msg->cycleTime;					\
    state_next(set, S0);						\
    status_next(set, RCS_DONE);						\
  } else {								\
    state_default(set);							\
  }
  CTCODE;
}

static void
doServoMobJACfgCycleTimeAirBot (ServoMobJACfgCycleTime * msg,
				ServoMobJASetAirBot * set)
{
  CTCODE;
#undef CTCODE
}

static void
doServoSPCmdInit (ServoSPStat * stat)
{
  if (state_match (stat, NEW_COMMAND))
    {
      dbprintf ("doServoSPCmdInit\n");
      state_new (stat);
      stat->admin_state = ADMIN_INITIALIZED;
      status_next (stat, RCS_DONE);
      state_next (stat, S0);
    }
  else
    {				// S0
      state_default (stat);
    }
}

static void
doServoSPCmdGo (ServoSPStat * stat)
{
  sw_struct sw;

  if (state_match (stat, NEW_COMMAND))
    {
      state_new (stat);
      if (stat->admin_state != ADMIN_INITIALIZED)
	{
	  status_next (stat, RCS_ERROR);
	  state_next (stat, S0);
	}
      else
	{
	  state_next (stat, S1);
	  status_next (stat, RCS_EXEC);
	}
    }
  else if (state_match (stat, S1))
    {
      int tmax = servoSPSet.sSetElem_length;
      state_next (stat, S2);
      status_next (stat, RCS_DONE);
      ulapi_mutex_take (servoSPSetMutex);
      for (int count = 0; count < tmax; count++)
	{
	  if (servoSPSet.sSetElem[count].sType == SERVO_SP_LINEIMAGER_SENSOR)
	    {
	      printf ("moastsup: Sending go command to kinect\n");
	      sw_set_type (&sw, SW_SEN_RANGEIMAGER);
	      sw_set_op (&sw, SW_SEN_GO);
	      sw_set_name (&sw, servoSPSet.sSetElem[count].senName);
	      inf_tell (&sw);
	    }
	}
      ulapi_mutex_give (servoSPSetMutex);
    }
  else if (state_match (stat, S2))
    {
      // stay here
    }
  else
    {				// S0
      state_default (stat);
    }
}

static void
doServoSPCfgCycleTime (ServoSPCfgCycleTime * msg, ServoSPSet * set)
{
  if (state_match (set, NEW_COMMAND))
    {
      dbprintf ("doServoSPCfgCycleTime\n");
      servoSPCycleTime = static_cast < atomic_real > (msg->cycleTime);
      set->cycleTime = msg->cycleTime;
      state_next (set, S0);
      status_next (set, RCS_DONE);
    }
  else
    {				// S0
      state_default (set);
    }
}

static int
copySonar (SensorData3D * sonar, sw_struct * sw)
{
  int t;

  t = sonarIndex (sw->name);

  if (t >= 0)
    {
      sonar->element[t].time = sw->time;
      sonar->element[t].range = sw->data.sonar.range;
      sonar->element[t].pose.tran =
	PM_CARTESIAN (sw->data.sonar.mount.x, sw->data.sonar.mount.y,
		      sw->data.sonar.mount.z);
      sonar->element[t].pose.rot =
	PM_RPY (sw->data.sonar.mount.roll, sw->data.sonar.mount.pitch,
		sw->data.sonar.mount.yaw);
      sonar->element_length = sonarNumber;

      return 0;
    }

  return -1;
}


/* append information onto message until last frame is received. Then send out message
   returns 0: frame not finished
           1: frame finished
 */
static int
copyRangeimager (SensorData2D * range, double **dataIn, sw_struct * sw)
{
  int t;
  int num;
  int count;
  int done = 0;
  float flipImagerx = 1;	// if set to -1, flip range imager increment and start, if 1 leave alone
  float flipImagery = 1;	// if set to -1, flip range imager increment and start, if 1 leave alone
  double *data;

  /*
     printf( "moastsup: rangeImager fov( %f, %f), res( %f, %f)\n", 
     sw->data.rangeimager.fovx,
     sw->data.rangeimager.fovy,
     sw->data.rangeimager.resolutionx,
     sw->data.rangeimager.resolutiony);
   */
  range->lowerLeft.x = flipImagerx * -0.5 * sw->data.rangeimager.fovx;
  range->lowerLeft.y = flipImagery * -0.5 * sw->data.rangeimager.fovy;
  range->incrementX =
    flipImagerx * sw->data.rangeimager.fovx /
    sw->data.rangeimager.resolutionx;
  range->incrementY =
    flipImagery * sw->data.rangeimager.fovy /
    sw->data.rangeimager.resolutiony;
  range->minRange = sw->data.rangeimager.minrange;
  range->maxRange = sw->data.rangeimager.maxrange;
  range->x = sw->data.rangeimager.resolutionx;
  range->y = sw->data.rangeimager.resolutiony;
  if (*dataIn == NULL)
    *dataIn = new double[range->x * range->y];
  data = *dataIn;
  if (sw->data.rangeimager.frame == 0)
    range->collectTimeStart = sw->time;
  if (sw->data.rangeimager.frame == (sw->data.rangeimager.totalframes - 1))
    {
      done = 1;
      range->collectTimeDuration = sw->time - range->collectTimeStart;
    }

  if (sw->data.rangeimager.totalframes != 0)
    printf ("moastsup: received frame %d of %d\n", sw->data.rangeimager.frame,
	    sw->data.rangeimager.totalframes);

  num =
    sw->data.rangeimager.numberperframe * (sw->data.rangeimager.frame + 1);
  /*
     if (num > SW_SEN_RANGEIMAGER_MAX)
     num = SW_SEN_RANGEIMAGER_MAX;
     if (num > MAX_SENSOR_DATA_2D)
     num = MAX_SENSOR_DATA_2D;
   */
  count = 0;
  //  printf( "moastsup: frame number: %d has number per frame: %d max: %d\n", sw->data.rangeimager.frame, sw->data.rangeimager.numberperframe, num );
  for (t = sw->data.rangeimager.numberperframe * sw->data.rangeimager.frame;
       t < num; t++)
    {
      data[t] = sw->data.rangeimager.range[count++];
      //    printf( "(%d %f) ", t, range->featArray[t] );
    }
  //  printf("\n");
  range->featArray_length = num;

  return done;
}


static int
copyRangescanner (SensorData1D * range, sw_struct * sw)
{
  int t;
  int num;
  float flipScanner = -1;	// if set to -1, flip range scanner increment and start, if 1 leave alone
  range->start = flipScanner * -0.5 * sw->data.rangescanner.fov;
  range->increment = flipScanner * sw->data.rangescanner.resolution;
  range->minRange = sw->data.rangescanner.minrange;
  range->maxRange = sw->data.rangescanner.maxrange;
  range->collectTimeStart = sw->time;
  range->collectTimeDuration = 0;

  num = sw->data.rangescanner.number;
  if (num > SW_SEN_RANGESCANNER_MAX)
    num = SW_SEN_RANGESCANNER_MAX;
  if (num > MAX_SENSOR_DATA_1D)
    num = MAX_SENSOR_DATA_1D;
  for (t = 0; t < num; t++)
    {
      range->featArray[t] = sw->data.rangescanner.range[t];
    }
  range->featArray_length = num;

  return 0;
}

static int
copyTachometer (SensorData * sen, sw_struct * sw)
{
  int senindex;
  int swindex;

  sen->time = sw->time;
  sen->type = SERVO_SP_TACHOMETER_SENSOR;
  for (senindex = 0, swindex = 0;
       senindex < SENSOR_DATA_MAX &&
       swindex < SW_SEN_TACHOMETER_MAX; senindex += 2, swindex += 1)
    {
      sen->data[senindex] = sw->data.tachometer.speed[swindex];
      sen->data[senindex + 1] = sw->data.tachometer.position[swindex];
    }
  sen->data_length = senindex;

  return 0;
}

/*
  We use the all-purpose SensorData class to store our acoustic
  sensor data, with this mapping:

  data[0] = azimuth angle of sound source wrt sensor frame, [rad]
  data[1] = altitude angle, likewise
  data[2] = average sound volume, [dB]
  data[3] = sound duration, [s]
*/
static int
copyAcoustic (SensorData * sen, sw_struct * sw)
{
  int count = 0;

  sen->time = sw->time;
  sen->type = SERVO_SP_ACOUSTIC_SENSOR;
  sen->data[count++] = sw->data.acoustic.azimuth;
  sen->data[count++] = sw->data.acoustic.altitude;
  sen->data[count++] = sw->data.acoustic.volume;
  sen->data[count++] = sw->data.acoustic.duration;
  sen->data_length = count;

  return 0;
}

static int
copyOdometer (SensorData * sen, sw_struct * sw)
{
  int count = 0;

  sen->time = sw->time;
  sen->type = SERVO_SP_ODOMETER_SENSOR;
  sen->data[count++] = sw->data.odometer.position.x;
  sen->data[count++] = sw->data.odometer.position.y;
  sen->data[count++] = sw->data.odometer.position.z;
  sen->data[count++] = sw->data.odometer.position.roll;
  sen->data[count++] = sw->data.odometer.position.pitch;
  sen->data[count++] = sw->data.odometer.position.yaw;
  sen->data_length = count;

  return 0;
}

static int
copyEncoder (SensorData * sen, sw_struct * sw)
{
  int count = 0;

  sen->time = sw->time;
  sen->type = SERVO_SP_ENCODER_SENSOR;
  sen->data[count++] = sw->data.encoder.tick;
  sen->data_length = count;

  return 0;
}

static int
copyGroundtruth (SensorData * sen, sw_struct * sw)
{
  int count = 0;
  PM_POSE sensor;
  PM_POSE mountInv;
  PM_POSE truth;
  PM_RPY rpy;

  /*
     .       wor    wor    sen
     truth =    T =    T *    T = sensor * inv(mount)
     .       veh    sen    veh
   */

  sensor =
    PM_POSE (PM_CARTESIAN
	     (sw->data.groundtruth.position.x,
	      sw->data.groundtruth.position.y,
	      sw->data.groundtruth.position.z),
	     PM_RPY (sw->data.groundtruth.position.roll,
		     sw->data.groundtruth.position.pitch,
		     sw->data.groundtruth.position.yaw));
  mountInv =
    inv (PM_POSE
	 (PM_CARTESIAN
	  (sw->data.groundtruth.mount.x, sw->data.groundtruth.mount.y,
	   sw->data.groundtruth.mount.z),
	  PM_RPY (sw->data.groundtruth.mount.roll,
		  sw->data.groundtruth.mount.pitch,
		  sw->data.groundtruth.mount.yaw)));
  truth = sensor * mountInv;
  rpy = truth.rot;

  sen->time = sw->time;
  sen->type = SERVO_SP_GRD_TRUTH_SENSOR;
  sen->data[count++] = truth.tran.x;
  sen->data[count++] = truth.tran.y;
  sen->data[count++] = truth.tran.z;
  sen->data[count++] = rpy.r;
  sen->data[count++] = rpy.p;
  sen->data[count++] = rpy.y;
  sen->data_length = count;

  return 0;
}

static int
copyServoMobJASetGrdVeh (ServoMobJASetGrdVeh * set, sw_struct * sw)
{
  switch (sw->data.groundvehicle.steertype)
    {
    case SW_STEER_SKID:
      set->steerType = MOAST_SKID_STEER_TYPE;
      break;
    case SW_STEER_ACKERMAN:
      set->steerType = MOAST_ACKERMAN_STEER_TYPE;
      break;
    case SW_STEER_OMNI:
      set->steerType = MOAST_OMNI_STEER_TYPE;
      break;
    default:
      set->steerType = MOAST_INVALID_STEER_TYPE;
      break;
    }
  strncpy (set->platformName, sw->name, sizeof (set->platformName));
  set->platformName[sizeof (set->platformName) - 1] = 0;
  set->length = sw->data.groundvehicle.length;
  set->width = sw->data.groundvehicle.width;
  set->height = sw->data.groundvehicle.height;
  set->mass = sw->data.groundvehicle.mass;
  set->cg.x = sw->data.groundvehicle.cg.x;
  set->cg.y = sw->data.groundvehicle.cg.y;
  set->cg.z = sw->data.groundvehicle.cg.z;
  set->maxWheelRot = sw->data.groundvehicle.max_speed;
  set->maxTorque = sw->data.groundvehicle.max_torque;
  set->wheelSeparation = sw->data.groundvehicle.wheel_separation;
  set->wheelRadius = sw->data.groundvehicle.wheel_radius;
  set->wheelBase = sw->data.groundvehicle.wheel_base;
  set->maxSteerAngle = sw->data.groundvehicle.max_steer_angle;
  set->minTurningRadius = sw->data.groundvehicle.min_turning_radius;

  return 0;
}

static void
cleanup ()
{
  cleanupNml ();

  if (freeNmlFile)
    free (nmlFile);
  nmlFile = NULL;

  if (freeIniFile)
    free (iniFile);
  iniFile = NULL;

  if (NULL != servoMobJAStatMutex)
    {
      ulapi_mutex_delete (servoMobJAStatMutex);
      servoMobJAStatMutex = NULL;
    }

  if (NULL != servoMobJASetMutex)
    {
      ulapi_mutex_delete (servoMobJASetMutex);
      servoMobJASetMutex = NULL;
    }

  if (NULL != servoSPStatMutex)
    {
      ulapi_mutex_delete (servoSPStatMutex);
      servoSPStatMutex = NULL;
    }

  if (NULL != servoSPSetMutex)
    {
      ulapi_mutex_delete (servoSPSetMutex);
      servoSPSetMutex = NULL;
    }

  return;
}

static int
servoMobJACmdPass (void)
{
  NMLTYPE cmdType = 0;

  ulapi_mutex_take (servoMobJAStatMutex);

  if (SERVO_MOB_JA_STAT_GRD_VEH_TYPE == botType)
    {
      cmdType = servoMobJAStatGrdVeh.command_type;
    }
  else if (SERVO_MOB_JA_STAT_AIR_BOT_TYPE == botType)
    {
      cmdType = servoMobJAStatAirBot.command_type;
    }

#define CAST_IT(TYPE) reinterpret_cast<TYPE>(servoMobJACmdBuf->get_address())
  switch (cmdType)
    {
    case 0:
      // no new command
      break;
    case -1:
      // comm error
      break;

    case SERVO_MOB_JA_CMD_INIT_TYPE:
      if (SERVO_MOB_JA_STAT_GRD_VEH_TYPE == botType)
	doServoMobJACmdInitGrdVeh (&servoMobJAStatGrdVeh);
      else if (SERVO_MOB_JA_STAT_AIR_BOT_TYPE == botType)
	doServoMobJACmdInitAirBot (&servoMobJAStatAirBot);
      break;

    case SERVO_MOB_JA_CMD_SKID_TYPE:
      doServoMobJACmdSkid (CAST_IT (ServoMobJACmdSkid *),
			   &servoMobJAStatGrdVeh, &servoMobJASetGrdVeh);
      break;

    case SERVO_MOB_JA_CMD_ACKERMAN_TYPE:
      doServoMobJACmdAckerman (CAST_IT (ServoMobJACmdAckerman *),
			       &servoMobJAStatGrdVeh, &servoMobJASetGrdVeh);
      break;
    }
#undef CAST_IT

  if (SERVO_MOB_JA_STAT_GRD_VEH_TYPE == botType)
    {
      cycle_time_tracker (&servoMobJAStatGrdVeh.tt);
      servoMobJAStatBuf->write (&servoMobJAStatGrdVeh);
    }
  else if (SERVO_MOB_JA_STAT_AIR_BOT_TYPE == botType)
    {
      cycle_time_tracker (&servoMobJAStatAirBot.tt);
      servoMobJAStatBuf->write (&servoMobJAStatAirBot);
    }
  // else don't do anything, since we don't know our robot type yet

  ulapi_mutex_give (servoMobJAStatMutex);

  return 0;
}

static int
servoMobJACmdRead (void)
{
  NMLTYPE cmdType;
  int cmdSerialNumber;
  int retval = 0;

  if (NULL == servoMobJACmdBuf)
    return -1;
  if (NULL == servoMobJAStatBuf)
    return -1;

  cmdType = servoMobJACmdBuf->blocking_read (-1);
  cmdSerialNumber = servoMobJACmdBuf->get_address ()->serial_number;

  ulapi_mutex_take (servoMobJAStatMutex);

  switch (cmdType)
    {
    case 0:
      // no new command
      break;
    case -1:
      // comm error
      retval = -1;
      break;

    case SERVO_MOB_JA_CMD_INIT_TYPE:
    case SERVO_MOB_JA_CMD_SKID_TYPE:
    case SERVO_MOB_JA_CMD_ACKERMAN_TYPE:
#define STATIT(TYPE)					\
    TYPE.command_type = cmdType;			\
    if (cmdSerialNumber != TYPE.echo_serial_number) {	\
      TYPE.echo_serial_number = cmdSerialNumber;	\
      TYPE.state = NEW_COMMAND;				\
    }
      if (SERVO_MOB_JA_STAT_GRD_VEH_TYPE == botType)
	{
	  STATIT (servoMobJAStatGrdVeh);
	}
      else if (SERVO_MOB_JA_STAT_AIR_BOT_TYPE == botType)
	{
	  STATIT (servoMobJAStatAirBot);
	}
#undef STATIT
      break;

    default:
      errprintf ("unknown mob command %s\n",
		 servoMobJA_symbol_lookup (cmdType));
      break;
    }				// switch (cmdType)

  ulapi_mutex_give (servoMobJAStatMutex);

  return servoMobJACmdPass ();
}

static void
servoMobJACmdPassThr (void *arg)
{
  while (true)
    {
      (void) servoMobJACmdPass ();
      esleep (servoMobJACycleTime);
    }
}

/*
  no need for a 'servoMobJACmdReadThr' thread, since it's called
  explicitly in 'sup_ask'
*/

static int
servoMobJACfgPass (void)
{
  NMLTYPE cmdType = 0;

  ulapi_mutex_take (servoMobJASetMutex);

  if (SERVO_MOB_JA_STAT_GRD_VEH_TYPE == botType)
    {
      cmdType = servoMobJASetGrdVeh.command_type;
    }
  else if (SERVO_MOB_JA_STAT_AIR_BOT_TYPE == botType)
    {
      cmdType = servoMobJASetAirBot.command_type;
    }

#define CAST_IT(TYPE) reinterpret_cast<TYPE>(servoMobJACfgBuf->get_address())
  switch (cmdType)
    {
    case SERVO_MOB_JA_CFG_CYCLE_TIME_TYPE:
      if (SERVO_MOB_JA_STAT_GRD_VEH_TYPE == botType)
	doServoMobJACfgCycleTimeGrdVeh (CAST_IT (ServoMobJACfgCycleTime *),
					&servoMobJASetGrdVeh);
      else if (SERVO_MOB_JA_STAT_AIR_BOT_TYPE == botType)
	doServoMobJACfgCycleTimeAirBot (CAST_IT (ServoMobJACfgCycleTime *),
					&servoMobJASetAirBot);
      break;
    }
#undef CAST_IT

  if (SERVO_MOB_JA_STAT_GRD_VEH_TYPE == botType)
    {
      servoMobJASetBuf->write (&servoMobJASetGrdVeh);
    }
  else if (SERVO_MOB_JA_STAT_AIR_BOT_TYPE == botType)
    {
      servoMobJASetBuf->write (&servoMobJASetAirBot);
    }

  ulapi_mutex_give (servoMobJASetMutex);

  return 0;
}

static int
servoMobJACfgRead (void)
{
  NMLTYPE cfgType;
  int cfgSerialNumber;

  if (NULL == servoMobJACfgBuf)
    return -1;
  if (NULL == servoMobJASetBuf)
    return -1;

  cfgType = servoMobJACfgBuf->blocking_read (-1);
  cfgSerialNumber = servoMobJACfgBuf->get_address ()->serial_number;

  ulapi_mutex_take (servoMobJASetMutex);

  switch (cfgType)
    {
    case 0:
      // no new command
      break;
    case -1:
      // comm error
      break;

    case SERVO_MOB_JA_CFG_CYCLE_TIME_TYPE:
#define SETIT(TYPE)					\
    TYPE.command_type = cfgType;			\
    if (cfgSerialNumber != TYPE.echo_serial_number) {	\
      TYPE.echo_serial_number = cfgSerialNumber;	\
      TYPE.state = NEW_COMMAND;				\
    }
      if (SERVO_MOB_JA_STAT_GRD_VEH_TYPE == botType)
	{
	  SETIT (servoMobJASetGrdVeh);
	}
      else if (SERVO_MOB_JA_STAT_AIR_BOT_TYPE == botType)
	{
	  SETIT (servoMobJASetAirBot);
	}
#undef SETIT
      break;

    default:
      errprintf ("unknown mob config %s\n",
		 servoMobJA_symbol_lookup (cfgType));
      break;
    }				// switch (cfgType)

  ulapi_mutex_give (servoMobJASetMutex);

  return servoMobJACfgPass ();
}

static void
servoMobJACfgPassThr (void *arg)
{
  while (true)
    {
      (void) servoMobJACfgPass ();
      esleep (servoMobJACycleTime);
    }
}

static void
servoMobJACfgReadThr (void *arg)
{
  while (true)
    {
      if (0 != servoMobJACfgRead ())
	{
	  /*
	     this sleep shouldn't be necessary, since we're doing a blocking
	     read, but in case there's a read error and it returns immediately,
	     we'll throttle back with this sleep
	   */
	  esleep (servoMobJACycleTime);
	}
    }
}

static void
servoMobJASetUpdate (void)
{
  ulapi_mutex_take (servoMobJASetMutex);
  switch (botType)
    {
    case SERVO_MOB_JA_STAT_GRD_VEH_TYPE:
      servoMobJASetBuf->write (&servoMobJASetGrdVeh);
      break;
    case SERVO_MOB_JA_STAT_AIR_BOT_TYPE:
      servoMobJASetBuf->write (&servoMobJASetAirBot);
      break;
    default:
      servoMobJASetBuf->write (&servoMobJASet);
      break;
    }
  ulapi_mutex_give (servoMobJASetMutex);
}

static int
servoSPCmdPass (void)
{
  ulapi_mutex_take (servoSPStatMutex);

#define CAST_IT(TYPE) reinterpret_cast<TYPE>(servoSPCmdBuf->get_address())
  switch (servoSPStat.command_type)
    {
    case 0:
      // no new command
      break;
    case -1:
      // comm error
      break;

    case SERVO_SP_CMD_INIT_TYPE:
      doServoSPCmdInit (&servoSPStat);
      break;

    case SERVO_SP_CMD_GO_TYPE:
      doServoSPCmdGo (&servoSPStat);
      break;
    }
#undef CAST_IT

  cycle_time_tracker (&servoSPStat.tt);
  servoSPStatBuf->write (&servoSPStat);
  ulapi_mutex_give (servoSPStatMutex);

  return 0;
}

static int
servoSPCmdRead (void)
{
  NMLTYPE cmdType;
  int cmdSerialNumber;
  int retval = 0;

  if (NULL == servoSPCmdBuf)
    return -1;
  if (NULL == servoSPStatBuf)
    return -1;

  cmdType = servoSPCmdBuf->blocking_read (-1);
  cmdSerialNumber = servoSPCmdBuf->get_address ()->serial_number;

  ulapi_mutex_take (servoSPStatMutex);

  switch (cmdType)
    {
    case 0:
      // no new command
      break;
    case -1:
      // comm error
      retval = -1;
      break;

    case SERVO_SP_CMD_INIT_TYPE:
    case SERVO_SP_CMD_GO_TYPE:
      servoSPStat.command_type = cmdType;
      if (cmdSerialNumber != servoSPStat.echo_serial_number)
	{
	  servoSPStat.echo_serial_number = cmdSerialNumber;
	  servoSPStat.state = NEW_COMMAND;
	}
      break;

    default:
      errprintf ("unknown sp command %s\n", servoSP_symbol_lookup (cmdType));
      break;
    }				// switch (cmdType)

  ulapi_mutex_give (servoSPStatMutex);

  return servoSPCmdPass ();
}

static void
servoSPCmdPassThr (void *arg)
{
  while (true)
    {
      (void) servoSPCmdPass ();
      esleep (servoSPCycleTime);
    }
}

static void
servoSPCmdReadThr (void *arg)
{
  while (true)
    {
      if (0 != servoSPCmdRead ())
	{
	  esleep (servoSPCycleTime);
	}
    }
}

static int
servoSPCfgPass (void)
{
  ulapi_mutex_take (servoSPSetMutex);

#define CAST_IT(TYPE) reinterpret_cast<TYPE>(servoSPCfgBuf->get_address())
  switch (servoSPSet.command_type)
    {
    case SERVO_SP_CFG_CYCLE_TIME_TYPE:
      doServoSPCfgCycleTime (CAST_IT (ServoSPCfgCycleTime *), &servoSPSet);
      break;
    }
#undef CAST_IT

  servoSPSetBuf->write (&servoSPSet);
  ulapi_mutex_give (servoSPSetMutex);

  return 0;
}

static int
servoSPCfgRead (void)
{
  NMLTYPE cfgType;
  int cfgSerialNumber;
  int retval = 0;

  if (NULL == servoSPCfgBuf)
    return -1;
  if (NULL == servoSPSetBuf)
    return -1;

  cfgType = servoSPCfgBuf->blocking_read (-1);
  cfgSerialNumber = servoSPCfgBuf->get_address ()->serial_number;

  ulapi_mutex_take (servoSPSetMutex);

  switch (cfgType)
    {
    case 0:
      // no new command
      break;
    case -1:
      // comm error
      retval = -1;
      break;

    case SERVO_SP_CFG_CYCLE_TIME_TYPE:
      servoSPSet.command_type = cfgType;
      if (cfgSerialNumber != servoSPSet.echo_serial_number)
	{
	  servoSPSet.echo_serial_number = cfgSerialNumber;
	  servoSPSet.state = NEW_COMMAND;
	}
      break;

    default:
      errprintf ("unknown sp config %s\n", servoSP_symbol_lookup (cfgType));
      break;
    }				// switch (cfgType)

  ulapi_mutex_give (servoSPSetMutex);

  return servoSPCfgPass ();
}

static void
servoSPCfgPassThr (void *arg)
{
  while (true)
    {
      (void) servoSPCfgPass ();
      esleep (servoSPCycleTime);
    }
}

static void
servoSPCfgReadThr (void *arg)
{
  while (true)
    {
      if (0 != servoSPCfgRead ())
	{
	  esleep (servoSPCycleTime);
	}
    }
}

static int
iniLoad (const char *filename,
	 ServoMobJASet * servoMobJASetPtr,
	 ServoMobJASetGrdVeh * servoMobJASetGrdVehPtr,
	 ServoMobJASetAirBot * servoMobJASetAirBotPtr,
	 // FIXME -- figure out how to iniLoad the mis pkg
	 // and effector
	 ServoSPSet * servoSPSetPtr)
{
  INIFILE inifile;
  const char *inistring;
  const char *section;
  const char *key;
  double d;
  int retval = INI_OK;

  servoMobJASetPtr->cycleTime = DEFAULT_CYCLE_TIME;
  servoMobJASetGrdVehPtr->cycleTime = DEFAULT_CYCLE_TIME;
  servoMobJASetAirBotPtr->cycleTime = DEFAULT_CYCLE_TIME;
  servoSPSetPtr->cycleTime = DEFAULT_CYCLE_TIME;
  servoSPSetPtr->vehicleID = 0;
  servoSPSetPtr->startPose =
    PM_POSE (PM_CARTESIAN (0, 0, 0), PM_QUATERNION (1, 0, 0, 0));
#if 0
  strncpy (servoSPSetPtr->worldName, "(unknown)",
	   sizeof (servoSPSetPtr->worldName));
  servoSPSetPtr->worldName[sizeof (servoSPSetPtr->worldName) - 1] = 0;
#endif
  servoSPSetPtr->sSetElem_length = 0;

  if (0 != inifile.open (filename))
    {
      errprintf ("can't open INI file %s, using defaults\n", filename);
      return INI_INVALID;
    }

  section = "SERVO_MOB_JA";
  key = "CYCLE_TIME";

  if (NULL != (inistring = inifile.find (key, section)))
    {
      if (1 == sscanf (inistring, "%lf", &d) && d > 0.0)
	{
	  // found, and valid
	  servoMobJACycleTime = static_cast < atomic_real > (d);
	  servoMobJASetPtr->cycleTime = d;
	}
      else
	{
	  // found, but invalid
	  errprintf ("invalid value '%s' for [%s] %s in '%s'\n", inistring,
		     section, key, filename);
	  retval = INI_INVALID;
	}
    }
  else
    {
      // not found at all
      errprintf ("can't find [%s] %s in '%s', using %f\n", section, key,
		 filename, servoMobJASetPtr->cycleTime);
      retval = (retval == INI_OK ? INI_DEFAULT : retval);
    }

  section = "SERVO_SP";
  key = "CYCLE_TIME";

  if (NULL != (inistring = inifile.find (key, section)))
    {
      if (1 == sscanf (inistring, "%lf", &d) && d > 0.0)
	{
	  // found, and valid
	  servoSPCycleTime = static_cast < atomic_real > (d);
	  servoSPSetPtr->cycleTime = d;
	}
      else
	{
	  // found, but invalid
	  errprintf ("invalid value '%s' for [%s] %s in '%s'\n", inistring,
		     section, key, filename);
	  retval = INI_INVALID;
	}
    }
  else
    {
      // not found at all
      errprintf ("can't find [%s] %s in '%s', using %f\n", section, key,
		 filename, servoSPSetPtr->cycleTime);
      retval = (retval == INI_OK ? INI_DEFAULT : retval);
    }

  inifile.close ();

  return retval;
}

#ifdef __cplusplus
extern "C"
{
#endif
#if 0
}
#endif

int
skin_sup_init (char *initString, int (*itell) (sw_struct * sw))
{
  int t;
  int argc;
  char **argv;
  int i;
  int retval;

  retval = 0;

  if (NULL == initString)
    {
      argc = 0;
    }
  else
    {
      argc = ulapi_to_argv (initString, &argv);
      // would like to use getopt here, but the 'optreset' feature to
      // reset it after being used by main isn't working
      retval = 0;
      for (t = 0; t < argc; t++)
	{
	  if (!strcmp (argv[t], "-d") || !strcmp (argv[t], "--debug"))
	    {
	      debug = true;
	    }
	  else if (!strcmp (argv[t], "-v") || !strcmp (argv[t], "--verbose"))
	    {
	      verbose = true;
	    }
	  else if (!strcmp (argv[t], "-n") || !strcmp (argv[t], "--nml"))
	    {
	      t++;
	      if (t == argc)
		{
		  errprintf ("sup_init: missing argument to %s\n",
			     argv[t - 1]);
		  retval = -1;
		  break;
		}
	      nmlFile =
		reinterpret_cast < char *>(malloc (strlen (argv[t]) + 1));
	      strcpy (nmlFile, argv[t]);
	      freeNmlFile = true;
	    }
	  else if (!strcmp (argv[t], "-i") || !strcmp (argv[t], "--ini"))
	    {
	      t++;
	      if (t == argc)
		{
		  errprintf ("sup_init: missing argument to %s\n",
			     argv[t - 1]);
		  retval = -1;
		  break;
		}
	      iniFile =
		reinterpret_cast < char *>(malloc (strlen (argv[t]) + 1));
	      strcpy (iniFile, argv[t]);
	      freeIniFile = true;
	    }
	  else if (!strcmp (argv[t], "-m") || !strcmp (argv[t], "--mission"))
	    {
	      t++;
	      if (t == argc)
		{
		  errprintf ("sup_init: missing argument to %s\n",
			     argv[t - 1]);
		  retval = -1;
		  break;
		}
	      if (1 != sscanf (argv[t], "%i", &i))
		{
		  errprintf ("sup_init: bad argument to %s: %s\n",
			     argv[t - 1], argv[t]);
		  retval = -1;
		  break;
		}
	      missionNumber = i;
	      printf ("simware using mission %d\n", missionNumber);
	    }
	  else
	    {
	      errprintf ("sup_init: unrecognized option: %s\n", argv[t]);
	      retval = -1;
	      break;
	    }
	}
      ulapi_free_argv (argc, argv);
      // now we can't reference argv anymore
    }

  if (0 != retval)
    {
      cleanup ();
      return retval;
    }

  if (NULL == nmlFile)
    nmlFile = getenv ("CONFIG_NML");
  if (NULL == nmlFile)
    nmlFile = const_cast < char *>(DEFAULT_NML_FILE);

  if (NULL == iniFile)
    iniFile = getenv ("CONFIG_INI");
  if (NULL == iniFile)
    iniFile = const_cast < char *>(DEFAULT_INI_FILE);

  dbprintf
    ("sup_init: using NML file %s, INI file %s, debug %s, verbose %s\n",
     nmlFile, iniFile, debug ? "true" : "false", verbose ? "true" : "false");

  inf_tell = itell;

  if (INI_INVALID == iniLoad (iniFile,
			      &servoMobJASet,
			      &servoMobJASetGrdVeh,
			      &servoMobJASetAirBot, &servoSPSet))
    {
      cleanup ();
      return -1;
    }

  /*
     initialize all the globals -- no need to mutex this now, since
     no other threads are yet running
   */

  for (t = 0; t < MAX_SENSOR_DATA_3D; t++)
    {
      sonarNames[t][0] = 0;
    }
  sonarNumber = 0;

  for (t = 0; t < MAX_SENSOR_DATA_1D_BUFFERS; t++)
    {
      rangescannerBuf[t] = NULL;
      rangescannerNames[t][0] = 0;
    }

  for (t = 0; t < MAX_SENSOR_DATA_2D_BUFFERS; t++)
    {
      rangeimagerBuf[t] = NULL;
      rangeimagerNames[t][0] = 0;
    }

  for (t = 0; t < MAX_SENSOR_DATA_BUFFERS; t++)
    {
      sensorDataBuf[t] = NULL;
      sensorDataNames[t][0] = 0;
    }

  for (t = 0; t < SERVO_MIS_JA_MIS_PKG_MAX; t++)
    {
      servoMisJANodes[t] = NULL;
      servoMisJANames[t][0] = 0;
      servoMisJANodes[t] = NULL;
    }

  for (t = 0; t < SERVO_EFF_JA_EFFECTOR_MAX; t++)
    {
      servoEffJANodes[t] = NULL;
      servoEffJANames[t][0] = 0;
      servoEffJANodes[t] = NULL;
    }

  servoFactJANode = NULL;

  reset_time_tracker (&servoMobJAStat.tt);
  reset_time_tracker (&servoMobJAStatGrdVeh.tt);
  reset_time_tracker (&servoMobJAStatAirBot.tt);
  reset_time_tracker (&servoSPStat.tt);

#define INIT_MUTEX(NAME,KEY)			\
  if (NULL == (NAME = ulapi_mutex_new(KEY))) {	\
    cleanup();					\
    return -1;					\
  }
  INIT_MUTEX (servoMobJAStatMutex, SERVO_MOB_JA_STAT_KEY);
  INIT_MUTEX (servoMobJASetMutex, SERVO_MOB_JA_SET_KEY);
  INIT_MUTEX (servoSPStatMutex, SERVO_SP_STAT_KEY);
  INIT_MUTEX (servoSPSetMutex, SERVO_SP_SET_KEY);

  if (!getServoMobJABufs (thisProcess, nmlFile, missionNumber))
    {
      cleanup ();
      return -1;
    }

  if (!getServoSPBufs (thisProcess, nmlFile, missionNumber))
    {
      cleanup ();
      return -1;
    }

  /*
     We have to defer starting the other 'ask' threads until after
     the first call to our 'skin_sup_ask' function, since we don't
     know if the inferior skin init has happened until that point.
   */

  return 0;
}

/*
  In sup_ask, we run one pass through the mobility command handling
  function that blocks on the NML read. Since we have more blocking
  reads to do, we need threads for those.
*/
int
skin_sup_ask (void)
{
  // here is where we start the other 'ask' threads
#define START_THREAD_IF_NEEDED(TASK,THREAD)				\
  if (NULL == TASK) {							\
    TASK = ulapi_task_new();						\
    if (NULL != TASK) {							\
      if (ULAPI_OK == ulapi_task_start(TASK,				\
				       THREAD,				\
				       NULL,				\
				       ulapi_prio_lowest(),		\
				       1)) {				\
      } else {								\
	ulapi_task_delete(TASK);					\
	TASK = NULL;							\
	errprintf("can't start one of the superior threads\n");	\
      }									\
    } else {								\
      errprintf("can't create one of the superior tasks\n");	\
    }									\
  }

  // no 'servoMobJACmdReadTask' - this is part of 'sup_ask'
  START_THREAD_IF_NEEDED (servoMobJACmdPassTask, servoMobJACmdPassThr);
  START_THREAD_IF_NEEDED (servoMobJACfgReadTask, servoMobJACfgReadThr);
  START_THREAD_IF_NEEDED (servoMobJACfgPassTask, servoMobJACfgPassThr);

  START_THREAD_IF_NEEDED (servoSPCmdReadTask, servoSPCmdReadThr);
  START_THREAD_IF_NEEDED (servoSPCmdPassTask, servoSPCmdPassThr);
  START_THREAD_IF_NEEDED (servoSPCfgReadTask, servoSPCfgReadThr);
  START_THREAD_IF_NEEDED (servoSPCfgPassTask, servoSPCfgPassThr);

  if (0 != servoMobJACmdRead ())
    {
      esleep (servoMobJACycleTime);
    }

  return 0;
}

int
skin_sup_tell (sw_struct * sw)
{
  /*
     sonarData needs to be static, since each sonar is reported
     in a separate call to sup_tell, and they must be aggregated
     into a single SensorData3D buffer
   */
  static SensorData3D sonarData;
  SensorData1D rangescannerData;
  static SensorData2D rangeimagerData;
  static double *rangeData = NULL;
  SensorData sensorData;
  int t;
  int num;
  int index;
  bool foundIt;

  switch (sw_get_type (sw))
    {
    case SW_SEN_ENCODER:
      switch (sw_get_op (sw))
	{
	case SW_SEN_ENCODER_STAT:
	  vbprintf ("Encoder status for %s at time %f: %d\n", sw->name,
		    sw->time, sw->data.encoder.tick);
	  if (0 == copyEncoder (&sensorData, sw))
	    {
	      num = sensorDataIndex (sw->name);
	      if (getSensorDataBuf (thisProcess, nmlFile, num, missionNumber))
		{
		  sensorDataBuf[num]->write (&sensorData);
		}
	      else
		{
		  errprintf ("Encoder error for %s: can't find NML buffer\n",
			     sw->name);
		  return -1;
		}
	    }
	  else
	    {
	      errprintf ("Encoder error for %s: can't copy it\n", sw->name);
	      return -1;
	    }
	  break;
	case SW_SEN_ENCODER_SET:
	  vbprintf ("Encoder settings for %s: ", sw->name);
	  vbprintf ("%f %f,%f,%f %f,%f,%f\n",
		    sw->data.encoder.resolution,
		    sw->data.encoder.mount.x,
		    sw->data.encoder.mount.y,
		    sw->data.encoder.mount.z,
		    sw->data.encoder.mount.roll,
		    sw->data.encoder.mount.pitch, sw->data.encoder.mount.yaw);

	  if (0 == copyEncoder (&sensorData, sw))
	    {
	      num = sensorDataIndex (sw->name);
	      if (getSensorDataBuf (thisProcess, nmlFile, num, missionNumber))
		{
		  sensorDataBuf[num]->write (&sensorData);
		}
	      else
		{
		  errprintf ("Encoder error for %s: can't get NML buffer\n",
			     sw->name);
		  return -1;
		}

	      // add it to servo's list of sensors if it's new
	      ulapi_mutex_take (servoSPSetMutex);
	      foundIt = false;
	      for (t = 0; t < servoSPSet.sSetElem_length; t++)
		{
		  if (servoSPSet.sSetElem[t].sType == SERVO_SP_ENCODER_SENSOR
		      && servoSPSet.sSetElem[t].sensorID == num)
		    {
		      foundIt = true;
		      break;
		    }
		}
	      if (!foundIt)
		{
		  index = servoSPSet.sSetElem_length;
		  servoSPSet.sSetElem[index].sType = SERVO_SP_ENCODER_SENSOR;
		  ulapi_strncpy (servoSPSet.sSetElem[index].senName, sw->name,
				 MOAST_NML_BUFFER_NAME_LEN);
		  servoSPSet.sSetElem[index].sensorID = num;
		  servoSPSet.sSetElem[index].sensorToMount =
		    PM_POSE (PM_CARTESIAN (sw->data.encoder.mount.x,
					   sw->data.encoder.mount.y,
					   sw->data.encoder.mount.z),
			     PM_RPY (sw->data.encoder.mount.roll,
				     sw->data.encoder.mount.pitch,
				     sw->data.encoder.mount.yaw));
		  servoSPSet.sSetElem_length++;
		  servoSPSetBuf->write (&servoSPSet);
		}
	      ulapi_mutex_give (servoSPSetMutex);

	    }
	  else
	    {
	      errprintf ("Encoder error for %s: can't copy it\n", sw->name);
	      return -1;
	    }
	  // update the mobility's sensor count
	  servoMobJASetUpdate ();
	  break;
	default:
	  errprintf ("invalid operation: %d\n", sw_get_op (sw));
	  return -1;
	  break;
	}
      break;

    case SW_SEN_SONAR:
      switch (sw_get_op (sw))
	{
	case SW_SEN_SONAR_STAT:
	  vbprintf ("Sonar status for %s at time %f: %f\n", sw->name,
		    sw->time, sw->data.sonar.range);
	  if (0 == copySonar (&sonarData, sw))
	    {
	      if (getSonarBuf (thisProcess, nmlFile, missionNumber))
		{
		  sonarBuf->write (&sonarData);
		}
	      else
		{
		  errprintf ("Sonar error for %s: can't get NML buffer\n",
			     sw->name);
		  return -1;
		}
	    }
	  else
	    {
	      errprintf ("Sonar error for %s: can't find its index\n",
			 sw->name);
	      return -1;
	    }
	  break;
	case SW_SEN_SONAR_SET:
	  vbprintf ("Sonar settings for %s: ", sw->name);
	  vbprintf ("%f %f %f %f,%f,%f %f,%f,%f\n",
		    sw->data.sonar.minrange,
		    sw->data.sonar.maxrange,
		    sw->data.sonar.beamangle,
		    sw->data.sonar.mount.x,
		    sw->data.sonar.mount.y,
		    sw->data.sonar.mount.z,
		    sw->data.sonar.mount.roll,
		    sw->data.sonar.mount.pitch, sw->data.sonar.mount.yaw);
	  if (0 == copySonar (&sonarData, sw))
	    {
	      if (getSonarBuf (thisProcess, nmlFile, missionNumber))
		{
		  sonarBuf->write (&sonarData);
		}
	      else
		{
		  errprintf ("Sonar error for %s: can't get NML buffer\n",
			     sw->name);
		  return -1;
		}
	    }
	  else
	    {
	      errprintf ("Sonar error for %s: can't find its index\n",
			 sw->name);
	      return -1;
	    }
	  // update the mobility's sensor count
	  servoMobJASetUpdate ();
	  break;
	default:
	  errprintf ("invalid operation: %d\n", sw_get_op (sw));
	  return -1;
	  break;
	}
      break;

    case SW_SEN_RANGEIMAGER:
      switch (sw_get_op (sw))
	{
	case SW_SEN_RANGEIMAGER_STAT:
	  num = sw->data.rangeimager.numberperframe;
	  if (num > SW_SEN_RANGEIMAGER_MAX)
	    num = SW_SEN_RANGEIMAGER_MAX;
	  if (num > MAX_SENSOR_DATA_2D)
	    num = MAX_SENSOR_DATA_2D;
	  if (verbose)
	    {
	      for (t = 0; t < num; t++)
		{
		  vbprintf ("%f ", sw->data.rangeimager.range[t]);
		}
	      vbprintf ("\n");
	    }
	  if (1 == copyRangeimager (&rangeimagerData, &rangeData, sw))
	    {
	      num = rangeimagerIndex (sw->name);
	      if (getRangeimagerBuf
		  (thisProcess, nmlFile, num, missionNumber))
		{
		  rangeimagerBuf[num]->write (&rangeimagerData);
		  writeRangeImageData ((char *) "rangeData.dat",
				       &rangeimagerData, rangeData);
		  printf ("moastsup: wrote range data\n");
		}
	      else
		{
		  errprintf
		    ("RangeImager error for %s: can't find NML buffer\n",
		     sw->name);
		  return -1;
		}
	    }
	  break;

	case SW_SEN_RANGEIMAGER_SET:
	  printf ("moastsup: RangeImager settings for %s: ", sw->name);
	  printf ("%f %f %f %f %f,%f,%f %f,%f,%f, %f, %f\n",
		  sw->data.rangeimager.minrange,
		  sw->data.rangeimager.maxrange,
		  sw->data.rangeimager.resolutionx,
		  sw->data.rangeimager.resolutiony,
		  sw->data.rangeimager.fovx,
		  sw->data.rangeimager.fovy,
		  sw->data.rangeimager.mount.x,
		  sw->data.rangeimager.mount.y,
		  sw->data.rangeimager.mount.z,
		  sw->data.rangeimager.mount.roll,
		  sw->data.rangeimager.mount.pitch,
		  sw->data.rangeimager.mount.yaw);
	  if (0 == copyRangeimager (&rangeimagerData, &rangeData, sw))
	    {
	      num = rangeimagerIndex (sw->name);
	      if (getRangeimagerBuf
		  (thisProcess, nmlFile, num, missionNumber))
		{
		  rangeimagerBuf[num]->write (&rangeimagerData);
		}
	      else
		{
		  errprintf
		    ("moastsup: RangeImager error for %s: can't find NML buffer\n",
		     sw->name);
		  return -1;
		}

	      // add it to servo's list of sensors if it's new
	      ulapi_mutex_take (servoSPSetMutex);
	      foundIt = false;
	      for (t = 0; t < servoSPSet.sSetElem_length; t++)
		{
		  if (servoSPSet.sSetElem[t].sType ==
		      SERVO_SP_LINEIMAGER_SENSOR
		      && servoSPSet.sSetElem[t].sensorID == num)
		    {
		      foundIt = true;
		      break;
		    }
		}
	      if (!foundIt)
		{
		  index = servoSPSet.sSetElem_length;
		  servoSPSet.sSetElem[index].sType =
		    SERVO_SP_LINEIMAGER_SENSOR;
		  ulapi_strncpy (servoSPSet.sSetElem[index].senName, sw->name,
				 MOAST_NML_BUFFER_NAME_LEN);
		  servoSPSet.sSetElem[index].sensorID = num;
		  servoSPSet.sSetElem[index].sensorToMount =
		    PM_POSE (PM_CARTESIAN (sw->data.rangeimager.mount.x,
					   sw->data.rangeimager.mount.y,
					   sw->data.rangeimager.mount.z),
			     PM_RPY (sw->data.rangeimager.mount.roll,
				     sw->data.rangeimager.mount.pitch,
				     sw->data.rangeimager.mount.yaw));
		  servoSPSet.sSetElem_length++;
		  servoSPSetBuf->write (&servoSPSet);
		}
	      ulapi_mutex_give (servoSPSetMutex);

	    }
	  else
	    {
	      errprintf ("RangeImager error for %s: can't copy it\n",
			 sw->name);
	      return -1;
	    }
	  // update the mobility's sensor count
	  servoMobJASetUpdate ();
	  break;
	default:
	  errprintf ("invalid operation for rangeImager: %d\n",
		     sw_get_op (sw));
	  return -1;
	  break;
	}

      break;

    case SW_SEN_RANGESCANNER:
      switch (sw_get_op (sw))
	{
	case SW_SEN_RANGESCANNER_STAT:
	  vbprintf ("RangeScanner status for %s at time %f: ", sw->name,
		    sw->time);
	  num = sw->data.rangescanner.number;
	  if (num > SW_SEN_RANGESCANNER_MAX)
	    num = SW_SEN_RANGESCANNER_MAX;
	  if (num > MAX_SENSOR_DATA_1D)
	    num = MAX_SENSOR_DATA_1D;
	  if (verbose)
	    {
	      for (t = 0; t < num; t++)
		{
		  vbprintf ("%f ", sw->data.rangescanner.range[t]);
		}
	      vbprintf ("\n");
	    }
	  if (0 == copyRangescanner (&rangescannerData, sw))
	    {
	      num = rangescannerIndex (sw->name);
	      if (getRangescannerBuf
		  (thisProcess, nmlFile, num, missionNumber))
		{
		  rangescannerBuf[num]->write (&rangescannerData);
		}
	      else
		{
		  errprintf
		    ("RangeScanner error for %s: can't find NML buffer\n",
		     sw->name);
		  return -1;
		}
	    }
	  else
	    {
	      errprintf ("RangeScanner error for %s: can't copy it\n",
			 sw->name);
	      return -1;
	    }
	  break;

	case SW_SEN_RANGESCANNER_SET:
	  vbprintf ("RangeScanner settings for %s: ", sw->name);
	  vbprintf ("%f %f %f %f %f,%f,%f %f,%f,%f\n",
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
	  if (0 == copyRangescanner (&rangescannerData, sw))
	    {
	      num = rangescannerIndex (sw->name);
	      if (getRangescannerBuf
		  (thisProcess, nmlFile, num, missionNumber))
		{
		  rangescannerBuf[num]->write (&rangescannerData);
		}
	      else
		{
		  errprintf
		    ("RangeScanner error for %s: can't find NML buffer\n",
		     sw->name);
		  return -1;
		}

	      // add it to servo's list of sensors if it's new
	      ulapi_mutex_take (servoSPSetMutex);
	      foundIt = false;
	      for (t = 0; t < servoSPSet.sSetElem_length; t++)
		{
		  if (servoSPSet.sSetElem[t].sType ==
		      SERVO_SP_LINESCANNER_SENSOR
		      && servoSPSet.sSetElem[t].sensorID == num)
		    {
		      foundIt = true;
		      break;
		    }
		}
	      if (!foundIt)
		{
		  index = servoSPSet.sSetElem_length;
		  servoSPSet.sSetElem[index].sType =
		    SERVO_SP_LINESCANNER_SENSOR;
		  ulapi_strncpy (servoSPSet.sSetElem[index].senName, sw->name,
				 MOAST_NML_BUFFER_NAME_LEN);
		  servoSPSet.sSetElem[index].sensorID = num;
		  servoSPSet.sSetElem[index].sensorToMount =
		    PM_POSE (PM_CARTESIAN (sw->data.rangescanner.mount.x,
					   sw->data.rangescanner.mount.y,
					   sw->data.rangescanner.mount.z),
			     PM_RPY (sw->data.rangescanner.mount.roll,
				     sw->data.rangescanner.mount.pitch,
				     sw->data.rangescanner.mount.yaw));
		  servoSPSet.sSetElem_length++;
		  servoSPSetBuf->write (&servoSPSet);
		}
	      ulapi_mutex_give (servoSPSetMutex);

	    }
	  else
	    {
	      errprintf ("RangeScanner error for %s: can't copy it\n",
			 sw->name);
	      return -1;
	    }
	  // update the mobility's sensor count
	  servoMobJASetUpdate ();
	  break;
	default:
	  errprintf ("invalid operation: %d\n", sw_get_op (sw));
	  return -1;
	  break;
	}
      break;

    case SW_SEN_TOUCH:
      switch (sw_get_op (sw))
	{
	case SW_SEN_TOUCH_STAT:
	  break;
	case SW_SEN_TOUCH_SET:
	  break;
	default:
	  break;
	}
      break;

    case SW_SEN_CO2:
      switch (sw_get_op (sw))
	{
	case SW_SEN_CO2_STAT:
	  break;
	case SW_SEN_CO2_SET:
	  break;
	default:
	  break;
	}
      break;

    case SW_SEN_GROUNDTRUTH:
      switch (sw_get_op (sw))
	{
	case SW_SEN_GROUNDTRUTH_STAT:
	  vbprintf
	    ("GroundTruth status for %s at time %f: %f,%f,%f %f,%f,%f\n",
	     sw->name, sw->time, sw->data.groundtruth.position.x,
	     sw->data.groundtruth.position.y, sw->data.groundtruth.position.z,
	     sw->data.groundtruth.position.roll,
	     sw->data.groundtruth.position.pitch,
	     sw->data.groundtruth.position.yaw);
	  if (0 == copyGroundtruth (&sensorData, sw))
	    {
	      num = sensorDataIndex (sw->name);
	      if (getSensorDataBuf (thisProcess, nmlFile, num, missionNumber))
		{
		  sensorDataBuf[num]->write (&sensorData);
		}
	      else
		{
		  errprintf
		    ("GroundTruth error for %s: can't get NML buffer\n",
		     sw->name);
		  return -1;
		}
	    }
	  else
	    {
	      errprintf ("GroundTruth error for %s: can't copy it\n",
			 sw->name);
	      return -1;
	    }
	  break;
	case SW_SEN_GROUNDTRUTH_SET:
	  vbprintf ("GroundTruth settings for %s: %f %f,%f,%f %f,%f,%f\n",
		    sw->name,
		    sw->data.groundtruth.period,
		    sw->data.groundtruth.mount.x,
		    sw->data.groundtruth.mount.y,
		    sw->data.groundtruth.mount.z,
		    sw->data.groundtruth.mount.roll,
		    sw->data.groundtruth.mount.pitch,
		    sw->data.groundtruth.mount.yaw);
	  if (0 == copyGroundtruth (&sensorData, sw))
	    {
	      num = sensorDataIndex (sw->name);
	      if (getSensorDataBuf (thisProcess, nmlFile, num, missionNumber))
		{
		  sensorDataBuf[num]->write (&sensorData);
		}
	      else
		{
		  errprintf
		    ("GroundTruth error for %s: can't get NML buffer\n",
		     sw->name);
		  return -1;
		}

	      // add it to servo's list of sensors if it's new
	      ulapi_mutex_take (servoSPSetMutex);
	      foundIt = false;
	      for (t = 0; t < servoSPSet.sSetElem_length; t++)
		{
		  if (servoSPSet.sSetElem[t].sType ==
		      SERVO_SP_GRD_TRUTH_SENSOR
		      && servoSPSet.sSetElem[t].sensorID == num)
		    {
		      foundIt = true;
		      break;
		    }
		}
	      if (!foundIt)
		{
		  index = servoSPSet.sSetElem_length;
		  servoSPSet.sSetElem[index].sType =
		    SERVO_SP_GRD_TRUTH_SENSOR;
		  ulapi_strncpy (servoSPSet.sSetElem[index].senName, sw->name,
				 MOAST_NML_BUFFER_NAME_LEN);
		  servoSPSet.sSetElem[index].sensorID = num;
		  servoSPSet.sSetElem[index].sensorToMount =
		    PM_POSE (PM_CARTESIAN (sw->data.groundtruth.mount.x,
					   sw->data.groundtruth.mount.y,
					   sw->data.groundtruth.mount.z),
			     PM_RPY (sw->data.groundtruth.mount.roll,
				     sw->data.groundtruth.mount.pitch,
				     sw->data.groundtruth.mount.yaw));
		  servoSPSet.sSetElem_length++;
		  servoSPSetBuf->write (&servoSPSet);
		}
	      ulapi_mutex_give (servoSPSetMutex);

	    }
	  else
	    {
	      errprintf ("GroundTruth error for %s: can't copy it\n",
			 sw->name);
	      return -1;
	    }
	  // update the mobility's sensor count
	  servoMobJASetUpdate ();
	  break;
	default:
	  break;
	}
      break;

    case SW_SEN_INS:
      switch (sw_get_op (sw))
	{
	case SW_SEN_INS_STAT:
	  break;
	case SW_SEN_INS_SET:
	  break;
	default:
	  break;
	}
      break;

    case SW_SEN_GPS:
      switch (sw_get_op (sw))
	{
	case SW_SEN_GPS_STAT:
	  break;
	case SW_SEN_GPS_SET:
	  break;
	default:
	  break;
	}
      break;

    case SW_SEN_ODOMETER:
      switch (sw_get_op (sw))
	{
	case SW_SEN_ODOMETER_STAT:
	  vbprintf ("Odometer status for %s at time %f: %f,%f,%f", sw->name,
		    sw->time, sw->data.odometer.position.x,
		    sw->data.odometer.position.y,
		    sw->data.odometer.position.z);
	  if (0 == copyOdometer (&sensorData, sw))
	    {
	      num = sensorDataIndex (sw->name);
	      if (getSensorDataBuf (thisProcess, nmlFile, num, missionNumber))
		{
		  sensorDataBuf[num]->write (&sensorData);
		}
	      else
		{
		  errprintf ("Odometer error for %s: can't find NML buffer\n",
			     sw->name);
		  return -1;
		}
	    }
	  else
	    {
	      errprintf ("Odometer error for %s: can't copy it\n", sw->name);
	      return -1;
	    }
	  break;
	case SW_SEN_ODOMETER_SET:
	  vbprintf ("not handling odometer settings for %s\n", sw->name);
	  // update the mobility's sensor count
	  servoMobJASetUpdate ();
	  break;
	default:
	  errprintf ("invalid operation: %d\n", sw_get_op (sw));
	  return -1;
	  break;
	}
      break;

    case SW_SEN_TACHOMETER:
      switch (sw_get_op (sw))
	{
	case SW_SEN_TACHOMETER_STAT:
	  vbprintf ("Tachometer status for %s at time %f: ", sw->name,
		    sw->time);
	  num = sw->data.tachometer.number;
	  if (num > SW_SEN_TACHOMETER_MAX)
	    num = SW_SEN_TACHOMETER_MAX;
	  for (t = 0; t < num; t++)
	    {
	      vbprintf ("%f,%f ", sw->data.tachometer.speed[t],
			sw->data.tachometer.position[t]);
	    }
	  vbprintf ("\n");
	  if (0 == copyTachometer (&sensorData, sw))
	    {
	      num = sensorDataIndex (sw->name);
	      if (getSensorDataBuf (thisProcess, nmlFile, num, missionNumber))
		{
		  sensorDataBuf[num]->write (&sensorData);
		}
	      else
		{
		  errprintf
		    ("Tachometer error for %s: can't find NML buffer\n",
		     sw->name);
		  return -1;
		}
	    }
	  else
	    {
	      errprintf ("Tachometer error for %s: can't copy it\n",
			 sw->name);
	      return -1;
	    }
	  break;
	case SW_SEN_TACHOMETER_SET:
	  vbprintf ("not handling tachometer settings for %s\n", sw->name);
	  // update the mobility's effector count
	  servoMobJASetUpdate ();
	  break;
	default:
	  errprintf ("invalid operation: %d\n", sw_get_op (sw));
	  return -1;
	  break;
	}
      break;

    case SW_SEN_ACOUSTIC:
      switch (sw_get_op (sw))
	{
	case SW_SEN_ACOUSTIC_STAT:
	  vbprintf ("Acoustic status for %s at time %f: ", sw->name,
		    sw->time);
	  vbprintf ("%f,%f,%f,%f\n", sw->data.acoustic.azimuth,
		    sw->data.acoustic.altitude, sw->data.acoustic.volume,
		    sw->data.acoustic.duration);
	  if (0 == copyAcoustic (&sensorData, sw))
	    {
	      num = sensorDataIndex (sw->name);
	      if (getSensorDataBuf (thisProcess, nmlFile, num, missionNumber))
		{
		  sensorDataBuf[num]->write (&sensorData);
		}
	      else
		{
		  errprintf ("Acoustic error for %s: can't find NML buffer\n",
			     sw->name);
		  return -1;
		}
	    }
	  else
	    {
	      errprintf ("Acoustic error for %s: can't copy it\n", sw->name);
	      return -1;
	    }
	  break;
	case SW_SEN_ACOUSTIC_SET:
	  vbprintf ("not handling acoustic settings for %s\n", sw->name);
	  // update the mobility's effector count
	  servoMobJASetUpdate ();
	  break;
	default:
	  errprintf ("invalid operation: %d\n", sw_get_op (sw));
	  return -1;
	  break;
	}
      break;

    case SW_SEN_VICTIM:
      switch (sw_get_op (sw))
	{
	case SW_SEN_VICTIM_STAT:
	  break;
	case SW_SEN_VICTIM_SET:
	  break;
	default:
	  break;
	}
      break;

    case SW_ACT:

      switch (sw_get_op (sw))
	{

	case SW_ACT_STAT:
	  t = servoMisJAIndex (sw->name);
	  if (t < 0)
	    {
	      errprintf ("can't allocate ServoMisJANode for %s\n", sw->name);
	      break;
	    }
	  if (NULL == servoMisJANodes[t])
	    {
	      servoMisJANodes[t] = new ServoMisJANode ();
	      if (0 !=
		  (servoMisJANodes[t])->init (thisProcess, nmlFile,
					      missionNumber, t + 1, sw->name,
					      inf_tell))
		{
		  delete servoMisJANodes[t];
		  servoMisJANodes[t] = NULL;
		  break;
		}
	    }
	  servoMisJANodes[t]->takeStatMutex ();
	  if (0 == servoMisJANodes[t]->setStat (sw))
	    {
	      servoMisJANodes[t]->writeStat ();
	      servoMisJANodes[t]->giveStatMutex ();
	    }
	  else
	    {
	      errprintf ("Mispkg error for %s: can't copy it\n", sw->name);
	      servoMisJANodes[t]->giveStatMutex ();
	      return -1;
	    }
	  break;

	case SW_ACT_SET:
	  t = servoMisJAIndex (sw->name);
	  if (t < 0)
	    {
	      errprintf ("can't allocate ServoMisJANode for %s\n", sw->name);
	      break;
	    }
	  if (NULL == servoMisJANodes[t])
	    {
	      servoMisJANodes[t] = new ServoMisJANode ();
	      if (0 !=
		  (servoMisJANodes[t])->init (thisProcess, nmlFile,
					      missionNumber, t + 1, sw->name,
					      inf_tell))
		{
		  delete servoMisJANodes[t];
		  servoMisJANodes[t] = NULL;
		  break;
		}
	    }
	  servoMisJANodes[t]->takeSetMutex ();
	  if (0 == servoMisJANodes[t]->setSet (sw))
	    {
	      servoMisJANodes[t]->writeSet ();
	      servoMisJANodes[t]->giveSetMutex ();
	    }
	  else
	    {
	      errprintf ("Mispkg error for %s: can't copy it\n", sw->name);
	      servoMisJANodes[t]->giveSetMutex ();
	      return -1;
	    }
	  // update the mobility's effector count
	  servoMobJASetUpdate ();
	  break;

	default:
	  break;
	}
      break;

    case SW_EFF_GRIPPER:
      switch (sw_get_op (sw))
	{

	case SW_EFF_GRIPPER_STAT:
	  t = servoEffJAIndex (sw->name);
	  if (t < 0)
	    {
	      errprintf ("can't allocate ServoEffJANode for %s\n", sw->name);
	      break;
	    }
	  if (NULL == servoEffJANodes[t])
	    {
	      servoEffJANodes[t] = new ServoEffJANode ();
	      if (0 !=
		  (servoEffJANodes[t])->init (thisProcess, nmlFile,
					      missionNumber, t + 1, sw->name,
					      inf_tell))
		{
		  delete servoEffJANodes[t];
		  servoEffJANodes[t] = NULL;
		  break;
		}
	    }
	  servoEffJANodes[t]->takeStatMutex ();
	  if (0 == servoEffJANodes[t]->setStat (sw))
	    {
	      servoEffJANodes[t]->writeStat ();
	      servoEffJANodes[t]->giveStatMutex ();
	    }
	  else
	    {
	      errprintf ("Effector error for %s: can't copy it\n", sw->name);
	      servoEffJANodes[t]->giveStatMutex ();
	      return -1;
	    }
	  break;

	case SW_EFF_GRIPPER_SET:
	  t = servoEffJAIndex (sw->name);
	  if (t < 0)
	    {
	      errprintf ("can't allocate ServoEffJANode for %s\n", sw->name);
	      break;
	    }
	  if (NULL == servoEffJANodes[t])
	    {
	      servoEffJANodes[t] = new ServoEffJANode ();
	      if (0 !=
		  (servoEffJANodes[t])->init (thisProcess, nmlFile,
					      missionNumber, t + 1, sw->name,
					      inf_tell))
		{
		  delete servoEffJANodes[t];
		  servoEffJANodes[t] = NULL;
		  break;
		}
	    }
	  servoEffJANodes[t]->takeSetMutex ();
	  if (0 == servoEffJANodes[t]->setSet (sw))
	    {
	      servoEffJANodes[t]->writeSet ();
	      servoEffJANodes[t]->giveSetMutex ();
	    }
	  else
	    {
	      errprintf ("Effector error for %s: can't copy it\n", sw->name);
	      servoEffJANodes[t]->giveSetMutex ();
	      return -1;
	    }
	  // update the mobility's effector count
	  servoMobJASetUpdate ();
	  break;

	default:
	  break;
	}
      break;

    case SW_ROBOT_FIXED:
      switch (sw_get_op (sw))
	{

	case SW_DEVICE_STAT:
	  if (NULL == servoFactJANode)
	    {
	      servoFactJANode = new ServoFactJANode ();
	      if (0 !=
		  (servoFactJANode)->init (thisProcess, nmlFile,
					   missionNumber, sw->name, inf_tell))
		{
		  delete servoFactJANode;
		  servoFactJANode = NULL;
		  break;
		}
	    }
	  servoFactJANode->takeStatMutex ();
	  if (0 == servoFactJANode->setStat (sw))
	    {
	      servoFactJANode->writeStat ();
	      servoFactJANode->giveStatMutex ();
	    }
	  else
	    {
	      errprintf ("Device error for %s: can't copy it\n", sw->name);
	      servoFactJANode->giveStatMutex ();
	      return -1;
	    }
	  break;

	case SW_DEVICE_SET:
	  if (NULL == servoFactJANode)
	    {
	      servoFactJANode = new ServoFactJANode ();
	      if (0 !=
		  (servoFactJANode)->init (thisProcess, nmlFile,
					   missionNumber, sw->name, inf_tell))
		{
		  delete servoFactJANode;
		  servoFactJANode = NULL;
		  break;
		}
	    }
	  servoFactJANode->takeSetMutex ();
	  if (0 == servoFactJANode->setSet (sw))
	    {
	      servoFactJANode->writeSet ();
	      servoFactJANode->giveSetMutex ();
	    }
	  else
	    {
	      errprintf ("Device error for %s: can't copy it\n", sw->name);
	      servoFactJANode->giveSetMutex ();
	      return -1;
	    }
	  break;

	default:
	  break;
	}
      break;

    case SW_ROBOT_GROUNDVEHICLE:
      switch (sw_get_op (sw))
	{
	case SW_ROBOT_STAT:
	  if (SERVO_MOB_JA_STAT_TYPE == botType)
	    {
	      // first time we know about the robot type
	      botType = SERVO_MOB_JA_STAT_GRD_VEH_TYPE;
	    }
	  else if (SERVO_MOB_JA_STAT_GRD_VEH_TYPE != botType)
	    {
	      // it was already set to something else
	      errprintf ("GroundVehicle overriding %d\n", botType);
	    }
	  break;
	case SW_ROBOT_SET:
	  if (SERVO_MOB_JA_STAT_TYPE == botType)
	    {
	      // first time we know about the robot type
	      botType = SERVO_MOB_JA_STAT_GRD_VEH_TYPE;
	    }
	  else if (SERVO_MOB_JA_STAT_GRD_VEH_TYPE != botType)
	    {
	      // it was already set to something else
	      errprintf ("GroundVehicle overriding %d\n", botType);
	    }
	  ulapi_mutex_take (servoMobJASetMutex);
	  copyServoMobJASetGrdVeh (&servoMobJASetGrdVeh, sw);
	  servoMobJASetBuf->write (&servoMobJASetGrdVeh);
	  ulapi_mutex_give (servoMobJASetMutex);
	  break;
	}
      break;

    case SW_ROBOT_AIRBOT:
      switch (sw_get_op (sw))
	{
	case SW_ROBOT_STAT:
	  if (SERVO_MOB_JA_STAT_TYPE == botType)
	    {
	      // first time we know about the robot type
	      botType = SERVO_MOB_JA_STAT_AIR_BOT_TYPE;
	    }
	  else if (SERVO_MOB_JA_STAT_AIR_BOT_TYPE != botType)
	    {
	      // it was already set to something else
	      errprintf ("AirBot overriding %d\n", botType);
	    }
	  break;
	case SW_ROBOT_SET:
	  if (SERVO_MOB_JA_STAT_TYPE == botType)
	    {
	      // first time we know about the robot type
	      botType = SERVO_MOB_JA_STAT_AIR_BOT_TYPE;
	    }
	  else if (SERVO_MOB_JA_STAT_AIR_BOT_TYPE != botType)
	    {
	      // it was already set to something else
	      errprintf ("AirBot overriding %d\n", botType);
	    }
	  break;
	}
      dbprintf ("not yet fully handling AirBots\n");
      break;

    default:
      errprintf ("Simware class %s op %d not handled\n",
		 sw_type_to_string (sw->type), sw->op);
      return -1;
      break;
    }				// switch (sw_get_type(sw))

  return 0;
}

int
skin_sup_fini (void)
{
  dbprintf ("moastsup: sup_fini\n");

  cleanup ();

  return 0;
}

#if 0
{
#endif
#ifdef __cplusplus
}
#endif
