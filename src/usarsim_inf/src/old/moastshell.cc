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
  \file moastshell.cc

  \brief Command line shell to test Moast superior skin.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <stdio.h>
#include <stddef.h>		// sizeof(), NULL
#include <string.h>		// strncmp()
#include <stdlib.h>		// getenv()
#include <ctype.h>		// isspace
#ifdef HAVE_GETOPT_H
#include <getopt.h>
#else
#include "getopt.h"
#endif
#ifdef HAVE_READLINE_READLINE_H
#include <readline/readline.h>
#include <readline/history.h>
#endif
#include <rcs.hh>
#include <navDataExt.hh>
#include <sensorData.hh>
#include <servoMobJA.hh>
#include <servoMisJA.hh>
#include <servoEffJA.hh>
#include <servoSP.hh>
#include "moastbufs.h"
#include "ulapi.h"
#include "gotypes.h"
#include "gomath.h"
#include "genserkins.h"

static char thisProcess[] = "servoShell";

#define DEFAULT_NML_FILE "moast.nml"
static char nmlFile[MOAST_FILE_NAME_LEN] = DEFAULT_NML_FILE;
static bool nmlFileArg = false;

static int packageIndex = 0;
static int effectorIndex = 0;

static int missionNumber = 1;

static enum
{ WHICH_MOB, WHICH_MIS, WHICH_EFF, WHICH_SP, WHICH_FACT, WHICH_SONAR,
    WHICH_RANGE, WHICH_NAV, WHICH_SENSOR_DATA } whichSub = WHICH_MOB;
static enum
{ WHICH_STAT, WHICH_SET } whichLook = WHICH_STAT;
static int whichRange = 1;
static int whichSensor = 1;

static RCS_PRINT_DESTINATION_TYPE savedest;

void
rcs_noprint (void)
{
  savedest = get_rcs_print_destination ();
  set_rcs_print_destination (RCS_PRINT_TO_NULL);
}

void
rcs_doprint (void)
{
  set_rcs_print_destination (savedest);
}

static void
printServoMobJAStat (ServoMobJAStat * stat)
{
  printf ("command_type:       %s\n",
	  servoMobJA_symbol_lookup (stat->command_type));
  printf ("echo_serial_number: %d\n", (int) stat->echo_serial_number);
  printf ("status:             %s\n", rcs_stat_to_string (stat->status));
  printf ("state:              %d\n", (int) stat->state);
  printf ("admin_state:        %s\n",
	  rcs_admin_state_to_string (stat->admin_state));
  printf ("line:               %d\n", (int) stat->line);
  printf ("source_line:        %d\n", (int) stat->source_line);
  printf ("source_file:        %s\n", (char *) stat->source_file);
  printf ("timetracker:        %d\n", (int) stat->tt.count);
}

static void
printServoMobJAStatGrdVeh (ServoMobJAStatGrdVeh * stat)
{
  printf ("                    Ground Vehicle\n");
  printf ("command_type:       %s\n",
	  servoMobJA_symbol_lookup (stat->command_type));
  printf ("echo_serial_number: %d\n", (int) stat->echo_serial_number);
  printf ("status:             %s\n", rcs_stat_to_string (stat->status));
  printf ("state:              %d\n", (int) stat->state);
  printf ("admin_state:        %s\n",
	  rcs_admin_state_to_string (stat->admin_state));
  printf ("line:               %d\n", (int) stat->line);
  printf ("source_line:        %d\n", (int) stat->source_line);
  printf ("source_file:        %s\n", (char *) stat->source_file);
  printf ("timetracker:        %d\n", (int) stat->tt.count);
}

static void
printServoMobJAStatAirBot (ServoMobJAStatAirBot * stat)
{
  printf ("                    Air Robot\n");
  printf ("command_type:       %s\n",
	  servoMobJA_symbol_lookup (stat->command_type));
  printf ("echo_serial_number: %d\n", (int) stat->echo_serial_number);
  printf ("status:             %s\n", rcs_stat_to_string (stat->status));
  printf ("state:              %d\n", (int) stat->state);
  printf ("admin_state:        %s\n",
	  rcs_admin_state_to_string (stat->admin_state));
  printf ("line:               %d\n", (int) stat->line);
  printf ("source_line:        %d\n", (int) stat->source_line);
  printf ("source_file:        %s\n", (char *) stat->source_file);
  printf ("timetracker:        %d\n", (int) stat->tt.count);
}

static void
printServoMobJASet (ServoMobJASet * set)
{
  printf ("command_type:       %s\n",
	  servoMobJA_symbol_lookup (set->command_type));
  printf ("echo_serial_number: %d\n", (int) set->echo_serial_number);
  printf ("status:             %s\n", rcs_stat_to_string (set->status));
  printf ("state:              %d\n", (int) set->state);
  printf ("cycle_time:         %f\n", (double) set->cycleTime);

  printf ("platform name:      %s\n", set->platformName);
  printf ("L/W/H/M:            %f/%f/%f/%f\n", (double) set->length,
	  (double) set->width, (double) set->height, (double) set->mass);

  printf ("sensors:            %d\n", set->senCount);
  printf ("effectors:          %d\n", set->effCount);
  printf ("mission packages:   %d\n", set->misCount);
}

static void
printServoMobJASetGrdVeh (ServoMobJASetGrdVeh * set)
{
  printf ("                    Ground Vehicle\n");
  printf ("command_type:       %s\n",
	  servoMobJA_symbol_lookup (set->command_type));
  printf ("echo_serial_number: %d\n", (int) set->echo_serial_number);
  printf ("status:             %s\n", rcs_stat_to_string (set->status));
  printf ("state:              %d\n", (int) set->state);
  printf ("cycle_time:         %f\n", (double) set->cycleTime);

  printf ("platform name:      %s\n", set->platformName);
  printf ("steerType:          %s\n",
	  set->steerType == MOAST_SKID_STEER_TYPE ? "Skid" : set->steerType ==
	  MOAST_ACKERMAN_STEER_TYPE ? "Ackerman" : set->steerType ==
	  MOAST_OMNI_STEER_TYPE ? "Omni" : "?");

  printf ("L/W/H/M:            %f/%f/%f/%f\n", (double) set->length,
	  (double) set->width, (double) set->height, (double) set->mass);

  printf ("sensors:            %d\n", set->senCount);
  printf ("effectors:          %d\n", set->effCount);
  printf ("mission packages:   %d\n", set->misCount);
}

static void
printServoMobJASetAirBot (ServoMobJASetAirBot * set)
{
  printf ("                    Air Robot\n");
  printf ("command_type:       %s\n",
	  servoMobJA_symbol_lookup (set->command_type));
  printf ("echo_serial_number: %d\n", (int) set->echo_serial_number);
  printf ("status:             %s\n", rcs_stat_to_string (set->status));
  printf ("state:              %d\n", (int) set->state);
  printf ("cycle_time:         %f\n", (double) set->cycleTime);

  printf ("platform name:      %s\n", set->platformName);
  printf ("L/W/H/M:            %f/%f/%f/%f\n", (double) set->length,
	  (double) set->width, (double) set->height, (double) set->mass);

  printf ("sensors:            %d\n", set->senCount);
  printf ("effectors:          %d\n", set->effCount);
  printf ("mission packages:   %d\n", set->misCount);
}

static void
go_matrix_print (go_matrix * matrix)
{
  int row, col;

  for (row = 0; row < matrix->rows; row++)
    {
      for (col = 0; col < matrix->cols; col++)
	{
	  printf ("%.2f\t", matrix->el[row][col]);
	}
      printf ("\n");
    }
}

static int
invKins (go_pose * world, go_real * joints, ServoMisJAStat * stat,
	 ServoMisJASet * set)
{
  genser_struct kins;
  go_integer link_number;
  go_link link_params[SERVO_MIS_JA_LINK_MAX];
  go_pose pose;
  go_rpy rpy;
  int t;

  genser_kin_init (&kins);

  link_number = set->linkSet_length;
  if (link_number > SERVO_MIS_JA_LINK_MAX)
    link_number = SERVO_MIS_JA_LINK_MAX;

  for (t = 0; t < link_number; t++)
    {
      rpy.r = set->linkSet[t].rot.r;
      rpy.p = set->linkSet[t].rot.p;
      rpy.y = set->linkSet[t].rot.y;
      go_rpy_quat_convert (&rpy, &pose.rot);

      pose.tran.x = set->linkSet[t].loc.x;
      pose.tran.y = set->linkSet[t].loc.y;
      pose.tran.z = set->linkSet[t].loc.z;

      link_params[t].u.pp.pose = pose;
      link_params[t].type = GO_LINK_PP;
      link_params[t].quantity =
	(SERVO_MIS_JA_PRISMATIC_JOINT_TYPE ==
	 set->linkSet[t].jType ? GO_QUANTITY_LENGTH : GO_QUANTITY_ANGLE);
      joints[t] = stat->linkStat[t].jointVal;
    }

  genser_kin_set_parameters (&kins, link_params, link_number);

  if (GO_RESULT_OK != genser_kin_inv (&kins, world, joints))
    {
      return -1;
    }

  return 0;
}

static void
printKins (ServoMisJAStat * stat, ServoMisJASet * set)
{
  genser_struct kins;
  go_integer link_number;
  go_link link_params[SERVO_MIS_JA_LINK_MAX];
  go_real joints[SERVO_MIS_JA_LINK_MAX];
  go_pose pose;
  go_rpy rpy;
  int t;

  genser_kin_init (&kins);

  link_number = set->linkSet_length;
  if (link_number > SERVO_MIS_JA_LINK_MAX)
    link_number = SERVO_MIS_JA_LINK_MAX;

  for (t = 0; t < link_number; t++)
    {
      rpy.r = set->linkSet[t].rot.r;
      rpy.p = set->linkSet[t].rot.p;
      rpy.y = set->linkSet[t].rot.y;
      go_rpy_quat_convert (&rpy, &pose.rot);

      pose.tran.x = set->linkSet[t].loc.x;
      pose.tran.y = set->linkSet[t].loc.y;
      pose.tran.z = set->linkSet[t].loc.z;

      link_params[t].u.pp.pose = pose;
      link_params[t].type = GO_LINK_PP;
      link_params[t].quantity =
	(SERVO_MIS_JA_PRISMATIC_JOINT_TYPE ==
	 set->linkSet[t].jType ? GO_QUANTITY_LENGTH : GO_QUANTITY_ANGLE);
      joints[t] = stat->linkStat[t].jointVal;
    }

  genser_kin_set_parameters (&kins, link_params, link_number);

  if (GO_RESULT_OK != genser_kin_fwd (&kins, joints, &pose))
    {
      printf ("can't compute forward kinematics\n");
      return;
    }

  go_quat_rpy_convert (&pose.rot, &rpy);

  printf ("%f %f %f %f %f %f\n",
	  (double) pose.tran.x,
	  (double) pose.tran.y,
	  (double) pose.tran.z,
	  (double) rpy.r, (double) rpy.p, (double) rpy.y);

  return;
}

static void
printJacs (ServoMisJAStat * stat, ServoMisJASet * set)
{
  GO_MATRIX_DECLARE (Jfwd, Jfwdstg, 6, SERVO_MIS_JA_LINK_MAX);
  GO_MATRIX_DECLARE (Jinv, Jinvstg, SERVO_MIS_JA_LINK_MAX, 6);
  go_vector vw[6];
  go_vector weights[SERVO_MIS_JA_LINK_MAX];
  go_vector jv[SERVO_MIS_JA_LINK_MAX];
  go_integer link_number;
  go_link link_params[SERVO_MIS_JA_LINK_MAX];
  go_link linkout[SERVO_MIS_JA_LINK_MAX];
  go_pose pose;
  go_rpy rpy;
  int t;

  link_number = set->linkSet_length;
  if (link_number > SERVO_MIS_JA_LINK_MAX)
    link_number = SERVO_MIS_JA_LINK_MAX;
  if (link_number > 7)
    link_number = 7;		// FIXME -- hard-coded for Telemax

  go_matrix_init (Jfwd, Jfwdstg, 6, link_number);
  go_matrix_init (Jinv, Jinvstg, link_number, 6);

  for (t = 0; t < link_number; t++)
    {
      rpy.r = set->linkSet[t].rot.r;
      rpy.p = set->linkSet[t].rot.p;
      rpy.y = set->linkSet[t].rot.y;
      go_rpy_quat_convert (&rpy, &pose.rot);

      pose.tran.x = set->linkSet[t].loc.x;
      pose.tran.y = set->linkSet[t].loc.y;
      pose.tran.z = set->linkSet[t].loc.z;

      link_params[t].u.pp.pose = pose;
      link_params[t].type = GO_LINK_PP;
      link_params[t].quantity =
	(SERVO_MIS_JA_PRISMATIC_JOINT_TYPE ==
	 set->linkSet[t].jType ? GO_QUANTITY_LENGTH : GO_QUANTITY_ANGLE);
      go_link_joint_set (&link_params[t],
			 (go_real) stat->linkStat[t].jointVal, &linkout[t]);
      weights[t] = 1;
    }

  if (GO_RESULT_OK !=
      genser_kin_compute_jfwd (linkout, link_number, &Jfwd, &pose))
    {
      printf ("can't compute forward Jacobian\n");
      return;
    }

  go_matrix_print (&Jfwd);

  printf ("  ---\n");

  if (GO_RESULT_OK != genser_kin_compute_jinv (&Jfwd, &Jinv, weights))
    {
      printf ("can't invert Jacobian\n");
      return;
    }

  go_matrix_print (&Jinv);

  printf ("  ---\n");

  vw[0] = vw[1] = vw[2] = vw[3] = vw[4] = vw[5] = 0;
  vw[1] = 0.005;
  go_matrix_vector_mult (&Jinv, vw, jv);
  for (t = 0; t < link_number; t++)
    {
      printf ("%f ", jv[t]);
    }
  printf ("\n");
}

static void
printServoMisJAStat (ServoMisJAStat * stat)
{
  int t;

  printf ("command_type:       %s\n",
	  servoMisJA_symbol_lookup (stat->command_type));
  printf ("echo_serial_number: %d\n", (int) stat->echo_serial_number);
  printf ("status:             %s\n", rcs_stat_to_string (stat->status));
  printf ("state:              %d\n", (int) stat->state);
  printf ("admin_state:        %s\n",
	  rcs_admin_state_to_string (stat->admin_state));
  printf ("line:               %d\n", (int) stat->line);
  printf ("source_line:        %d\n", (int) stat->source_line);
  printf ("source_file:        %s\n", (char *) stat->source_file);
  printf ("timetracker:        %d\n", (int) stat->tt.count);
  for (t = 0; t < stat->linkStat_length && t < SERVO_MIS_JA_LINK_MAX; t++)
    {
      printf ("joint %d value:     %f\n", stat->linkStat[t].linkID,
	      stat->linkStat[t].jointVal);
      printf ("        torque:    %f\n", stat->linkStat[t].torque);
    }
}

static char *
itoa (int i)
{
  static char str[3 * sizeof (i)];
  sprintf (str, "%d", i);
  return str;
}

static void
toDH (PM_CARTESIAN loc, PM_RPY rot, go_dh * dh)
{
  go_pose pose;
  go_rpy rpy;

  pose.tran.x = loc.x;
  pose.tran.y = loc.y;
  pose.tran.z = loc.z;
  rpy.r = rot.r;
  rpy.p = rot.p;
  rpy.y = rot.y;
  go_rpy_quat_convert (&rpy, &pose.rot);
  go_pose_dh_convert (&pose, dh);
}

static void
printServoMisJASet (ServoMisJASet * set)
{
  int t;
  go_dh dh;

  printf ("command_type:       %s\n",
	  servoMisJA_symbol_lookup (set->command_type));
  printf ("echo_serial_number: %d\n", (int) set->echo_serial_number);
  printf ("status:             %s\n", rcs_stat_to_string (set->status));
  printf ("state:              %d\n", (int) set->state);
  printf ("cycle_time:         %f\n", (double) set->cycleTime);
  printf ("name:               %s\n", set->name);
  for (t = 0; t < set->linkSet_length && t < SERVO_MIS_JA_LINK_MAX; t++)
    {
      printf ("joint %d parent:    %d\n", set->linkSet[t].linkID,
	      set->linkSet[t].parentLinkID);
      printf ("      type:         %s\n",
	      set->linkSet[t].jType ==
	      SERVO_MIS_JA_REVOLUTE_JOINT_TYPE ? "revolute" : set->linkSet[t].
	      jType ==
	      SERVO_MIS_JA_PRISMATIC_JOINT_TYPE ? "prismatic" : set->
	      linkSet[t].jType ==
	      SERVO_MIS_JA_SCISSOR_JOINT_TYPE ? "scissor" : itoa (set->
								  linkSet[t].
								  jType));
      printf ("      max speed:    %f\n", set->linkSet[t].maxVelocity);
      printf ("      max torque:   %f\n", set->linkSet[t].maxTorque);
      printf ("      max range:    %f\n", set->linkSet[t].maxRange);
      printf ("      min range:    %f\n", set->linkSet[t].minRange);
      printf ("      pose:         %f %f %f %f %f %f\n",
	      set->linkSet[t].loc.x, set->linkSet[t].loc.y,
	      set->linkSet[t].loc.z, set->linkSet[t].rot.r,
	      set->linkSet[t].rot.p, set->linkSet[t].rot.y);
      toDH (set->linkSet[t].loc, set->linkSet[t].rot, &dh);
      printf ("      DH:           %f %f %f %f\n", (double) dh.a,
	      (double) dh.alpha, (double) dh.d, (double) dh.theta);
    }
}

static void
printServoEffJAStat (ServoEffJAStat * stat)
{
  printf ("command_type:       %s\n",
	  servoEffJA_symbol_lookup (stat->command_type));
  printf ("echo_serial_number: %d\n", (int) stat->echo_serial_number);
  printf ("status:             %s\n", rcs_stat_to_string (stat->status));
  printf ("state:              %d\n", (int) stat->state);
  printf ("admin_state:        %s\n",
	  rcs_admin_state_to_string (stat->admin_state));
  printf ("line:               %d\n", (int) stat->line);
  printf ("source_line:        %d\n", (int) stat->source_line);
  printf ("source_file:        %s\n", (char *) stat->source_file);
  printf ("timetracker:        %d\n", (int) stat->tt.count);
}

static void
printServoEffJASet (ServoEffJASet * set)
{
  printf ("command_type:       %s\n",
	  servoEffJA_symbol_lookup (set->command_type));
  printf ("echo_serial_number: %d\n", (int) set->echo_serial_number);
  printf ("status:             %s\n", rcs_stat_to_string (set->status));
  printf ("state:              %d\n", (int) set->state);
  printf ("cycle_time:         %f\n", (double) set->cycleTime);
}

static void
printServoSPStat (ServoSPStat * stat)
{
  printf ("command_type:       %s\n",
	  servoSP_symbol_lookup (stat->command_type));
  printf ("echo_serial_number: %d\n", (int) stat->echo_serial_number);
  printf ("status:             %s\n", rcs_stat_to_string (stat->status));
  printf ("state:              %d\n", (int) stat->state);
  printf ("admin_state:        %s\n",
	  rcs_admin_state_to_string (stat->admin_state));
  printf ("line:               %d\n", (int) stat->line);
  printf ("source_line:        %d\n", (int) stat->source_line);
  printf ("source_file:        %s\n", (char *) stat->source_file);
  printf ("timetracker:        %d\n", (int) stat->tt.count);
}

#define ROUNDEG(rads) ((int) (TO_DEG*(rads)))

static void
printServoSPSet (ServoSPSet * set)
{
  int t, tmax;
  PM_RPY rpy;

  printf ("command_type:       %s\n",
	  servoSP_symbol_lookup (set->command_type));
  printf ("echo_serial_number: %d\n", (int) set->echo_serial_number);
  printf ("status:             %s\n", rcs_stat_to_string (set->status));
  printf ("state:              %d\n", (int) set->state);
  printf ("cycle_time:         %f\n", (double) set->cycleTime);

  printf ("vehicleID:          %d\n", (int) set->vehicleID);

  tmax = set->sSetElem_length;
  if (tmax > SERVO_SP_SENSOR_MAX)
    tmax = SERVO_SP_SENSOR_MAX;
  for (t = 0; t < tmax; t++)
    {
      printf ("sensor %d type:      %s\n", t,
	      servoSP_enum_ServoSensorType_symbol_lookup (set->sSetElem[t].
							  sType));
      printf ("sensor %d name:      %s\n", t, set->sSetElem[t].senName);
      rpy = set->sSetElem[t].sensorToMount.rot;
      printf ("sensor %d mount:     %.2f %.2f %.2f, %d %d %d\n", t,
	      (double) set->sSetElem[t].sensorToMount.tran.x,
	      (double) set->sSetElem[t].sensorToMount.tran.y,
	      (double) set->sSetElem[t].sensorToMount.tran.z,
	      ROUNDEG (rpy.r), ROUNDEG (rpy.p), ROUNDEG (rpy.y));
    }
}

static void
printServoFactJAStat (ServoFactJAStat * stat)
{
  printf ("command_type:       %s\n",
	  servoFactJA_symbol_lookup (stat->command_type));
  printf ("echo_serial_number: %d\n", (int) stat->echo_serial_number);
  printf ("status:             %s\n", rcs_stat_to_string (stat->status));
  printf ("state:              %d\n", (int) stat->state);
  printf ("admin_state:        %s\n",
	  rcs_admin_state_to_string (stat->admin_state));
  printf ("line:               %d\n", (int) stat->line);
  printf ("source_line:        %d\n", (int) stat->source_line);
  printf ("source_file:        %s\n", (char *) stat->source_file);
  printf ("timetracker:        %d\n", (int) stat->tt.count);
}

static void
printServoFactJASet (ServoFactJASet * set)
{
  printf ("command_type:       %s\n",
	  servoFactJA_symbol_lookup (set->command_type));
  printf ("echo_serial_number: %d\n", (int) set->echo_serial_number);
  printf ("status:             %s\n", rcs_stat_to_string (set->status));
  printf ("state:              %d\n", (int) set->state);
  printf ("cycle_time:         %f\n", (double) set->cycleTime);
}

static void
printSonarData (void)
{
  SensorData3D sonarData;
  NMLTYPE type;
  int t;
  int num;

  if (getSonarBuf (thisProcess, nmlFile, missionNumber))
    {
      type = sonarBuf->read ();
      switch (type)
	{
	case 0:
	  printf ("no new data\n");
	  break;
	case -1:
	  printf ("read error\n");
	  break;
	case SENSOR_DATA_3D_TYPE:
	  sonarData =
	    *(reinterpret_cast < SensorData3D * >(sonarBuf->get_address ()));
	  num = MAX_SENSOR_DATA_3D;
	  if (sonarData.element_length < num)
	    num = sonarData.element_length;
	  printf ("%d sonars:\n", num);
	  for (t = 0; t < num; t++)
	    {
	      printf ("%f : %f %f,%f,%f\n", sonarData.element[t].time,
		      sonarData.element[t].range,
		      sonarData.element[t].pose.tran.x,
		      sonarData.element[t].pose.tran.y,
		      sonarData.element[t].pose.tran.z);
	    }
	  break;
	}
    }
  else
    {
      printf ("data not available\n");
    }

  return;
}

static void
printRangescannerData (int number)
{
  NMLTYPE type;
  SensorData1D range;
  int index = number - 1;
  int t;
  int num;

  if (getRangescannerBuf (thisProcess, nmlFile, index, missionNumber))
    {
      type = rangescannerBuf[index]->read ();
      switch (type)
	{
	case 0:
	  printf ("no new data\n");
	  break;
	case -1:
	  printf ("read error\n");
	  break;
	case SENSOR_DATA_1D_TYPE:
	  range =
	    *(reinterpret_cast <
	      SensorData1D * >(rangescannerBuf[index]->get_address ()));
	  num = MAX_SENSOR_DATA_1D;
	  if (range.featArray_length < num)
	    num = range.featArray_length;
	  printf ("%d ranges at time %f:\n", num, range.collectTimeStart);
	  for (t = 0; t < num; t++)
	    {
	      printf ("%f %f\n", range.start + t * range.increment,
		      range.featArray[t]);
	    }
	  printf ("\n");
	  break;
	}
    }
  else
    {
      printf ("data not available\n");
    }

  return;
}

static void
printSensorData (int number)
{
  NMLTYPE type;
  SensorData sensorData;
  ServoSensorType sensorType;
  int index = number - 1;
  int t;
  int num;
#define ARRAYELS(array) ((int) (sizeof(array)/sizeof(*(array))))

  if (getSensorDataBuf (thisProcess, nmlFile, index, missionNumber))
    {
      type = sensorDataBuf[index]->read ();
      switch (type)
	{
	case 0:
	  printf ("no new data\n");
	  break;
	case -1:
	  printf ("read error\n");
	  break;
	case SENSOR_DATA_TYPE:
	  sensorData =
	    *(reinterpret_cast <
	      SensorData * >(sensorDataBuf[index]->get_address ()));
	  sensorType = sensorData.type;
	  switch (sensorType)
	    {
	    case SERVO_SP_ODOMETER_SENSOR:
	      printf ("odometer at time %f: %f,%f,%f\n",
		      (double) sensorData.time, (double) sensorData.data[0],
		      (double) sensorData.data[1],
		      (double) sensorData.data[2]);
	      break;
	    case SERVO_SP_TACHOMETER_SENSOR:
	      num = sensorData.data_length;
	      if (num > ARRAYELS (sensorData.data))
		num = ARRAYELS (sensorData.data);
	      printf ("%d of %d tachs at time %f:\n", num,
		      ARRAYELS (sensorData.data), (double) sensorData.time);
	      for (t = 0; t < num; t += 2)
		{
		  printf ("%f,%f ", (double) sensorData.data[t],
			  (double) sensorData.data[t + 1]);
		}
	      printf ("\n");
	      break;
	    case SERVO_SP_ENCODER_SENSOR:
	      printf ("encoder at time %f: %f\n", (double) sensorData.time,
		      (double) sensorData.data[0]);
	      break;
	    case SERVO_SP_GRD_TRUTH_SENSOR:
	      printf ("ground truth at time %f: %f,%f,%f %f,%f,%f\n",
		      (double) sensorData.time,
		      (double) sensorData.data[0],
		      (double) sensorData.data[1],
		      (double) sensorData.data[2],
		      (double) sensorData.data[3],
		      (double) sensorData.data[4],
		      (double) sensorData.data[5]);
	      break;
	    case SERVO_SP_ACOUSTIC_SENSOR:
	      printf ("acoustic sensor at time %f: %f,%f,%f,%f\n",
		      (double) sensorData.time,
		      (double) sensorData.data[0],
		      (double) sensorData.data[1],
		      (double) sensorData.data[2],
		      (double) sensorData.data[3]);
	      break;
	    default:
	      printf ("unknown sensor data type %d\n", (int) type);
	      break;
	    }
	  break;
	}
    }
  else
    {
      printf ("unknown sensor type\n");
    }

  return;
}

static void
printNavDataExt (void)
{
  NavDataExt nav;
  NMLTYPE type;

  if (getNavDataExtBuf (thisProcess, nmlFile, missionNumber))
    {
      type = navDataExtBuf->read ();
      switch (type)
	{
	case 0:
	  printf ("no new data\n");
	  break;
	case -1:
	  printf ("read error\n");
	  break;
	case NAV_DATA_EXT_TYPE:
	  nav =
	    *(reinterpret_cast <
	      NavDataExt * >(navDataExtBuf->get_address ()));
	  printf ("%f : %f,%f,%f %f,%f,%f\n", nav.time, nav.tranAbs.x,
		  nav.tranAbs.y, nav.tranAbs.z, nav.rpyAbs.r, nav.rpyAbs.p,
		  nav.rpyAbs.y);
	  break;
	}
    }
  else
    {
      printf ("data not available\n");
    }

  return;
}

static int
strwcmp (const char *test, const char *match)
{
  while (0 != *test && 0 != *match)
    {
      if (*test != *match)
	return 1;
      test++, match++;
    }

  if (0 == *match && (0 == *test || isspace (*test)))
    return 0;
  return 1;
}

int
main (int argc, char *argv[])
{
  int c;
  int this_option_optind;
  int option_index;
  struct option long_options[] = {
    /* --option, needs arg, 0, 'shortcut char' or 0 */
    {"nml", required_argument, 0, 'n'},
    {"mission", required_argument, 0, 'm'},
    {0, 0, 0, 0}
  };
#ifndef HAVE_READLINE_READLINE_H
  enum
  { BUFFERSIZE = 256 };
  char buffer[BUFFERSIZE];
#endif
  enum
  { PROMPTSIZE = 16 };
  char prompt[PROMPTSIZE];
  char *line;
  char *ptr;
  int t;
  NMLTYPE mobStatType = 0;
  NMLTYPE mobSetType = 0;

  ulapi_init (UL_USE_DEFAULT);

  opterr = 0;
  while (1)
    {
      this_option_optind = optind ? optind : 1;
      option_index = 0;

      /* the leading colon means generate a ':' for missing parameters */
      /* colons mean a required argument for the preceding option */
      c = getopt_long (argc, argv, ":n:m:", long_options, &option_index);
      if (c == -1)
	break;

      switch (c)
	{
	  /* this handles --long args with no shortcut characters */
	case 0:
	  if (!strcmp ("nml", long_options[option_index].name))
	    {
	      strncpy (nmlFile, optarg, sizeof (nmlFile));
	      nmlFile[sizeof (nmlFile) - 1] = 0;
	      nmlFileArg = true;
	    }
	  else if (!strcmp ("mission", long_options[option_index].name))
	    {
	      if (1 != sscanf (optarg, "%i", &missionNumber))
		{
		  fprintf (stderr, "bad value for mission number: %s\n",
			   optarg);
		  return 1;
		}
	    }
	  else
	    {
	      /* should never get here, since 0 above means OK */
	      fprintf (stderr, "unknown option `%s'",
		       long_options[option_index].name);
	      if (optarg)
		{
		  fprintf (stderr, " with arg %s", optarg);
		}
	      fprintf (stderr, "\n");
	      return 1;
	    }
	  break;

	  /* the rest handle -short args */
	case 'n':
	  strncpy (nmlFile, optarg, sizeof (nmlFile));
	  nmlFile[sizeof (nmlFile) - 1] = 0;
	  nmlFileArg = true;
	  break;

	case 'm':
	  if (1 != sscanf (optarg, "%i", &missionNumber))
	    {
	      fprintf (stderr, "bad value for mission number: %s\n", optarg);
	      return 1;
	    }
	  break;

	case '?':
	  fprintf (stderr, "unknown option `%s'\n", argv[this_option_optind]);
	  return 1;
	  break;

	case ':':
	  fprintf (stderr, "missing parameter to `%s'\n",
		   argv[this_option_optind]);
	  return 1;
	  break;

	default:
	  fprintf (stderr, "bad argument `%s'\n", argv[this_option_optind]);
	  return 1;
	  break;
	}
    }
  if (optind < argc)
    {
      fprintf (stderr, "extra arguments provided:");
      while (optind < argc)
	{
	  fprintf (stderr, " %s", argv[optind++]);
	}
      fprintf (stderr, "\n");
      return 1;
    }

  // if NML file wasn't provided on the command line, get
  // it first from the environment, then from the default

  if (!nmlFileArg)
    {
      char *ptr = getenv ("CONFIG_NML");
      if (NULL != ptr)
	{
	  strncpy (nmlFile, ptr, sizeof (nmlFile));
	}
      else
	{
	  strncpy (nmlFile, DEFAULT_NML_FILE, sizeof (nmlFile));
	}
      nmlFile[sizeof (nmlFile) - 1] = 0;
    }

  initMoastBufs ();

  rcs_noprint ();

  if (getServoMobJABufs (thisProcess, nmlFile, missionNumber))
    {
      printf ("connected to mobility mission %d\n", missionNumber);
      whichSub = WHICH_MOB;
    }

  for (t = 1; t <= SERVO_MIS_JA_MIS_PKG_MAX; t++)
    {
      if (getServoMisJABufs (thisProcess, nmlFile, t, missionNumber))
	{
	  printf ("connected to mission package %d, mission %d\n", t,
		  missionNumber);
	  whichSub = WHICH_MIS;
	}
    }

  for (t = 1; t <= SERVO_EFF_JA_EFFECTOR_MAX; t++)
    {
      if (getServoEffJABufs (thisProcess, nmlFile, t, missionNumber))
	{
	  printf ("connected to effector %d, mission %d\n", t, missionNumber);
	  whichSub = WHICH_EFF;
	}
    }

  if (getServoFactJABufs (thisProcess, nmlFile, missionNumber))
    {
      printf ("connected to factory %d\n", missionNumber);
      whichSub = WHICH_FACT;
    }

  if (getServoSPBufs (thisProcess, nmlFile, missionNumber))
    {
      printf ("connected to SP mission %d\n", missionNumber);
      whichSub = WHICH_SP;
    }

  rcs_doprint ();

#ifdef HAVE_READLINE_READLINE_H
  using_history ();
#endif

  while (!feof (stdin))
    {
      if (WHICH_SONAR == whichSub)
	ulapi_snprintf (prompt, sizeof (prompt), "sonar> ");
      else if (WHICH_RANGE == whichSub)
	ulapi_snprintf (prompt, sizeof (prompt), "range> ");
      else if (WHICH_NAV == whichSub)
	ulapi_snprintf (prompt, sizeof (prompt), "nav> ");
      else if (WHICH_SENSOR_DATA == whichSub)
	ulapi_snprintf (prompt, sizeof (prompt), "sensor> ");
      else
	ulapi_snprintf (prompt, sizeof (prompt), "%s%s> ",
			whichSub == WHICH_MIS ? "mis" :
			whichSub == WHICH_EFF ? "eff" :
			whichSub == WHICH_SP ? "sp" :
			whichSub == WHICH_FACT ? "fact" : "mob",
			whichLook == WHICH_SET ? "set" : "stat");
#ifndef HAVE_READLINE_READLINE_H
      printf (prompt);
      fflush (stdout);
      if (NULL == fgets (buffer, BUFFERSIZE, stdin))
	break;
      buffer[BUFFERSIZE - 1] = 0;
      line = buffer;
#else
      line = readline (prompt);
      if (NULL == line)
	break;
      if (*line)
	add_history (line);
#endif

      if (NULL != servoMobJAStatBuf)
	{
	  servoMobJAStatBuf->read ();
	  mobStatType = servoMobJAStatBuf->get_address ()->type;
	  if (SERVO_MOB_JA_STAT_TYPE == mobStatType)
	    {
	      servoMobJAStat =
		*(reinterpret_cast <
		  ServoMobJAStat * >(servoMobJAStatBuf->get_address ()));
	    }
	  else if (SERVO_MOB_JA_STAT_GRD_VEH_TYPE == mobStatType)
	    {
	      servoMobJAStatGrdVeh =
		*(reinterpret_cast <
		  ServoMobJAStatGrdVeh *
		  >(servoMobJAStatBuf->get_address ()));
	    }
	  else if (SERVO_MOB_JA_STAT_AIR_BOT_TYPE == mobStatType)
	    {
	      servoMobJAStatAirBot =
		*(reinterpret_cast <
		  ServoMobJAStatAirBot *
		  >(servoMobJAStatBuf->get_address ()));
	    }
	}

      if (NULL != servoMobJASetBuf)
	{
	  servoMobJASetBuf->read ();
	  mobSetType = servoMobJASetBuf->get_address ()->type;
	  if (SERVO_MOB_JA_SET_TYPE == mobSetType)
	    {
	      servoMobJASet =
		*(reinterpret_cast <
		  ServoMobJASet * >(servoMobJASetBuf->get_address ()));
	    }
	  else if (SERVO_MOB_JA_SET_GRD_VEH_TYPE == mobSetType)
	    {
	      servoMobJASetGrdVeh =
		*(reinterpret_cast <
		  ServoMobJASetGrdVeh * >(servoMobJASetBuf->get_address ()));
	    }
	  else if (SERVO_MOB_JA_SET_AIR_BOT_TYPE == mobSetType)
	    {
	      servoMobJASetAirBot =
		*(reinterpret_cast <
		  ServoMobJASetAirBot * >(servoMobJASetBuf->get_address ()));
	    }
	}

      if (NULL != servoMisJAStatBuf[packageIndex])
	{
	  servoMisJAStatBuf[packageIndex]->read ();
	  if (SERVO_MIS_JA_STAT_TYPE ==
	      servoMisJAStatBuf[packageIndex]->get_address ()->type)
	    {
	      servoMisJAStat[packageIndex] =
		*(reinterpret_cast <
		  ServoMisJAStat *
		  >(servoMisJAStatBuf[packageIndex]->get_address ()));
	    }
	}

      if (NULL != servoMisJASetBuf[packageIndex])
	{
	  servoMisJASetBuf[packageIndex]->read ();
	  if (SERVO_MIS_JA_SET_TYPE ==
	      servoMisJASetBuf[packageIndex]->get_address ()->type)
	    {
	      servoMisJASet[packageIndex] =
		*(reinterpret_cast <
		  ServoMisJASet *
		  >(servoMisJASetBuf[packageIndex]->get_address ()));
	    }
	}

      if (NULL != servoEffJAStatBuf[effectorIndex])
	{
	  servoEffJAStatBuf[effectorIndex]->read ();
	  if (SERVO_EFF_JA_STAT_TYPE ==
	      servoEffJAStatBuf[effectorIndex]->get_address ()->type)
	    {
	      servoEffJAStat[effectorIndex] =
		*(reinterpret_cast <
		  ServoEffJAStat *
		  >(servoEffJAStatBuf[effectorIndex]->get_address ()));
	    }
	}

      if (NULL != servoEffJASetBuf[effectorIndex])
	{
	  servoEffJASetBuf[effectorIndex]->read ();
	  if (SERVO_EFF_JA_SET_TYPE ==
	      servoEffJASetBuf[effectorIndex]->get_address ()->type)
	    {
	      servoEffJASet[effectorIndex] =
		*(reinterpret_cast <
		  ServoEffJASet *
		  >(servoEffJASetBuf[effectorIndex]->get_address ()));
	    }
	}

      if (NULL != servoFactJAStatBuf)
	{
	  servoFactJAStatBuf->read ();
	  if (SERVO_FACT_JA_STAT_TYPE ==
	      servoFactJAStatBuf->get_address ()->type)
	    {
	      servoFactJAStat =
		*(reinterpret_cast <
		  ServoFactJAStat * >(servoFactJAStatBuf->get_address ()));
	    }
	}

      if (NULL != servoFactJASetBuf)
	{
	  servoFactJASetBuf->read ();
	  if (SERVO_FACT_JA_SET_TYPE ==
	      servoFactJASetBuf->get_address ()->type)
	    {
	      servoFactJASet =
		*(reinterpret_cast <
		  ServoFactJASet * >(servoFactJASetBuf->get_address ()));
	    }
	}

      if (NULL != servoSPStatBuf)
	{
	  servoSPStatBuf->read ();
	  if (SERVO_SP_STAT_TYPE == servoSPStatBuf->get_address ()->type)
	    {
	      servoSPStat =
		*(reinterpret_cast <
		  ServoSPStat * >(servoSPStatBuf->get_address ()));
	    }
	}

      if (NULL != servoSPSetBuf)
	{
	  servoSPSetBuf->read ();
	  if (SERVO_SP_SET_TYPE == servoSPSetBuf->get_address ()->type)
	    {
	      servoSPSet =
		*(reinterpret_cast <
		  ServoSPSet * >(servoSPSetBuf->get_address ()));
	    }
	}

      ptr = line;
      while (isspace (*ptr))
	ptr++;			// skip whitespace
      if (0 == *ptr)
	{
	  // blank line
#define PRINT_IT							\
      if (WHICH_SONAR == whichSub) printSonarData();			\
      else if (WHICH_RANGE == whichSub) printRangescannerData(1);	\
      else if (WHICH_NAV == whichSub) printNavDataExt();		\
      else if (WHICH_SENSOR_DATA == whichSub) printSensorData(whichSensor); \
      else if (WHICH_MIS == whichSub) {					\
	if (WHICH_SET == whichLook) printServoMisJASet(&servoMisJASet[packageIndex]);	\
	else printServoMisJAStat(&servoMisJAStat[packageIndex]);			\
      } else if (WHICH_EFF == whichSub) {				\
	if (WHICH_SET == whichLook) printServoEffJASet(&servoEffJASet[effectorIndex]);	\
	else printServoEffJAStat(&servoEffJAStat[effectorIndex]);			\
      } else if (WHICH_SP == whichSub) {				\
	if (WHICH_SET == whichLook) printServoSPSet(&servoSPSet);	\
	else printServoSPStat(&servoSPStat);				\
      } else if (WHICH_FACT == whichSub) {				\
	if (WHICH_SET == whichLook) printServoFactJASet(&servoFactJASet);	\
	else printServoFactJAStat(&servoFactJAStat);				\
      } else {								\
	if (WHICH_SET == whichLook) {					\
	  if (SERVO_MOB_JA_SET_TYPE == mobSetType) printServoMobJASet(&servoMobJASet); \
	  else if (SERVO_MOB_JA_SET_GRD_VEH_TYPE == mobSetType) printServoMobJASetGrdVeh(&servoMobJASetGrdVeh);	\
	  else if (SERVO_MOB_JA_SET_AIR_BOT_TYPE == mobSetType) printServoMobJASetAirBot(&servoMobJASetAirBot);	\
	} else {							\
	  if (SERVO_MOB_JA_STAT_TYPE == mobStatType) printServoMobJAStat(&servoMobJAStat); \
	  else if (SERVO_MOB_JA_STAT_GRD_VEH_TYPE == mobStatType) printServoMobJAStatGrdVeh(&servoMobJAStatGrdVeh); \
	  else if (SERVO_MOB_JA_STAT_AIR_BOT_TYPE == mobStatType) printServoMobJAStatAirBot(&servoMobJAStatAirBot); \
	}								\
      }
	  PRINT_IT;
	}
      else if (ptr[0] == 'q' || !strwcmp (ptr, "quit"))
	{
	  break;
	}
      else if (ptr[0] == '?' || !strwcmp (ptr, "help"))
	{
	  printf ("q, quit: quit\n");
	  printf ("h, help: print this help message\n");
	  printf
	    ("mob, mis, eff, sp, fact: switch focus to that subsystem\n");
	  printf ("stat, set: switch to status or settings\n");
	  printf ("movej <# ...>: move mission package to joint values\n");
	  printf ("movev <# ...>: move mission package at joint speeds\n");
	  printf ("moveq<# ...>: apply torque values to mission package\n");
	  printf ("movew <X Y Z R P W>: move to Cartesian position\n");
	  printf ("kins, jacs: print kinematics or jacobians\n");
	}
      else if (!strwcmp (ptr, "mob"))
	{
	  whichSub = WHICH_MOB;
	  PRINT_IT;
	}
      else if (!strwcmp (ptr, "mis"))
	{
	  whichSub = WHICH_MIS;
	  PRINT_IT;
	}
      else if (!strwcmp (ptr, "eff"))
	{
	  whichSub = WHICH_EFF;
	  PRINT_IT;
	}
      else if (!strwcmp (ptr, "sp"))
	{
	  whichSub = WHICH_SP;
	  PRINT_IT;
	}
      else if (!strwcmp (ptr, "fact"))
	{
	  whichSub = WHICH_FACT;
	  PRINT_IT;
	}
      else if (!strwcmp (ptr, "stat"))
	{
	  whichLook = WHICH_STAT;
	  PRINT_IT;
	}
      else if (!strwcmp (ptr, "set"))
	{
	  whichLook = WHICH_SET;
	  PRINT_IT;
	}
      else if (!strwcmp (ptr, "init"))
	{
	  ServoMobJACmdInit mobMsg;
	  ServoMisJACmdInit misMsg;
	  ServoEffJACmdInit effMsg;
	  ServoSPCmdInit spMsg;
	  ServoFactJACmdInit factMsg;
	  if (WHICH_MIS == whichSub)
	    {
	      SERVO_MIS_JA_CMD_WRITE (packageIndex, misMsg);
	    }
	  else if (WHICH_EFF == whichSub)
	    {
	      SERVO_EFF_JA_CMD_WRITE (effectorIndex, effMsg);
	    }
	  else if (WHICH_SP == whichSub)
	    {
	      SERVO_SP_CMD_WRITE (spMsg);
	    }
	  else if (WHICH_FACT == whichSub)
	    {
	      SERVO_FACT_JA_CMD_WRITE (factMsg);
	    }
	  else
	    {
	      SERVO_MOB_JA_CMD_WRITE (mobMsg);
	    }
	  whichLook = WHICH_STAT;
	}
      else if (!strwcmp (ptr, "go"))
	{
	  ServoSPCmdGo goMsg;
	  SERVO_SP_CMD_WRITE (goMsg);
	  whichSub = WHICH_SP;
	}
      else if (!strwcmp (ptr, "num"))
	{
	  int i;
	  if (1 == sscanf (ptr, "%*s %i", &i))
	    {
	      if (WHICH_MIS == whichSub)
		{
		  if (i <= 0 || i > SERVO_MIS_JA_MIS_PKG_MAX)
		    {
		      printf ("syntax: num <1 .. %d>\n",
			      SERVO_MIS_JA_MIS_PKG_MAX);
		    }
		  else
		    {
		      if (0 !=
			  getServoMisJABufs (thisProcess, nmlFile, i,
					     missionNumber))
			{
			  printf ("can't get servo mis bufs for %d\n", i);
			}
		      else
			{
			  packageIndex = i - 1;
			}
		    }
		}
	      else if (WHICH_EFF == whichSub)
		{
		  if (i <= 0 || i > SERVO_EFF_JA_EFFECTOR_MAX)
		    {
		      printf ("syntax: num <1 .. %d>\n",
			      SERVO_EFF_JA_EFFECTOR_MAX);
		    }
		  else
		    {
		      if (0 !=
			  getServoEffJABufs (thisProcess, nmlFile, i,
					     missionNumber))
			{
			  printf ("can't get servo eff bufs for %d\n", i);
			}
		      else
			{
			  effectorIndex = i - 1;
			}
		    }
		}
	    }
	  else
	    {
	      printf ("syntax: num <subsystem number>\n");
	    }
	}
      else if (!strwcmp (ptr, "skid"))
	{
	  double d1, d2;
	  ServoMobJACmdSkid skidMsg;
	  if (2 == sscanf (ptr, "%*s %lf %lf", &d1, &d2))
	    {
	      skidMsg.wLeft = d1, skidMsg.wRight = d2;
	      SERVO_MOB_JA_CMD_WRITE (skidMsg);
	      whichSub = WHICH_MOB;
	      whichLook = WHICH_STAT;
	    }
	  else
	    {
	      printf ("syntax: skid <left speed> <right speed>\n");
	    }
	}
      else if (!strwcmp (ptr, "ackerman"))
	{
	  double d1, d2;
	  ServoMobJACmdAckerman ackermanMsg;
	  if (2 == sscanf (ptr, "%*s %lf %lf", &d1, &d2))
	    {
	      ackermanMsg.velocity = d1, ackermanMsg.steerAngle = d2;
	      SERVO_MOB_JA_CMD_WRITE (ackermanMsg);
	      whichSub = WHICH_MOB;
	      whichLook = WHICH_STAT;
	    }
	  else
	    {
	      printf ("syntax: ackerman <speed> <steering angle>\n");
	    }
#define MISIT(CMD,TYPE)							\
    } else if (!strwcmp(ptr, CMD)) {					\
      double d1;							\
      ServoMisJACmdMove moveMsg;					\
      for (t = 0; t < SERVO_MIS_JA_LINK_MAX; t++) {			\
	while (! isspace(*ptr) && 0 != *ptr) ptr++;			\
	while (isspace(*ptr)) ptr++;					\
	if (1 != sscanf(ptr, "%lf", &d1)) break;			\
	moveMsg.linkCmd[t].value = d1;					\
	moveMsg.linkCmd[t].type = TYPE;					\
	moveMsg.linkCmd_length = t+1;                   		\
      }									\
      if (moveMsg.linkCmd_length > 0) {					\
	SERVO_MIS_JA_CMD_WRITE(packageIndex, moveMsg);			\
	whichSub = WHICH_MIS;						\
	whichLook = WHICH_STAT;						\
      } else {								\
	printf("syntax: %s <sequence of joint values>\n", CMD);		\
      }
	  MISIT ("movej", SERVO_MIS_JA_LINK_CMD_ABS_VALUE_TYPE);
	  MISIT ("movev", SERVO_MIS_JA_LINK_CMD_VELOCITY_TYPE);
	  MISIT ("moveq", SERVO_MIS_JA_LINK_CMD_TORQUE_TYPE);
	}
      else if (!strwcmp (ptr, "movew"))
	{
	  double d1, d2, d3, d4, d5, d6;
	  go_real joints[SERVO_MIS_JA_LINK_MAX];
	  go_pose pose;
	  go_rpy rpy;
	  ServoMisJACmdMove moveMsg;
	  if (6 ==
	      sscanf (ptr, "%*s %lf %lf %lf %lf %lf %lf", &d1, &d2, &d3, &d4,
		      &d5, &d6))
	    {
	      pose.tran.x = d1, pose.tran.y = d2, pose.tran.z = d3;
	      rpy.r = d4, rpy.p = d5, rpy.y = d6;
	      go_rpy_quat_convert (&rpy, &pose.rot);
	      if (0 ==
		  invKins (&pose, joints, &servoMisJAStat[packageIndex],
			   &servoMisJASet[packageIndex]))
		{
		  for (t = 0; t < servoMisJASet[packageIndex].linkSet_length;
		       t++)
		    {
		      moveMsg.linkCmd[t].value = joints[t];
		      moveMsg.linkCmd[t].type =
			SERVO_MIS_JA_LINK_CMD_ABS_VALUE_TYPE;
		    }
		  moveMsg.linkCmd_length =
		    servoMisJASet[packageIndex].linkSet_length;
		  SERVO_MIS_JA_CMD_WRITE (packageIndex, moveMsg);
		  whichSub = WHICH_MIS;
		  whichLook = WHICH_STAT;
		}
	      else
		{
		  printf ("can't compute inverse kinematics\n");
		}
	    }
	  else
	    {
	      printf ("syntax: mismove <X Y Z R P W>\n");
	    }
	}
      else if (!strwcmp (ptr, "kins"))
	{
	  printKins (&servoMisJAStat[packageIndex],
		     &servoMisJASet[packageIndex]);
	}
      else if (!strwcmp (ptr, "jacs"))
	{
	  printJacs (&servoMisJAStat[packageIndex],
		     &servoMisJASet[packageIndex]);
	}
      else if (!strwcmp (ptr, "opcode"))
	{
	  double param;
	  ServoEffJACmdOpcode msg;
	  if (1 == sscanf (ptr, "%*s %lf", &param))
	    {
	      msg.effID = effectorIndex;
	      msg.opcode = SERVO_EFF_JA_ACTIVATE_OPCODE_TYPE;
	      msg.param = param;
	      SERVO_EFF_JA_CMD_WRITE (effectorIndex, msg);
	      whichSub = WHICH_EFF;
	    }
	  else
	    {
	      printf ("syntax: opcode <parameter>\n");
	    }
	}
      else if (!strwcmp (ptr, "ct"))
	{
	  double d1;
	  ServoMobJACfgCycleTime mobMsg;
	  ServoMisJACfgCycleTime misMsg;
	  ServoEffJACfgCycleTime effMsg;
	  ServoSPCfgCycleTime spMsg;
	  ServoFactJACfgCycleTime factMsg;
	  if (1 == sscanf (ptr, "%*s %lf", &d1))
	    {
	      if (WHICH_MIS == whichSub)
		{
		  misMsg.cycleTime = d1;
		  SERVO_MIS_JA_CFG_WRITE (packageIndex, misMsg);
		}
	      else if (WHICH_EFF == whichSub)
		{
		  effMsg.cycleTime = d1;
		  SERVO_EFF_JA_CFG_WRITE (effectorIndex, effMsg);
		}
	      else if (WHICH_SP == whichSub)
		{
		  spMsg.cycleTime = d1;
		  SERVO_SP_CFG_WRITE (spMsg);
		}
	      else if (WHICH_FACT == whichSub)
		{
		  factMsg.cycleTime = d1;
		  SERVO_FACT_JA_CFG_WRITE (factMsg);
		}
	      else
		{
		  mobMsg.cycleTime = d1;
		  SERVO_MOB_JA_CFG_WRITE (mobMsg);
		}
	      whichLook = WHICH_SET;
	    }
	  else
	    {
	      printf ("syntax: ct <cycle time>\n");
	    }
	}
      else if (!strwcmp (ptr, "sonar"))
	{
	  whichSub = WHICH_SONAR;
	  printSonarData ();
	}
      else if (!strwcmp (ptr, "range"))
	{
	  int i1;
	  if (1 == sscanf (ptr, "%*s %i", &i1))
	    {
	      if (i1 > 0 && i1 <= MAX_SENSOR_DATA_1D_BUFFERS)
		{
		  whichRange = i1;
		  whichSub = WHICH_RANGE;
		  printRangescannerData (whichRange);
		}
	      else
		{
		  printf ("bad range sensor number: %d\n", i1);
		}
	    }
	  else
	    {
	      printf ("need range sensor number 1 to %d\n",
		      MAX_SENSOR_DATA_1D_BUFFERS);
	    }
	}
      else if (!strwcmp (ptr, "nav"))
	{
	  whichSub = WHICH_NAV;
	  printNavDataExt ();
	}
      else if (!strwcmp (ptr, "sensor"))
	{
	  int i1;
	  if (1 == sscanf (ptr, "%*s %i", &i1))
	    {
	      if (i1 > 0 && i1 <= MAX_SENSOR_DATA_BUFFERS)
		{
		  whichSensor = i1;
		  whichSub = WHICH_SENSOR_DATA;
		  printSensorData (whichSensor);
		}
	      else
		{
		  printf ("bad sensor number: %d\n", i1);
		}
	    }
	  else
	    {
	      printf ("need sensor number 1 to %d\n",
		      MAX_SENSOR_DATA_BUFFERS);
	    }
	}
      else
	{
	  printf ("?\n");
	}

#ifdef HAVE_READLINE_READLINE_H
      free (line);
#endif
    }

  cleanupNml ();

  ulapi_exit ();

  return 0;
}
