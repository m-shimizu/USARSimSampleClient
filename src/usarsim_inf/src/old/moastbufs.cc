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
  \file moastbufs.cc

  \brief Collection of global NML buffers, for convenience
*/

#include <rcs.hh>
#include <navDataExt.hh>
#include <sensorData.hh>
#include <servoMobJA.hh>
#include <servoMisJA.hh>
#include <servoEffJA.hh>
#include <servoSP.hh>
#include "moastbufs.h"
#include "ulapi.h"

NML *rangescannerBuf[MAX_SENSOR_DATA_1D_BUFFERS];
NML *rangeimagerBuf[MAX_SENSOR_DATA_2D_BUFFERS];
NML *sonarBuf = NULL;
NML *navDataExtBuf = NULL;
NML *sensorDataBuf[MAX_SENSOR_DATA_BUFFERS];

RCS_CMD_CHANNEL *servoMobJACmdBuf = NULL;
RCS_STAT_CHANNEL *servoMobJAStatBuf = NULL;
ServoMobJAStat servoMobJAStat;
ServoMobJAStatGrdVeh servoMobJAStatGrdVeh;
ServoMobJAStatAirBot servoMobJAStatAirBot;
int servoMobJACmdSerialNumber = 0;

RCS_CMD_CHANNEL *servoMobJACfgBuf = NULL;
RCS_STAT_CHANNEL *servoMobJASetBuf = NULL;
ServoMobJASet servoMobJASet;
ServoMobJASetGrdVeh servoMobJASetGrdVeh;
ServoMobJASetAirBot servoMobJASetAirBot;
int servoMobJACfgSerialNumber = 0;

RCS_CMD_CHANNEL *servoMisJACmdBuf[SERVO_MIS_JA_MIS_PKG_MAX];
RCS_STAT_CHANNEL *servoMisJAStatBuf[SERVO_MIS_JA_MIS_PKG_MAX];
ServoMisJAStat servoMisJAStat[SERVO_MIS_JA_MIS_PKG_MAX];
int servoMisJACmdSerialNumber[SERVO_MIS_JA_MIS_PKG_MAX];

RCS_CMD_CHANNEL *servoMisJACfgBuf[SERVO_MIS_JA_MIS_PKG_MAX];
RCS_STAT_CHANNEL *servoMisJASetBuf[SERVO_MIS_JA_MIS_PKG_MAX];
ServoMisJASet servoMisJASet[SERVO_MIS_JA_MIS_PKG_MAX];
int servoMisJACfgSerialNumber[SERVO_MIS_JA_MIS_PKG_MAX];

RCS_CMD_CHANNEL *servoEffJACmdBuf[SERVO_EFF_JA_EFFECTOR_MAX];
RCS_STAT_CHANNEL *servoEffJAStatBuf[SERVO_EFF_JA_EFFECTOR_MAX];
ServoEffJAStat servoEffJAStat[SERVO_EFF_JA_EFFECTOR_MAX];
int servoEffJACmdSerialNumber[SERVO_EFF_JA_EFFECTOR_MAX];

RCS_CMD_CHANNEL *servoEffJACfgBuf[SERVO_EFF_JA_EFFECTOR_MAX];
RCS_STAT_CHANNEL *servoEffJASetBuf[SERVO_EFF_JA_EFFECTOR_MAX];
ServoEffJASet servoEffJASet[SERVO_EFF_JA_EFFECTOR_MAX];
int servoEffJACfgSerialNumber[SERVO_EFF_JA_EFFECTOR_MAX];

RCS_CMD_CHANNEL *servoSPCmdBuf = NULL;
RCS_STAT_CHANNEL *servoSPStatBuf = NULL;
ServoSPStat servoSPStat;
int servoSPCmdSerialNumber = 0;

RCS_CMD_CHANNEL *servoSPCfgBuf = NULL;
RCS_STAT_CHANNEL *servoSPSetBuf = NULL;
ServoSPSet servoSPSet;
int servoSPCfgSerialNumber = 0;

RCS_CMD_CHANNEL *servoFactJACmdBuf = NULL;
RCS_STAT_CHANNEL *servoFactJAStatBuf = NULL;
ServoFactJAStat servoFactJAStat;
int servoFactJACmdSerialNumber = 0;

RCS_CMD_CHANNEL *servoFactJACfgBuf = NULL;
RCS_STAT_CHANNEL *servoFactJASetBuf = NULL;
ServoFactJASet servoFactJASet;
int servoFactJACfgSerialNumber = 0;

bool
initMoastBufs (void)
{
  int t;

  for (t = 0; t < MAX_SENSOR_DATA_1D_BUFFERS; t++)
    {
      rangescannerBuf[t] = NULL;
    }

  for (t = 0; t < MAX_SENSOR_DATA_2D_BUFFERS; t++)
    {
      rangeimagerBuf[t] = NULL;
    }

  for (t = 0; t < MAX_SENSOR_DATA_BUFFERS; t++)
    {
      sensorDataBuf[t] = NULL;
    }

  for (t = 0; t < SERVO_MIS_JA_MIS_PKG_MAX; t++)
    {
      servoMisJACmdBuf[t] = NULL;
      servoMisJAStatBuf[t] = NULL;
      servoMisJACmdSerialNumber[t] = 0;
      servoMisJACfgBuf[t] = NULL;
      servoMisJASetBuf[t] = NULL;
      servoMisJACfgSerialNumber[t] = 0;
    }

  for (t = 0; t < SERVO_EFF_JA_EFFECTOR_MAX; t++)
    {
      servoEffJACmdBuf[t] = NULL;
      servoEffJAStatBuf[t] = NULL;
      servoEffJACmdSerialNumber[t] = 0;
      servoEffJACfgBuf[t] = NULL;
      servoEffJASetBuf[t] = NULL;
      servoEffJACfgSerialNumber[t] = 0;
    }

  return true;
}

bool
getSonarBuf (char *thisProcess, char *nmlFile, int missionNumber)
{
  char bufferName[MOAST_NML_BUFFER_NAME_LEN];

  if (NULL == sonarBuf)
    {
      ulapi_snprintf (bufferName, sizeof (bufferName), "servoSPSonarArr%d_1",
		      missionNumber);
      bufferName[sizeof (bufferName) - 1] = 0;
      sonarBuf =
	new NML (sensorData_format, bufferName, thisProcess, nmlFile);
      if (NULL == sonarBuf || !sonarBuf->valid ())
	{
	  rcs_print_warning ("can't open NML buffer %s\n", bufferName);
	  if (NULL != sonarBuf)
	    delete sonarBuf;
	  sonarBuf = NULL;
	  return false;
	}
    }

  return true;
}

bool
getRangescannerBuf (char *thisProcess, char *nmlFile, int index,
		    int missionNumber)
{
  char bufferName[MOAST_NML_BUFFER_NAME_LEN];

  if (index < 0 || index >= MAX_SENSOR_DATA_1D_BUFFERS)
    return false;

  if (NULL == rangescannerBuf[index])
    {
      ulapi_snprintf (bufferName, sizeof (bufferName), "servoSPLinescan%d_%d",
		      missionNumber, index + 1);
      bufferName[sizeof (bufferName) - 1] = 0;
      rangescannerBuf[index] =
	new NML (sensorData_format, bufferName, thisProcess, nmlFile);
      if (NULL == rangescannerBuf[index] || !rangescannerBuf[index]->valid ())
	{
	  rcs_print_warning ("can't open NML buffer %s\n", bufferName);
	  if (NULL != rangescannerBuf[index])
	    delete rangescannerBuf[index];
	  rangescannerBuf[index] = NULL;
	  return false;
	}
    }

  return true;
}

bool
getRangeimagerBuf (char *thisProcess, char *nmlFile, int index,
		   int missionNumber)
{
  char bufferName[MOAST_NML_BUFFER_NAME_LEN];

  if (index < 0 || index >= MAX_SENSOR_DATA_2D_BUFFERS)
    return false;

  if (NULL == rangeimagerBuf[index])
    {
      ulapi_snprintf (bufferName, sizeof (bufferName), "%s%d_%d",
		      SERVO_SP_DATA_2D_BUF_NAME, missionNumber, index + 1);
      bufferName[sizeof (bufferName) - 1] = 0;
      rangeimagerBuf[index] =
	new NML (sensorData_format, bufferName, thisProcess, nmlFile);
      if (NULL == rangeimagerBuf[index] || !rangeimagerBuf[index]->valid ())
	{
	  rcs_print_warning ("can't open NML buffer %s\n", bufferName);
	  if (NULL != rangeimagerBuf[index])
	    delete rangeimagerBuf[index];
	  rangeimagerBuf[index] = NULL;
	  return false;
	}
    }

  return true;
}


bool
getNavDataExtBuf (char *thisProcess, char *nmlFile, int missionNumber)
{
  char bufferName[MOAST_NML_BUFFER_NAME_LEN];

  if (NULL == navDataExtBuf)
    {
      ulapi_snprintf (bufferName, sizeof (bufferName), "navDataExt%d",
		      missionNumber);
      bufferName[sizeof (bufferName) - 1] = 0;
      navDataExtBuf =
	new NML (navDataExt_format, bufferName, thisProcess, nmlFile);
      if (NULL == navDataExtBuf || !navDataExtBuf->valid ())
	{
	  rcs_print_warning ("can't open NML buffer %s\n", bufferName);
	  if (NULL != navDataExtBuf)
	    delete navDataExtBuf;
	  navDataExtBuf = NULL;
	  return false;
	}
    }

  return true;
}

bool
getSensorDataBuf (char *thisProcess, char *nmlFile, int index,
		  int missionNumber)
{
  char bufferName[MOAST_NML_BUFFER_NAME_LEN];

  if (index < 0 || index >= MAX_SENSOR_DATA_BUFFERS)
    return false;

  if (NULL == sensorDataBuf[index])
    {
      ulapi_snprintf (bufferName, sizeof (bufferName), "sensorData%d_%d",
		      missionNumber, index + 1);
      bufferName[sizeof (bufferName) - 1] = 0;
      sensorDataBuf[index] =
	new NML (sensorData_format, bufferName, thisProcess, nmlFile);
      if (NULL == sensorDataBuf[index] || !sensorDataBuf[index]->valid ())
	{
	  rcs_print_warning ("can't open NML buffer %s\n", bufferName);
	  if (NULL != sensorDataBuf[index])
	    delete sensorDataBuf[index];
	  sensorDataBuf[index] = NULL;
	  return false;
	}
    }

  return true;
}

bool
getServoMobJABufs (char *thisProcess, char *nmlFile, int missionNumber)
{
  char bufferName[MOAST_NML_BUFFER_NAME_LEN];

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoMobJACmd%d",
		  missionNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoMobJACmdBuf =
    new RCS_CMD_CHANNEL (servoMobJA_format, bufferName, thisProcess, nmlFile);
  if (NULL == servoMobJACmdBuf || !servoMobJACmdBuf->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoMobJAStat%d",
		  missionNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoMobJAStatBuf =
    new RCS_STAT_CHANNEL (servoMobJA_format, bufferName, thisProcess,
			  nmlFile);
  if (NULL == servoMobJAStatBuf || !servoMobJAStatBuf->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoMobJACfg%d",
		  missionNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoMobJACfgBuf =
    new RCS_CMD_CHANNEL (servoMobJA_format, bufferName, thisProcess, nmlFile);
  if (NULL == servoMobJACfgBuf || !servoMobJACfgBuf->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoMobJASet%d",
		  missionNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoMobJASetBuf =
    new RCS_STAT_CHANNEL (servoMobJA_format, bufferName, thisProcess,
			  nmlFile);
  if (NULL == servoMobJASetBuf || !servoMobJASetBuf->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }
  servoMobJASet.senCount = 0;
  servoMobJASet.effCount = 0;
  servoMobJASet.misCount = 0;

  return true;
}

bool
getServoMisJABufs (char *thisProcess, char *nmlFile, int packageNumber,
		   int missionNumber)
{
  char bufferName[MOAST_NML_BUFFER_NAME_LEN];
  int packageIndex;

  packageIndex = packageNumber - 1;
  if (packageIndex < 0 || packageIndex >= SERVO_MIS_JA_MIS_PKG_MAX)
    {
      rcs_print_warning ("bad mission package number: %d\n", packageNumber);
      return false;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoMisJACmd%d_%d",
		  missionNumber, packageNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoMisJACmdBuf[packageIndex] =
    new RCS_CMD_CHANNEL (servoMisJA_format, bufferName, thisProcess, nmlFile);
  if (NULL == servoMisJACmdBuf[packageIndex]
      || !servoMisJACmdBuf[packageIndex]->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }
  servoMisJACmdSerialNumber[packageIndex] = 0;

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoMisJAStat%d_%d",
		  missionNumber, packageNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoMisJAStatBuf[packageIndex] =
    new RCS_STAT_CHANNEL (servoMisJA_format, bufferName, thisProcess,
			  nmlFile);
  if (NULL == servoMisJAStatBuf[packageIndex]
      || !servoMisJAStatBuf[packageIndex]->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoMisJACfg%d_%d",
		  missionNumber, packageNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoMisJACfgBuf[packageIndex] =
    new RCS_CMD_CHANNEL (servoMisJA_format, bufferName, thisProcess, nmlFile);
  if (NULL == servoMisJACfgBuf[packageIndex]
      || !servoMisJACfgBuf[packageIndex]->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }
  servoMisJACfgSerialNumber[packageIndex] = 0;

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoMisJASet%d_%d",
		  missionNumber, packageNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoMisJASetBuf[packageIndex] =
    new RCS_STAT_CHANNEL (servoMisJA_format, bufferName, thisProcess,
			  nmlFile);
  if (NULL == servoMisJASetBuf[packageIndex]
      || !servoMisJASetBuf[packageIndex]->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }

  return true;
}

bool
getServoEffJABufs (char *thisProcess, char *nmlFile, int effectorNumber,
		   int missionNumber)
{
  char bufferName[MOAST_NML_BUFFER_NAME_LEN];
  int effectorIndex;

  effectorIndex = effectorNumber - 1;
  if (effectorIndex < 0 || effectorIndex >= SERVO_EFF_JA_EFFECTOR_MAX)
    {
      rcs_print_warning ("bad effector number: %d\n", effectorNumber);
      return false;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoEffJACmd%d_%d",
		  missionNumber, effectorNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoEffJACmdBuf[effectorIndex] =
    new RCS_CMD_CHANNEL (servoEffJA_format, bufferName, thisProcess, nmlFile);
  if (NULL == servoEffJACmdBuf[effectorIndex]
      || !servoEffJACmdBuf[effectorIndex]->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }
  servoEffJACmdSerialNumber[effectorIndex] = 0;

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoEffJAStat%d_%d",
		  missionNumber, effectorNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoEffJAStatBuf[effectorIndex] =
    new RCS_STAT_CHANNEL (servoEffJA_format, bufferName, thisProcess,
			  nmlFile);
  if (NULL == servoEffJAStatBuf[effectorIndex]
      || !servoEffJAStatBuf[effectorIndex]->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoEffJACfg%d_%d",
		  missionNumber, effectorNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoEffJACfgBuf[effectorIndex] =
    new RCS_CMD_CHANNEL (servoEffJA_format, bufferName, thisProcess, nmlFile);
  if (NULL == servoEffJACfgBuf[effectorIndex]
      || !servoEffJACfgBuf[effectorIndex]->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }
  servoEffJACfgSerialNumber[effectorIndex] = 0;

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoEffJASet%d_%d",
		  missionNumber, effectorNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoEffJASetBuf[effectorIndex] =
    new RCS_STAT_CHANNEL (servoEffJA_format, bufferName, thisProcess,
			  nmlFile);
  if (NULL == servoEffJASetBuf[effectorIndex]
      || !servoEffJASetBuf[effectorIndex]->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }

  return true;
}

bool
getServoSPBufs (char *thisProcess, char *nmlFile, int missionNumber)
{
  char bufferName[MOAST_NML_BUFFER_NAME_LEN];

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoSPCmd%d",
		  missionNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoSPCmdBuf =
    new RCS_CMD_CHANNEL (servoSP_format, bufferName, thisProcess, nmlFile);
  if (NULL == servoSPCmdBuf || !servoSPCmdBuf->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoSPStat%d",
		  missionNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoSPStatBuf =
    new RCS_STAT_CHANNEL (servoSP_format, bufferName, thisProcess, nmlFile);
  if (NULL == servoSPStatBuf || !servoSPStatBuf->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoSPCfg%d",
		  missionNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoSPCfgBuf =
    new RCS_CMD_CHANNEL (servoSP_format, bufferName, thisProcess, nmlFile);
  if (NULL == servoSPCfgBuf || !servoSPCfgBuf->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoSPSet%d",
		  missionNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoSPSetBuf =
    new RCS_STAT_CHANNEL (servoSP_format, bufferName, thisProcess, nmlFile);
  if (NULL == servoSPSetBuf || !servoSPSetBuf->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }

  return true;
}

bool
getServoFactJABufs (char *thisProcess, char *nmlFile, int missionNumber)
{
  char bufferName[MOAST_NML_BUFFER_NAME_LEN];

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoFactJACmd%d",
		  missionNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoFactJACmdBuf =
    new RCS_CMD_CHANNEL (servoFactJA_format, bufferName, thisProcess,
			 nmlFile);
  if (NULL == servoFactJACmdBuf || !servoFactJACmdBuf->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoFactJAStat%d",
		  missionNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoFactJAStatBuf =
    new RCS_STAT_CHANNEL (servoFactJA_format, bufferName, thisProcess,
			  nmlFile);
  if (NULL == servoFactJAStatBuf || !servoFactJAStatBuf->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoFactJACfg%d",
		  missionNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoFactJACfgBuf =
    new RCS_CMD_CHANNEL (servoFactJA_format, bufferName, thisProcess,
			 nmlFile);
  if (NULL == servoFactJACfgBuf || !servoFactJACfgBuf->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }

  ulapi_snprintf (bufferName, sizeof (bufferName), "servoFactJASet%d",
		  missionNumber);
  bufferName[sizeof (bufferName) - 1] = 0;
  servoFactJASetBuf =
    new RCS_STAT_CHANNEL (servoFactJA_format, bufferName, thisProcess,
			  nmlFile);
  if (NULL == servoFactJASetBuf || !servoFactJASetBuf->valid ())
    {
      rcs_print_warning ("can't open NML buffer %s\n", bufferName);
      return false;
    }

  return true;
}

void
cleanupNml (void)
{
  int t;

#define DISCONNECT_NML(BUF)			\
  if (NULL != BUF) {				\
    delete BUF;					\
    BUF = NULL;					\
  }

  for (t = 0; t < MAX_SENSOR_DATA_1D_BUFFERS; t++)
    {
      DISCONNECT_NML (rangescannerBuf[t]);
    }
  for (t = 0; t < MAX_SENSOR_DATA_2D_BUFFERS; t++)
    {
      DISCONNECT_NML (rangeimagerBuf[t]);
    }
  for (t = 0; t < MAX_SENSOR_DATA_BUFFERS; t++)
    {
      DISCONNECT_NML (sensorDataBuf[t]);
    }
  DISCONNECT_NML (sonarBuf);
  DISCONNECT_NML (navDataExtBuf);

  DISCONNECT_NML (servoMobJACmdBuf);
  DISCONNECT_NML (servoMobJAStatBuf);
  DISCONNECT_NML (servoMobJACfgBuf);
  DISCONNECT_NML (servoMobJASetBuf);

  for (t = 0; t < SERVO_MIS_JA_MIS_PKG_MAX; t++)
    {
      DISCONNECT_NML (servoMisJACmdBuf[t]);
      DISCONNECT_NML (servoMisJAStatBuf[t]);
      DISCONNECT_NML (servoMisJACfgBuf[t]);
      DISCONNECT_NML (servoMisJASetBuf[t]);
    }

  DISCONNECT_NML (servoSPCmdBuf);
  DISCONNECT_NML (servoSPStatBuf);
  DISCONNECT_NML (servoSPCfgBuf);
  DISCONNECT_NML (servoSPSetBuf);

  DISCONNECT_NML (servoFactJACmdBuf);
  DISCONNECT_NML (servoFactJAStatBuf);
  DISCONNECT_NML (servoFactJACfgBuf);
  DISCONNECT_NML (servoFactJASetBuf);

  return;
}
