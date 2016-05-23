#ifndef MOASTBUFS_H
#define MOASTBUFS_H

#include <rcs.hh>
#include <navDataExt.hh>
#include <sensorData.hh>
#include <servoMobJA.hh>
#include <servoMisJA.hh>
#include <servoEffJA.hh>
#include <servoFactJA.hh>
#include <servoSP.hh>

/*
  NML buffer name syntax is

  buffername<M>

  where <M> is the mission (vehicle or robot) number, e.g., 

  servoMobJACmd1 for the first mission,
  servoMobJACmd2 for the second one.

  If there are several subsystems of the same type on a vehicle or robot,
  e.g., for several linescanners, the syntax is

  buffername<M>_<N>

  where <N> is the subsystem number, e.g., 

  servoSPLinescan1_1 for the first linescanner in mission 1,
  servoSPLinescan1_2 for the second linescanner in mission 1,
  servoSPLinescan4_3 for the third linescanner in mission 4,

  servoMisJACmd1_1 for the command buffer to the first arm in mission 1,
  servoMisJAStat3_2 for the status buffer to the second arm in mission 3.
*/

// The maximum number of 1D sensor data buffers
#define MAX_SENSOR_DATA_1D_BUFFERS 4

// We can have many rangescanner buffers
extern NML *rangescannerBuf[MAX_SENSOR_DATA_1D_BUFFERS];

// The maximum number of 2D sensor data buffers
#define MAX_SENSOR_DATA_2D_BUFFERS 4

// We can have many rangeimager buffers
extern NML *rangeimagerBuf[MAX_SENSOR_DATA_2D_BUFFERS];


/*
  Each sonar sensor is independent of the others, but they are
  aggregated in MOAST into a single SensorData3D buffer. This
  buffer is optional, and will be allocated only if needed.
*/
extern NML *sonarBuf;

/*
  We only have one NavDataExt buffer, and it's optional. It will
  be allocated if needed.
  FIXME -- eventually this will be created by PrimSP, and we'll
  need to define a servo-level nav buffer for GPS, INS and ground
  truth sensors at this lower level that Prim will use when building
  the real NavDataExt structure.
*/
extern NML *navDataExtBuf;
extern NML *sensorDataBuf[MAX_SENSOR_DATA_BUFFERS];
extern NML *rangeimagerBuf[MAX_SENSOR_DATA_2D_BUFFERS];

/*
  We can have many SensorSetElem types.
 */
extern NML *sensorSetElemBuf[SERVO_SP_SENSOR_MAX];

extern RCS_CMD_CHANNEL *servoMobJACmdBuf;
extern RCS_STAT_CHANNEL *servoMobJAStatBuf;
extern ServoMobJAStat servoMobJAStat;
extern ServoMobJAStatGrdVeh servoMobJAStatGrdVeh;
extern ServoMobJAStatAirBot servoMobJAStatAirBot;
extern int servoMobJACmdSerialNumber;

extern RCS_CMD_CHANNEL *servoMobJACfgBuf;
extern RCS_STAT_CHANNEL *servoMobJASetBuf;
extern ServoMobJASet servoMobJASet;
extern ServoMobJASetGrdVeh servoMobJASetGrdVeh;
extern ServoMobJASetAirBot servoMobJASetAirBot;
extern int servoMobJACfgSerialNumber;

extern RCS_CMD_CHANNEL *servoMisJACmdBuf[SERVO_MIS_JA_MIS_PKG_MAX];
extern RCS_STAT_CHANNEL *servoMisJAStatBuf[SERVO_MIS_JA_MIS_PKG_MAX];
extern ServoMisJAStat servoMisJAStat[SERVO_MIS_JA_MIS_PKG_MAX];
extern int servoMisJACmdSerialNumber[SERVO_MIS_JA_MIS_PKG_MAX];

extern RCS_CMD_CHANNEL *servoMisJACfgBuf[SERVO_MIS_JA_MIS_PKG_MAX];
extern RCS_STAT_CHANNEL *servoMisJASetBuf[SERVO_MIS_JA_MIS_PKG_MAX];
extern ServoMisJASet servoMisJASet[SERVO_MIS_JA_MIS_PKG_MAX];
extern int servoMisJACfgSerialNumber[SERVO_MIS_JA_MIS_PKG_MAX];

extern RCS_CMD_CHANNEL *servoEffJACmdBuf[SERVO_EFF_JA_EFFECTOR_MAX];
extern RCS_STAT_CHANNEL *servoEffJAStatBuf[SERVO_EFF_JA_EFFECTOR_MAX];
extern ServoEffJAStat servoEffJAStat[SERVO_EFF_JA_EFFECTOR_MAX];
extern int servoEffJACmdSerialNumber[SERVO_EFF_JA_EFFECTOR_MAX];

extern RCS_CMD_CHANNEL *servoEffJACfgBuf[SERVO_EFF_JA_EFFECTOR_MAX];
extern RCS_STAT_CHANNEL *servoEffJASetBuf[SERVO_EFF_JA_EFFECTOR_MAX];
extern ServoEffJASet servoEffJASet[SERVO_EFF_JA_EFFECTOR_MAX];
extern int servoEffJACfgSerialNumber[SERVO_EFF_JA_EFFECTOR_MAX];

extern RCS_CMD_CHANNEL *servoSPCmdBuf;
extern RCS_STAT_CHANNEL *servoSPStatBuf;
extern ServoSPStat servoSPStat;
extern int servoSPCmdSerialNumber;

extern RCS_CMD_CHANNEL *servoSPCfgBuf;
extern RCS_STAT_CHANNEL *servoSPSetBuf;
extern ServoSPSet servoSPSet;
extern int servoSPCfgSerialNumber;

extern RCS_CMD_CHANNEL *servoFactJACmdBuf;
extern RCS_STAT_CHANNEL *servoFactJAStatBuf;
extern ServoFactJAStat servoFactJAStat;
extern int servoFactJACmdSerialNumber;

extern RCS_CMD_CHANNEL *servoFactJACfgBuf;
extern RCS_STAT_CHANNEL *servoFactJASetBuf;
extern ServoFactJASet servoFactJASet;
extern int servoFactJACfgSerialNumber;

#define rcs_stat_to_string(s) (s) == RCS_DONE ? "RCS_DONE" : (s) == RCS_EXEC ? "RCS_EXEC" : (s) == RCS_ERROR ? "RCS_ERROR" : "?"

#define rcs_admin_state_to_string(s) (s) == ADMIN_INITIALIZED ? "Initialized" : (s) == ADMIN_UNINITIALIZED ? "Uninitialized" : (s) == ADMIN_SHUT_DOWN ? "Shut Down" : "?"

#define SERVO_MOB_JA_CMD_WRITE(msg) servoMobJACmdSerialNumber += 2; (msg).serial_number = servoMobJACmdSerialNumber; servoMobJACmdBuf->write(&(msg))

#define SERVO_MOB_JA_CFG_WRITE(msg) servoMobJACfgSerialNumber += 2; (msg).serial_number = servoMobJACfgSerialNumber; servoMobJACfgBuf->write(&(msg))

#define SERVO_MIS_JA_CMD_WRITE(index,msg) servoMisJACmdSerialNumber[index] += 2; (msg).serial_number = servoMisJACmdSerialNumber[index]; servoMisJACmdBuf[index]->write(&(msg))

#define SERVO_MIS_JA_CFG_WRITE(index,msg) servoMisJACfgSerialNumber[index] += 2; (msg).serial_number = servoMisJACfgSerialNumber[index]; servoMisJACfgBuf[index]->write(&(msg))

#define SERVO_EFF_JA_CMD_WRITE(index,msg) servoEffJACmdSerialNumber[index] += 2; (msg).serial_number = servoEffJACmdSerialNumber[index]; servoEffJACmdBuf[index]->write(&(msg))

#define SERVO_EFF_JA_CFG_WRITE(index,msg) servoEffJACfgSerialNumber[index] += 2; (msg).serial_number = servoEffJACfgSerialNumber[index]; servoEffJACfgBuf[index]->write(&(msg))

#define SERVO_SP_CMD_WRITE(msg) servoSPCmdSerialNumber += 2; (msg).serial_number = servoSPCmdSerialNumber; servoSPCmdBuf->write(&(msg))

#define SERVO_SP_CFG_WRITE(msg) servoSPCfgSerialNumber += 2; (msg).serial_number = servoSPCfgSerialNumber; servoSPCfgBuf->write(&(msg))

#define SERVO_FACT_JA_CMD_WRITE(msg) servoFactJACmdSerialNumber += 2; (msg).serial_number = servoFactJACmdSerialNumber; servoFactJACmdBuf->write(&(msg))

#define SERVO_FACT_JA_CFG_WRITE(msg) servoFactJACfgSerialNumber += 2; (msg).serial_number = servoFactJACfgSerialNumber; servoFactJACfgBuf->write(&(msg))

extern bool initMoastBufs(void);

extern bool getSonarBuf(char *thisProcess, char *nmlFile, int missionNumber);

extern bool getRangescannerBuf(char *thisProcess, char *nmlFile, int index, int missionNumber);

extern bool getRangeimagerBuf(char *thisProcess, char *nmlFile, int index, int missionNumber);

extern bool getNavDataExtBuf(char *thisProcess, char *nmlFile, int missionNumber);

extern bool getSensorDataBuf(char *thisProcess, char *nmlFile, int index, int missionNumber);

extern bool getRangeImagerBuf(char *thisProcess, char *nmlFile, int index, int missionNumber);

extern bool getServoMobJABufs(char *thisProcess, char *nmlFile, int missionNumber);

extern bool getServoMobJABufs(char *thisProcess, char *nmlFile, int missionNumber);

extern bool getServoMisJABufs(char *thisProcess, char *nmlFile, int packageNumber, int missionNumber);

extern bool getServoEffJABufs(char *thisProcess, char *nmlFile, int effectorNumber, int missionNumber);

extern bool getServoSPBufs(char *thisProcess, char *nmlFile, int missionNumber);

extern bool getServoFactJABufs(char *thisProcess, char *nmlFile, int missionNumber);

extern void cleanupNml(void);

#endif

