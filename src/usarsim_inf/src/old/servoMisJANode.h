#ifndef SERVO_MIS_JA_NODE_H
#define SERVO_MIS_JA_NODE_H

#include <rcs.hh>
#include "servoMisJA.hh"
#include "skin.h"

class ServoMisJANode {
public:
  ServoMisJANode(void);
  ~ServoMisJANode(void);
  int init(char *process, char *nmlfile, int packageNumber, int argMission, char *argName, int (*argInfTell)(sw_struct *sw));
  char *getName(void);

  void takeStatMutex(void);
  void giveStatMutex(void);
  void takeSetMutex(void);
  void giveSetMutex(void);

  int setStat(sw_struct *sw);
  int setSet(sw_struct *sw);

  int writeStat(void);
  int writeSet(void);

  int cmdPass(void);
  int cmdRead(void);
  int cfgPass(void);
  int cfgRead(void);

  void doCmdInit(void);
  void doCmdMove(void);

  void doCfgCycleTime(void);

  void sleep(void);

private:
  RCS_CMD_CHANNEL *cmdBuf;
  RCS_STAT_CHANNEL *statBuf;
  RCS_CMD_CHANNEL *cfgBuf;
  RCS_STAT_CHANNEL *setBuf;
  ServoMisJAStat stat;
  ServoMisJASet set;
  void *cmdReadTask;
  void *cmdPassTask;
  void *cfgReadTask;
  void *cfgPassTask;
  void *statMutex;
  void *setMutex;
  double cycleTime;

  int number;
  char name[SW_NAME_MAX];
  int (*inf_tell)(sw_struct *sw);
};

#endif

