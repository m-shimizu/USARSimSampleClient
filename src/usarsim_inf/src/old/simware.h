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
  \file simware.h

  \brief Declarations of simware data structures exchanged between
  the superior and inferior skins.
*/

#ifndef SIMWARE_H
#define SIMWARE_H

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

#define SW_NAME_MAX 80
#define SW_NONE 0

/*
  The resources, such as vehicles, machines and sensors.
*/  
enum {
  SW_TYPE_UNINITIALIZED = 0,
  SW_SEN_ENCODER,
  SW_SEN_SONAR,
  SW_SEN_RANGESCANNER,
  SW_SEN_RANGEIMAGER,
  SW_SEN_TOUCH,
  SW_SEN_CO2,
  SW_SEN_GROUNDTRUTH,
  SW_SEN_INS,
  SW_SEN_GPS,
  SW_SEN_ODOMETER,
  SW_SEN_VICTIM,
  SW_SEN_TACHOMETER,
  SW_SEN_ACOUSTIC,
  SW_EFF_GRIPPER,
  SW_ACT,
  SW_ROBOT_FIXED,
  SW_ROBOT_GROUNDVEHICLE,
  SW_ROBOT_AIRBOT,
  SW_DEVICE_BOXCHUTE,
  SW_DEVICE_CONVEYOR,
  SW_OBJECT_CARGO
};
typedef int sw_type;

extern char *sw_type_to_string(sw_type type);

/*
  The operation to perform.
*/
typedef int sw_op;

typedef struct {
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
} sw_pose;

/*
  Each resource has an associated structure that contains everything
  neeed to command it, configure it and report settings and status.
  The \a sw_op determines which fields are actually used.
*/

enum {
  SW_ROBOT_SKID_MOVE = 1,	/*!< move the vehicle the skid way */
  SW_ROBOT_ACKERMAN_MOVE,    /*!< move the vehicle the Ackerman way */
  SW_ROBOT_STAT,
  SW_ROBOT_SET
};

typedef enum {
  SW_STEER_UNKNOWN = 0,
  SW_STEER_SKID,
  SW_STEER_ACKERMAN,
  SW_STEER_OMNI
} sw_steer_type;

typedef struct {
  sw_pose mount;
} sw_robot_fixed_struct;

typedef struct {
  /* settings */
  double length;
  double width;
  double height;
  double mass;
  sw_pose cg;
  sw_steer_type steertype;
  double wheel_separation;
  double wheel_radius;
  double wheel_base;
  double max_speed;
  double max_torque;
  double max_steer_angle;
  double min_turning_radius;
  /* status */
  double speed;
  double heading;
  double left_speed;
  double right_speed;
} sw_robot_groundvehicle_struct;

typedef struct {
  double speed;
  double heading;
} sw_robot_airbot_struct;

enum {
  SW_DEVICE_STAT = 1,
  SW_DEVICE_SET
};

enum {
  SW_BOXCHUTE_RELEASE = 1
};
typedef struct {
  int rfid;
} sw_boxchute_struct;

enum {
  SW_CONVEYOR_SPEED = 1
};
typedef struct {
  double speed;			/*!< commanded speed  */
  double minSpeed;		/*!< minimum speed */
} sw_conveyor_struct;

enum {
  SW_CLEAR_CARGO = 1
};

#define SW_MAX_CARGO 20
typedef struct {
  int numCargo;
  int id[SW_MAX_CARGO];
  int memory[SW_MAX_CARGO];
  sw_pose position[SW_MAX_CARGO];
} sw_cargo_struct;

enum {
  SW_SEN_ENCODER_STAT = 1,
  SW_SEN_ENCODER_SET
};
typedef struct {
  /* status */
  int tick;
  /* settings */
  double resolution;
  sw_pose mount;
} sw_sen_encoder_struct;

enum {
  SW_SEN_SONAR_STAT = 1,
  SW_SEN_SONAR_SET,
  SW_SEN_GO
};
typedef struct {
  double range;
  double minrange;
  double maxrange;
  double beamangle;
  sw_pose mount;
} sw_sen_sonar_struct;

enum {
  SW_SEN_RANGEIMAGER_STAT = 1,
  SW_SEN_RANGEIMAGER_SET
};
#define SW_SEN_RANGEIMAGER_MAX 6000
typedef struct {
  int frame;                    /*!< Frame number (out of totalframes), frames must break on a line boundary */
  int totalframes;              /*!< Total number of frames */
  int numberperframe;		/*!< how many elements in this message  */
  double range[SW_SEN_RANGEIMAGER_MAX];
  double maxrange;
  double minrange;
  double resolutionx;
  double resolutiony;
  double fovx;
  double fovy;
  sw_pose mount;
} sw_sen_rangeimager_struct;


enum {
  SW_SEN_RANGESCANNER_STAT = 1,
  SW_SEN_RANGESCANNER_SET
};
#define SW_SEN_RANGESCANNER_MAX 192	/*!< how many ranges we can have  */
typedef struct {
  double range[SW_SEN_RANGESCANNER_MAX];
  double maxrange;
  double minrange;
  double resolution;
  double fov;
  sw_pose mount;
  int number;			/*!< how many there are  */
} sw_sen_rangescanner_struct;

enum {
  SW_SEN_TOUCH_STAT = 1,
  SW_SEN_TOUCH_SET
};
typedef struct {
  int touched;			/*!< non-zero means touching  */
} sw_sen_touch_struct;

enum {
  SW_SEN_CO2_STAT = 1,
  SW_SEN_CO2_SET
};
typedef struct {
  double density;		/*!< density of gas */
} sw_sen_co2_struct;

enum {
  SW_SEN_GROUNDTRUTH_STAT = 1,
  SW_SEN_GROUNDTRUTH_SET
};
typedef struct {
  /* status */
  sw_pose position;
  /* settings */
  sw_pose mount;
  double period;
} sw_sen_groundtruth_struct;

enum {
  SW_SEN_INS_STAT = 1,
  SW_SEN_INS_SET
};
typedef struct {
  sw_pose mount;
  sw_pose position;
  double period;
} sw_sen_ins_struct;

enum {
  SW_SEN_GPS_STAT = 1,
  SW_SEN_GPS_SET
};
typedef struct {
  double latitude;
  double longitude;
  int fix;
  int satellites;
  sw_pose mount;
  sw_pose position;
  double period;
} sw_sen_gps_struct;

enum {
  SW_SEN_ODOMETER_STAT = 1,
  SW_SEN_ODOMETER_SET
};
typedef struct {
  sw_pose mount;
  sw_pose position;
  double period;
  double resolution;
} sw_sen_odometer_struct;

enum {
  SW_SEN_VICTIM_STAT = 1,
  SW_SEN_VICTIM_SET
};
typedef struct {
  char parent[SW_NAME_MAX];
  sw_pose mount;
  double maxrange;
  double hfov;
  double vfov;
  int victims;
} sw_sen_victim_struct;

enum {
  SW_SEN_TACHOMETER_STAT = 1,
  SW_SEN_TACHOMETER_SET
};
#define SW_SEN_TACHOMETER_MAX 4	/*!< how many tachometers we can have  */
typedef struct {
  sw_pose mount;		/*!< where the tachometer set is located  */
  double speed[SW_SEN_TACHOMETER_MAX];
  double position[SW_SEN_TACHOMETER_MAX];
  int number;			/*!< how many there are  */
} sw_sen_tachometer_struct;

enum {
  SW_SEN_ACOUSTIC_STAT = 1,
  SW_SEN_ACOUSTIC_SET
};
typedef struct {
  sw_pose mount;		/*!< where the acoustic set is located  */
  double azimuth;		/*!< azimuth (yaw or pan angle) to source */
  double altitude;		/*!< altitude(pitch or tilt angle) to source */
  double volume;		/*!< average loudness of sound */
  double duration;		/*!< how long the sound was */
} sw_sen_acoustic_struct;

enum {
  SW_EFF_GRIPPER_OPEN = 1,
  SW_EFF_GRIPPER_CLOSE,
  SW_EFF_GRIPPER_STAT,
  SW_EFF_GRIPPER_SET
};
typedef struct {
  sw_pose mount;
  char open;			/*!< non-zero means the gripper is open  */
} sw_eff_gripper_struct;

#define SW_ACT_LINK_MAX 16	/*!< how many links a mission package can have */

enum {
  SW_LINK_REVOLUTE = 1,
  SW_LINK_PRISMATIC,
  SW_LINK_SCISSOR
};
typedef int link_type;

#define sw_link_to_string(TYPE) ((TYPE) == SW_LINK_REVOLUTE ? "Revolute" : (TYPE) == SW_LINK_PRISMATIC ? "Prismatic" : (TYPE) == SW_LINK_SCISSOR ? "Scissor" : "Unknown")

typedef struct {
  int parent;			/*!< Index of parent, 0 for the base. */
  link_type type;
  sw_pose mount;		/*!< Pose with respect to parent link. */
  double minrange;
  double maxrange;
  double maxspeed;
  double maxtorque;
  double position;		/*!< Linear or angular position. */
  double speed;
  double torque;
} sw_link_struct;

enum {
  SW_ACT_POSITION = 1,	/*!< Move the joints to a position. */
  SW_ACT_SPEED,		/*!< Move the joints at a speed. */
  SW_ACT_TORQUE,		/*!< Move the joints by force or torque. */
  SW_ACT_STAT,
  SW_ACT_SET
};
typedef struct {
  sw_link_struct link[SW_ACT_LINK_MAX];
  int number;			/*!< how many links */
} sw_mispkg_struct;

typedef struct {
  double time;
  sw_type type;			/*!< Which resource is selected. */
  sw_op op;			/*!< What operation to do on the resource.  */
  char name[SW_NAME_MAX]; /*!< Resources are identified by string names. */
  union {
    /* robots */
    sw_robot_groundvehicle_struct groundvehicle;
    sw_robot_airbot_struct airbot;
    /* machines */
    sw_boxchute_struct boxchute;
    sw_conveyor_struct conveyor;
    /* things */
    sw_cargo_struct cargo;
    /* effectors */
    sw_eff_gripper_struct gripper;
    /* sensors */
    sw_sen_encoder_struct encoder;
    sw_sen_sonar_struct sonar;
    sw_sen_rangescanner_struct rangescanner;
    sw_sen_rangeimager_struct rangeimager;
    sw_sen_touch_struct touch;
    sw_sen_co2_struct co2sensor;
    sw_sen_groundtruth_struct groundtruth;
    sw_sen_ins_struct ins;
    sw_sen_gps_struct gps;
    sw_sen_odometer_struct odometer;
    sw_sen_victim_struct victim;
    sw_sen_tachometer_struct tachometer;
    sw_sen_acoustic_struct acoustic;
    /* mission packages */
    sw_mispkg_struct mispkg;
  } data;
} sw_struct;

#define sw_set_time(PTR,TIME) (PTR)->time = (TIME)
#define sw_set_op(PTR,OP) (PTR)->op = (OP)
#define sw_set_type(PTR,TYPE) (PTR)->type = (TYPE)
#define sw_set_name(PTR,STR) strncpy((PTR)->name,(STR),SW_NAME_MAX); (PTR)->name[SW_NAME_MAX - 1] = 0
#define sw_clear_name(PTR) ((PTR)->name[0] = 0)
#define sw_no_name(PTR) (0 == (PTR)->name[0])
#define sw_match_name(PTR,STR) (! strcmp((PTR)->name,(STR)))

/* the sw_get_xxx macros need to resolve to compile-time constants,
   since we switch on them */
#define sw_get_time(PTR) (PTR)->time
#define sw_get_op(PTR) (PTR)->op
#define sw_get_type(PTR) (PTR)->type
#define sw_get_name(PTR) (PTR)->name

#define sw_init(PTR) sw_set_time(PTR,0); sw_set_op(PTR,SW_NONE); sw_set_type(PTR,SW_NONE); sw_clear_name(PTR)

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif
