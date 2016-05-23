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
  \file simware.hh

  \brief Declarations of simware data structures exchanged between
  the superior and inferior skins.
*/

#ifndef SIMWARE_H
#define SIMWARE_H

#include <string>

#define SW_NAME_MAX 80
#define SW_NONE 0

/* 
   enum for the type of robot
*/
enum ROBOT_TYPE
{
  SW_ROBOT_UNKNOWN = 0,
  SW_ROBOT_GRD_VEH,
  SW_ROBOT_AIR_VEH,
  SW_ROBOT_STATIC_VEH
};

/*
  The resources, such as vehicles, machines and sensors.
*/
enum
{
  SW_TYPE_UNINITIALIZED = 0,
  SW_SEN_ENCODER,
  SW_SEN_SONAR,
  SW_SEN_RANGESCANNER,
  SW_SEN_RANGEIMAGER,
  SW_SEN_TOUCH,
  SW_SEN_CO2,
  SW_SEN_INS,
  SW_SEN_GPS,
  SW_SEN_ODOMETER,
  SW_SEN_VICTIM,
  SW_SEN_TACHOMETER,
  SW_SEN_ACOUSTIC,
  SW_SEN_OBJECTSENSOR,
  SW_EFF_GRIPPER,
  SW_EFF_TOOLCHANGER,
  SW_ACT,
  SW_ROBOT_FIXED,
  SW_ROBOT_GROUNDVEHICLE,
  SW_ROBOT_AIRBOT,
  SW_DEVICE_BOXCHUTE,
  SW_DEVICE_CONVEYOR,
  SW_OBJECT_CARGO
};
typedef int sw_type;

extern char *sw_type_to_string (sw_type type);

/*
  The operation to perform.
*/
typedef int sw_op;

typedef struct
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  char offsetFrom[SW_NAME_MAX];	// parent of pose
  int linkOffset;
} sw_pose;

/*
  Each resource has an associated structure that contains everything
  neeed to command it, configure it and report settings and status.
  The \a sw_op determines which fields are actually used.
*/

// ros command types
enum
{
  SW_ROS_CMD_VEL = 1,
  SW_ROS_CMD_TRAJ,
  SW_ROS_CMD_GRIP,
  SW_ROS_CMD_TOOLCHANGE,
  SW_ROS_CMD_SCAN
};

enum
{
  SW_ROBOT_SKID_MOVE = 1,	/*!< move the vehicle the skid way */
  SW_ROBOT_ACKERMAN_MOVE,	/*!< move the vehicle the Ackerman way */
  SW_ROBOT_STAT,
  SW_ROBOT_SET
};

typedef enum
{
  SW_STEER_UNKNOWN = 0,
  SW_STEER_SKID,
  SW_STEER_ACKERMAN,
  SW_STEER_OMNI
} sw_steer_type;

typedef struct
{
  double linearx;
  double lineary;
  double linearz;
  double angularx;
  double angulary;
  double angularz;
} sw_ros_cmd_vel_struct;
typedef struct
{
  sw_pose mount;
} sw_robot_fixed_struct;

typedef struct
{
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

typedef struct
{
  double speed;
  double heading;
} sw_robot_airbot_struct;

enum
{
  SW_DEVICE_STAT = 1,
  SW_DEVICE_SET
};

enum
{
  SW_BOXCHUTE_RELEASE = 1
};
typedef struct
{
  int rfid;
} sw_boxchute_struct;

enum
{
  SW_CONVEYOR_SPEED = 1
};
typedef struct
{
  double speed;			/*!< commanded speed  */
  double minSpeed;		/*!< minimum speed */
} sw_conveyor_struct;

enum
{
  SW_CLEAR_CARGO = 1
};

#define SW_MAX_CARGO 20
typedef struct
{
  int numCargo;
  int id[SW_MAX_CARGO];
  int memory[SW_MAX_CARGO];
  sw_pose position[SW_MAX_CARGO];
} sw_cargo_struct;

enum
{
  SW_SEN_ENCODER_STAT = 1,
  SW_SEN_ENCODER_SET
};
typedef struct
{
  /* status */
  int tick;
  /* settings */
  double resolution;
  sw_pose mount;
} sw_sen_encoder_struct;

enum
{
  SW_SEN_SONAR_STAT = 1,
  SW_SEN_SONAR_SET,
  SW_SEN_GO
};
typedef struct
{
  double range;
  double minrange;
  double maxrange;
  double beamangle;
  sw_pose mount;
} sw_sen_sonar_struct;

enum
{
  SW_SEN_RANGEIMAGER_STAT = 1,
  SW_SEN_RANGEIMAGER_SET
};
#define SW_SEN_RANGEIMAGER_MAX 6000
typedef struct
{
  int frame;			/*!< Frame number (out of totalframes), frames must break on a line boundary */
  int totalframes;		/*!< Total number of frames */
  int numberperframe;		/*!< how many elements in this message  */
  float range[SW_SEN_RANGEIMAGER_MAX];
  double maxrange;
  double minrange;
  double resolutionx;
  double resolutiony;
  double fovx;
  double fovy;
  sw_pose mount;
} sw_sen_rangeimager_struct;


enum
{
  SW_SEN_RANGESCANNER_STAT = 1,
  SW_SEN_RANGESCANNER_SET
};
#define SW_SEN_RANGESCANNER_MAX 192	/*!< how many ranges we can have  */
typedef struct
{
  double range[SW_SEN_RANGESCANNER_MAX];
  double maxrange;
  double minrange;
  double resolution;
  double fov;
  sw_pose mount;
  int number;			/*!< how many there are  */
} sw_sen_rangescanner_struct;

enum
{
  SW_SEN_TOUCH_STAT = 1,
  SW_SEN_TOUCH_SET
};
typedef struct
{
  int touched;			/*!< non-zero means touching  */
} sw_sen_touch_struct;

enum
{
  SW_SEN_CO2_STAT = 1,
  SW_SEN_CO2_SET
};
typedef struct
{
  double density;		/*!< density of gas */
} sw_sen_co2_struct;

enum
{
  SW_SEN_INS_STAT = 1,
  SW_SEN_INS_SET
};
typedef struct
{
  sw_pose mount;
  sw_pose position;
  double period;
} sw_sen_ins_struct;

enum
{
  SW_SEN_GPS_STAT = 1,
  SW_SEN_GPS_SET
};
typedef struct
{
  double latitude;
  double longitude;
  int fix;
  int satellites;
  sw_pose mount;
  sw_pose position;
  double period;
} sw_sen_gps_struct;

enum
{
  SW_SEN_ODOMETER_STAT = 1,
  SW_SEN_ODOMETER_SET
};
typedef struct
{
  sw_pose mount;
  sw_pose position;
  double period;
  double resolution;
} sw_sen_odometer_struct;

enum
{
  SW_SEN_VICTIM_STAT = 1,
  SW_SEN_VICTIM_SET
};
typedef struct
{
  char parent[SW_NAME_MAX];
  sw_pose mount;
  double maxrange;
  double hfov;
  double vfov;
  int victims;
} sw_sen_victim_struct;

enum
{
  SW_SEN_TACHOMETER_STAT = 1,
  SW_SEN_TACHOMETER_SET
};
#define SW_SEN_TACHOMETER_MAX 4	/*!< how many tachometers we can have  */
typedef struct
{
  sw_pose mount;		/*!< where the tachometer set is located  */
  double speed[SW_SEN_TACHOMETER_MAX];
  double position[SW_SEN_TACHOMETER_MAX];
  int number;			/*!< how many there are  */
} sw_sen_tachometer_struct;

enum
{
  SW_SEN_ACOUSTIC_STAT = 1,
  SW_SEN_ACOUSTIC_SET
};
typedef struct
{
  sw_pose mount;		/*!< where the acoustic set is located  */
  double azimuth;		/*!< azimuth (yaw or pan angle) to source */
  double altitude;		/*!< altitude(pitch or tilt angle) to source */
  double volume;		/*!< average loudness of sound */
  double duration;		/*!< how long the sound was */
} sw_sen_acoustic_struct;

typedef struct
{
  char tag[SW_NAME_MAX];
  char material_name[SW_NAME_MAX];
  sw_pose position;
  sw_pose hit_location;
} sw_sen_object_struct;
enum
{
	SW_SEN_OBJECTSENSOR_STAT = 1,
	SW_SEN_OBJECTSENSOR_SET
};
typedef struct
{
  sw_sen_object_struct objects[SW_SEN_RANGESCANNER_MAX];
  sw_pose mount;
  double fov;
  int number; //the number of objects detected by the sensor
} sw_sen_objectsensor_struct;
enum
{
  SW_EFF_OPEN = 1,
  SW_EFF_CLOSE
};
typedef int effector_status;
enum
{
  SW_EFF_GRIPPER_STAT=1,
  SW_EFF_GRIPPER_SET
};
typedef struct
{
  sw_pose mount;
  sw_pose tip;
  effector_status status;
} sw_eff_gripper_struct;
enum
{
  SW_EFF_TOOLCHANGER_STAT=1,
  SW_EFF_TOOLCHANGER_SET,
  SW_EFF_TOOLCHANGER_UNKNOWN_TYPE,
  SW_EFF_TOOLCHANGER_GRIPPER,
  SW_EFF_TOOLCHANGER_VACUUM,
  SW_EFF_TOOLCHANGER_TOOLCHANGER
};
typedef int toolchanger_tool_type;
typedef struct
{
  sw_pose mount;
  toolchanger_tool_type tooltype;
  effector_status status;
} sw_eff_toolchanger_struct;
#define SW_ACT_LINK_MAX 16	/*!< how many links a mission package can have */

enum
{
  SW_LINK_REVOLUTE = 1,
  SW_LINK_PRISMATIC,
  SW_LINK_SCISSOR
};
typedef int link_type;

#define sw_link_to_string(TYPE) ((TYPE) == SW_LINK_REVOLUTE ? "Revolute" : (TYPE) == SW_LINK_PRISMATIC ? "Prismatic" : (TYPE) == SW_LINK_SCISSOR ? "Scissor" : "Unknown")

typedef struct
{
  int parent;			/*!< Index of parent, 0 for the base. */
  link_type type;
  sw_pose mount;		/*!< Pose with respect to parent link. */
  double minvalue;
  double maxvalue;
  double maxspeed;
  double maxtorque;
  double position;		/*!< Linear or angular position. */
  double speed;
  double torque;
} sw_link_struct;

enum
{
  SW_ACT_POSITION = 1,		/*!< Move the joints to a position. */
  SW_ACT_SPEED,			/*!< Move the joints at a speed. */
  SW_ACT_TORQUE,		/*!< Move the joints by force or torque. */
  SW_ACT_STAT,
  SW_ACT_SET
};
typedef struct
{
  sw_pose mount;
  sw_pose tip;
  sw_link_struct link[SW_ACT_LINK_MAX];
  int number;			/*!< how many links */
} sw_actuator_struct;

typedef struct
{
  double goal[SW_ACT_LINK_MAX];
  int number;
} sw_ros_cmd_traj_struct;
typedef struct
{
  effector_status goal;
} sw_ros_cmd_effector_struct;
typedef struct
{
  int dummy;
} sw_ros_cmd_scan_struct;
typedef struct
{
  double time;
  sw_type type;			/*!< Which resource is selected. */
  sw_op op;			/*!< What operation to do on the resource.  */
  std::string name;		/*!< Resources are identified by string names. */
  union
  {
    /* ros */
    sw_ros_cmd_vel_struct roscmdvel;
    sw_ros_cmd_traj_struct roscmdtraj;
    sw_ros_cmd_effector_struct roscmdeff;
    sw_ros_cmd_scan_struct roscmdscan;
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
    sw_eff_toolchanger_struct toolchanger;
    /* sensors */
    sw_sen_encoder_struct encoder;
    sw_sen_sonar_struct sonar;
    sw_sen_rangescanner_struct rangescanner;
    sw_sen_rangeimager_struct rangeimager;
    sw_sen_touch_struct touch;
    sw_sen_co2_struct co2sensor;
    sw_sen_ins_struct ins;
    sw_sen_gps_struct gps;
    sw_sen_odometer_struct odometer;
    sw_sen_victim_struct victim;
    sw_sen_tachometer_struct tachometer;
    sw_sen_acoustic_struct acoustic;
    sw_sen_objectsensor_struct objectsensor;
    /* actuators */
    sw_actuator_struct actuator;
  } data;
} sw_struct;

extern const char *swTypeToString (sw_type type);
extern const char *swRobotType (ROBOT_TYPE type);
#endif
