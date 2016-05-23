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
  \file usarsiminf.c

  \brief The inferior skin to USARSim.
*/
#define _USE_MATH_DEFINES

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <ctype.h>
#include <stdarg.h>
#include <float.h>
#include <math.h>		/* atan2, sqrt */
#include "ulapi.h"
#include "gotypes.h"
#include "gomath.h"
#include "simware.h"
#include "skin.h"

static int debug = 0;

static void
dbprintf (const char *fmt, ...)
{
  va_list ap;

  if (debug)
    {
      va_start (ap, fmt);
      fprintf (stderr, "usarsiminf: ");
      vfprintf (stderr, fmt, ap);
      va_end (ap);
    }
}

static int verbose = 0;

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

  fprintf (stderr, "usarsiminf: ");
  va_start (ap, fmt);
  vfprintf (stderr, fmt, ap);
  va_end (ap);
}

static int
no_sup_tell (sw_struct * sw)
{
  errprintf ("no sup_tell function defined\n");

  return -1;
}

static int (*sup_tell) (sw_struct * sw) = no_sup_tell;

static int socket_fd = -1;

/* wraps socket writing with a verbose printf, for debugging */
static ulapi_integer
usarsim_socket_write (ulapi_integer id, char *buf, ulapi_integer len)
{
  vbprintf (buf);

  return ulapi_socket_write (id, buf, len);
}

static int waiting_for_conf = 0;
static int waiting_for_geo = 0;

#define SOCKET_MUTEX_KEY 1
static void *socket_mutex = NULL;

#define BUFFERLEN 8
static int buildlen = BUFFERLEN;
static char *build = NULL;
static char *build_ptr;
static char *build_end;

#define DELIMITER 10
#define MAX_MSG_LEN 1024
#define MAX_TOKEN_LEN 1024

/* only works with arrays, not heap */
#define NULLTERM(s) (s)[sizeof(s)-1]=0

static char *
get_key (char *msg, char *key)
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

static char *
get_value (char *msg, char *value)
{
  char *ptr = msg;

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
      *value++ = *ptr++;
    }
  *value = 0;			/* null terminate  */
  // printf( "get_value: value: %s ptr: %s\n", value, ptr );
  return ptr;
}

typedef struct _usarsim_class_struct
{
  sw_struct sw;
  int did_conf;
  int did_geo;
  struct _usarsim_class_struct *next;
} usarsim_class_struct, *usarsim_class_list;

#define usarsim_class_init(TYPE,CS) (CS) = malloc(sizeof(*(CS))); sw_init(&(CS)->sw); sw_set_type(&(CS)->sw,TYPE); (CS)->did_conf = 0; (CS)->did_geo = 0

static int
usarsim_class_find (usarsim_class_list ptr, char *name,
		    usarsim_class_list * where)
{
  if (NULL == ptr)
    return -1;

  while (!sw_no_name (&ptr->sw))
    {
      if (sw_match_name (&ptr->sw, name))
	{
	  /* found it */
	  *where = ptr;
	  return 0;
	}
      ptr = ptr->next;
    }

  /* a new one-- fill in the terminal empty structure... */
  sw_set_name (&ptr->sw, name);
  ptr->did_conf = 0;
  ptr->did_geo = 0;

  /* ...and get a new terminal empty structure */
  usarsim_class_init (sw_get_type (&ptr->sw), ptr->next);
  if (NULL == ptr->next)
    return -1;

  *where = ptr;
  return 0;
}

static usarsim_class_list encoders;
static usarsim_class_list sonars;
static usarsim_class_list rangescanners;
static usarsim_class_list rangeimagers;
static usarsim_class_list touches;
static usarsim_class_list co2sensors;
static usarsim_class_list groundtruths;
static usarsim_class_list inses;
static usarsim_class_list gpses;
static usarsim_class_list odometers;
static usarsim_class_list victims;
static usarsim_class_list tachometers;
static usarsim_class_list acoustics;

static usarsim_class_list misstas;

static usarsim_class_list grippers;

static usarsim_class_struct robot;

/*
  SEN {Type Encoder} 

  {Name EncTest1} {Tick 0}
*/
static int
handle_sen_encoder (char *msg)
{
#define VAR_DECL				\
  char token[MAX_TOKEN_LEN];			\
  char *ptr;					\
  char *nextptr;				\
  int sawname;					\
  int count;					\
  double time;					\
  double d;					\
  usarsim_class_struct def;			\
  usarsim_class_list where

#define VAR_INIT				\
  ptr = msg;					\
  sawname = 0;					\
  count = 0;					\
  time = 0;					\
  d = 0;					\
  where = &def

  VAR_DECL;
  int i;

  VAR_INIT;

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

#define TELL(OP)				\
  if (sw_no_name(&where->sw)) {			\
    dbprintf("nothing configured for %s\n", msg); \
  } else {					\
    sw_set_time(&where->sw, time);		\
    sw_set_op(&where->sw, OP);			\
    sup_tell(&where->sw);			\
  }

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
#define EXPECT(TOKEN)				\
      nextptr = get_value(ptr, token);		\
      if (nextptr == ptr) {			\
	errprintf("EXPECT: nextptr = ptr for token %s and pointer %s\n", token, ptr); \
	return -1;}				\
      if (strcmp(token, TOKEN)) {		\
	errprintf("EXPECT: found %s wanted %s\n", token, TOKEN );	\
	return -1;}						\
      ptr = nextptr;
	  EXPECT ("Encoder");
	}
      else if (!strcmp (token, "Name"))
	{
#define GET_NAME(LIST,OP)						\
      if (sawname) {							\
	TELL(OP);							\
      }									\
      sawname = 1;							\
      nextptr = get_value(ptr, token);					\
      if (nextptr == ptr) return -1;					\
      if (0 != usarsim_class_find(LIST, token, &where)) {               \
	errprintf("error from GET_NAME macro on usarsim_class_find");   \
	return -1;}							\
      ptr = nextptr;
	  GET_NAME (encoders, SW_SEN_ENCODER_STAT);
	}
      else if (!strcmp (token, "Tick"))
	{
#define GET_INTEGER(VAR)				\
      nextptr = get_value(ptr, token);			\
      if (nextptr == ptr) return -1;			\
      if (1 != sscanf(token, "%i", &i)) return -1;	\
      where->sw.data.VAR = i;				\
      count++;						\
      ptr = nextptr;
	  GET_INTEGER (encoder.tick);
	}
      else if (!strcmp (token, "Time"))
	{
#define GET_TIME					\
      nextptr = get_value(ptr, token);			\
      if (nextptr == ptr) return -1;			\
      if (1 != sscanf(token, "%lf", &d)) return -1;	\
      time = d;						\
      ptr = nextptr;
	  GET_TIME;
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_ENCODER_STAT);

  return count;
}

/*
  SEN {Time 5608.8500} {Type Sonar}

  {Name F1 Range 2.5098} {Name F2 Range 3.3153} {Name F3 Range 4.9950} {Name F4 Range 2.0774} {Name F5 Range 2.0780} {Name F6 Range 5.0000} {Name F7 Range 5.0000} {Name F8 Range 4.9978} {Name R1 Range 4.9978} {Name R2 Range 4.0775} {Name R3 Range 2.9782} {Name R4 Range 2.6137} {Name R5 Range 2.6148} {Name R6 Range 2.9942} {Name R7 Range 3.3052} {Name R8 Range 2.5098}
*/

static int
handle_sen_sonar (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("Sonar");
	}
      else if (!strcmp (token, "Time"))
	{
	  GET_TIME;
	}
      else if (!strcmp (token, "Name"))
	{
	  GET_NAME (sonars, SW_SEN_SONAR_STAT);
	  EXPECT ("Range");
#define GET_REAL(VAR)					\
      nextptr = get_value(ptr, token);			\
      if (nextptr == ptr) return -1;			\
      if (1 != sscanf(token, "%lf", &d)) return -1;	\
      where->sw.data.VAR = d;				\
      count++;						\
      ptr = nextptr;
	  GET_REAL (sonar.range);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_SONAR_STAT);

  return count;
}

/*
  SEN {Time 5608.85} {Type RangeScanner} 

  {Name Scanner1} {Resolution 0.0174} {FOV 3.1415} {Range 11.3059,11.3014,11.3002,...}
*/

static int
handle_sen_rangescanner (char *msg)
{
  VAR_DECL;
  int number = 0;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("RangeScanner");
	}
      else if (!strcmp (token, "Name"))
	{
	  GET_NAME (rangescanners, SW_SEN_RANGESCANNER_STAT);
	}
      else if (!strcmp (token, "Time"))
	{
	  GET_TIME;
	}
      else if (!strcmp (token, "Resolution"))
	{
	  GET_REAL (rangescanner.resolution);
	}
      else if (!strcmp (token, "FOV"))
	{
	  GET_REAL (rangescanner.fov);
	}
      else if (!strcmp (token, "Range"))
	{
	  /*
	     We won't use the usual GET_REAL macro to get range values,
	     since there are a bunch separated by commas, and we'll also
	     want to keep track of the cumulative number.
	   */
	  while (1)
	    {
	      nextptr = get_value (ptr, token);
	      if (nextptr == ptr)
		{
		  if (0 == number)
		    return -1;	/* need at least one range value */
		  else
		    break;
		}
	      if (1 != sscanf (token, "%lf", &d))
		return -1;
	      if (number >= SW_SEN_RANGESCANNER_MAX)
		{
		  /* drop it */
		}
	      else
		{
		  where->sw.data.rangescanner.range[number] = d;
		  number++;
		}
	      ptr = nextptr;
	    }
	  /* all ok, so credit the count */
	  count++;
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  where->sw.data.rangescanner.number = number;
  TELL (SW_SEN_RANGESCANNER_STAT);

  return count;
}


/*
  SEN {Time 5608.85} {Type RangeImager} 

  {Name Scanner1} {Frames 10} {Resolution 0.0174} {FOV 3.1415} {Range 11.3059,11.3014,11.3002,...}
*/

static int
handle_sen_rangeimager (char *msg)
{
  VAR_DECL;
  int number;
  int i;

  VAR_INIT;

  //  printf( "usarsiminf: handle_sen_rangeimager: %s\n", msg );

  number = 0;
  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("RangeImager");
	}
      else if (!strcmp (token, "Frame"))
	{
	  GET_INTEGER (rangeimager.frame);
	}
      else if (!strcmp (token, "Frames"))
	{
	  GET_INTEGER (rangeimager.totalframes);
	}
      else if (!strcmp (token, "Name"))
	{
	  GET_NAME (rangeimagers, SW_SEN_RANGESCANNER_STAT);
	}
      else if (!strcmp (token, "Time"))
	{
	  GET_TIME;
	}
      else if (!strcmp (token, "Resolution"))
	{
	  GET_REAL (rangeimager.resolutionx);
	  GET_REAL (rangeimager.resolutiony);
	}
      else if (!strcmp (token, "FOV"))
	{
	  GET_REAL (rangeimager.fovx);
	  GET_REAL (rangeimager.fovy);
	}
      else if (!strcmp (token, "Range"))
	{
	  /*
	     We won't use the usual GET_REAL macro to get range values,
	     since there are a bunch separated by commas, and we'll also
	     want to keep track of the cumulative number.
	   */
	  while (1)
	    {
	      nextptr = get_value (ptr, token);
	      if (nextptr == ptr)
		{
		  if (0 == number)
		    return -1;	/* need at least one range value */
		  else
		    break;
		}
	      if (1 != sscanf (token, "%lf", &d))
		return -1;
	      if (number >= SW_SEN_RANGEIMAGER_MAX)
		{
		  /* drop it */
		  printf ("usarsiminf: error, dropping data\n");
		}
	      else
		{
		  where->sw.data.rangeimager.range[number] = d;
		  number++;
		}
	      ptr = nextptr;
	    }
	  /* all ok, so credit the count */
	  count++;
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  where->sw.data.rangeimager.numberperframe = number;
  TELL (SW_SEN_RANGEIMAGER_STAT);

  return count;
}

/*
  SEN {Type Touch}

  {Name Touch Touch False}
*/

static int
handle_sen_touch (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("Touch");
	}
      else if (!strcmp (token, "Name"))
	{
	  GET_NAME (touches, SW_SEN_TOUCH_STAT);
	  /* FIXME -- conf and geo are not handled inside usarsim */
	  where->did_conf = where->did_geo = 1;
	  EXPECT ("Touch");
	  /* expecting "True" or "False" */
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if (!strcmp (token, "False"))
	    where->sw.data.touch.touched = 0;
	  else if (!strcmp (token, "True"))
	    where->sw.data.touch.touched = 1;
	  else
	    return -1;
	  ptr = nextptr;
	  /* all ok, so credit the count */
	  count++;
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_TOUCH_STAT);

  return count;
}

/*
  SEN {Type CO2Sensor}

  {Name CO2} {Gas CO2} {Density 0.00}
*/
static int
handle_sen_co2sensor (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("CO2Sensor");
	}
      else if (!strcmp (token, "Name"))
	{
	  GET_NAME (co2sensors, SW_SEN_CO2_STAT);
	  /* FIXME -- conf and geo are not handled inside usarsim */
	  where->did_conf = where->did_geo = 1;
	}
      else if (!strcmp (token, "Gas"))
	{
	  EXPECT ("CO2");
	}
      else if (!strcmp (token, "Density"))
	{
	  GET_REAL (co2sensor.density);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_CO2_STAT);

  return count;
}

/*
  SEN {Time 5608.8500} {Type GroundTruth} {Name GroundTruth} {Location 12.61,-2.68,1.64} {Orientation 0.00,6.23,0.00}
*/

static int
handle_sen_groundtruth (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("GroundTruth");
	}
      else if (!strcmp (token, "Time"))
	{
	  GET_TIME;
	}
      else if (!strcmp (token, "Name"))
	{
	  GET_NAME (groundtruths, SW_SEN_GROUNDTRUTH_STAT);
	}
      else if (!strcmp (token, "Location"))
	{
	  GET_REAL (groundtruth.position.x);
	  GET_REAL (groundtruth.position.y);
	  GET_REAL (groundtruth.position.z);
	}
      else if (!strcmp (token, "Orientation"))
	{
	  GET_REAL (groundtruth.position.roll);
	  GET_REAL (groundtruth.position.pitch);
	  GET_REAL (groundtruth.position.yaw);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_GROUNDTRUTH_STAT);

  return count;
}

/*
  SEN {Type GPS}

  {Name GPS} {Latitude 39,8.0341,N} {Longitude 77,13.0001,W} {Fix 1} {Satellites 11}
*/

static int
handle_sen_gps (char *msg)
{
  VAR_DECL;
  double latdeg;
  double latmin;
  double londeg;
  double lonmin;
  int i;
  int south;
  int west;

  VAR_INIT;
  latdeg = londeg = latmin = lonmin = 0;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("GPS");
	}
      else if (!strcmp (token, "Name"))
	{
	  GET_NAME (gpses, SW_SEN_GPS_STAT);
	}
      else if (!strcmp (token, "Latitude"))
	{
	  /*
	     Because our GET_REAL macro prefixes the argument with 'data.',
	     we can't use it for local vars. Let's have more macro fun
	     with GET_LOCAL_REAL.
	   */
#define GET_LOCAL_REAL(VAR)				\
      nextptr = get_value(ptr, token);			\
      if (nextptr == ptr) return -1;			\
      if (1 != sscanf(token, "%lf", &d)) return -1;	\
      VAR = d;						\
      count++;						\
      ptr = nextptr;
	  GET_LOCAL_REAL (latdeg);
	  GET_LOCAL_REAL (latmin);
	  /* this is too specific to define convenience macro */
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if ('N' == token[0])
	    south = 0;
	  else if ('S' == token[0])
	    south = 1;
	  else
	    return -1;
	  count++;
	  ptr = nextptr;
	}
      else if (!strcmp (token, "Longitude"))
	{
	  GET_LOCAL_REAL (londeg);
	  GET_LOCAL_REAL (lonmin);
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if ('E' == token[0])
	    west = 0;
	  else if ('W' == token[0])
	    west = 1;
	  else
	    return -1;
	  count++;
	  ptr = nextptr;
	}
      else if (!strcmp (token, "Fix"))
	{
	  GET_INTEGER (gps.fix);
	}
      else if (!strcmp (token, "Satellites"))
	{
	  GET_INTEGER (gps.satellites);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  where->sw.data.gps.latitude = latdeg + (latmin / 60.0);
  if (south)
    where->sw.data.gps.latitude = -where->sw.data.gps.latitude;
  where->sw.data.gps.longitude = londeg + (lonmin / 60.0);
  if (west)
    where->sw.data.gps.longitude = -where->sw.data.gps.longitude;

  TELL (SW_SEN_GPS_STAT);

  return count;
}

/*
  SEN {Type INS}

  {Name INS} {Location 12.61,-2.68,1.84} {Orientation 0.00,0.00,0.00}
*/

static int
handle_sen_ins (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("INS");
	}
      else if (!strcmp (token, "Name"))
	{
	  GET_NAME (inses, SW_SEN_INS_STAT);
	}
      else if (!strcmp (token, "Location"))
	{
	  GET_REAL (ins.position.x);
	  GET_REAL (ins.position.y);
	  GET_REAL (ins.position.z);
	}
      else if (!strcmp (token, "Orientation"))
	{
	  GET_REAL (ins.position.roll);
	  GET_REAL (ins.position.pitch);
	  GET_REAL (ins.position.yaw);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_INS_STAT);

  return count;
}

/*
  SEN {Time 44.5467} {Type Odometry} 
  {Name Odometer} {Pose  0.0000,0.0000,0.0000}

  FIXME -- this needs to have full x y z r p w info
*/

static int
handle_sen_odometry (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("Odometry");
	}
      else if (!strcmp (token, "Time"))
	{
	  GET_TIME;
	}
      else if (!strcmp (token, "Name"))
	{
	  GET_NAME (odometers, SW_SEN_ODOMETER_STAT);
	}
      else if (!strcmp (token, "Pose"))
	{
	  GET_REAL (odometer.position.x);
	  GET_REAL (odometer.position.y);
	  GET_REAL (odometer.position.yaw);
	  where->sw.data.odometer.position.roll = 0;
	  where->sw.data.odometer.position.pitch = 0;
	  where->sw.data.odometer.position.z = 0;
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_ODOMETER_STAT);

  return count;
}

/*
  SEN {Time 5609.4467} {Type VictSensor} {Status NoVictims}

  FIXME -- the name is required to be VictSensor in the .uc code.
*/

static int
handle_sen_victim (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  /* FIXME per comment above */
  if (0 != usarsim_class_find (victims, "VictSensor", &where))
    return -1;
  sw_set_name (&where->sw, "VictSensor");

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("VictSensor");
	}
      else if (!strcmp (token, "Time"))
	{
	  GET_TIME;
	}
      else if (!strcmp (token, "Status"))
	{
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if (!strcmp (token, "Victims"))
	    where->sw.data.victim.victims = 1;
	  else if (!strcmp (token, "NoVictims"))
	    where->sw.data.victim.victims = 0;
	  else
	    return -1;
	  ptr = nextptr;
	  /* all ok, so credit the count */
	  count++;
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_VICTIM_STAT);

  return count;
}

/*
  SEN {Type Tachometer} {Name TachTest} {Vel 0.0000,0.0000,0.0000,0.0000} {Pos 6.2832,6.2832,6.2832,6.2832}
*/

static int
handle_sen_tachometer (char *msg)
{
  VAR_DECL;
  int vel_number = 0;
  int pos_number = 0;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("Tachometer");
	}
      else if (!strcmp (token, "Name"))
	{
	  GET_NAME (tachometers, SW_SEN_TACHOMETER_STAT);
	}
      else if (!strcmp (token, "Vel"))
	{
	  /*
	     We won't use the usual GET_REAL macro to get range values,
	     since there are a bunch separated by commas, and we'll also
	     want to keep track of the cumulative number.
	   */
	  while (1)
	    {
	      nextptr = get_value (ptr, token);
	      if (nextptr == ptr)
		{
		  if (0 == vel_number)
		    return -1;	/* need at least one speed value */
		  else
		    break;
		}
	      if (1 != sscanf (token, "%lf", &d))
		return -1;
	      if (vel_number >= SW_SEN_TACHOMETER_MAX)
		{
		  /* drop it */
		}
	      else
		{
		  where->sw.data.tachometer.speed[vel_number] = d;
		  vel_number++;
		}
	      ptr = nextptr;
	    }
	}
      else if (!strcmp (token, "Pos"))
	{
	  while (1)
	    {
	      nextptr = get_value (ptr, token);
	      if (nextptr == ptr)
		{
		  if (0 == pos_number)
		    return -1;	/* need at least one speed value */
		  else
		    break;
		}
	      if (1 != sscanf (token, "%lf", &d))
		return -1;
	      if (pos_number >= SW_SEN_TACHOMETER_MAX)
		{
		  /* drop it */
		}
	      else
		{
		  where->sw.data.tachometer.position[pos_number] = d;
		  pos_number++;
		}
	      ptr = nextptr;
	    }
	  /* credit the count here, after the last expected data type 'Pos' */
	  count++;
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  where->sw.data.tachometer.number =
    pos_number > vel_number ? pos_number : vel_number;
  TELL (SW_SEN_TACHOMETER_STAT);

  return count;
}

/*
  SEN {Time 390.5659} {Type Acoustic} {Name Test}
  {None}
  or
  {Far}
  or
  {Direction 1,0,0} {Volume 0.5} {Duration 1.4} {Delay 0.1}
*/

static int
handle_sen_acoustic (char *msg)
{
  VAR_DECL;
  double x, y, z;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("Acoustic");
	}
      else if (!strcmp (token, "Time"))
	{
	  GET_TIME;
	}
      else if (!strcmp (token, "Name"))
	{
	  GET_NAME (acoustics, SW_SEN_ACOUSTIC_STAT);
	}
      else if (!strcmp (token, "Direction"))
	{
	  GET_LOCAL_REAL (x);
	  GET_LOCAL_REAL (y);
	  GET_LOCAL_REAL (z);
	  where->sw.data.acoustic.azimuth = atan2 (y, x);
	  where->sw.data.acoustic.altitude = atan2 (z, sqrt (x * x + y * y));
	}
      else if (!strcmp (token, "Volume"))
	{
	  GET_REAL (acoustic.volume);
	}
      else if (!strcmp (token, "Duration"))
	{
	  GET_REAL (acoustic.duration);
	  /* ignore "Delay" since that's not obtainable by a real sensor */
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_ACOUSTIC_STAT);

  return count;
}

static int
handle_sen (char *msg)
{
  char token[MAX_TOKEN_LEN];
  char *ptr = msg;
  char *nextptr;
  int count = 0;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      /* look for {Type <name>}, and pass the whole msg to the sensor */
      if (!strcmp (token, "Type"))
	{
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if (!strcmp (token, "Sonar"))
	    {
	      return handle_sen_sonar (msg);
	    }
	  else if (!strcmp (token, "RangeScanner"))
	    {
	      return handle_sen_rangescanner (msg);
	    }
	  else if (!strcmp (token, "RangeImager"))
	    {
	      return handle_sen_rangeimager (msg);
	    }
	  else if (!strcmp (token, "Encoder"))
	    {
	      return handle_sen_encoder (msg);
	    }
	  else if (!strcmp (token, "Touch"))
	    {
	      return handle_sen_touch (msg);
	    }
	  else if (!strcmp (token, "CO2Sensor"))
	    {
	      return handle_sen_co2sensor (msg);
	    }
	  else if (!strcmp (token, "GroundTruth"))
	    {
	      return handle_sen_groundtruth (msg);
	    }
	  else if (!strcmp (token, "GPS"))
	    {
	      return handle_sen_gps (msg);
	    }
	  else if (!strcmp (token, "INS"))
	    {
	      return handle_sen_ins (msg);
	    }
	  else if (!strcmp (token, "Odometry"))
	    {
	      return handle_sen_odometry (msg);
	    }
	  else if (!strcmp (token, "VictSensor"))
	    {
	      return handle_sen_victim (msg);
	    }
	  else if (!strcmp (token, "Tachometer"))
	    {
	      return handle_sen_tachometer (msg);
	    }
	  else if (!strcmp (token, "Acoustic"))
	    {
	      return handle_sen_acoustic (msg);
	    }
	  else if (!strcmp (token, "Camera"))
	    {
	      return count;
	    }
	  else
	    {
	      errprintf ("Unknown sensor type %s\n", token);
	      /* skip it and keep going */
	    }
	}
      /* else something else to be handled by sensor, probably {Time #} */
    }

  return count;
}

/*
  EFF {Type Gripper} {Name Gripper1} {Open | Closed}
*/

static int
handle_eff_gripper (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("Gripper");
	}
      else if (!strcmp (token, "Name"))
	{
	  GET_NAME (grippers, SW_EFF_GRIPPER_STAT);
	}
      else if (!strcmp (token, "Open"))
	{
	  where->sw.data.gripper.open = 1;
	  count++;
	}
      else if (!strcmp (token, "Closed"))
	{
	  where->sw.data.gripper.open = 0;
	  count++;
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_EFF_GRIPPER_STAT);

  return count;
}

static int
handle_eff (char *msg)
{
  char token[MAX_TOKEN_LEN];
  char *ptr = msg;
  char *nextptr;
  int count = 0;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      /* look for {Type <name>}, and pass the whole msg to the effector */
      if (!strcmp (token, "Type"))
	{
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if (!strcmp (token, "Gripper"))
	    {
	      return handle_eff_gripper (msg);
	    }
	  else
	    {
	      errprintf ("Unknown effector type %s\n", token);
	      /* skip it and keep going */
	    }
	}
      /* else something else to be handled by effector */
    }

  return count;
}

/*
  STA {Type GroundVehicle}

  {Time 5609.45} {FrontSteer 0.0000} {RearSteer 0.0000} {LightToggle False} {LightIntensity 0} {Battery 1199} {View -1}

  {Time 48.22} {FrontSteer 0.0000} {RearSteer 0.0000} {FRFlipper -0.0006} {FLFlipper -0.0005} {RRFlipper 0.0005} {RLFlipper 0.0006} {LightToggle False} {LightIntensity 0} {Battery 1196} {View -1}

  FIXME -- there's no name here, consequently we can only have one
  vehicle. The name of the vehicle is irrelevant.
*/

static int
handle_sta_groundvehicle (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  /* since we only have one robot, we just set it explicitly here
     without going through 'usarsim_class_find' and needing a name */
  where = &robot;
  where->sw.type = SW_ROBOT_GROUNDVEHICLE;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("GroundVehicle");
	}
      else if (!strcmp (token, "Time"))
	{
	  GET_TIME;
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_ROBOT_STAT);

  return count;
}

/*
  STA {Type BaseMachine}

  {Time 5609.45}

  FIXME -- there's no name here.
*/
static int
handle_sta_basemachine (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  where = &robot;
  where->sw.type = SW_ROBOT_FIXED;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("BaseMachine");
	}
      else if (!strcmp (token, "Time"))
	{
	  GET_TIME;
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_DEVICE_STAT);
  /*
     FIXME-- there is some weirdness with SW_ROBOT_FIXED,GROUNDVEHICLE,
     devices, basemachines, etc. Rather than using a fixed robot, which
     suggests an industrial robot arm, try something like
     SW_ENVIRONMENT, which could be a factory, or a collapsed building.
   */

  return count;
}

/*
  STA {Type staticplatform}

  {Time 5609.45}

  FIXME -- there's no name here.
*/
static int
handle_sta_staticplatform (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  where = &robot;
  where->sw.type = SW_ROBOT_FIXED;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("StaticPlatform");
	}
      else if (!strcmp (token, "Time"))
	{
	  GET_TIME;
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_DEVICE_STAT);
  return count;
}


static int
handle_sta (char *msg)
{
  char token[MAX_TOKEN_LEN];
  char *ptr = msg;
  char *nextptr;
  int count = 0;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      /* look for {Type <name>}, and pass the whole msg to the STA */
      if (!strcmp (token, "Type"))
	{
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if (!strcmp (token, "GroundVehicle"))
	    {
	      return handle_sta_groundvehicle (msg);
	    }
	  else if (!strcmp (token, "BaseMachine"))
	    {
	      return handle_sta_basemachine (msg);
	    }
	  else if (!strcmp (token, "StaticPlatform"))
	    {
	      return handle_sta_staticplatform (msg);
	    }
	  else
	    {
	      errprintf ("Unknown STA type %s\n", token);
	      /* skip it and keep going */
	    }
	}
      /* else something else, probably {Time #} */
    }

  return count;
}

/*
  MISSTA {Time 5609.45} {Name CameraPanTilt} {Link 1} {Value 0.0000} {Torque -20.00} {Link 2} {Value 0.0000} {Torque -20.00}

  MISSTA {Time 48.22} {Name TeleMaxArm} {Link 1} {Value 0.0000} {Torque -300.00} {Link 2} {Value -0.0000} {Torque -300.00} {Link 3} {Value 0.0002} {Torque -300.00} {Link 4} {Value 0.0000} {Torque -300.00} {Link 5} {Value 0.0000} {Torque -300.00} {Link 6} {Value 0.0000} {Torque -300.00} {Link 7} {Value 0.0000} {Torque -300.00} {Link 8} {Value 0.0000} {Torque -300.00} {Link 9} {Value 0.0000} {Torque -300.00} {Link 10} {Value 0.0000} {Torque -300.00} {Link 11} {Value 0.0000} {Torque -300.00}
*/

static int
handle_missta (char *msg)
{
  VAR_DECL;
  int i;
  int linkindex = 0;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Time"))
	{
	  GET_TIME;
	}
      else if (!strcmp (token, "Name"))
	{
	  GET_NAME (misstas, SW_ACT_STAT);
	  where->sw.data.mispkg.number = 0;
	}
      else if (!strcmp (token, "Link"))
	{
	  /* expecting number */
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if (1 != sscanf (token, "%i", &i))
	    return -1;
	  if (i < 1)
	    {
	      errprintf ("invalid link number for %s: %d\n", where->sw.name,
			 i);
	      i = 1;
	    }
	  else if (i > SW_ACT_LINK_MAX)
	    {
	      errprintf ("invalid link number for %s: %d\n", where->sw.name,
			 i);
	      i = SW_ACT_LINK_MAX;
	    }
	  linkindex = i - 1;
	  if (i > where->sw.data.mispkg.number)
	    where->sw.data.mispkg.number = i;
	  count++;
	  ptr = nextptr;
	}
      else if (!strcmp (token, "Value"))
	{
	  GET_REAL (mispkg.link[linkindex].position);
	}
      else if (!strcmp (token, "Torque"))
	{
	  GET_REAL (mispkg.link[linkindex].torque);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_ACT_STAT);

  return count;
}

/*
  RES {Type Effector} {Name Loading} {Status OK}
*/
static int
handle_res (char *msg)
{
  static int didit = 0;

  if (!didit)
    {
      errprintf ("not handling %s\n", msg);
      didit = 1;
    }
  return 0;
}

/*
  CONF {Type Encoder} {Name ECLeft} {Resolution 0.0174}
*/
static int
handle_conf_encoder (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("Encoder");
	}
      else if (!strcmp (token, "Name"))
	{
#define CONF_NAME(TYPE,OP)			\
      GET_NAME(TYPE, OP);			\
      where->did_conf = 1;
	  CONF_NAME (encoders, SW_SEN_ENCODER_SET);
	}
      else if (!strcmp (token, "Resolution"))
	{
	  GET_REAL (encoder.resolution);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_ENCODER_SET);

  return count;
}

/*
  CONF {Type Sonar} {Name R8} {MaxRange 5.0000} {MinRange 0.1000} {BeamAngle 0.3491}
*/

static int
handle_conf_sonar (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("Sonar");
	}
      else if (!strcmp (token, "Name"))
	{
	  CONF_NAME (sonars, SW_SEN_SONAR_SET);
	}
      else if (!strcmp (token, "MaxRange"))
	{
	  GET_REAL (sonar.maxrange);
	}
      else if (!strcmp (token, "MinRange"))
	{
	  GET_REAL (sonar.minrange);
	}
      else if (!strcmp (token, "BeamAngle"))
	{
	  GET_REAL (sonar.beamangle);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_SONAR_SET);

  return count;
}

/*
  CONF {Type RangeImager} {Name Scanner1} {MaxRange 20.0000} {MinRange 0.1000} {Resolution 0.0174} {Fov 3.1415} {Paning True} {Tilting False}
*/
static int
handle_conf_rangeimager (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  //  printf( "usarsiminf: in handle_conf for rangeimager: %s\n", msg );
  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("RangeImager");
	}
      else if (!strcmp (token, "Name"))
	{
	  CONF_NAME (rangeimagers, SW_SEN_RANGEIMAGER_SET);
	}
      else if (!strcmp (token, "MaxRange"))
	{
	  GET_REAL (rangeimager.maxrange);
	}
      else if (!strcmp (token, "MinRange"))
	{
	  GET_REAL (rangeimager.minrange);
	}
      else if (!strcmp (token, "Resolution"))
	{
	  GET_REAL (rangeimager.resolutionx);
	  GET_REAL (rangeimager.resolutiony);
	}
      else if (!strcmp (token, "Fov"))
	{
	  GET_REAL (rangeimager.fovx);
	  GET_REAL (rangeimager.fovy);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  //  printf( "usarsiminf: going to tell in handle_conf for rangeimager\n");
  TELL (SW_SEN_RANGEIMAGER_SET);

  return count;
}


/*
  CONF {Type RangeScanner} {Name Scanner1} {MaxRange 20.0000} {MinRange 0.1000} {Resolution 0.0174} {Fov 3.1415} {Paning True} {Tilting False}
*/
static int
handle_conf_rangescanner (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("RangeScanner");
	}
      else if (!strcmp (token, "Name"))
	{
	  CONF_NAME (rangescanners, SW_SEN_RANGESCANNER_SET);
	}
      else if (!strcmp (token, "MaxRange"))
	{
	  GET_REAL (rangescanner.maxrange);
	}
      else if (!strcmp (token, "MinRange"))
	{
	  GET_REAL (rangescanner.minrange);
	}
      else if (!strcmp (token, "Resolution"))
	{
	  GET_REAL (rangescanner.resolution);
	}
      else if (!strcmp (token, "Fov"))
	{
	  GET_REAL (rangescanner.fov);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_RANGESCANNER_SET);

  return count;
}

static int
handle_conf_touch (char *msg)
{
  static int didit = 0;

  if (!didit)
    {
      errprintf ("not handling %s\n", msg);
      didit = 1;
    }
  return 0;
}

static int
handle_conf_co2sensor (char *msg)
{
  static int didit = 0;

  if (!didit)
    {
      errprintf ("not handling %s\n", msg);
      didit = 1;
    }
  return 0;
}

/*
  CONF {Type GroundTruth} {Name GroundTruth} {ScanInterval 0.05}
*/
static int
handle_conf_groundtruth (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("GroundTruth");
	}
      else if (!strcmp (token, "Name"))
	{
	  CONF_NAME (groundtruths, SW_SEN_GROUNDTRUTH_SET);
	}
      else if (!strcmp (token, "ScanInterval"))
	{
	  GET_REAL (groundtruth.period);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_GROUNDTRUTH_SET);

  return count;
}

/*
  CONF {Type GPS} {Name GPS} {ScanInterval 0.20}
*/
static int
handle_conf_gps (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("GPS");
	}
      else if (!strcmp (token, "Name"))
	{
	  CONF_NAME (gpses, SW_SEN_GPS_SET);
	}
      else if (!strcmp (token, "ScanInterval"))
	{
	  GET_REAL (gps.period);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_GPS_SET);

  return count;
}

/*
  CONF {Type INS} {Name INS} {ScanInterval 0.20}
*/
static int
handle_conf_ins (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("INS");
	}
      else if (!strcmp (token, "Name"))
	{
	  CONF_NAME (inses, SW_SEN_INS_SET);
	}
      else if (!strcmp (token, "ScanInterval"))
	{
	  GET_REAL (ins.period);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_INS_SET);

  return count;
}

/*
  CONF {Type Odometry} {Name Odometry} {ScanInterval 0.2000} {EncoderResolution 0.0099}
*/
static int
handle_conf_odometry (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("Odometry");
	}
      else if (!strcmp (token, "Name"))
	{
	  CONF_NAME (odometers, SW_SEN_ODOMETER_SET);
	}
      else if (!strcmp (token, "ScanInterval"))
	{
	  GET_REAL (odometer.period);
	}
      else if (!strcmp (token, "EncoderResolution"))
	{
	  GET_REAL (odometer.resolution);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_ODOMETER_SET);

  return count;
}

/*
  CONF {Type Tachometer} {Name TachTest} {ScanInterval 0.2000}
*/
static int
handle_conf_tachometer (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("Tachometer");
	}
      else if (!strcmp (token, "Name"))
	{
	  CONF_NAME (tachometers, SW_SEN_TACHOMETER_SET);
	}
      else if (!strcmp (token, "ScanInterval"))
	{
	  GET_REAL (odometer.period);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_TACHOMETER_SET);

  return count;
}

/*
  CONF {Type Acoustic} {Name Test}
*/
static int
handle_conf_acoustic (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("Acoustic");
	}
      else if (!strcmp (token, "Name"))
	{
	  CONF_NAME (acoustics, SW_SEN_ACOUSTIC_SET);
	}
      else if (!strcmp (token, "ScanInterval"))
	{
	  GET_REAL (odometer.period);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_ACOUSTIC_SET);

  return count;
}

/*
  CONF {Type VictSensor} {Name VictSensor} {MaxRange 6.0000} {HorizontalFOV 0.6981} {VerticalFOV 0.6981}
*/
static int
handle_conf_victim (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("VictSensor");
	}
      else if (!strcmp (token, "Name"))
	{
	  CONF_NAME (victims, SW_SEN_VICTIM_SET);
	}
      else if (!strcmp (token, "MaxRange"))
	{
	  GET_REAL (victim.maxrange);
	}
      else if (!strcmp (token, "HorizontalFOV"))
	{
	  GET_REAL (victim.hfov);
	}
      else if (!strcmp (token, "VerticalFOV"))
	{
	  GET_REAL (victim.vfov);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_VICTIM_SET);

  return count;
}

/*
  CONF {Type Gripper} {Name Gripper1}
*/
static int
handle_conf_gripper (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("Gripper");
	}
      else if (!strcmp (token, "Name"))
	{
	  CONF_NAME (grippers, SW_SEN_INS_SET);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_EFF_GRIPPER_SET);

  return count;
}

/*
  CONF {Type MisPkg} {Name TeleMaxArm} {Link 1} {JointType Revolute} {MaxSpeed 0.17} {MaxTorque 300.00} {MinRange 1.00} {MaxRange 0.00} ...
*/
static int
handle_conf_mispkg (char *msg)
{
  VAR_DECL;
  int i;
  int linkindex;

  VAR_INIT;
  linkindex = 0;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("MisPkg");
	}
      else if (!strcmp (token, "Name"))
	{
	  CONF_NAME (misstas, SW_ACT_SET);
	  where->sw.data.mispkg.number = 0;
	}
      else if (!strcmp (token, "Link"))
	{
	  /* expecting number */
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if (1 != sscanf (token, "%i", &i))
	    return -1;
	  if (i < 1)
	    {
	      errprintf ("invalid link number for %s: %d\n", where->sw.name,
			 i);
	      i = 1;
	    }
	  else if (i > SW_ACT_LINK_MAX)
	    {
	      errprintf ("invalid link number for %s: %d\n", where->sw.name,
			 i);
	      i = SW_ACT_LINK_MAX;
	    }
	  linkindex = i - 1;
	  ptr = nextptr;
	  if (i > where->sw.data.mispkg.number)
	    where->sw.data.mispkg.number = i;
	  count++;
	}
      else if (!strcmp (token, "JointType"))
	{
	  /* expecting "Revolute" or "Prismatic" */
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if (!strcmp (token, "Prismatic"))
	    {
	      where->sw.data.mispkg.link[linkindex].type = SW_LINK_PRISMATIC;
	    }
	  else if (!strcmp (token, "Revolute"))
	    {
	      where->sw.data.mispkg.link[linkindex].type = SW_LINK_REVOLUTE;
	    }
	  else if (!strcmp (token, "Scissor"))
	    {
	      where->sw.data.mispkg.link[linkindex].type = SW_LINK_SCISSOR;
	    }
	  else
	    {
	      errprintf ("bad value for %s JointType: %s\n", where->sw.name,
			 token);
	      where->sw.data.mispkg.link[linkindex].type = SW_NONE;
	    }
	  ptr = nextptr;
	  /* all ok, so credit the count */
	  count++;
	}
      else if (!strcmp (token, "MaxSpeed"))
	{
	  GET_REAL (mispkg.link[linkindex].maxspeed);
	}
      else if (!strcmp (token, "MaxTorque"))
	{
	  GET_REAL (mispkg.link[linkindex].maxtorque);
	}
      else if (!strcmp (token, "MinRange"))
	{
	  GET_REAL (mispkg.link[linkindex].minrange);
	}
      else if (!strcmp (token, "MaxRange"))
	{
	  GET_REAL (mispkg.link[linkindex].maxrange);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  /*
     Some mission packages don't report any link information. For these,
     set the number of links to be the max.
   */

  if (0 == where->sw.data.mispkg.number)
    {
      where->sw.data.mispkg.number = SW_ACT_LINK_MAX;
    }

  /*
     Only do the 'tell' operation if we got data -- this will ignore
     empty "GEO {Type MisPkg}" strings that may be sent out after a
     "GETGEO {Type MisPkg}" if there are no mission packages.

     FIXME -- should do this for all messages
   */
  if (count > 0)
    {
      TELL (SW_ACT_SET);
    }

  return count;
}

/*
  CONF {Type GroundVehicle}

  {Name P2AT} {SteeringType SkidSteered} {Mass 14.0000} {MaxSpeed 5.3850} {MaxTorque 60.0000} {MaxFrontSteer 0.0000} {MaxRearSteer 0.0000}
*/
static int
handle_conf_groundvehicle (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  where = &robot;		/* one robot, no need to find it */
  where->sw.type = SW_ROBOT_GROUNDVEHICLE;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("GroundVehicle");
	  where->did_conf = 1;
	  ptr = nextptr;
	}
      else if (!strcmp (token, "Name"))
	{
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  strncpy (where->sw.name, token, sizeof (where->sw.name));
	  where->sw.name[sizeof (where->sw.name) - 1] = 0;
	  ptr = nextptr;
	}
      else if (!strcmp (token, "SteeringType"))
	{
	  /* expecting "SkidSteered", "AckermanSteered" or "OmniDrive" */
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if (!strcmp (token, "SkidSteered"))
	    {
	      where->sw.data.groundvehicle.steertype = SW_STEER_SKID;
	    }
	  else if (!strcmp (token, "AckermanSteered"))
	    {
	      where->sw.data.groundvehicle.steertype = SW_STEER_ACKERMAN;
	    }
	  else if (!strcmp (token, "OmniDrive"))
	    {
	      where->sw.data.groundvehicle.steertype = SW_STEER_OMNI;
	    }
	  else
	    {
	      where->sw.data.groundvehicle.steertype = SW_STEER_UNKNOWN;
	      errprintf ("bad value for SteeringType: %s\n", token);
	    }
	  ptr = nextptr;
	  count++;
	}
      else if (!strcmp (token, "Mass"))
	{
	  GET_REAL (groundvehicle.mass);
	}
      else if (!strcmp (token, "MaxSpeed"))
	{
	  GET_REAL (groundvehicle.max_speed);
	}
      else if (!strcmp (token, "MaxTorque"))
	{
	  GET_REAL (groundvehicle.max_torque);
	}
      else if (!strcmp (token, "MaxFrontSteer"))
	{
	  GET_REAL (groundvehicle.max_steer_angle);
	}
      else
	{
	  /* skip MaxRearSteer, other unknown entries  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_ROBOT_SET);

  return count;
}

/*
  GEO {Type StaticPlatform} {Name FactoryControlBot} {Dimensions 0.0000,0.0000,0.0000}
*/
static int
handle_geo_staticplatform (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  where = &robot;
  where->sw.type = SW_ROBOT_FIXED;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("StaticPlatform");
	  where->did_geo = 1;
	  ptr = nextptr;
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_ROBOT_SET);

  return count;
}


/*
  CONF {Type BaseMachine} {Name FactoryControlBot}
*/
static int
handle_conf_basemachine (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  where = &robot;		/* one robot, no need to find it */
  where->sw.type = SW_ROBOT_FIXED;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("BaseMachine");
	  where->did_conf = 1;
	  ptr = nextptr;
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_ROBOT_SET);

  return count;
}

/*
  CONF {Type StaticPlatform} {Name FactoryControlBot}
*/
static int
handle_conf_staticplatform (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  where = &robot;		/* one robot, no need to find it */
  where->sw.type = SW_ROBOT_FIXED;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("StaticPlatform");
	  where->did_conf = 1;
	  ptr = nextptr;
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_ROBOT_SET);

  return count;
}


static int
handle_conf (char *msg)
{
  char token[MAX_TOKEN_LEN];
  char *ptr = msg;
  char *nextptr;
  int count = 0;

  waiting_for_conf = 0;
  //  printf( "usarsiminf: (0) waiting_for_conf reset w/ msg %s\n", msg );

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      /* look for {Type <name>}, and pass the whole msg to the sensor */
      if (!strcmp (token, "Type"))
	{
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if (!strcmp (token, "Sonar"))
	    {
	      return handle_conf_sonar (msg);
	    }
	  else if (!strcmp (token, "RangeScanner"))
	    {
	      return handle_conf_rangescanner (msg);
	    }
	  else if (!strcmp (token, "RangeImager"))
	    {
	      return handle_conf_rangeimager (msg);
	    }
	  else if (!strcmp (token, "Encoder"))
	    {
	      return handle_conf_encoder (msg);
	    }
	  else if (!strcmp (token, "Touch"))
	    {
	      return handle_conf_touch (msg);
	    }
	  else if (!strcmp (token, "CO2Sensor"))
	    {
	      return handle_conf_co2sensor (msg);
	    }
	  else if (!strcmp (token, "GroundTruth"))
	    {
	      return handle_conf_groundtruth (msg);
	    }
	  else if (!strcmp (token, "GPS"))
	    {
	      return handle_conf_gps (msg);
	    }
	  else if (!strcmp (token, "INS"))
	    {
	      return handle_conf_ins (msg);
	    }
	  else if (!strcmp (token, "Odometry"))
	    {
	      return handle_conf_odometry (msg);
	    }
	  else if (!strcmp (token, "Tachometer"))
	    {
	      return handle_conf_tachometer (msg);
	    }
	  else if (!strcmp (token, "Acoustic"))
	    {
	      return handle_conf_acoustic (msg);
	    }
	  else if (!strcmp (token, "VictSensor"))
	    {
	      return handle_conf_victim (msg);
	    }
	  else if (!strcmp (token, "Gripper"))
	    {
	      return handle_conf_gripper (msg);
	    }
	  else if (!strcmp (token, "MisPkg"))
	    {
	      return handle_conf_mispkg (msg);
	    }
	  else if (!strcmp (token, "GroundVehicle"))
	    {
	      return handle_conf_groundvehicle (msg);
	    }
	  else if (!strcmp (token, "BaseMachine"))
	    {
	      return handle_conf_basemachine (msg);
	    }
	  else if (!strcmp (token, "StaticPlatform"))
	    {
	      return handle_conf_staticplatform (msg);
	    }
	  else
	    {
	      errprintf ("Unknown conf type %s\n", token);
	      /* skip it and keep going */
	    }
	}
    }

  return count;
}

/*
  GEO {Type Encoder} {Name ECRight Location 0.0000,0.0000,0.0000 Orientation 0.0000,0.0000,1.5707 Mount RightFWheel} < {Name ...} >
*/
static int
handle_geo_encoder (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("Encoder");
	}
      else if (!strcmp (token, "Name"))
	{
#define GEO_NAME(TYPE,OP)			\
      GET_NAME(TYPE,OP)				\
      where->did_geo = 1;
	  GEO_NAME (encoders, SW_SEN_ENCODER_SET);
	  EXPECT ("Location");
	  GET_REAL (encoder.mount.x);
	  GET_REAL (encoder.mount.y);
	  GET_REAL (encoder.mount.z);
	  EXPECT ("Orientation");
	  GET_REAL (encoder.mount.roll);
	  GET_REAL (encoder.mount.pitch);
	  GET_REAL (encoder.mount.yaw);
	  /* ignore MOUNT RightFWheel */
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_ENCODER_SET);

  return count;
}

/*
  GEO {Type Sonar} {Name F2 Location 0.1850,-0.1150,0.0000 Orientation 0.0000,0.0000,-0.8727 Mount HARD}

  Why didn't the original USARSim designers use the same tag convention
  as for the CONF message?
*/

static int
handle_geo_sonar (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("Sonar");
	}
      else if (!strcmp (token, "Name"))
	{
	  GEO_NAME (sonars, SW_SEN_SONAR_SET);
	  EXPECT ("Location");
	  GET_REAL (sonar.mount.x);
	  GET_REAL (sonar.mount.y);
	  GET_REAL (sonar.mount.z);
	  EXPECT ("Orientation");
	  GET_REAL (sonar.mount.roll);
	  GET_REAL (sonar.mount.pitch);
	  GET_REAL (sonar.mount.yaw);
	  /* ignore MOUNT HARD */
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_SONAR_SET);

  return count;
}

/*
  GEO {Type RangeImager} {Name Scanner1 Location 0.1439,0.0000,-0.0920 Orientation 0.0000,0.0000,0.0000 Mount HARD}
*/

static int
handle_geo_rangeimager (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  //  printf( "usarsiminf: in handle_geo for range imager: %s\n", msg );
  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("RangeImager");
	}
      else if (!strcmp (token, "Name"))
	{
	  GEO_NAME (rangeimagers, SW_SEN_RANGEIMAGER_SET);
	  EXPECT ("Location");
	  GET_REAL (rangeimager.mount.x);
	  GET_REAL (rangeimager.mount.y);
	  GET_REAL (rangeimager.mount.z);
	  /*
	     printf( "location %f %f %f\n", where->sw.data.rangescanner.mount.x,
	     where->sw.data.rangescanner.mount.y,
	     where->sw.data.rangescanner.mount.z);
	   */
	  EXPECT ("Orientation");
	  GET_REAL (rangeimager.mount.roll);
	  GET_REAL (rangeimager.mount.pitch);
	  GET_REAL (rangeimager.mount.yaw);

	  where->sw.data.rangeimager.mount.roll = M_PI;
	  /*
	     printf( "orientation %f %f %f\n", where->sw.data.rangescanner.mount.roll,
	     where->sw.data.rangescanner.mount.pitch,
	     where->sw.data.rangescanner.mount.yaw);
	   */
	  /* ignore MOUNT HARD */
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_RANGEIMAGER_SET);

  return count;
}


/*
  GEO {Type RangeScanner} {Name Scanner1 Location 0.1439,0.0000,-0.0920 Orientation 0.0000,0.0000,0.0000 Mount HARD}
*/

static int
handle_geo_rangescanner (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("RangeScanner");
	}
      else if (!strcmp (token, "Name"))
	{
	  GEO_NAME (rangescanners, SW_SEN_RANGESCANNER_SET);
	  EXPECT ("Location");
	  GET_REAL (rangescanner.mount.x);
	  GET_REAL (rangescanner.mount.y);
	  GET_REAL (rangescanner.mount.z);
	  /*
	     printf( "location %f %f %f\n", where->sw.data.rangescanner.mount.x,
	     where->sw.data.rangescanner.mount.y,
	     where->sw.data.rangescanner.mount.z);
	   */
	  EXPECT ("Orientation");
	  GET_REAL (rangescanner.mount.roll);
	  GET_REAL (rangescanner.mount.pitch);
	  GET_REAL (rangescanner.mount.yaw);

	  where->sw.data.rangescanner.mount.roll = M_PI;
	  /*
	     printf( "orientation %f %f %f\n", where->sw.data.rangescanner.mount.roll,
	     where->sw.data.rangescanner.mount.pitch,
	     where->sw.data.rangescanner.mount.yaw);
	   */
	  /* ignore MOUNT HARD */
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_RANGESCANNER_SET);

  return count;
}

static int
handle_geo_touch (char *msg)
{
  static int didit = 0;

  if (!didit)
    {
      errprintf ("not handling %s\n", msg);
      didit = 1;
    }
  return 0;
}

static int
handle_geo_co2sensor (char *msg)
{
  static int didit = 0;

  if (!didit)
    {
      errprintf ("not handling %s\n", msg);
      didit = 1;
    }
  return 0;
}

/*
  GEO {Type GroundTruth} {Name GroundTruth Location 0.0000,0.0000,0.0000 Orientation 0.0000,0.0000,0.0000 Mount HARD}
*/
static int
handle_geo_groundtruth (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("GroundTruth");
	}
      else if (!strcmp (token, "Name"))
	{
	  GEO_NAME (groundtruths, SW_SEN_GROUNDTRUTH_SET);
	  EXPECT ("Location");
	  GET_REAL (groundtruth.mount.x);
	  GET_REAL (groundtruth.mount.y);
	  GET_REAL (groundtruth.mount.z);
	  EXPECT ("Orientation");
	  GET_REAL (groundtruth.mount.roll);
	  GET_REAL (groundtruth.mount.pitch);
	  GET_REAL (groundtruth.mount.yaw);
	  /* ignore MOUNT HARD */
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_GROUNDTRUTH_SET);

  return count;
}

/*
  GEO {Type GPS} {Name GPS Location 0.0000,0.0000,0.0000 Orientation 0.0000,-0.0000,0.0000 Mount HARD}
*/
static int
handle_geo_gps (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("GPS");
	}
      else if (!strcmp (token, "Name"))
	{
	  GEO_NAME (gpses, SW_SEN_GPS_SET);
	  EXPECT ("Location");
	  GET_REAL (gps.mount.x);
	  GET_REAL (gps.mount.y);
	  GET_REAL (gps.mount.z);
	  EXPECT ("Orientation");
	  GET_REAL (gps.mount.roll);
	  GET_REAL (gps.mount.pitch);
	  GET_REAL (gps.mount.yaw);
	  /* ignore MOUNT HARD */
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_GPS_SET);

  return count;
}

/*
  GEO {Type INS} {Name INS Location 0.0000,0.0000,0.0000 Orientation 0.0000,-0.0000,0.0000 Mount HARD}
*/
static int
handle_geo_ins (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("INS");
	}
      else if (!strcmp (token, "Name"))
	{
	  GEO_NAME (inses, SW_SEN_INS_SET);
	  EXPECT ("Location");
	  GET_REAL (ins.mount.x);
	  GET_REAL (ins.mount.y);
	  GET_REAL (ins.mount.z);
	  EXPECT ("Orientation");
	  GET_REAL (ins.mount.roll);
	  GET_REAL (ins.mount.pitch);
	  GET_REAL (ins.mount.yaw);
	  /* ignore MOUNT HARD */
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_INS_SET);

  return count;
}

/*
  GEO {Type Odometry} {Name Odometry 
  Location 0.0000,0.0000,0.0000 Orientation 0.0000,-0.0000,0.0000 Mount HARD}
*/
static int
handle_geo_odometry (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("Odometry");
	}
      else if (!strcmp (token, "Name"))
	{
	  GEO_NAME (odometers, SW_SEN_ODOMETER_SET);
	  EXPECT ("Location");
	  GET_REAL (odometer.mount.x);
	  GET_REAL (odometer.mount.y);
	  GET_REAL (odometer.mount.z);
	  EXPECT ("Orientation");
	  GET_REAL (odometer.mount.roll);
	  GET_REAL (odometer.mount.pitch);
	  GET_REAL (odometer.mount.yaw);
	  /* ignore MOUNT HARD */
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_ODOMETER_SET);

  return count;
}

/*
  GEO {Type Tachometer} {Name TachTest Location 0.0000,0.0000,0.0000 Orientation 0.0000,0.0000,0.0000 Mount USARbot.P3AT}
*/
static int
handle_geo_tachometer (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("Tachometer");
	}
      else if (!strcmp (token, "Name"))
	{
	  GEO_NAME (tachometers, SW_SEN_TACHOMETER_SET);
	  EXPECT ("Location");
	  GET_REAL (tachometer.mount.x);
	  GET_REAL (tachometer.mount.y);
	  GET_REAL (tachometer.mount.z);
	  EXPECT ("Orientation");
	  GET_REAL (tachometer.mount.roll);
	  GET_REAL (tachometer.mount.pitch);
	  GET_REAL (tachometer.mount.yaw);
	  /* ignore MOUNT <name> */
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_TACHOMETER_SET);

  return count;
}

/*
  GEO {Type Acoustic} {Name Test Location 0.0000,0.0000,0.0000 Orientation 0.0000,0.0000,0.0000 Mount USARbot.P3AT}
*/
static int
handle_geo_acoustic (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("Acoustic");
	}
      else if (!strcmp (token, "Name"))
	{
	  GEO_NAME (acoustics, SW_SEN_ACOUSTIC_SET);
	  EXPECT ("Location");
	  GET_REAL (acoustic.mount.x);
	  GET_REAL (acoustic.mount.y);
	  GET_REAL (acoustic.mount.z);
	  EXPECT ("Orientation");
	  GET_REAL (acoustic.mount.roll);
	  GET_REAL (acoustic.mount.pitch);
	  GET_REAL (acoustic.mount.yaw);
	  /* ignore MOUNT <name> */
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_ACOUSTIC_SET);

  return count;
}

/*
  GEO {Type VictSensor} {Name VictSensor Location 0.0600,0.0000,-0.0087 Orientation 0.0000,0.0000,0.0000 Mount CameraPanTilt_Link2}
*/
static int
handle_geo_victim (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("VictSensor");
	}
      else if (!strcmp (token, "Name"))
	{
	  GEO_NAME (victims, SW_SEN_VICTIM_SET);
	  EXPECT ("Location");
	  GET_REAL (victim.mount.x);
	  GET_REAL (victim.mount.y);
	  GET_REAL (victim.mount.z);
	  EXPECT ("Orientation");
	  GET_REAL (victim.mount.roll);
	  GET_REAL (victim.mount.pitch);
	  GET_REAL (victim.mount.yaw);
	  EXPECT ("Mount");
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  strncpy (where->sw.data.victim.parent, token,
		   sizeof (where->sw.data.victim.parent));
	  NULLTERM (where->sw.data.victim.parent);
	  count++;
	  ptr = nextptr;
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_SEN_VICTIM_SET);

  return count;
}

/*
  GEO {Type Gripper} {Name Gripper Location 0.0600,0.0000,-0.0087 Orientation 0.0000,0.0000,0.0000}
*/
static int
handle_geo_gripper (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("Gripper");
	}
      else if (!strcmp (token, "Name"))
	{
	  GEO_NAME (grippers, SW_EFF_GRIPPER_SET);
	  EXPECT ("Location");
	  GET_REAL (gripper.mount.x);
	  GET_REAL (gripper.mount.y);
	  GET_REAL (gripper.mount.z);
	  EXPECT ("Orientation");
	  GET_REAL (gripper.mount.roll);
	  GET_REAL (gripper.mount.pitch);
	  GET_REAL (gripper.mount.yaw);
	  count++;
	  ptr = nextptr;
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_EFF_GRIPPER_SET);

  return count;
}

/*
  GEO {Type MisPkg} {Name TeleMaxArm} {Link 1} {ParentLink -1} {Location 0.1789,-0.0014,-0.0905} {Orientation 3.1415,0.0000,0.0000} {Link 2} {ParentLink 1} {Location 0.0258,-0.0720,0.1566} {Orientation 1.5707,0.0000,0.0000} ...

  where the location and orientation of each link are with respect to
  the base frame of the mission package at the current location. This
  means that when the arm moves, these values will change. However, we
  compute the mounting as the transform from the link to its parent,
  which won't change.
*/

static void
sw_go_pose_convert (sw_pose * swp, go_pose * gop)
{
  go_rpy rpy;

  gop->tran.x = swp->x;
  gop->tran.y = swp->y;
  gop->tran.z = swp->z;
  rpy.r = swp->roll;
  rpy.p = swp->pitch;
  rpy.y = swp->yaw;
  go_rpy_quat_convert (&rpy, &gop->rot);

  return;
}

static void
go_sw_pose_convert (go_pose * gop, sw_pose * swp)
{
  go_rpy rpy;

  swp->x = gop->tran.x;
  swp->y = gop->tran.y;
  swp->z = gop->tran.z;
  go_quat_rpy_convert (&gop->rot, &rpy);
  swp->roll = rpy.r;
  swp->pitch = rpy.p;
  swp->yaw = rpy.y;

  return;
}

static int
handle_geo_mispkg (char *msg)
{
  VAR_DECL;
  int i;
  int linkindex;
  go_pose T_0_im1 = go_pose_identity ();
  go_pose T_i_0;
  go_pose T_i_im1;

  VAR_INIT;
  linkindex = 0;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("MisPkg");
	}
      else if (!strcmp (token, "Name"))
	{
	  GEO_NAME (misstas, SW_ACT_SET);
	  where->sw.data.mispkg.number = 0;
	}
      else if (!strcmp (token, "Link"))
	{
	  /* expecting number */
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if (1 != sscanf (token, "%i", &i))
	    return -1;
	  if (i < 1)
	    {
	      errprintf ("invalid link number for %s: %d\n", where->sw.name,
			 i);
	      i = 1;
	    }
	  else if (i > SW_ACT_LINK_MAX)
	    {
	      errprintf ("invalid link number for %s: %d\n", where->sw.name,
			 i);
	      i = SW_ACT_LINK_MAX;
	    }
	  linkindex = i - 1;
	  if (i > where->sw.data.mispkg.number)
	    where->sw.data.mispkg.number = i;
	  count++;
	  ptr = nextptr;
	}
      else if (!strcmp (token, "ParentLink"))
	{
	  /* expecting number */
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if (1 != sscanf (token, "%i", &i))
	    return -1;
	  if (i < -1)
	    {
	      errprintf ("invalid link number for %s: %d\n", where->sw.name,
			 i);
	      i = -1;
	    }
	  else if (i > SW_ACT_LINK_MAX)
	    {
	      errprintf ("invalid link number for %s: %d\n", where->sw.name,
			 i);
	      i = SW_ACT_LINK_MAX;
	    }
	  /* change convention of base at -1 to base at 0 */
	  if (i < 0)
	    i = 0;
	  where->sw.data.mispkg.link[linkindex].parent = i;
	  count++;
	  ptr = nextptr;
	}
      else if (!strcmp (token, "Location"))
	{
	  GET_REAL (mispkg.link[linkindex].mount.x);
	  GET_REAL (mispkg.link[linkindex].mount.y);
	  GET_REAL (mispkg.link[linkindex].mount.z);
	}
      else if (!strcmp (token, "Orientation"))
	{
	  GET_REAL (mispkg.link[linkindex].mount.roll);
	  GET_REAL (mispkg.link[linkindex].mount.pitch);
	  GET_REAL (mispkg.link[linkindex].mount.yaw);
	  /* 
	     Now we should have the full pose of this link wrt the base
	     frame, which we'll call T_i_0. To convert this to the pose
	     of the link in the parent frame, pre-multiply by the inverse
	     of the pose of the parent link in the base frame, which we
	     have saved from last cycle:

	     . i-1    i-1    0
	     .    T =    T *  T
	     .   i      0    i
	   */
	  sw_go_pose_convert (&where->sw.data.mispkg.link[linkindex].mount,
			      &T_i_0);
	  go_pose_pose_mult (&T_0_im1, &T_i_0, &T_i_im1);
	  go_sw_pose_convert (&T_i_im1,
			      &where->sw.data.mispkg.link[linkindex].mount);
	  /* invert this cycle's pose wrt base to be the parent for the
	     next cycle */
	  go_pose_inv (&T_i_0, &T_0_im1);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  /*
     Check for no reported links and set to max, as with handle_conf_mispkg.
   */
  if (0 == where->sw.data.mispkg.number)
    {
      where->sw.data.mispkg.number = SW_ACT_LINK_MAX;
    }

  /* as with handle_conf_mispkg, we inhibit telling of blank messages */
  if (count > 0)
    {
      TELL (SW_ACT_SET);
    }

  return count;
}

/*
  GEO {Type GroundVehicle}

  {Name P2AT} {Dimensions 0.5238,0.4968,0.2913} {COG 0.0000,0.0000,0.0000} {WheelRadius 0.1300} {WheelSeparation 0.4712} {WheelBase 0.2884}
*/
static int
handle_geo_groundvehicle (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  where = &robot;
  where->sw.type = SW_ROBOT_GROUNDVEHICLE;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("GroundVehicle");
	  where->did_geo = 1;
	  ptr = nextptr;
	}
      else if (!strcmp (token, "Name"))
	{
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  strncpy (where->sw.name, token, sizeof (where->sw.name));
	  where->sw.name[sizeof (where->sw.name) - 1] = 0;
	  ptr = nextptr;
	}
      else if (!strcmp (token, "Dimensions"))
	{
	  GET_REAL (groundvehicle.length);
	  GET_REAL (groundvehicle.width);
	  GET_REAL (groundvehicle.height);
	}
      else if (!strcmp (token, "COG"))
	{
	  where->sw.data.groundvehicle.cg.roll = 0;
	  where->sw.data.groundvehicle.cg.pitch = 0;
	  where->sw.data.groundvehicle.cg.yaw = 0;
	  GET_REAL (groundvehicle.cg.x);
	  GET_REAL (groundvehicle.cg.y);
	  GET_REAL (groundvehicle.cg.z);
	}
      else if (!strcmp (token, "WheelRadius"))
	{
	  GET_REAL (groundvehicle.wheel_radius);
	}
      else if (!strcmp (token, "WheelSeparation"))
	{
	  GET_REAL (groundvehicle.wheel_separation);
	}
      else if (!strcmp (token, "WheelBase"))
	{
	  GET_REAL (groundvehicle.wheel_base);
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_ROBOT_SET);

  return count;
}

/*
  GEO {Type BaseMachine} {Name FactoryControlBot} {Dimensions 0.0000,0.0000,0.0000}
*/
static int
handle_geo_basemachine (char *msg)
{
  VAR_DECL;

  VAR_INIT;

  where = &robot;
  where->sw.type = SW_ROBOT_FIXED;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  EXPECT ("BaseMachine");
	  where->did_geo = 1;
	  ptr = nextptr;
	}
      else
	{
	  /* skip unknown entry  */
	  nextptr = get_value (ptr, token);
	}
    }

  TELL (SW_ROBOT_SET);

  return count;
}

static int
handle_geo (char *msg)
{
  char token[MAX_TOKEN_LEN];
  char *ptr = msg;
  char *nextptr;
  int count = 0;

  waiting_for_geo = 0;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      /* look for {Type <name>}, and pass the whole msg to the sensor */
      if (!strcmp (token, "Type"))
	{
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    return -1;
	  if (!strcmp (token, "Sonar"))
	    {
	      return handle_geo_sonar (msg);
	    }
	  else if (!strcmp (token, "RangeScanner"))
	    {
	      return handle_geo_rangescanner (msg);
	    }
	  else if (!strcmp (token, "RangeImager"))
	    {
	      return handle_geo_rangeimager (msg);
	    }
	  else if (!strcmp (token, "Encoder"))
	    {
	      return handle_geo_encoder (msg);
	    }
	  else if (!strcmp (token, "Touch"))
	    {
	      return handle_geo_touch (msg);
	    }
	  else if (!strcmp (token, "CO2Sensor"))
	    {
	      return handle_geo_co2sensor (msg);
	    }
	  else if (!strcmp (token, "GroundTruth"))
	    {
	      return handle_geo_groundtruth (msg);
	    }
	  else if (!strcmp (token, "GPS"))
	    {
	      return handle_geo_gps (msg);
	    }
	  else if (!strcmp (token, "INS"))
	    {
	      return handle_geo_ins (msg);
	    }
	  else if (!strcmp (token, "Odometry"))
	    {
	      return handle_geo_odometry (msg);
	    }
	  else if (!strcmp (token, "Tachometer"))
	    {
	      return handle_geo_tachometer (msg);
	    }
	  else if (!strcmp (token, "Acoustic"))
	    {
	      return handle_geo_acoustic (msg);
	    }
	  else if (!strcmp (token, "VictSensor"))
	    {
	      return handle_geo_victim (msg);
	    }
	  else if (!strcmp (token, "Gripper"))
	    {
	      return handle_geo_gripper (msg);
	    }
	  else if (!strcmp (token, "MisPkg"))
	    {
	      return handle_geo_mispkg (msg);
	    }
	  else if (!strcmp (token, "GroundVehicle"))
	    {
	      return handle_geo_groundvehicle (msg);
	    }
	  else if (!strcmp (token, "BaseMachine"))
	    {
	      return handle_geo_basemachine (msg);
	    }
	  else if (!strcmp (token, "StaticPlatform"))
	    {
	      return handle_geo_staticplatform (msg);
	    }
	  else
	    {
	      errprintf ("Unknown geo type %s\n", token);
	      /* skip it and keep going */
	    }
	}
    }

  return count;
}

/*
  NFO {Gametype BotDeathMatch} {Level DM-Factory_250} {TimeLimit 0}

  NFO {StartPoses 3} {Name UnitLoader1 Location 10.24,3.54,10.05 Orientation 0.00,0.00,1.60} {Name UnitLoader2 Location 10.24,3.54,10.05 Orientation 0.00,0.00,1.60} {Name UnitLoader3 Location 10.24,3.54,10.05 Orientation 0.00,0.00,1.60}

*/
static int
handle_nfo (char *msg)
{
  dbprintf (msg);

  return 1;
}

static int
handle_em (char *msg)
{
  char token[MAX_TOKEN_LEN];
  char *ptr = msg;
  char *nextptr = ptr;
  int count = 0;

  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      dbprintf ("key %s\n", token);
      ptr = nextptr;
      nextptr = get_value (ptr, token);
      if (nextptr == ptr)
	break;
      dbprintf ("value %s\n", token);
      ptr = nextptr;
      count++;
      while (1)
	{
	  nextptr = get_value (ptr, token);
	  if (nextptr == ptr)
	    break;
	  dbprintf ("value %s\n", token);
	  ptr = nextptr;
	}
    }

  return count;
}

static int
do_sen_confs (usarsim_class_list where, char *type)
{
  char str[MAX_MSG_LEN];

  if (NULL == where)
    return -1;

  while (0 != where->sw.name[0])
    {
      if (!where->did_conf && !waiting_for_conf)
	{
	  //      printf( "usarsiminf: Getting sen conf for type %s, name %s\n", type, where->sw.name );
	  ulapi_snprintf (str, sizeof (str),
			  "GETCONF {Type %s} {Name %s}\r\n", type,
			  where->sw.name);
	  NULLTERM (str);
	  ulapi_mutex_take (socket_mutex);
	  usarsim_socket_write (socket_fd, str, strlen (str));
	  ulapi_mutex_give (socket_mutex);
	  waiting_for_conf = 1;
	  //      printf( "usarsiminf: (1) waiting_for_conf set \n" );
	}
      else if (!where->did_geo && !waiting_for_geo)
	{
	  //      printf( "usarsiminf: Getting geo for type %s, name %s\n", type, where->sw.name );
	  ulapi_snprintf (str, sizeof (str), "GETGEO {Type %s} {Name %s}\r\n",
			  type, where->sw.name);
	  NULLTERM (str);
	  ulapi_mutex_take (socket_mutex);
	  usarsim_socket_write (socket_fd, str, strlen (str));
	  ulapi_mutex_give (socket_mutex);
	  waiting_for_geo = 1;
	}
      where = where->next;
    }

  return 0;
}

static int
do_eff_confs (usarsim_class_list where, char *type)
{
  char str[MAX_MSG_LEN];

  if (NULL == where)
    return -1;

  while (0 != where->sw.name[0])
    {
      if (!where->did_conf && !waiting_for_conf)
	{
	  //      printf( "usarsiminf: Getting eff conf for type %s, name %s\n", type, where->sw.name );
	  ulapi_snprintf (str, sizeof (str),
			  "GETCONF {Type %s} {Name %s}\r\n", type,
			  where->sw.name);
	  NULLTERM (str);
	  ulapi_mutex_take (socket_mutex);
	  usarsim_socket_write (socket_fd, str, strlen (str));
	  ulapi_mutex_give (socket_mutex);
	  waiting_for_conf = 1;
	  //      printf( "usarsiminf: (2) waiting_for_conf set \n" );
	}
      else if (!where->did_geo && !waiting_for_geo)
	{
	  ulapi_snprintf (str, sizeof (str), "GETGEO {Type %s} {Name %s}\r\n",
			  type, where->sw.name);
	  NULLTERM (str);
	  ulapi_mutex_take (socket_mutex);
	  usarsim_socket_write (socket_fd, str, strlen (str));
	  ulapi_mutex_give (socket_mutex);
	  waiting_for_geo = 1;
	}
      where = where->next;
    }

  return 0;
}

static int
do_robot_confs (usarsim_class_list where)
{
  char str[MAX_MSG_LEN];

  if (!where->did_conf && !waiting_for_conf)
    {
      //    printf( "usarsiminf: Getting robot conf for name %s\n", where->sw.name );
      ulapi_snprintf (str, sizeof (str), "GETCONF {Type Robot} {Name %s}\r\n",
		      where->sw.name);
      NULLTERM (str);
      ulapi_mutex_take (socket_mutex);
      usarsim_socket_write (socket_fd, str, strlen (str));
      ulapi_mutex_give (socket_mutex);
      waiting_for_conf = 1;
      //    printf( "usarsiminf: (3) waiting_for_conf set \n" );
    }
  else if (!where->did_geo && !waiting_for_geo)
    {
      ulapi_snprintf (str, sizeof (str), "GETGEO {Type Robot} {Name %s}\r\n",
		      where->sw.name);
      NULLTERM (str);
      ulapi_mutex_take (socket_mutex);
      usarsim_socket_write (socket_fd, str, strlen (str));
      ulapi_mutex_give (socket_mutex);
      waiting_for_geo = 1;
    }

  return 0;
}

/*!
  \return Returns -1 on error, otherwise returns a number indicating how
  many elements were handled.
*/
static int
handle_msg (char *msg)
{
  char head[MAX_TOKEN_LEN];
  char *ptr = msg;
  int headindex = 0;
  int count;

  vbprintf (msg);

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
	  errprintf ("no head on ``%s''\n", msg);
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

  if (!strcmp (head, "NFO"))
    {
      count = handle_nfo (msg);
    }
  else if (!strcmp (head, "SEN"))
    {
      count = handle_sen (msg);
    }
  else if (!strcmp (head, "EFF"))
    {
      count = handle_eff (msg);
    }
  else if (!strcmp (head, "STA") || !strcmp (head, "ASTA"))
    {
      count = handle_sta (msg);
    }
  else if (!strcmp (head, "MISSTA"))
    {
      count = handle_missta (msg);
    }
  else if (!strcmp (head, "RES"))
    {
      count = handle_res (msg);
    }
  else if (!strcmp (head, "CONF"))
    {
      count = handle_conf (msg);
    }
  else if (!strcmp (head, "GEO"))
    {
      count = handle_geo (msg);
    }
  else
    {
      errprintf ("unknown head: ``%s''\n", msg);
      count = handle_em (msg);
    }

  do_sen_confs (encoders, "Encoder");
  do_sen_confs (sonars, "Sonar");
  do_sen_confs (rangescanners, "RangeScanner");
  do_sen_confs (rangeimagers, "RangeImager");
  do_sen_confs (touches, "Touch");
  do_sen_confs (co2sensors, "CO2Sensor");
  do_sen_confs (groundtruths, "GroundTruth");
  do_sen_confs (inses, "INS");
  do_sen_confs (gpses, "GPS");
  do_sen_confs (odometers, "Odometry");
  do_sen_confs (victims, "VictSensor");
  do_sen_confs (tachometers, "Tachometer");
  do_sen_confs (acoustics, "Acoustic");
  do_sen_confs (misstas, "MisPkg");

  do_eff_confs (grippers, "Gripper");

  do_robot_confs (&robot);

  return count;
}

/*
  Init string options: 

  -d,--debug : turn debug printing on
  -v,--verbose : turn verbose printing on
  -h,--host <name> : set host name
  -p,--port <port> : set TCP/IP port number
  -t,--type <type name> : set robot class type name
  -n,--name <name> : set robot name
  -s,--start <x,y,z,r,p,w> : set start location
*/
int
skin_inf_init (char *init_string, int (*stell) (sw_struct * sw))
{
#define INIT_STRING_LEN 256
  char str[INIT_STRING_LEN];
  int argc;
  char **argv;
  char *host = "localhost";
  char *type = "P2AT";
  char *name = "MyRobot";
  char *startname = NULL;
  char freeHost = 0;
  char freeType = 0;
  char freeName = 0;
  char freeStartname = 0;
  double x = 0;
  double y = 0;
  double z = 0;
  double roll = 0;
  double pitch = 0;
  double yaw = 0;
  int port = 3000;
  int t;
  int retval;

  retval = 0;

  if (NULL == init_string)
    {
      argc = 0;
    }
  else
    {
      argc = ulapi_to_argv (init_string, &argv);
      /*
         Would like to use getopt here, but the 'optreset' feature to
         reset it after being used by main isn't working, so we have
         to do it manually.
       */
      retval = 0;
      for (t = 0; t < argc; t++)
	{
	  if (!strcmp (argv[t], "-d") || !strcmp (argv[t], "--debug"))
	    {
	      debug = 1;
	    }
	  else if (!strcmp (argv[t], "-v") || !strcmp (argv[t], "--verbose"))
	    {
	      verbose = 1;
	    }
	  else if (!strcmp (argv[t], "-h") || !strcmp (argv[t], "--host"))
	    {
	      t++;
	      if (t == argc)
		{
		  errprintf ("inf_init: missing argument to %s\n",
			     argv[t - 1]);
		  retval = -1;
		  break;
		}
	      host = malloc (strlen (argv[t]) + 1);
	      strcpy (host, argv[t]);
	      freeHost = 1;
	    }
	  else if (!strcmp (argv[t], "-p") || !strcmp (argv[t], "--port"))
	    {
	      t++;
	      if (t == argc)
		{
		  errprintf ("inf_init: missing argument to %s\n",
			     argv[t - 1]);
		  retval = -1;
		  break;
		}
	      port = atoi (argv[t]);
	    }
	  else if (!strcmp (argv[t], "-t") || !strcmp (argv[t], "--type"))
	    {
	      t++;
	      if (t == argc)
		{
		  errprintf ("inf_init: missing argument to %s\n",
			     argv[t - 1]);
		  retval = -1;
		  break;
		}
	      type = malloc (strlen (argv[t]) + 1);
	      strcpy (type, argv[t]);
	      freeType = 1;
	    }
	  else if (!strcmp (argv[t], "-n") || !strcmp (argv[t], "--name"))
	    {
	      t++;
	      if (t == argc)
		{
		  errprintf ("inf_init: missing argument to %s\n",
			     argv[t - 1]);
		  retval = -1;
		  break;
		}
	      name = malloc (strlen (argv[t]) + 1);
	      strcpy (name, argv[t]);
	      freeName = 1;
	    }
	  else if (!strcmp (argv[t], "-s") || !strcmp (argv[t], "--start"))
	    {
	      t++;
	      if (t == argc)
		{
		  errprintf ("inf_init: missing argument to %s\n",
			     argv[t - 1]);
		  retval = -1;
		  break;
		}
	      if (6 !=
		  sscanf (argv[t], "%lf,%lf,%lf,%lf,%lf,%lf", &x, &y, &z,
			  &roll, &pitch, &yaw))
		{
		  startname = malloc (strlen (argv[t]) + 1);
		  strcpy (startname, argv[t]);
		  freeStartname = 1;
		}
	    }
	  else
	    {
	      errprintf ("inf_init: unrecognized option: %s\n", argv[t]);
	      retval = -1;
	      break;
	    }
	}
    }

#undef CLEANUP
#define CLEANUP					\
  {						\
    ulapi_free_argv(argc, argv);		\
    if (freeHost) free(host);			\
    freeHost = 0;				\
    if (freeName) free(name);			\
    freeName = 0;				\
    if (freeStartname) free(startname);		\
    freeStartname = 0;				\
  }

  if (0 != retval)
    {
      CLEANUP;
      return -1;
    }

  dbprintf ("inf_init: using host %s, port %d, debug %s, verbose %s\n", host,
	    port, debug ? "true" : "false", verbose ? "true" : "false");

  sup_tell = stell;

  socket_fd = ulapi_socket_get_client_id (port, host);
  if (socket_fd < 0)
    {
      errprintf ("can't open socket to %s port %d\n", host, port);
      CLEANUP;
      return -1;
    }

  socket_mutex = ulapi_mutex_new (SOCKET_MUTEX_KEY);
  if (NULL == socket_mutex)
    {
      ulapi_socket_close (socket_fd);
      socket_fd = -1;
      CLEANUP;
      return -1;
    }

  if (NULL != startname)
    {
      ulapi_snprintf (str, sizeof (str),
		      "GETSTARTPOSES\r\nINIT {Classname USARBot.%s} {Name %s} {Start %s}\r\n",
		      type, name, startname);
    }
  else
    {
      ulapi_snprintf (str, sizeof (str),
		      "GETSTARTPOSES\r\nINIT {Classname USARBot.%s} {Name %s} {Location %f,%f,%f} {Rotation %f,%f,%f}\r\n",
		      type, name, x, y, z, roll, pitch, yaw);
    }
  NULLTERM (str);
  ulapi_mutex_take (socket_mutex);
  usarsim_socket_write (socket_fd, str, strlen (str));
  ulapi_mutex_give (socket_mutex);

  /*
     Note:

     Some of the USARsim resources don't announce themselves.  Sensors
     do, but effectors don't. For the ones that don't, like effectors,
     we need to force a request for geo and conf information. It may
     come back empty.
   */

  //  printf( "usarsiminf: Getting misPgk(2) conf\n");
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

  //  printf( "usarsiminf: Getting gripper(2) conf\n");
  ulapi_snprintf (str, sizeof (str), "GETCONF {Type Gripper}\r\n");
  NULLTERM (str);
  ulapi_mutex_take (socket_mutex);
  usarsim_socket_write (socket_fd, str, strlen (str));
  ulapi_mutex_give (socket_mutex);

  ulapi_snprintf (str, sizeof (str), "GETGEO {Type Gripper}\r\n");
  NULLTERM (str);
  ulapi_mutex_take (socket_mutex);
  usarsim_socket_write (socket_fd, str, strlen (str));
  ulapi_mutex_give (socket_mutex);

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

  build = realloc (build, buildlen * sizeof (char));
  build_ptr = build;
  build_end = build + buildlen;

  usarsim_class_init (SW_SEN_ENCODER, encoders);
  usarsim_class_init (SW_SEN_SONAR, sonars);
  usarsim_class_init (SW_SEN_RANGESCANNER, rangescanners);
  usarsim_class_init (SW_SEN_RANGEIMAGER, rangeimagers);
  usarsim_class_init (SW_SEN_TOUCH, touches);
  usarsim_class_init (SW_SEN_CO2, co2sensors);
  usarsim_class_init (SW_SEN_GROUNDTRUTH, groundtruths);
  usarsim_class_init (SW_SEN_INS, inses);
  usarsim_class_init (SW_SEN_GPS, gpses);
  usarsim_class_init (SW_SEN_ODOMETER, odometers);
  usarsim_class_init (SW_SEN_VICTIM, victims);
  usarsim_class_init (SW_SEN_TACHOMETER, tachometers);
  usarsim_class_init (SW_SEN_ACOUSTIC, acoustics);

  usarsim_class_init (SW_ACT, misstas);

  usarsim_class_init (SW_EFF_GRIPPER, grippers);

  /*
     Note: the name of a robot appears to be ignored, and USARSim will
     set the name to be the type. It's OK to pass the name in geo and
     conf requests; since there can be only one robot, the name is
     ignored.
   */
  sw_set_name (&robot.sw, 0 != name[0] ? name : type);
  /*
     don't know or care what type, since this will be set explicitly in
     the handle_sta_xxx functions
   */
  sw_set_type (&robot.sw, 0);
  robot.did_conf = 0;
  robot.did_geo = 0;

  CLEANUP;

  return 0;
}

int
skin_inf_ask (void)
{
  char buffer[BUFFERLEN];
  char *buffer_ptr;
  char *buffer_end;
  ptrdiff_t offset;
  int nchars;

  nchars = ulapi_socket_read (socket_fd, buffer, BUFFERLEN);
  if (-1 == nchars)
    {				/* bad read */
      return -1;
    }
  if (0 == nchars)
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
	  if (handle_msg (build) < 0)
	    {
	      errprintf ("inf_ask: error handling %s\n", build);
	    }
	}
    }

  return 0;
}

int
skin_inf_tell (sw_struct * sw)
{
  char str[MAX_MSG_LEN];
  char cmp[MAX_MSG_LEN];
  int t;

  switch (sw->type)
    {
    case SW_ROBOT_GROUNDVEHICLE:
      switch (sw->op)
	{

	case SW_ROBOT_SKID_MOVE:
	  ulapi_snprintf (str, sizeof (str), "Drive {Left %f} {Right %f}\r\n",
			  sw->data.groundvehicle.left_speed,
			  sw->data.groundvehicle.right_speed);
	  NULLTERM (str);
	  ulapi_mutex_take (socket_mutex);
	  usarsim_socket_write (socket_fd, str, strlen (str));
	  ulapi_mutex_give (socket_mutex);
	  break;

	case SW_ROBOT_ACKERMAN_MOVE:
	  /* FIXME -- how do we know if the vehicle is front steered or
	     rear steered? */
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

	default:
	  errprintf ("inf_tell: not handling groundvehicle op %d\n", sw->op);
	  break;
	}			/* switch SW_ROBOT_GROUNDVEHICLE op */
      break;

    case SW_ACT:
      switch (sw->op)
	{

	case SW_ACT_POSITION:
	case SW_ACT_SPEED:
	case SW_ACT_TORQUE:
	  /* e.g., ACT {Name P2AT} {Link 1} {Value 1.23} {Order 0 | 1 | 2} */
	  *str = 0;
	  ulapi_snprintf (cmp, sizeof (cmp), "ACT {Name %s} ", sw->name);
	  NULLTERM (cmp);
	  strcat (str, cmp);
	  for (t = 0; t < sw->data.mispkg.number; t++)
	    {
	      ulapi_snprintf (cmp, sizeof (cmp),
			      "{Link %d} {Value %f} {Order %d} ", t,
			      SW_ACT_SPEED ==
			      sw->op ? sw->data.mispkg.link[t].
			      speed : SW_ACT_TORQUE ==
			      sw->op ? sw->data.mispkg.link[t].torque : sw->
			      data.mispkg.link[t].position,
			      SW_ACT_SPEED == sw->op ? 1 : SW_ACT_TORQUE ==
			      sw->op ? 2 : 0);
	      NULLTERM (cmp);
	      if (sizeof (str) <= strlen (str) + strlen (cmp) + 10)
		{		/* plus extra */
		  errprintf ("inf_tell: usarsim message string overflow\n");
		  break;
		}
	      strcat (str, cmp);
	    }
	  strcat (str, "\r\n");	/* here's some of the 10 extra */
	  ulapi_mutex_take (socket_mutex);
	  usarsim_socket_write (socket_fd, str, strlen (str));
	  printf ("usarsiminf (act msg): %s\n", str);
	  ulapi_mutex_give (socket_mutex);
	  break;

	default:
	  errprintf ("inf_tell: not handling mispkg op %d\n", sw->op);
	  break;
	}			/* switch SW_ACT op */
      break;

    case SW_EFF_GRIPPER:
      switch (sw->op)
	{

	case SW_EFF_GRIPPER_OPEN:
	  /* e.g., ACT {Name KR60Arm} {Gripper 1} */
	  ulapi_mutex_take (socket_mutex);
	  ulapi_snprintf (str, sizeof (str), "ACT {Name %s} {Gripper 0}\r\n",
			  sw->name);
	  NULLTERM (str);
	  usarsim_socket_write (socket_fd, str, strlen (str));
	  ulapi_mutex_give (socket_mutex);
	  break;

	case SW_EFF_GRIPPER_CLOSE:
	  /* e.g., ACT {Name KR60Arm} {Gripper 0} */
	  ulapi_mutex_take (socket_mutex);
	  ulapi_snprintf (str, sizeof (str), "ACT {Name %s} {Gripper 1}\r\n",
			  sw->name);
	  NULLTERM (str);
	  usarsim_socket_write (socket_fd, str, strlen (str));
	  ulapi_mutex_give (socket_mutex);
	  break;

	default:
	  errprintf ("inf_tell: not handling mispkg op %d\n", sw->op);
	  break;
	}			/* switch SW_ACT op */
      break;

    case SW_SEN_RANGEIMAGER:
      switch (sw->op)
	{
	case SW_SEN_GO:
	  ulapi_mutex_take (socket_mutex);
	  ulapi_snprintf (str, sizeof (str),
			  "SET {Type RangeImager} {Name %s} {Opcode SCAN} {Value 0.0}\r\n",
			  sw->name);
	  NULLTERM (str);
	  usarsim_socket_write (socket_fd, str, strlen (str));
	  ulapi_mutex_give (socket_mutex);
	  printf ("usarsiminf: wrote rangeimager cmd :%s\n", str);
	  break;
	default:
	  errprintf ("usarsiminf: unknown command to range imager\n");
	  break;
	}
      break;

    default:
      errprintf ("inf_tell: not handling type %s\n",
		 sw_type_to_string (sw->type));
      break;
    }				/* switch (sw->type) */

  return 0;
}

int
skin_inf_fini (void)
{
  dbprintf ("inf_fini");

  return 0;
}
