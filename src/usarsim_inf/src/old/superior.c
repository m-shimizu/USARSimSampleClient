#include <stdio.h>
#include <string.h>
#include "ulapi.h"
#include "simware.h"
#include "skin.h"

static int
no_inf_tell (sw_struct * sw)
{
  fprintf (stderr, "no inf_tell function defined\n");

  return -1;
}

static int (*inf_tell) (sw_struct * sw) = no_inf_tell;

int
skin_sup_init (char *init_string, int (*itell) (sw_struct * sw))
{
  printf ("superior init");

  if (NULL != init_string && 0 != init_string[0])
    {
      printf (": %s", init_string);
    }
  printf ("\n");

  inf_tell = itell;

  return 0;
}

int
skin_sup_ask (void)
{
  sw_struct sw;

  printf ("superior ask\n");

  sw_init (&sw);
  sw_set_name (&sw, "MyRobot");
  sw.type = SW_ROBOT_GROUNDVEHICLE;
  sw.op = SW_ROBOT_SKID_MOVE;
  sw.data.groundvehicle.left_speed = 1;
  sw.data.groundvehicle.right_speed = -1;

  inf_tell (&sw);

  ulapi_wait (500000000);

  return 0;
}

int
skin_sup_tell (sw_struct * sw)
{
  switch (sw->type)
    {
    case SW_SEN_ENCODER:
      if (SW_SEN_ENCODER_STAT == sw->op)
	{
	  printf ("Encoder Status for %s at time %f: %d\n",
		  sw->name, sw->time, sw->data.encoder.tick);
	}
      else if (SW_SEN_ENCODER_SET == sw->op)
	{
	  printf ("Encoder Settings for %s: %f %f %f %f %f %f %f\n",
		  sw->name,
		  sw->data.encoder.resolution,
		  sw->data.encoder.mount.x,
		  sw->data.encoder.mount.y,
		  sw->data.encoder.mount.z,
		  sw->data.encoder.mount.roll,
		  sw->data.encoder.mount.pitch, sw->data.encoder.mount.yaw);
	}
      break;
    default:
      printf ("Simware class number %d not handled\n", sw->type);
      break;
    }
  return 0;
}

int
skin_sup_fini (void)
{
  printf ("superior fini");

  return 0;
}
