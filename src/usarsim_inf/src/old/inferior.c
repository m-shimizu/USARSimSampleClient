#include <stdio.h>
#include <string.h>
#include "ulapi.h"
#include "simware.h"
#include "skin.h"

static int
no_sup_tell (sw_struct * sw)
{
  fprintf (stderr, "no sup_tell function defined\n");

  return -1;
}

static int (*sup_tell) (sw_struct * sw) = no_sup_tell;

int
skin_inf_init (char *init_string, int (*stell) (sw_struct * sw))
{
  printf ("inferior init");

  if (NULL != init_string && 0 != init_string[0])
    {
      printf (": %s", init_string);
    }
  printf ("\n");

  sup_tell = stell;

  return 0;
}

int
skin_inf_ask (void)
{
  static int tick = 0;
  sw_struct sw;

  printf ("inferior ask\n");

  sw_init (&sw);
  sw_set_name (&sw, "MyEncoder");

  sw.type = SW_SEN_ENCODER;
  sw.op = SW_SEN_ENCODER_SET;
  sw.data.encoder.resolution = 0.001;
  sw.data.encoder.mount.x = 1;
  sw.data.encoder.mount.y = 2;
  sw.data.encoder.mount.z = -3;
  sw.data.encoder.mount.roll = -0.1;
  sw.data.encoder.mount.pitch = -0.2;
  sw.data.encoder.mount.yaw = 0.3;
  sup_tell (&sw);

  ulapi_wait (500000000);

  sw.op = SW_SEN_ENCODER_STAT;
  sw.time = ulapi_time ();
  sw.data.encoder.tick = tick++;

  sup_tell (&sw);

  ulapi_wait (500000000);

  return 0;
}

int
skin_inf_tell (sw_struct * sw)
{
  printf ("inferior tell\n");

  switch (sw->type)
    {
    case SW_ROBOT_GROUNDVEHICLE:
      if (SW_ROBOT_SKID_MOVE == sw->op)
	printf ("Skid %s : %f %f\n",
		sw->name,
		sw->data.groundvehicle.left_speed,
		sw->data.groundvehicle.right_speed);
      break;
    default:
      printf ("Simware class number %d not handled\n", sw->type);
      break;
    }

  return 0;
}

int
skin_inf_fini (void)
{
  printf ("inferior fini");

  return 0;
}
