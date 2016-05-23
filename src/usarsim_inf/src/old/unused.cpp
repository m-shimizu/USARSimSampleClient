
/*
  SEN {Time 5608.85} {Type RangeImager} 

  {Name Scanner1} {Frames 10} {Resolution 0.0174} {FOV 3.1415} {Range 11.3059,11.3014,11.3002,...}
*/

static int
handleSen_rangeimager (char *msg)
{
  sensorInfo info;
  int number;
  int i;

  sw_struct *sw = NULL;

  //  printf( "usarsiminf: handleSen_rangeimager: %s\n", msg );

  number = 0;
  while (1)
    {
      nextptr = get_key (ptr, token);
      if (nextptr == ptr)
	break;
      ptr = nextptr;

      if (!strcmp (token, "Type"))
	{
	  expect (&info, "RangeImager");
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
	     /
	     while (1)
	     {
	     nextptr = get_value (ptr, token);
	     if (nextptr == ptr)
	     {
	     if (0 == number)
	     return -1; // need at least one range value
	     else
	     break;
	     }
	     if (1 != sscanf (token, "%lf", &d))
	     return -1;
	     if (number >= SW_SEN_RANGEIMAGER_MAX)
	     {
	     // drop it 
	     printf ("usarsiminf: error, dropping data\n");
	     }
	     else
	     {
	     where->sw.data.rangeimager.range[number] = d;
	     number++;
	     }
	     ptr = nextptr;
	     }
	     // all ok, so credit the count
	     count++;
	     }
	     else
	     {
	     // skip unknown entry 
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

	  static int handleSen_touch (char *msg)
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
	  static int handleSen_co2sensor (char *msg)
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

	  static int handleSen_groundtruth (char *msg)
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

	  static int handleSen_gps (char *msg)
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

	  static int handleSen_ins (char *msg)
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

	  static int handleSen_odometry (char *msg)
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

	  static int handleSen_victim (char *msg)
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

	  static int handleSen_tachometer (char *msg)
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
			    where->sw.data.tachometer.position[pos_number] =
			      d;
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

	  static int handleSen_acoustic (char *msg)
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
		    where->sw.data.acoustic.altitude =
		      atan2 (z, sqrt (x * x + y * y));
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
