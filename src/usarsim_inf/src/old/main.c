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
  \file main.c

  \brief The top-level wrapper to Simware. Typically, run it like this: 

  \code
  ./newsimware --supargs "-i ../etc/moast.ini -n ../etc/moast.nml" \
  --infargs "-h abok -t P3AT -s 0.76,2.3,1.8,0,0,0 -v"
  \endcode
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "getopt.h"
#include "ulapi.h"
#include "skin.h"

static void
errprintf (const char *fmt, ...)
{
  va_list ap;

  fprintf (stderr, "simware: ");
  va_start (ap, fmt);
  vfprintf (stderr, fmt, ap);
  va_end (ap);
}

#define ERRSTR_LEN 80

static int
lib_load (void **handle, char *name)
{
  char errstr[ERRSTR_LEN];

  *handle = ulapi_dl_open (name, errstr, ERRSTR_LEN);
  if (NULL == *handle)
    {
      fprintf (stderr, "can't load %s: %s\n", name, errstr);
      return -1;
    }

  return 0;
}

static int
func_load (void **func, void *handle, char *name)
{
  char errstr[ERRSTR_LEN];

  *func = ulapi_dl_sym (handle, name, errstr, ERRSTR_LEN);
  if (0 != errstr[0])
    {
      fprintf (stderr, "can't look up %s: %s\n", name, errstr);
      return -1;
    }

  return 0;
}

void
sup_thread (void *arg)
{
  int (*sup_ask) (void) = arg;

  while (1)
    {
      sup_ask ();
    }
}

int
main (int argc, char *argv[])
{
  int c;
  int this_option_optind;
  int option_index;
  struct option long_options[] = {
    /* --option, needs arg, 0, 'shortcut char' or 0 */
    {"superior", required_argument, 0, 's'},
    {"supargs", required_argument, 0, 0},
    {"inferior", required_argument, 0, 'i'},
    {"infargs", required_argument, 0, 0},
    {"help", no_argument, 0, 'h'},
    {"h", no_argument, 0, 'h'},
    {"debug", no_argument, 0, 'd'},
    {"d", no_argument, 0, 'd'},
    {"version", no_argument, 0, 'v'},
    {"v", no_argument, 0, 'v'},
    {0, 0, 0, 0}
  };
  void *sup_handle = NULL;
  void *inf_handle = NULL;
  char *sup_name = NULL;
  char *inf_name = NULL;
  char *sup_init_string = NULL;
  char *inf_init_string = NULL;
  void *sup_task = NULL;
  int (*sup_init) (char *, int (*)(sw_struct *)) = skin_sup_init;
  int (*sup_ask) (void) = skin_sup_ask;
  int (*sup_tell) (sw_struct *) = skin_sup_tell;
  int (*sup_fini) (void) = skin_sup_fini;
  int (*inf_init) (char *, int (*)(sw_struct *)) = skin_inf_init;
  int (*inf_ask) (void) = skin_inf_ask;
  int (*inf_tell) (sw_struct *) = skin_inf_tell;
  int (*inf_fini) (void) = skin_inf_fini;
  int retval;

  opterr = 0;
  while (1)
    {
      this_option_optind = optind ? optind : 1;
      option_index = 0;

      /* the leading colon means generate a ':' for missing parameters */
      /* colons mean a required argument for the preceding option */
      c = getopt_long (argc, argv, ":s:i:hdv", long_options, &option_index);
      if (c == -1)
	break;

      switch (c)
	{
	  /* this handles --long args with no shortcut characters */
	case 0:
	  if (!strcmp ("supargs", long_options[option_index].name))
	    {
	      sup_init_string = malloc (strlen (optarg) + 1);
	      strcpy (sup_init_string, optarg);
	    }
	  else if (!strcmp ("infargs", long_options[option_index].name))
	    {
	      inf_init_string = malloc (strlen (optarg) + 1);
	      strcpy (inf_init_string, optarg);
	    }
	  /* else should never get here, since 0 above means OK */
	  break;

	  /* the rest handle -short args */
	case 's':
	  sup_name = malloc (strlen (optarg));
	  strcpy (sup_name, optarg);
	  break;

	case 'i':
	  inf_name = malloc (strlen (optarg));
	  strcpy (inf_name, optarg);
	  break;

	case 'h':
	  printf ("run with something like:\n");
	  printf
	    ("./main -s ./moastsup.so --supargs \"-d -n ./moast.nml\" -i ./usarsiminf.so --infargs \"-h abok.mel.nist.gov -p 3000 -d -t Telemax -n Robot1 -s10,-2,9.9,0,0,0\"\n");
	  return 0;
	  break;

	case 'd':
	  printf ("got debug\n");
	  break;

	case 'v':
	  printf ("sizeof(sw_struct) = %d\n", (int) sizeof (sw_struct));
	  return 0;
	  break;

	case '?':
	  errprintf ("unknown option `%s'\n", argv[this_option_optind]);
	  return 1;
	  break;

	case ':':
	  errprintf ("missing parameter to `%s'\n", argv[this_option_optind]);
	  return 1;
	  break;

	default:
	  errprintf ("bad argument `%s'\n", argv[this_option_optind]);
	  return 1;
	  break;
	}
    }
  if (optind < argc)
    {
      errprintf ("extra arguments provided:");
      while (optind < argc)
	{
	  errprintf (" %s", argv[optind++]);
	}
      errprintf ("\n");
      return 1;
    }

  if (ULAPI_OK != ulapi_init (UL_USE_DEFAULT))
    {
      errprintf ("can't initialize ulapi\n");
      return 1;
    }

  if (NULL != sup_name)
    {
      if (0 != lib_load (&sup_handle, sup_name))
	{
	  return -1;
	}
      if (0 != func_load ((void **) &sup_init, sup_handle, "skin_sup_init"))
	{
	  return -1;
	}
      if (0 != func_load ((void **) &sup_ask, sup_handle, "skin_sup_ask"))
	{
	  return -1;
	}
      if (0 != func_load ((void **) &sup_tell, sup_handle, "skin_sup_tell"))
	{
	  return -1;
	}
      if (0 != func_load ((void **) &sup_fini, sup_handle, "skin_sup_fini"))
	{
	  return -1;
	}
    }

  if (NULL != inf_name)
    {
      if (0 != lib_load (&inf_handle, inf_name))
	{
	  return -1;
	}
      if (0 != func_load ((void **) &inf_init, inf_handle, "skin_inf_init"))
	{
	  return -1;
	}
      if (0 != func_load ((void **) &inf_ask, inf_handle, "skin_inf_ask"))
	{
	  return -1;
	}
      if (0 != func_load ((void **) &inf_tell, inf_handle, "skin_inf_tell"))
	{
	  return -1;
	}
      if (0 != func_load ((void **) &inf_fini, inf_handle, "skin_inf_fini"))
	{
	  return -1;
	}
    }

  retval = sup_init (sup_init_string, inf_tell);
  if (0 != retval)
    {
      errprintf ("can't init superior skin\n");
      return 1;
    }

  retval = inf_init (inf_init_string, sup_tell);
  if (0 != retval)
    {
      errprintf ("can't init inferior skin\n");
      return 1;
    }

  sup_task = ulapi_task_new ();
  ulapi_task_start (sup_task, sup_thread, sup_ask, ulapi_prio_lowest (), 1);

  while (1)
    {
      inf_ask ();
    }

  (void) ulapi_exit ();

  return 0;
}
