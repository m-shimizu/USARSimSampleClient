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
  \file skin.h

  \brief Declarations of the Simware skin interface functions.
*/

#ifndef SKIN_H
#define SKIN_H

#include "simware.h"

/*!
  The 'tell' function is provided by each of the superior and inferior
  skins, for its counterpart to call and notify ('tell') it what has
  happened. 
*/

/*!
  The 'ask' function is called by the main Simware process to ask the
  skin to query its underlying system for the latest information,
  place what it found in a \a sw_struct, and call the 'tell'
  function with that \a sw_struct.  A superior skin's 'ask'
  function will query the controller for commands. An inferior skin's
  'ask' function will query the sensors.
*/

/*! The initialization function for the skin. The argument is a place
  for any ad hoc skin information needed for initialization, passed
  via command-line arguments by the Simware process. */

/*! The finalization function is called by the Simware process to
  terminate the execution of a skin. */

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

/* The skin_sup_xxx functions are provided by the skins. */
extern int skin_sup_init(char *init_string, int (*inf_tell)(sw_struct *sw));
extern int skin_sup_ask(void);
extern int skin_sup_tell(sw_struct *sw);
extern int skin_sup_fini(void);

extern int skin_inf_init(char *init_string, int (*sup_tell)(sw_struct *sw));
extern int skin_inf_ask(void);
extern int skin_inf_tell(sw_struct *sw);
extern int skin_inf_fini(void);

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif
