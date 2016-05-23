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
  \file unix_ulapi.h

  \brief Declarations of Unix implementation of ULAPI
*/

#ifndef UNIX_ULAPI_H
#define UNIX_ULAPI_H

#include "ulapi.h"		/* ulapi_real, ... */

#ifdef __cplusplus
extern "C" {
#if 0
} /* just to match one above, for indenters */
#endif
#endif

extern ulapi_result unix_ulapi_init(ulapi_integer sel);

extern ulapi_result unix_ulapi_exit(void);

extern void * unix_ulapi_shm_new(ulapi_id key, ulapi_integer size);

extern void * unix_ulapi_shm_addr(void * shm);

extern ulapi_result unix_ulapi_shm_delete(void * shm);

extern ulapi_result unix_ulapi_fifo_new(ulapi_integer key, ulapi_integer * fd,
					ulapi_integer size);
extern ulapi_result unix_ulapi_fifo_delete(ulapi_integer key, ulapi_integer fd,
					   ulapi_integer size);
extern ulapi_integer unix_ulapi_fifo_write(ulapi_integer fd, const char *buf,
					   ulapi_integer size);
extern ulapi_integer unix_ulapi_fifo_read(ulapi_integer fd, char *buf,
					  ulapi_integer size);

#ifdef __cplusplus
#if 0
{			  /* just to match one below, for indenters */
#endif
}
#endif

#endif /* UNIX_ULAPI_H */
