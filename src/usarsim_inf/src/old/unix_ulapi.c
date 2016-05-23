/*
  DISCLAIMER:
  This software was produced by the National Institute of Standards
  and Technology (NIST), an agency of the U.S. government, and by statute is
  not subject to copyright in the United States.  Recipients of this software
  assume all responsibility associated with its operation, modification,
  maintenance, and subsequent redistribution.

  See NIST Administration Manual 4.09.07 b and Appendix I. 
*/

/*
  unix_ulapi.c

  Implementation of user-level API to Unix simulations of the
  realtime part, plus some non-realtime resources like threads
  for portability.

  This assumes Unix user <-> Unix realtime.
*/

#include <stdio.h>		/* perror */
#include <stddef.h>		/* NULL */
#include <stdlib.h>		/* malloc */
#include <sys/ipc.h>		/* IPC_CREAT */
#include <sys/shm.h>		/* shmget() */
#include "ulapi.h"

ulapi_result
unix_ulapi_init (ulapi_integer sel)
{
  return UL_USE_UNIX == sel ? ULAPI_OK :
    UL_USE_DEFAULT == sel ? ULAPI_OK : ULAPI_IMPL_ERROR;
}

ulapi_result
unix_ulapi_exit (void)
{
  return ULAPI_OK;
}

typedef struct
{
  ulapi_id key;
  ulapi_integer size;
  ulapi_id id;
  void *addr;
} unix_shm_struct;

void *
unix_ulapi_shm_new (ulapi_id key, ulapi_integer size)
{
  unix_shm_struct *shm;

  shm = malloc (sizeof (unix_shm_struct));
  if (NULL == (void *) shm)
    return NULL;

  shm->id = shmget ((key_t) key, (int) size, IPC_CREAT | 0666);
  if (-1 == shm->id)
    {
      perror ("shmget");
      free (shm);
      return NULL;
    }

  shm->addr = shmat (shm->id, NULL, 0);
  if ((void *) -1 == shm->addr)
    {
      perror ("shmat");
      free (shm);
      return NULL;
    }

  return (void *) shm;
}

void *
unix_ulapi_shm_addr (void *shm)
{
  return ((unix_shm_struct *) shm)->addr;
}

ulapi_result
unix_ulapi_shm_delete (void *shm)
{
  struct shmid_ds d;
  int r1, r2;

  if (NULL == shm)
    return ULAPI_OK;

  r1 = shmdt (((unix_shm_struct *) shm)->addr);
  r2 = shmctl (IPC_RMID, ((unix_shm_struct *) shm)->id, &d);

  free (shm);

  return (r1 || r2 ? ULAPI_ERROR : ULAPI_OK);
}

ulapi_result
unix_ulapi_fifo_new (ulapi_integer key, ulapi_integer * fd,
		     ulapi_integer size)
{
  return ULAPI_ERROR;
}

ulapi_result
unix_ulapi_fifo_delete (ulapi_integer key, ulapi_integer fd,
			ulapi_integer size)
{
  return ULAPI_ERROR;
}

ulapi_integer
unix_ulapi_fifo_write (ulapi_integer fd, const char *buf, ulapi_integer size)
{
  return 0;
}

ulapi_integer
unix_ulapi_fifo_read (ulapi_integer fd, char *buf, ulapi_integer size)
{
  return 0;
}
