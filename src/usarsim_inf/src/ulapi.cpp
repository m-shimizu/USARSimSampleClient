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

//#include <stdio.h>            /* perror */
#include <ros/ros.h>
#include <stddef.h>		/* NULL */
#include <stdlib.h>		/* malloc */
#include <sys/ipc.h>		/* IPC_CREAT */
#include <sys/shm.h>		/* shmget() */
#include <string.h>		/* memset */
#include <ctype.h>		/* isspace */
#include <pthread.h>		/* pthread_create(), pthread_mutex_t */
#include <time.h>		/* struct timespec, nanosleep */
#include <sys/time.h>		/* gettimeofday(), struct timeval */
#include <unistd.h>		/* select(), write() */
#include <sys/sem.h>
#include <errno.h>
#include <fcntl.h>		/* O_RDONLY, O_NONBLOCK */
#include <sys/types.h>		/* fd_set, FD_ISSET() */
#include <sys/socket.h>		/* PF_INET, socket(), listen(), bind(), etc. */
#include <netinet/in.h>		/* struct sockaddr_in */
#include <netdb.h>		/* gethostbyname */
#include <arpa/inet.h>		/* inet_addr */


#include "ulapi.hh"

ulapi_result
ulapi_init (ulapi_integer sel)
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

  shm = (unix_shm_struct *) malloc (sizeof (unix_shm_struct));
  if (NULL == (void *) shm)
    return NULL;

  shm->id = shmget ((key_t) key, (int) size, IPC_CREAT | 0666);
  if (-1 == shm->id)
    {
      ROS_ERROR ("shmget");
      free (shm);
      return NULL;
    }

  shm->addr = shmat (shm->id, NULL, 0);
  if ((void *) -1 == shm->addr)
    {
      ROS_ERROR ("shmat");
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

ulapi_integer
ulapi_socket_read (ulapi_integer id, char *buf, ulapi_integer len)
{
  return read (id, buf, len);
}

ulapi_integer
ulapi_socket_write (ulapi_integer id, const char *buf, ulapi_integer len)
{
  //  ROS_INFO("writing: %s", buf );
  return write (id, buf, len);
}

ulapi_integer
ulapi_socket_get_client_id (ulapi_integer port, const char *hostname)
{
  int socket_fd;
  struct sockaddr_in server_addr;
  struct hostent *ent;
  struct in_addr *in_a;

  if (NULL == (ent = gethostbyname (hostname)))
    {
      ROS_ERROR ("gethostbyname");
      return -1;
    }
  in_a = (struct in_addr *) ent->h_addr_list[0];

  memset (&server_addr, 0, sizeof (struct sockaddr_in));
  server_addr.sin_family = PF_INET;
  server_addr.sin_addr.s_addr = in_a->s_addr;
  server_addr.sin_port = htons (port);

  if (-1 == (socket_fd = socket (PF_INET, SOCK_STREAM, 0)))
    {
      ROS_ERROR ("socket");
      return -1;
    }

  if (-1 == connect (socket_fd,
		     (struct sockaddr *) &server_addr,
		     sizeof (struct sockaddr_in)))
    {
      ROS_ERROR ("connect");
      close (socket_fd);
      return -1;
    }

  return socket_fd;
}

ulapi_result
ulapi_mutex_delete (void *mutex)
{
  if (NULL == (void *) mutex)
    return ULAPI_ERROR;

  (void) pthread_mutex_destroy ((pthread_mutex_t *) mutex);
  free (mutex);

  return ULAPI_OK;
}

void *
ulapi_mutex_new (ulapi_id key)
{
  pthread_mutex_t *mutex;

  mutex = (pthread_mutex_t *) malloc (sizeof (pthread_mutex_t));
  if (NULL == (void *) mutex)
    return NULL;

  /* initialize mutex to default attributes, and give it */
  if (0 == pthread_mutex_init (mutex, NULL))
    {
      (void) pthread_mutex_unlock (mutex);
      return (void *) mutex;
    }
  /* else got an error, so free the mutex and return null */

  free (mutex);
  return NULL;
}

ulapi_result
ulapi_mutex_give (void *mutex)
{
  return (0 ==
	  pthread_mutex_unlock ((pthread_mutex_t *) mutex) ? ULAPI_OK :
	  ULAPI_ERROR);
}

ulapi_result
ulapi_mutex_take (void *mutex)
{
  return (0 ==
	  pthread_mutex_lock ((pthread_mutex_t *) mutex) ? ULAPI_OK :
	  ULAPI_ERROR);
}

ulapi_result
ulapi_socket_close (ulapi_integer id)
{
  return 0 == close ((int) id) ? ULAPI_OK : ULAPI_ERROR;
}

ulapi_result
ulapi_exit (void)
{
  return ULAPI_OK;
}

ulapi_prio
ulapi_prio_highest (void)
{
  return 1;
}

ulapi_prio
ulapi_prio_lowest (void)
{
  return 31;
}

ulapi_prio
ulapi_prio_next_higher (ulapi_prio prio)
{
  if (prio == ulapi_prio_highest ())
    return prio;

  return prio - 1;
}

ulapi_prio
ulapi_prio_next_lower (ulapi_prio prio)
{
  if (prio == ulapi_prio_lowest ())
    return prio;

  return prio + 1;
}

void *
ulapi_task_new (void)
{
  return malloc (sizeof (pthread_t));
}

ulapi_result
ulapi_task_delete (void *task)
{
  if (NULL != task)
    free (task);

  return ULAPI_OK;
}

typedef void *(*pthread_task_code) (void *);

ulapi_result
ulapi_task_start (void *task,
		  void (*taskcode) (void *),
		  void *taskarg, ulapi_prio prio, ulapi_integer period_nsec)
{
  pthread_attr_t attr;
  struct sched_param sched_param;

  pthread_attr_init (&attr);
  sched_param.sched_priority = prio;
  pthread_attr_setschedparam (&attr, &sched_param);
  pthread_create ((pthread_t *) task, &attr, (pthread_task_code) taskcode,
		  taskarg);
  pthread_setschedparam (*((pthread_t *) task), SCHED_OTHER, &sched_param);
  /* ignore period_nsec, since we can't handle it in pthreads, so the
     application must call ulapi_wait(period_nsec) */

  return ULAPI_OK;
}

ulapi_result
ulapi_task_stop (void *task)
{
  return (pthread_cancel (*((pthread_t *) task)) ==
	  0 ? ULAPI_OK : ULAPI_ERROR);
}

ulapi_result
ulapi_task_pause (void *task)
{
  return ULAPI_OK;
}

ulapi_result
ulapi_task_resume (void *task)
{
  return ULAPI_OK;
}

ulapi_result
ulapi_task_set_period (void *task, ulapi_integer period_nsec)
{
  return ULAPI_OK;
}

ulapi_result
ulapi_task_init (void)
{
  if (0 != pthread_setcancelstate (PTHREAD_CANCEL_ENABLE, NULL))
    return ULAPI_ERROR;
  if (0 != pthread_setcanceltype (PTHREAD_CANCEL_ASYNCHRONOUS, NULL))
    return ULAPI_ERROR;

  return ULAPI_OK;
}

void ulapi_sleep(ulapi_real secs)
{
  int isecs, insecs;
  struct timespec ts;

  isecs = (int) secs;
  insecs = (int) ((secs - isecs) * 1.0e9);

  ts.tv_sec = isecs;
  ts.tv_nsec = insecs;

  (void) nanosleep(&ts, NULL);
}
