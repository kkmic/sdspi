/**
*****************************************************************************
**
**  File        : syscalls.c
**
**  Abstract    : Atollic TrueSTUDIO Minimal System calls file
**
** 		          For more information about which c-functions
**                need which of these lowlevel functions
**                please consult the Newlib libc-manual
**
**  Environment : Atollic TrueSTUDIO
**
**  Distribution: The file is distributed �as is,� without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) Pro toolchain.
**
*****************************************************************************
*/

/* Includes */
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>


/* Variables */
#undef errno
extern int errno;

extern int __io_putchar(int ch) __attribute__((weak));

extern int __io_getchar(void) __attribute__((weak));

extern void print_address(caddr_t addr) __attribute__((weak));

char *__env[1] = {0};
char **environ = __env;


/* Functions */
void initialise_monitor_handles()
{
}

int _getpid(void)
{
  return 1;
}

int _kill(int pid, int sig)
{
  errno = EINVAL;
  return -1;
}

void _exit(int status)
{
  _kill(status, -1);
  while (1) {}    /* Make sure we hang here */
}

int _read(int file, char *ptr, int len)
{
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++) {
    *ptr++ = __io_getchar();
  }

  return len;
}

int _write(int file, char *ptr, int len)
{
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++) {
    __io_putchar(*ptr++);
  }
  return len;
}

//extern caddr_t HEAP_START asm("_end");
extern char end asm("end");
register caddr_t stack_ptr asm ("sp");
caddr_t heap_end = NULL;

caddr_t _sbrk(int incr)
{
  if (heap_end == NULL)
    heap_end = &end;

  char *prev_heap_end = heap_end;
  if (heap_end + incr > stack_ptr) {

#ifdef MEMORY_DEBUG
		_write(1, "Heap and stack collision\r\n", 26);
#endif

    errno = ENOMEM;
    return (caddr_t) -1;
  }

  heap_end += incr;
#ifdef MEMORY_DEBUG
  _write(1, "Allocated memory\r\n", 18);
#endif

  return (caddr_t) prev_heap_end;
}

//
//caddr_t _sbrk(int incr)
//{
//  static caddr_t heap = NULL;
//  caddr_t prevHeap;
//  caddr_t nextHeap;
//
//  if (heap == NULL) {
//    // first allocation
//    heap = (caddr_t)&HEAP_START;
//
//#ifdef MEMORY_DEBUG
//    _write(1, "Heap start ", 11);
//    print_address(heap);
//    _write(1, ":   ", 4);
//
//    _write(1, "Heap end ", 9);
//    print_address((caddr_t)&_HEAP_END);
//    _write(1, "\r\n", 2);
//#endif
//  }
//
//  prevHeap = heap;
//
//  // Always return data aligned on a 8 byte boundary
//  nextHeap = (caddr_t)(((unsigned int)(heap + incr) + 7) & ~7);
//
//#ifdef MEMORY_DEBUG
//  _write(1, "Heap ", 5);
//  print_address(heap);
//  _write(1, ": inc ", 6);
//  print_address((caddr_t)incr);
//  _write(1, ":   ", 4);
//
//  _write(1, "Next heap ", 10);
//  print_address(nextHeap);
//  _write(1, ":   ", 4);
//
//  // get current stack pointer
//  _write(1, "Stack ", 6);
//  print_address(stackPtr);
//  _write(1, "\r\n", 2);
//#endif
//
//  // Check enough space and there is no collision with stack coming the other way
//  // if stack is above start of heap
//  if (((caddr_t)&HEAP_START > stackPtr && nextHeap > stackPtr))
//  {
//#ifdef MEMORY_DEBUG
//		_write(1, "Heap and stack collision\r\n", 26);
//#endif
//    errno = ENOMEM;
//    return (caddr_t) -1; // error - no more memory
//  } else {
//    heap = nextHeap;
//    return (caddr_t) prevHeap;
//  }
//}

int _close(int file)
{
  return -1;
}


int _fstat(int file, struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file)
{
  return 1;
}

int _lseek(int file, int ptr, int dir)
{
  return 0;
}

int _open(char *path, int flags, ...)
{
  /* Pretend like we always fail */
  return -1;
}

int _wait(int *status)
{
  errno = ECHILD;
  return -1;
}

int _unlink(char *name)
{
  errno = ENOENT;
  return -1;
}

int _times(struct tms *buf)
{
  return -1;
}

int _stat(char *file, struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}

int _link(char *old, char *new)
{
  errno = EMLINK;
  return -1;
}

int _fork(void)
{
  errno = EAGAIN;
  return -1;
}

int _execve(char *name, char **argv, char **env)
{
  errno = ENOMEM;
  return -1;
}
