/* based on a example-code from Keil for CS G++ */

/* for caddr_t (typedef char * caddr_t;) */
//#include <sys/types.h>
#include "inc/hw_types.h"
extern int  __HEAP_START;
typedef char * caddr_t;

#define NULL 0


caddr_t _sbrk (int incr)
{
  static unsigned char *heap = NULL;
  unsigned char *prev_heap;

  if (heap == NULL) {
    heap = (unsigned char *)&__HEAP_START;
  }
  prev_heap = heap;
  // check removed to show basic approach 

  heap += incr;

  return (caddr_t) prev_heap;
}

