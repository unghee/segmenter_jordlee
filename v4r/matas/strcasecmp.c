/*
 * stricmp - compare string s1 to s2 without regard to case
 *
 * based on code by Henry Spencer
 * modified for ANSI by D'Arcy J.M. Cain
 */
/*----------------------------------------------------------------*/
static char sccsid[]="@(#)strcasecmp.c	1.4	94/09/02 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include  "strGM.h"

#include  <string.h>
#include  <ctype.h>

int    stricmp(const char *s1, const char *s2)
{
  while (*s1 && toupper(*s1) == toupper(*s2))
  {
    s1++;
    s2++;
  }

  /*
   * The following case analysis is necessary so that characters
   * which look negative collate low against normal characters but
   * high against the end-of-string NUL.
   */
  if (*s1 == '\0' && *s2 == '\0')
    return(0);
  else if (*s1 == '\0')
    return(-1);
  else if (*s2 == '\0')
    return(1);
  else
    return(toupper(*s1) - toupper(*s2));
}

/* ----- as above, but compare the first N chars ------- */
/* copied and modified from above by G. Matas            */

int    strnicmp(const char *s1, const char *s2, int num)
{
  while (num>0 && *s1 && toupper(*s1) == toupper(*s2))
  {
    s1++;
    s2++;
    num--;
  }

  if (0==num) return 0;  /* first N chars equal */

  /* if N is larger then the length of one of the strings analyse */

  /*
   * The following case analysis is necessary so that characters
   * which look negative collate low against normal characters but
   * high against the end-of-string NUL.
   */
  if (*s1 == '\0' && *s2 == '\0')
    return(0);
  else if (*s1 == '\0')
    return(-1);
  else if (*s2 == '\0')
    return(1);
  else
    return(toupper(*s1) - toupper(*s2));
}


