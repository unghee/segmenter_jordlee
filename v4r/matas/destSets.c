/*----------- Destroy (free) all sets in a list + the list ---------------*/
/* +-------------------------------------------------------------------+ */
/* | Copyright 1994, George (Jiri) Matas  (g.matas@ee.surrey.ac.uk)    | */
/* |   Permission to use, copy, modify, and distribute this software   | */
/* |   and its documentation for any purpose and without fee is hereby | */
/* |   granted, provided that the above copyright notice appear in all | */
/* |   copies and that both that copyright notice and this permission  | */
/* |   notice appear in supporting documentation.  This software is    | */
/* |   provided "as is" without express or implied warranty.           | */
/* +-------------------------------------------------------------------+ */
static const
char rcsid[] = "$Id: destSets.c,v 1.4 1996/02/02 18:12:55 ees2gm Exp $";
typedef char _r_foo[sizeof(rcsid)]; 
/*-----------------------------------------------------------------------*/

#include "LL.h"
#include "gfLL.h"
 
static  void * DestForApply(void * s)
{
   DestLLSet(*(t_LLSet*) s);
   return s;
}

void  DestLLSets(t_LL Sets)
{
  ApplyLL(Sets, DestForApply);
  DestLL(Sets);
}
