/*---------------------------------------------------------------------- */
/* +-------------------------------------------------------------------+ */
/* | Copyright 1994, George (Jiri) Matas  (g.matas@ee.surrey.ac.uk)    | */
/* |   Permission to use, copy, modify, and distribute this software   | */
/* |   and its documentation for any purpose and without fee is hereby | */
/* |   granted, provided that the above copyright notice appear in all | */
/* |   copies and that both that copyright notice and this permission  | */
/* |   notice appear in supporting documentation.  This software is    | */
/* |   provided "as is" without express or implied warranty.           | */
/* +-------------------------------------------------------------------+ */
/* $Id: PGutil.c,v 1.3 1996/02/02 09:24:56 ees2gm Exp matas $
 * Modifications:
 * $Log: PGutil.c,v $
 * Revision 1.3  1996/02/02 09:24:56  ees2gm
 * *** empty log message ***
 *
 * Revision 1.2  1995/09/08  12:19:28  ees1rm
 * The function GetLLSetNum(t_LL Sets, char *id) updated.
 *
*/

/*-----------------------------------------------------------------------*/

#include <stdlib.h>
#include "strGM.h"
#include "gfLL.h"

/* define non-ANSI string functions */
/* char * strcasecmp(char * s1, char *s2); */

t_LLSet* GetLLpSet(t_LL Sets, char *id)
{
  t_LLSet *Set;
  
  ForeachLL_M(Sets,Set)
    /*if (strcasecmp((*Set)->id,id)==0) return (Set);*/
    if (stricmp((*Set)->id,id)==0) return (Set);

  return NULL;
}

t_LLSet* GetPrefLLpSet(t_LL Sets, char *id)
{
  t_LLSet *Set;
  
  ForeachLL_M(Sets,Set)
    if (IsTokenPrefStr(id,(*Set)->id)) return (Set);

  return NULL;
}

t_LLSet GetLLSet(t_LL Sets, char *id)
{
   t_LLSet  *pSet = GetLLpSet(Sets,id);
   if (pSet == NULL) return NULL;
                     return *pSet;
}

t_LLSet GetPrefLLSet(t_LL Sets, char *id)
{
   t_LLSet  *pSet = GetPrefLLpSet(Sets,id);
   if (pSet == NULL) return NULL;
                     return *pSet;
}

int GetLLSetNum(t_LL Sets, char *id)
{
  int i=0;
  t_LLSet *Set;
  
  ForeachLL_M(Sets,Set)
    if (stricmp(id,(*Set)->id) == 0) return i;

  return -1;
}


