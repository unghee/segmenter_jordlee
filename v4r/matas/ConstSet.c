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
static const
char rscid[] = "$Id: ConstSet.c,v 1.4 1996/02/02 18:12:55 ees2gm Exp $";
typedef char _r_foo[sizeof(rscid)];
/*-----------------------------------------------------------------------*/

#include "strGM.h"
#include "LL.h"
#include "gfLL.h"

/*------------------------------------------------------------------------*/
t_LLSet ConsLLSet(char * SetName, t_LL GeomStruct, char * format)
{
  t_LLSet Set;

  Set = (t_LLSet) malloc (sizeof(*Set));
  Set->id = DupStr(SetName);
  Set->format = DupStr(format);

  if(NULL==GeomStruct)  Set->GeomStruct = ConsLL();
  else                  Set->GeomStruct = GeomStruct;

  Set->Attributes = ConsLL();
  Set->data = ConsLL();
  
  return Set;
}

void DestLLSet(t_LLSet Set)
{
 
  DestLL(Set->data);
  DestLL(Set->GeomStruct);
  DestLL(Set->Attributes);
  free(Set->id);
  free(Set->format);
  free(Set);
}
  
