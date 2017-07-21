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
char rcsid[] = "$Id: xyEdge2LL.c,v 1.4 1996/02/02 18:12:55 ees2gm Exp matas $";
typedef char _r_foo[sizeof(rcsid)]; 
/*-----------------------------------------------------------------------*/

#include "LL.h"
#include "gfLL.h"
#include "gfLLconv.h"
#include "vector2.h"
#include "strGM.h"

t_LL PointStr2xyLL(char * str)
{
  t_LL    edge = ConsLL();
  int  n;
  int  i;
  int  nPts;

  sscanf(str,"%d %n", &nPts,&n);
  str+=n;

  for(i=0;i<nPts;i++)
  {
    t_V2 pt;
    sscanf(str,"%f %f %n", &pt.x,&pt.y,&n);
    str+=n;
    InsLastLL(edge,pt);
  }
  return edge;
}

void XYLL2pointset(t_LL list)
{
  t_V2 * pPoint;
  AppBufStr("%d ",SizeLL(list));
  ForeachLL_M(list,pPoint)
    AppBufStr("%5.1f %5.1f ",pPoint->x,pPoint->y);
}
void XYintLL2pointset(t_LL list)
{
  t_V2i * pPoint;
  AppBufStr("%d ",SizeLL(list));
  ForeachLL_M(list,pPoint)
    AppBufStr("%5d %5d ",pPoint->x,pPoint->y);
}
void YXintLL2pointset(t_LL list)
{
  t_V2i * pPoint;
  AppBufStr("%d ",SizeLL(list));
  ForeachLL_M(list,pPoint)
    AppBufStr("%5d %5d ",pPoint->y,pPoint->x);
}

t_LL XYedgeSet2LLofV2(t_LLSet set) 
 {return XYpointSet2LLofV2(set,"pointset");}

t_LL XYpointSet2LLofV2(t_LLSet set, char * pointName)
{
  t_LL   edges = ConsLL();
  t_LL   edge;
  char  *str;

  char** tokens = TokenizeStr(set->format);
  int i;

  for(i=0;NULL != tokens[i]; i++) 
    if(0 == stricmp(tokens[i],pointName)) break;

  if (NULL==tokens[i]) return edges;  /* no edge if pointset not found */

  ForeachLL_M(set->data,str)
  {
    char * pointSetStart = FindNthTokenStr(str,i+1);
    edge = PointStr2xyLL(pointSetStart);
    InsLastLL(edges,edge);
  }
  return edges ;
}

