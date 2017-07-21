/*---------- simple 2d vector lib ---------------------------------- */
/*  author: G. Matas                           (g.matas@ee.surrey.ac.uk) */
/* +-------------------------------------------------------------------+ */
/* | Copyright 1992, George Matas.                                     | */
/* |   Permission to use, copy, modify, and distribute this software   | */
/* |   and its documentation for any purpose and without fee is hereby | */
/* |   granted, provided that the above copyright notice appear in all | */
/* |   copies and that both that copyright notice and this permission  | */
/* |   notice appear in supporting documentation.  This software is    | */
/* |   provided "as is" without express or implied warranty.           | */
/* +-------------------------------------------------------------------+ */
/*------------------------------------------------------------------------*/
static const
char rscid[] = "$Id: vector2i.c,v 1.2 1996/03/26 10:04:25 eevsspsoft Exp $";
   typedef char _s_foo[sizeof(rscid)];/*stop gcc warning: unused var rscid*/ 

/*--------------------------------------------------------------------------*/


#include "vector2.h"
#include <stdlib.h>
#include <math.h>

/*------------------------------------------------------------------------*/
t_V2i ConsV2i( float x, float y)
{
   t_V2i v;

   v.x = x;
   v.y = y;

   return v;
}

/*------------------------------------------------------------------------*/
t_V2i * DupV2i (t_V2i in)
{
  t_V2i * dup = (t_V2i*) malloc (sizeof(t_V2i));

  *dup = in;
  return dup;
}

/*------------------------------------------------------------------------*/
int  IsEqualV2i(t_V2i v1, t_V2i v2){ return (v1.x==v2.x && v1.y==v2.y);}

t_V2i DiffV2i( t_V2i v1, t_V2i v2)
  { t_V2i v = {v1.x - v2.x, v1.y - v2.y}; return v;}
t_V2i AddV2i( t_V2i v1, t_V2i v2)
  { t_V2i v = {v1.x + v2.x, v1.y + v2.y}; return v;}
t_V2i ScalV2i( t_V2i v1, int fact)
  { t_V2i v = {fact * v1.x, fact * v1.y}; return v;}
t_V2i DivV2i( t_V2i v1, int fact)
  { t_V2i v = {v1.x / fact, v1.y / fact}; return v;}

int DotPV2i ( t_V2i v1, t_V2i v2)  { return ( v1.x * v2.x + v1.y * v2.y); }
int CrossV2i ( t_V2i v1, t_V2i v2) { return ( v1.x * v2.y - v2.x * v1.y); }

t_V2i NormV2i( t_V2i v)            { t_V2i w = {v.y, -v.x}; return w;}
t_V2i NormClockV2i( t_V2i v)       { t_V2i w = {v.y, -v.x}; return w;}
t_V2i NormAClockV2i( t_V2i v)      { t_V2i w = {-v.y, v.x}; return w;}

int   MagnSqrV2i( t_V2i v1 )       { return (v1.x * v1.x + v1.y * v1.y);}

/* intersection of lines, given as point p and direction a */
t_V2i IntSecV2i(t_V2i p1, t_V2i a1, t_V2i p2, t_V2i a2)
{
  int d = CrossV2i(a2, a1);
  if(d != 0)
  {
    int n = CrossV2i(a1, DiffV2i(p2,p1))/d;
    a2 = ScalV2i(a2,n);
    return AddV2i(p2,a2);
  }
  else
  {
    t_V2i v = {UNDEF_V2I, UNDEF_V2I};
    return v;
  }
}
/* squared distance */
int DistSqrV2i(t_V2i v1, t_V2i v2)
{
  return (v2.x-v1.x)*(v2.x-v1.x) + (v2.y-v1.y)*(v2.y-v1.y);
}
