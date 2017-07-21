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
char rscid[] = "$Id: vector2.c,v 1.5 1996/01/24 17:54:30 ees1rm Exp $";
   typedef char _s_foo[sizeof(rscid)];/*stop gcc warning: unused var rscid*/ 

/* $Log: vector2.c,v $
 * Revision 1.5  1996/01/24 17:54:30  ees1rm
 * *** empty log message ***
 *
 * Revision 1.4  1996/01/24  17:47:45  ees1rm
 * *** empty log message ***
 *
 * Revision 1.3  1995/08/28  17:03:26  ees2gm
 * *** empty log message ***
 *
 * Revision 1.2  1995/08/28  12:09:55  ees2gm
 * rscid slightly modified
 *
 * Revision 1.1  1995/08/28  12:07:15  ees2gm
 * Initial revision
 * */
 
/* last version under SCCS "@(#)95/02/17 g.matas@ee.surrey.ac.uk 2.4 .c"    */
/*--------------------------------------------------------------------------*/


#include "vector2.h"
#include <stdlib.h>
#include <math.h>

/*------------------------------------------------------------------------*/
t_V2 ConsV2( float x, float y)
{
   t_V2 v;

   v.x = x;
   v.y = y;

   return v;
}

/*------------------------------------------------------------------------*/
t_V2 * DupV2 (t_V2 in)
{
  t_V2 * dup = (t_V2*) malloc (sizeof(t_V2));

  *dup = in;
  return dup;
}

/*------------------------------------------------------------------------*/
int  IsEqualV2(t_V2 v1, t_V2 v2)  { return (v1.x==v2.x && v1.y==v2.y);}

t_V2 DiffV2( t_V2 v1, t_V2 v2)    { return ConsV2(v1.x - v2.x, v1.y - v2.y);}
t_V2 AddV2( t_V2 v1,  t_V2 v2)    { return ConsV2(v1.x + v2.x, v1.y + v2.y);}
t_V2 ScalV2( t_V2 v1, float fact) { return ConsV2(fact * v1.x, fact * v1.y);}

float DotPV2 ( t_V2 v1, t_V2 v2)  { return ( v1.x * v2.x + v1.y * v2.y); }
float CrossV2 ( t_V2 v1, t_V2 v2) { return ( v1.x * v2.y - v2.x * v1.y); }

t_V2 NormV2( t_V2 v)              { return ConsV2( v.y, -v.x); }
t_V2 NormClockV2( t_V2 v)         { return ConsV2( v.y, -v.x); }
t_V2 NormAClockV2( t_V2 v)        { return ConsV2(-v.y,  v.x); }

float MagnV2( t_V2 v1 )           { return sqrt(v1.x * v1.x + v1.y * v1.y);}
float MagnSqrV2( t_V2 v1 )        { return     (v1.x * v1.x + v1.y * v1.y);}

float DistV2( t_V2 point1, t_V2 point2) {return MagnV2(DiffV2(point1,point2));}

/*------------------------------------------------------------------------*/
float LineAngleV2(t_V2 v1, t_V2 v2)
{
  float Mv1= MagnV2(v1);
  float Mv2= MagnV2(v2);
  if ((Mv1<ZERO_V2)||(Mv2<ZERO_V2))return UNDEF_V2;
 
  return asin(CrossV2(v1,v2)/(Mv1*Mv2));
}

/*------------------------------------------------------------------------*/
float AngleV2(t_V2 v1, t_V2 v2)
{
  float Mv1= MagnV2(v1);
  float Mv2= MagnV2(v2);
  if ((Mv1<ZERO_V2)||(Mv2<ZERO_V2))return UNDEF_V2;
 
  return atan2(CrossV2(v1,v2),DotPV2(v1,v2));
}
/*------------------------------------------------------------------------*/
float PolarAngleV2(t_V2 v)
{
  if (( MagnV2(v)<ZERO_V2))return UNDEF_V2;
 
  return atan2(v.y,v.x);
}


/*------------------------------------------------------------------------*/
t_V2 IntSecV2(t_V2 s1, t_V2 e1, t_V2 s2,t_V2 e2,float *p1,float *p2)
{
  float det1, det2;

  float det = CrossV2(DiffV2(e1,s1),DiffV2(s2,e2));
  if (det==0.0){
     *p1 = UNDEF_V2;
     *p1 = UNDEF_V2;
     return ConsV2(UNDEF_V2,UNDEF_V2);
  }
  det1= CrossV2(DiffV2(s2,s1),DiffV2(s2,e2));
  det2= CrossV2(DiffV2(e1,s1),DiffV2(s2,s1));
  *p1 = det1/det;
  *p2 = det2/det;
  return ConsV2(s1.x+*p1*(e1.x-s1.x),s1.y + *p1*(e1.y-s1.y));
}

/*------------------------------------------------------------------------*/
t_V2 CollIntSecV2(t_V2 s1, t_V2 e1, t_V2 s2,t_V2 e2, int *r1, int *r2)
{
  t_V2 c1 , c2;
  
  if (MagnV2(DiffV2(s1,s2))<=MagnV2(DiffV2(e1,s2))){ c1=s1; *r1=0;}
  else                                                     { c1=e1; *r1=1;}
  if (MagnV2(DiffV2(s1,s2))<=MagnV2(DiffV2(s1,e2))){ c2=s2; *r2=0;} 
  else                                                     { c2=e2; *r2=1;}

  return ScalV2(AddV2(c1,c2),0.5);
}


/*
t_V2 ProjAngleV2
	( t_V2 Point, t_V2 PonLine, float LineAngle)
{
    float PdotT, McrosT;
    t_V2 t;

    sincos(LineAngle, &t.y, &t.x );
    t.y = sin(LineAngle);
    t.x = cos(LineAngle);

    PdotT = DotPV2 ( Point, t);
    McrosT = CrossV2 ( PonLine, t);

    return
        ConsV2(DotPV2 (ConsV2(PdotT, McrosT), t),
		      DotPV2 (ConsV2(-McrosT, PdotT), t));
}
*/
/*------------------------------------------------------------------------*/
t_V2 ProjV2_v1( t_V2 Point, t_V2 PonLine, t_V2 LineDir)
{
    float PdotT, McrosT;
    t_V2 t;

    t= ScalV2(LineDir, 1/MagnV2(LineDir));

    PdotT = DotPV2 ( Point, t);
       /* distance from foot (closest pt to origin) to projection of Point */
       /* == to projection of vector ending at Point on the line */
    McrosT = CrossV2 ( PonLine, t);
      /* t is a unit vector, therefore McrosT equals the distance from origin */

    return
        ConsV2(DotPV2 (ConsV2(PdotT, McrosT), t),
	       DotPV2 (ConsV2(-McrosT, PdotT), t));
}

/*------------------------------------------------------------------------*/
/* projection (foot) of Point onto line defined by point on line and direct.*/

t_V2 ProjV2 ( t_V2 Point, t_V2 PonLine, t_V2 dir /* directional vector */)
{
  /* derivation :   
       1. view Point and PonLine as vectors from origin
       2. Projection of the difference vector (Point - PonLine) onto line
	  is equal to the distance between projection (foot) Proj and PonLine
       3. Find projection Proj  by adding vector dir (line direction)
	  scaled by quantity computed in 2. to PonLine
  */
  return
   AddV2(PonLine,   /*3*/
         ScalV2(dir,/*2*/ DotPV2(
		    /*1*/ DiffV2(Point,PonLine),dir)/DotPV2(dir,dir)));
}

/*------------------------------------------------------------------------*/
float SignDistVtoLine(t_V2 Point, t_V2 PonLine1, t_V2 PonLine2)
{
  t_V2 dir = DiffV2 (PonLine2, PonLine1);
  t_V2 PtoL = DiffV2(Point,PonLine1);

  return CrossV2(PtoL,dir)/MagnV2(dir);
}
/*------------------------------------------------------------------------*/
float DistVtoLine(t_V2 Point, t_V2 PonLine1, t_V2 PonLine2)
{
  return fabs(SignDistVtoLine(Point,PonLine1,PonLine2));
}

/*------------------------------------------------------------------------*/
t_V2 RotV2 (t_V2 v, float r)
{
  float c = cos (r), s = sin (r);
  t_V2 v1;

   v1.x = DotPV2 (v, ConsV2 (c, -s));
   v1.y = DotPV2 (v, ConsV2 (s,  c));

   return v1;
}

