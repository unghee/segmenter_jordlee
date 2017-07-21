/*---------------------------------------------------------------------*/
/*
  G. Matas, H. Dabis, 17-Jun-93
    created
*/
/*---------------------------------------------------------------------*/
static char sccsid[]="@(#)normFary.c	2.4	95/02/22 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <math.h>
#include "ary.h"


/* normalize FARY so that the some of its absolute values is 'value' */

void NormFary(FARY *mask,float value)
{
   int i,j;
   float sum = 0.0;

   for(i=mask->lb1; i<=mask->ub1; i++)
    for(j=mask->lb2; j<=mask->ub2; j++)
      sum += (fabs)(mask->el[i][j]); 
				   

   for(i=mask->lb1; i<=mask->ub1; i++)
    for(j=mask->lb2; j<=mask->ub2; j++)
      mask->el[i][j] = mask->el[i][j]*value/sum;

}
