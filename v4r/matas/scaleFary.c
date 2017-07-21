/*---------------------------------------------------------------------*/
/*
  G. Matas, 10-May-94
      ScaleMinMaxFary written by Rupert J. Young added. 

  G. Matas, 17-Jun-93
      created with the help of Homam Dabis

 Rupert Young
*/
/*---------------------------------------------------------------------*/
static char sccsid[]="@(#)scaleFary.c	2.6	95/08/29 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include "ary.h"

/* scale Fary so that it's maximum (positive, not absolute!) is equal */
/* to the requested_max value */

void ScaleMaxFary(FARY *ary,int requested_max)
{
   int i,j;
   float max = 0.0;

   for(i=ary->lb1; i<=ary->ub1; i++)
    for(j=ary->lb2; j<=ary->ub2; j++)
      if (ary->el[i][j]>max) max = ary->el[i][j]; 
				   

   for(i=ary->lb1; i<=ary->ub1; i++)
    for(j=ary->lb2; j<=ary->ub2; j++)
      ary->el[i][j] = requested_max*ary->el[i][j]/max;

}

float MinFary(FARY *ary)
{
  int i,j;
  float minimum = ary->el[ary->lb1][ary->ub1];

  for(i=ary->lb1; i<=ary->ub1; i++) 
    for(j=ary->lb2; j<=ary->ub2; j++)
      if (ary->el[i][j] < minimum)
	minimum = ary->el[i][j];

  return minimum;
}
float MinIary(IARY *ary)
{
  int i,j;
  float minimum = ary->el[ary->lb1][ary->ub1];

  for(i=ary->lb1; i<=ary->ub1; i++) 
    for(j=ary->lb2; j<=ary->ub2; j++)
      if (ary->el[i][j] < minimum)
	minimum = ary->el[i][j];

  return minimum;
}
float MaxFary(FARY *ary)
{
  int i,j;
  float maximum = ary->el[ary->lb1][ary->ub1];

  for(i=ary->lb1; i<=ary->ub1; i++) 
    for(j=ary->lb2; j<=ary->ub2; j++)
      if (ary->el[i][j] > maximum)
	maximum = ary->el[i][j];

  return maximum;
}
/*
 * ScaleMinMaxFary: Scales the positive and negative values in a FARY 
 * to between two input values, by finding the lowest and highest 
 * values and adjusting all values accordingly.
 *
 * Author: Rupert J. Young
 * Input: A FARY.
 * 
 * Output: The altered FARY.
 *
 */

void ScaleMinMaxFary(FARY * image, float min, float max)
{
   int i,j;
   float lowest  = MinFary(image);
   float highest = MaxFary(image);

   for(i=image->lb1; i<=image->ub1; i++) 
   {
      for(j=image->lb2; j<=image->ub2; j++)
      {
	  image->el[i][j] = min + (max - min) * (image->el[i][j] - lowest) / (highest - lowest);
      }    
   }
}

