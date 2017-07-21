/*
			gradient.c
			----------

	Detection of contours using Rachid Deriche's method
	Written by Radu Horaud (initial version by Rachid in fortran).
	April 12, 1988.

	The non-maximu suppression completely rewritten by D.Gentle
	and G. Matas at UoS .
	January 92

	Region of Interest option added by G.Matas
	February 92
*/

#include <stdio.h>
#include <stdlib.h>
#include "ary.h"
#include "strGM.h"
#include "math.h"
#include "canny.h"

#define CARRE(a)   ((a) * (a))

void WriteGrd(char * grad_file, FARY * image) ;


#define EPSILON                0.0001

#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))

int IsDiag(float A,float xp,float yp,float xyp,float dx,float dy);

FARY *FindGradient(FARY *image, float alpha, float omega,
                   float *high_thresh, float *low_thresh)
{
  return FindGradientDir(image,alpha,omega,NULL,high_thresh,low_thresh);
}

FARY *FindGradientDir (FARY *image, float alpha, float omega, FARY* grad_dir,
    float *high_thresh, float *low_thresh)
/* standard paremeter values: alpha = 2. omega = .001 */
{
  register int i, j;
  FARY *a1 = NULL, *a2 = NULL, *a3 = NULL, *a4 = NULL;
  int  lines, columns, i_min, j_min, i_max, j_max, points = 0;
  float cst, cst0, cst1, cst2, ad1, ad2, an1, an2, an3, an4, an11   ;
  float d_x, d_y;
  float meanval = 0;
  int is_a_maximum;
 
  // MZ: this seems to be the default
  *high_thresh = *low_thresh = 1.0;

  lines = image->ub1 - image->lb1 + 1;
  columns = image->ub2 - image->lb2 + 1;

  if (doRofI) {
    j_min = max(0,RofI[0]);
    i_min = max(0,RofI[1]);
    j_max = min(columns,RofI[2]);
    i_max = min(lines,RofI[3]);
  }
  else{
    i_min = 0;
    j_min = 0;
    i_max = lines;
    j_max = columns;
  }


  /* creat some working images */
  if ( (a1 = (FARY *) makFary (lines, columns)) == NULL )
     { printf ("Unable to make a1...\n"); return (NULL);  }
  if ( (a2 = (FARY *) makFary (lines, columns)) == NULL )
     { printf ("Unable to make a2...\n"); destAry(a1); return (NULL);  }
  if ( (a3 = (FARY *) makFary (lines, columns)) == NULL )
     { printf ("Unable to make a3...\n");
       destAry(a1); destAry(a2); return (NULL);  }
  if ( (a4 = (FARY *) makFary (lines, columns)) == NULL )
     { printf ("Unable to make a4...\n");
       destAry(a1); destAry(a2); destAry(a3); return (NULL);  }


  cst0 = alpha * alpha + omega * omega;
  cst  = cst0 * cst0 / (4 * alpha * omega);
  cst1 = -cst * alpha / cst0;
  cst2 = -cst * omega / cst0;
  ad1  = -2 * cos (omega) * exp (-alpha);
  ad2  = exp (-2 * alpha);
  an1  = cst2;
  an2  = -cst2 * cos (omega) * exp (-alpha) + cst1 * sin (omega) * exp (-alpha);
  an3  = an2 - cst2 * ad1;
  an4  = -cst2 * ad2;
  an11 = cst * sin (omega) * exp (-alpha);


  /* 1-st passage: Y-derivative */
  for (i= i_min; i < i_max; ++i)
    {
    a2->el[i][j_min+0] = an1 * image->el[i][j_min+0];
    a2->el[i][j_min+1] = an1 * image->el[i][j_min+1] + 
       an2 * image->el[i][j_min+0] - ad1 * a2->el[i][j_min+0];
    for (j = j_min+2; j < j_max; ++j)
      a2->el[i][j] = an1 * image->el[i][j] + an2 * image->el[i][j-1] -
         ad1 * a2->el[i][j-1] - ad2 * a2->el[i][j-2];
    }


  for (i= i_min; i < i_max; ++i)
    {
    a3->el[i][j_max-1] = 0;
    a3->el[i][j_max-2] = an3 * image->el[i][j_max-1];
    for (j = j_max-3; j >= j_min; --j)
      a3->el[i][j] = an3 * image->el[i][j+1] + an4 * image->el[i][j+2] -
         ad1 * a3->el[i][j+1] - ad2 * a3->el[i][j+2];
    }

  for (i= i_min; i < i_max; ++i)
  for (j = j_min; j < j_max; ++j)
      a2->el[i][j] += a3->el[i][j];

  for (j = j_min; j < j_max; ++j)
    {
    a3->el[i_min+0][j] = 0.0;
    a3->el[i_min+1][j] = an11 * a2->el[i_min+0][j];
    for (i = i_min+2; i < i_max; ++i)
      a3->el[i][j] = an11 * a2->el[i-1][j] - ad1 * a3->el[i-1][j] -
         ad2 * a3->el[i-2][j];
    }

  for (j = j_min; j < j_max; ++j)
    {
    a4->el[i_max-1][j] = 0.0;
    a4->el[i_max-2][j] = -an11 * a2->el[i_max-1][j];
    for (i = i_max-3; i >= i_min+0; --i)
      a4->el[i][j] = -an11 * a2->el[i+1][j] - ad1 * a4->el[i+1][j] -
         ad2 * a4->el[i+2][j];
    }

  /* stock results of 1-st passage */
  for (i= i_min; i < i_max; ++i)
  for (j = j_min; j < j_max; ++j)
      a1->el[i][j] = a4->el[i][j] + a3->el[i][j];

  /* 2-nd passage: X-derivative */
  for (i= i_min; i < i_max; ++i)
    {
    a2->el[i][j_min+0] = 0.0;
    a2->el[i][j_min+1] = an11 * image->el[i][j_min+0];
    for (j = j_min+2; j < j_max; ++j)
      a2->el[i][j] = an11 * image->el[i][j-1] -
         ad1 * a2->el[i][j-1] - ad2 * a2->el[i][j-2];
    }

  for (i= i_min; i < i_max; ++i)
    {
    a3->el[i][j_max-1] = 0.0;
    a3->el[i][j_max-2] = -an11 * image->el[i][j_max-1];
    for (j = j_max-3; j >= 0; --j)
      a3->el[i][j] = -an11 * image->el[i][j+1] -
         ad1 * a3->el[i][j+1] - ad2 * a3->el[i][j+2];
    }

  for (i= i_min; i < i_max; ++i)
  for (j = j_min; j < j_max; ++j)
      a2->el[i][j] += a3->el[i][j];

  for (j = j_min; j < j_max; ++j)
    {
    a3->el[i_min+0][j] = an1 * a2->el[i_min+0][j];
    a3->el[i_min+1][j] = an1 * a2->el[i_min+1][j] + an2 * a2->el[i_min+0][j]
            - ad1 * a3->el[i_min+0][j];
    for (i = i_min+2; i < i_max; ++i)
      a3->el[i][j] = an1 * a2->el[i][j] + an2 * a2->el[i-1][j] -
                     ad1 * a3->el[i-1][j] - ad2 * a3->el[i-2][j];
    }

  for (j = j_min; j < j_max; ++j)
    {
    a4->el[i_max-1][j] = 0.0;
    a4->el[i_max-2][j] = an3 * a2->el[i_max-1][j];
    for (i = i_max-3; i >= i_min; --i)
      a4->el[i][j] = an3 * a2->el[i+1][j] + an4 * a2->el[i+2][j] -
                     ad1 * a4->el[i+1][j] - ad2 * a4->el[i+2][j];
    }

  for (i= i_min; i < i_max; ++i)
  for (j = j_min; j < j_max; ++j)
      a4->el[i][j] += a3->el[i][j];

  /* compute gradient */
  for (i= i_min; i < i_max; ++i)
  for (j = j_min; j < j_max; ++j)
      {
       a2->el[i][j] = sqrt( CARRE (a4->el[i][j]) + CARRE (a1->el[i][j]));
      }

  if (grad_non_name != NULL)
    ScaleFary2P5pgm(a2,grad_non_name);
  /*     WriteGrd(grad_non_name, a2); */

  if(derivative_name != NULL)
  {
     WriteAry(a4,ConsStr("%s_dx.ary",derivative_name));   /* dx */
     WriteAry(a1,ConsStr("%s_dy.ary",derivative_name));   /* dy */
     if (derivatives_only) {exit(0);}
  }


  /* 0 the whole array, not only the region of interest  */
  for(i = 0; i < lines; i++)
   for(j = 0; j < columns; j++)
      a3->el[i][j]=0.0;


  /* research of local maxima with linear interpolation */
  for (i = i_min+1; i <= i_max-2; ++i)
  for (j = j_min+1; j <= j_max-2; ++j)

  {
  is_a_maximum = 0;
  d_x = a4->el[i][j];
  d_y = a1->el[i][j];

  if (fabs(d_y) < EPSILON) {
     if (fabs(d_x) < EPSILON) continue; 
     if ((a2->el[i][j+1]<=a2->el[i][j]) && (a2->el[i][j-1]<a2->el[i][j]))
         is_a_maximum = 1;
  }
  else {
     if (fabs(d_x) >= fabs(d_y) ) {
       if (d_x*d_y > 0) {
          if ((fabs(d_x*a2->el[i][j+1]+(a2->el[i+1][j+1]-a2->el[i][j+1])*d_y) <= fabs(d_x*a2->el[i][j])) 
    && (fabs(d_x*a2->el[i][j-1]+(a2->el[i-1][j-1]-a2->el[i][j-1])*d_y) <fabs(d_x*a2->el[i][j])) )
          {
                is_a_maximum = 1;
          }
        } 
        else {
    if ((fabs(d_x*a2->el[i][j-1]-(a2->el[i+1][j-1]-a2->el[i][j-1])*d_y) < fabs(d_x*a2->el[i][j])) 
    && (fabs(d_x*a2->el[i][j+1]-(a2->el[i-1][j+1]-a2->el[i][j+1])*d_y) <=fabs(d_x*a2->el[i][j])) ) 
          { 
                is_a_maximum = 1;
          }
        }
     }
     else {
        if (d_x*d_y > 0) {
    if ((fabs(d_y*a2->el[i+1][j]+(a2->el[i+1][j+1]-a2->el[i+1][j])*d_x) <= fabs(d_y*a2->el[i][j])) 
    && (fabs(d_y*a2->el[i-1][j]+(a2->el[i-1][j-1]-a2->el[i-1][j])*d_x) <fabs(d_y*a2->el[i][j])) )
       { 
                is_a_maximum = 1;
       }
        }
        else {
    if ((fabs(d_y*a2->el[i+1][j]-(a2->el[i+1][j-1]-a2->el[i+1][j])*d_x) <= fabs(d_y*a2->el[i][j])) 
    && (fabs(d_y*a2->el[i-1][j]-(a2->el[i-1][j+1]-a2->el[i-1][j])*d_x) < fabs(d_y*a2->el[i][j])) )
       { 
                is_a_maximum = 1;
       }
        }
     
     } /* end of if dx>dy */

  } /* end of if d_y < epsilon */

  if (is_a_maximum >= 1 ) 
     {
      a3->el[i][j] =(a2->el[i][j]); 
      if(grad_dir !=NULL) 
        grad_dir->el[i][j] = atan2(a1->el[i][j],a4->el[i][j]); 

     meanval += a2->el[i][j];
     points++;
  }

  } /* end of for loops */

  if ( points == 0 ) fprintf(stderr,"points = 0!!!!\n");
  else {
    meanval = meanval / points;

  /* user control of thresholds changed 14/3/91 by G. Matas */
  /* originally (R. Horaud), if high/low thresh was 0 the value 
     was computed automatically from meanval, otherwise it was left
     unchanged. 
       Now the threshold multiplies the estimated values  
  */
  /*
          if (*high_thresh == 0) *high_thresh = meanval;
    if (*low_thresh == 0) *low_thresh = meanval/10;
  */
          if (fix_thresh){              /* use the HARD threshold */
            *low_thresh = *high_thresh/10;
          }
          else {
            *high_thresh *= meanval;
            *low_thresh *= meanval/10;
          } 

  /*
    if (debug_general) fprintf(stderr,"The mean value of gradient: %f\n", meanval);
          if (debug_general) fprintf(stderr,"For %d points\n", points);
  */
       }

  /*   printf("converting from 4 to 8 way connectivity...\n\n"); */

#define image a3->el
#define BLACK 0
  if (!four_con) 
  /* if (0) */
  {
  float i1,i2,i3,i4;
  int is_connected;

  for (i = i_min+1; i <= i_max-2; ++i)
    for (j = j_min+1; j <= j_max-2; ++j)
      if(image[i][j] != 0){


        i1 = image[i-1][j];
        i2 = image[i][j-1];
        i3 = image[i+1][j];
        i4 = image[i][j+1];
        is_connected = 0;
        if     ((i1  > 0) && (i2  > 0))is_connected=1;
        else if((i2  > 0) && (i3  > 0))is_connected=1;
        else if((i3  > 0) && (i4  > 0))is_connected=1;
        else if((i4  > 0) && (i1  > 0))is_connected=1;
    if (is_connected){
      is_a_maximum = 0;
      d_x = a4->el[i][j];
      d_y = a1->el[i][j];
     if (fabs(d_x) >= fabs(d_y) ) {
       if (d_x*d_y > 0) {
  if (IsDiag(a2->el[i][j],a2->el[i][j+1],a2->el[i+1][j],a2->el[i+1][j+1],d_x,d_y))
  if (IsDiag(a2->el[i][j],a2->el[i][j-1],a2->el[i-1][j],a2->el[i-1][j-1],d_x,d_y))
          is_a_maximum = 1;
        } 
        else {
  if (IsDiag(a2->el[i][j],a2->el[i][j+1],a2->el[i-1][j],a2->el[i-1][j+1],d_x,d_y))
  if (IsDiag(a2->el[i][j],a2->el[i][j-1],a2->el[i+1][j],a2->el[i+1][j-1],d_x,d_y))
          is_a_maximum = 1;
        }
     }
     else {
        if (d_x*d_y > 0) {
  if (IsDiag(a2->el[i][j],a2->el[i+1][j],a2->el[i+1][j],a2->el[i+1][j+1],d_y,d_x))
  if (IsDiag(a2->el[i][j],a2->el[i-1][j],a2->el[i][j-1],a2->el[i-1][j-1],d_y,d_x))
       is_a_maximum = 1;
        }
        else {
  if (IsDiag(a2->el[i][j],a2->el[i+1][j],a2->el[i][j-1],a2->el[i+1][j-1],d_y,d_x))
  if (IsDiag(a2->el[i][j],a2->el[i-1][j],a2->el[i][j+1],a2->el[i-1][j+1],d_y,d_x))
       is_a_maximum = 1;
        }
     
     } /* end of if dx>dy */

     if (is_a_maximum == 0) image[i][j] = BLACK;
    } /* end of connect */ 
  }   /* end of image > 0 */
  }   /* end of do four conn suppresion */


  destAry(a1);
  destAry(a2);
  destAry(a4);
  return (a3);
}

int IsDiag(float A,float xp,float yp,float xyp,float dx,float dy)
{
  float val;
  float adx = fabs(dx);
  float ady = fabs(dy);

  val = xp + ((A+xp+yp+xyp)/4 - xp)*ady/(adx+ady);
  return (A>val);
}

