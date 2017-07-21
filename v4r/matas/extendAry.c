#include "ary.h"

/*
 *  InitBorderFary: Initialise the border of a FARY.
 * 
 *  Input: The FARY, border width and pixel value.
 *
 */  

void InitBorderFary(FARY * image, int border, int pixel)
{
  int i, j;

  for(i = image->lb1; i <= image->ub1; i++) 
       {
     for(j = image->lb2; j<=image->lb2+border; j++) image->el[i][j] = pixel;
     for(j = image->ub2-border; j<=image->ub2; j++) image->el[i][j] = pixel;
  } 
 

  for(j = image->lb2+border; j <= image->ub2-border; j++)
  {
     for(i = image->lb1; i<=image->lb1+border; i++) image->el[i][j] = pixel;
     for(i = image->ub1-border; i<=image->ub1; i++) image->el[i][j] = pixel;
  }
}


/* 
 * ExtendAry: Adds a border around an array (FARY).
 *
 * Input: A FARY and the width of the border in pixels.
 * 
 * Output: The extended FARY.
 *
 */

FARY * ExtendFary(FARY * in_image, int border)
{
   FARY  * out_image = (FARY *)makary(in_image->lb1-border,in_image->ub1+border,in_image->lb2-border,in_image->ub2+border, FLOATMATRIX);
   int i,j;
     
   /* Read image array into out_image array with extended border */
  
   for(i=in_image->lb1; i<=in_image->ub1; i++) 
   {
      for(j=in_image->lb2; j<=in_image->ub2; j++)
      {
        out_image->el[i][j] = in_image->el[i][j];  
      }           
   }
   return out_image;
}

