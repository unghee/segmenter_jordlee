/*---------------------------------------------------------------------*/
/*
  G. Matas, 1-Sep-93
    changed to use el_size field of ary instead of AryElemSize

  G. Matas, 17-Jun-93
    created
*/
/*---------------------------------------------------------------------*/
static char sccsid[]="@(#)copyAry.c	2.4	95/02/22 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <stdlib.h>
#include <string.h>
#include "ary.h" 

ARY * CopyAry(ARY * in)
{
  ARY  *out       = makary(in->lb1,in->ub1,in->lb2,in->ub2,in->ary_type) ;
  int elSize     = in->el_size;
  int  n_of_bytes = (in->ub1-in->lb1+1)*(in->ub2-in->lb2+1)* elSize;

  if(NULL==out) ErrHandle("makary returned NULL in CopyAry",-1);

  memcpy((char*)(out->el[out->lb1])+elSize*out->lb2,
	  (char*)(in->el[in->lb1])+elSize*in->lb2,
	  n_of_bytes);

  return(out) ;
}


void  ScaleFillBary(BARY *dst, BARY * src, float f_zoom_i, float f_zoom_j)
{
   int k,l;
	 int src_i,src_j;
	 int dst_i,dst_j;
   int src_i_inc, src_j_inc;
   int dst_i_inc, dst_j_inc;
   int zoom_i, zoom_j;
    

   if (f_zoom_i >= 1 )
   {
     zoom_i = (int) f_zoom_i; src_i_inc = 0; dst_i_inc = 1;
   }
   else if(f_zoom_i>0)
   {
     zoom_i =(int)1/f_zoom_i;src_i_inc = 1; dst_i_inc = 0;
   }
   else return;
 

   if (f_zoom_j >= 1 ) 
   {
      zoom_j = (int) f_zoom_j;src_j_inc = 0; dst_j_inc = 1; 
   }
   else if(f_zoom_j>0)
   { 
      zoom_j =(int)1/f_zoom_j;src_j_inc = 1; dst_j_inc = 0;
   }
   else return;

   for(src_i=src->lb1, dst_i=dst->lb1; src_i <= src->ub1;
       src_i += dst_i_inc,    /* the dst to src is correct */
       dst_i += src_i_inc
      )
   {
     for(k=1; k<=zoom_i; k++)
     {
       if (src_i <= src->ub1 &&  dst_i <= dst->ub1){
       for(src_j=src->lb2, dst_j=dst->lb2; src_j<=src->ub2;

           src_j += dst_j_inc,    /* the dst to src is correct */
           dst_j += src_j_inc
          )
         for(l=1; l<=zoom_j; l++)
         {
           if (src_j <= src->ub2 &&  dst_j <= dst->ub2) {
			       dst->el[dst_i][dst_j] = src->el[src_i][src_j];
           }
           src_j += src_j_inc;
           dst_j += dst_j_inc;
         }
       }
       src_i += src_i_inc;
       dst_i += dst_i_inc;
     }
   }
}

 
