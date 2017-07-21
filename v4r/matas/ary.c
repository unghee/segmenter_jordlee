/*----------------- dynamic 2-dimensional array lib -------------------*/
/*
  G.Matas, 1-Sep-93
     - generic ary allocator ConsAry1 created
     - makXary moved to a makAry.c
     - error handling modified to "returned on error"

  G.Matas, 17-Jun-93
     - ARY matrix allocation split to increase portability

  1992
    Written initially by Ron Cain at SRI, this version by Radu Horaud.
*/
/*---------------------------------------------------------------------*/
static char sccsid[]="@(#)ary.c	2.7	95/08/29 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <stdlib.h>
#include "ary.h"
/* zvire Marik */
void *consAry1
(int firstrow,int lastrow, int firstcol, int lastcol, int elSize, void * mem)
{
  ARY *matrix;		
  int rows        = lastrow - firstrow + 1;
  int cols        = lastcol - firstcol + 1;

  if ((rows <= 0) || (cols <= 0) || (elSize <= 0 ))
    ErrHandle("Incorrect number of rows/cols or matrix type in makary",-1);


  if (NULL == (matrix = malloc(sizeof (ARY) + rows * (sizeof (void*)))))
    ErrHandle("makary: unable to allocate ary header",-1);

/* set up the newly created matrix */
   matrix->lb1 = firstrow;			/* fill the header  */
   matrix->ub1 = lastrow;
   matrix->lb2 = firstcol;
   matrix->ub2 = lastcol;

   matrix->ary_type = USER_DEFINED;
   matrix->el_size = elSize;

/* build the edge list and return address of matrix	*/
  {
    int  r;

    if (NULL == mem)
      if (NULL == (  mem = malloc(elSize*rows*cols)))
        ErrHandle("makary: unable to allocate ary matrix",-1);

    mem = (char*)mem - ((matrix->lb2) * elSize);
	  /* align it so reference by firstcol is correct */
     
    /* the edge list begins right after the ARY structure	*/
    /* adjust for non-zero minimum index                       */

    matrix->el = (void*) ((void **)(matrix +1) - (matrix->lb1));

    for (r = firstrow; r <= lastrow; ++r)
    {
       matrix->el[r] = mem;
       mem = (char*)mem +  cols * elSize;
    }
  }

  return (matrix);
}

/*-------------  free memory alloced by ARY --------------------*/
void destAry(void * ary)
{
  void * st = startAddrAry((ARY*)ary);
  if(0 != st) free(st);
  free(ary);
}

