/*---------- Hysteresis thres. and link -------------------------------- */
/*  author: G. Matas                           (g.matas@ee.surrey.ac.uk) */
/* +-------------------------------------------------------------------+ */
/* | Copyright 1992, George Matas.                                     | */
/* |   Permission to use, copy, modify, and distribute this software   | */
/* |   and its documentation for any purpose and without fee is hereby | */
/* |   granted, provided that the above copyright notice appear in all | */
/* |   copies and that both that copyright notice and this permission  | */
/* |   notice appear in supporting documentation.  This software is    | */
/* |   provided "as is" without express or implied warranty.           | */
/* +-------------------------------------------------------------------+
*/

/*
      G.Matas, 16-Nov-92
       - created
*/

#include <stdio.h>
#include "canny.h"

enum pixelStates {BELLOW_THR, ABOVE_THR, JUNCT,PROC_STR};

#define TWO_BIT_MASK 3
#define THREE_BIT_MASK 7

/*-------------- direction offsets to neighbouring pixels -------------*/
typedef enum {N,NE,E,SE,S,SW,W,NW} t_directions  ;
#define NUMBER_OF_DIR 8

static int i_ofset[NUMBER_OF_DIR] = { 0, 1, 1, 1, 0,-1,-1,-1};
static int j_ofset[NUMBER_OF_DIR] = {-1,-1, 0, 1, 1, 1, 0,-1};

static t_directions OppositeDir(t_directions dir)
{
  return (t_directions)(((int)dir+NUMBER_OF_DIR/2)%NUMBER_OF_DIR); 
                            /*let's make it general.. */
}

/*-------- store/retrieve direction in one byte -----------------------*/
/* #ifdef DEBUG */
#if 1
static t_directions GetDir(unsigned char source,  int number) 
{
/*   int s = source; */
  if     (number == 0) 
    return (t_directions)( (source>>2)&THREE_BIT_MASK);
  else if(number == 1)
    return (t_directions)(  (source>>5)&THREE_BIT_MASK);

  fprintf(stderr,"trouble in GetDir, number= %d\n",number); exit(-1);
  return (t_directions)0;  /* unreachable, just to shut warnings on SGI */
}

static void PutDir(unsigned char *dest, t_directions dir, int number)
{
  if     (number == 0) (*dest)|= (dir<<2);
  else if(number == 1) (*dest)|= (dir<<5);
  else   { fprintf(stderr,"trouble in PutDir, number= %d\n",number);exit(-1);}
}  

#else
/* fast macros for GetDir and PutDir used after debugging */
/* not tested yet! */
#define GetDir(source,number)   ((source)>>((number)==0?2:5))
#define PutDir(dest,dir,number) ((*dest)|=(dir<<((number)==0?2:5)))
#endif

/*------- use 2 lower bits to store pixel status -----------------*/
static enum pixelStates GetState(unsigned char pixel)
{
  return (enum pixelStates)(pixel & TWO_BIT_MASK);
}
 
static void PutState(unsigned char * pixel, enum pixelStates State)
{
  *pixel = (*pixel & ~TWO_BIT_MASK)|State;
}


/*--------------- hysteresis thresholding part ------------------*/

/*---------------- define a user stack --------------------------*/
/* the depth of the stack is at most the number of junctions in  */
/* a connected component                                         */

#define MAX_DEPTH 1000
static t_i2D stack[MAX_DEPTH];
static int  top = -1;
static int   maxDiagnostic = 0;

static void InitStack() {top = -1;}  /* not used, stack is empty after proc. */

static void PushStack(int i, int j) 
{
  if(++top >= MAX_DEPTH) 
  {
    fprintf(stderr,"user stack overflow\n"); exit(-10);
  }
  if (top > maxDiagnostic)
  {
    if (0==maxDiagnostic && debug_general)
      fprintf(stderr,"user stack max depth: ");
    maxDiagnostic = top;
    if (debug_general) fprintf(stderr,"%d ",maxDiagnostic);
  }
  stack[top].x  = i;
  stack[top].y  = j;
}

static int PopStack(int *i, int *j)
{
  if (top < 0)  return 0;
  *i = stack[top].x;
  *j = stack[top].y;
  top--;

  return 1;
}

/*---------------- label a connected set of points --------------------------*/
#define IsEdge(di,dj)      (g->el[i+(di)][j+(dj)]>lr_thr)
#define NotExpanded(i,j)   (GetState(c->el[(i)][(j)]) == BELLOW_THR)
static int In(int di, int dj, int i, int j,BARY*c)
{
  return (di+i<=c->ub1 && di+i>=c->lb1 && dj+j<=c->ub2 && dj+j>=c->lb2 ) ;
}

static void GetContour(int i, int j, BARY * c, FARY * g, double lr_thr)
{
  t_directions dir[NUMBER_OF_DIR];
 
  InitStack();
  PushStack(i,j);
  PutState(&c->el[i][j],ABOVE_THR);
  while (PopStack(&i,&j))
  {
    int neighbours = 0;
    int ne,se,sw,nw;
    ne=se=sw=nw=0;

    if (In( 1,0,i,j,c) && IsEdge( 1, 0)) { dir[neighbours++]=E; ne=se=1;}
    if (In( 0,1,i,j,c) && IsEdge( 0, 1)) { dir[neighbours++]=S; se=sw=1;}
    if (In(-1,0,i,j,c) && IsEdge(-1, 0)) { dir[neighbours++]=W; sw=nw=1;}
    if (In(0,-1,i,j,c) && IsEdge( 0,-1)) { dir[neighbours++]=N; nw=ne=1;}

    if (In( 1, 1,i,j,c) && se==0 && IsEdge( 1, 1)) dir[neighbours++]=SE;
    if (In( 1,-1,i,j,c) && ne==0 && IsEdge( 1,-1)) dir[neighbours++]=NE;
    if (In(-1, 1,i,j,c) && sw==0 && IsEdge(-1, 1)) dir[neighbours++]=SW;
    if (In(-1,-1,i,j,c) && nw==0 && IsEdge(-1,-1)) dir[neighbours++]=NW;
   
/* put all neighbours not expanded yet stored in dir on stack */
    {
      int k, i_next, j_next;
      for(k=0; k<neighbours; k++)
      {
	i_next = i+i_ofset[dir[k]];
	j_next = j+j_ofset[dir[k]];
        if (NotExpanded(i_next,j_next))
        {
	  PushStack(i_next, j_next);
	  PutState(&c->el[i_next][j_next], ABOVE_THR);
        }
      }
    }

/* label edge point according to the number of neighbours */

    if (neighbours!=2)
    /*if (  neighboursr==1 || neighbours>2 )*/
   /* note that this leaves out single points */
      PutState(&c->el[i][j],JUNCT);
    else
    {
      PutDir(&c->el[i][j],dir[0],0);
      PutDir(&c->el[i][j],dir[1],1);
    } 
   }
}


/* -----------------main of hysteresis thresholding.  -----------------------*/
/*
 *  1. Look  for and mark pixels above upper threshold up_thr
 *  2. mark all pixels 8-connected to 1. above lower threshold
*/

BARY *HysteresisThresh (FARY *grad, double up_thr, double down_thr)
{
  int lines  = grad->ub1 + 1;
  int columns= grad->ub2 + 1;
  BARY* c    = makBary(lines,columns); /* allocate space for label image */

  {
    int i1,j1;
    
    InitialiseBary(c,BELLOW_THR);
    
/* WARNING: Instead of InitialiseBary a loop using PutState(..) */
/*          should be used to work only on the two bits         */
/*          that are reserved for 'state'                       */
/*          Because the direction info in the upper to bits     */
/*          needs no initialisation per se, the code above      */
/*          sets the it to 0 as a side-effect                   */

/* don't create frame, handle in linking */
/*
   for(i1=0;i1<lines;i1+=lines-1)
     for(j1=0; j1<columns; j1++) 
       PutState(&c->el[i1][j1],PROC_STR);


   for(j1=0;j1<columns;j1+=columns-1)
     for(i1=0; i1<lines; i1++) 
       PutState(&c->el[i1][j1],PROC_STR);
*/

    for(i1=0; i1<lines; i1++)
      for(j1=0; j1<columns; j1++)
	if(grad->el[i1][j1]>up_thr && GetState(c->el[i1][j1])==BELLOW_THR)
	  GetContour(i1,j1,c,grad,down_thr);
  }

  return c;
}

/*------------ Link into strings, create a graph of strings   ----*/ 

/*------- auxilary functions ----------*/
static void InsPoint(int  dir,t_LL string,int i, int j)
{
  t_i2D point;

  point.x=i;
  point.y=j;
  if(0==dir) InsLastLL(string,point);
  else       InsFirstLL(string,point);
}

static 
t_nodeGR ExistingOrNewNode(t_graphGR graph, PARY * nodePts, int i,  int j)
{
   if(NULL == nodePts->el[i][j]) 
	 {
	   t_i2D point;
     t_nodeGR node = nodePts->el[i][j]   = InsNodeGR(graph);

	   node->att           = ConsLL();
	   point.x = i, point.y = j;
	   InsLastLL((t_LL)(node->att),point);
   }
   return nodePts->el[i][j];
}

static 
void InsEdge(t_graphGR graph, PARY* nodePts, int i,  int j, int di, int dj)
{
  t_nodeGR center = ExistingOrNewNode(graph,nodePts,i,j);
  t_nodeGR other  = ExistingOrNewNode(graph,nodePts,i+di,j+dj);

  t_LL     string_c = ConsCopyLL((t_LL)(center->att));
  t_LL     string_o = ConsCopyLL((t_LL)(other->att) );
  MoveListLastLL(string_c,string_o);

  InsEdgeAttGR(graph,center,other,string_c);
  DestLL(string_o);
}

/*------- the linking proper --------*/
/* This routine doesn't output strings that have no inner pts */
/* isolated pts and point pairs (in 8-neigh)( are not output  */

t_graphGR LinkEdges(BARY * l)
{
   return LinkEdgesMask(l,1);
}

t_graphGR LinkEdgesMask(BARY * l, int mask)
/* 2. detect junctions - pts with 1 and 3,4 .. more neighbours */
{
  t_graphGR graph=ConsGR();
  PARY * nodePts = makPary(rowsAry(l),colsAry(l));
  InitialisePary(nodePts,NULL);

  {
    int i,j;
    t_nodeGR node[2];

    for(i=0; i<l->ub1 + 1; i++)
     for(j=0; j<l->ub2 + 1; j++)
       if( GetState(l->el[i][j]) == ABOVE_THR)
	 {
	   int dir_num;
	   t_LL string = ConsLL();

	   InsPoint(0,string,i,j);
	   PutState(&l->el[i][j],PROC_STR);
	   for(dir_num=0; dir_num<=1; dir_num++)
	   {
	     int i1, j1;
	     t_directions dir = GetDir(l->el[i][j],dir_num);

	     i1 = i + i_ofset[dir];
	     j1 = j + j_ofset[dir];
	     while (GetState(l->el[i1][j1]) == ABOVE_THR)
	     {
	       t_directions old_dir = dir;

	       PutState(&l->el[i1][j1],PROC_STR);
	       InsPoint(dir_num,string,i1,j1);

	       dir = GetDir(l->el[i1][j1],0);
	       if (dir == OppositeDir(old_dir)) dir = GetDir(l->el[i1][j1],1);

	       i1 += i_ofset[dir];
	       j1 += j_ofset[dir];
	     }
	     /* append the junction point */
	     if (GetState(l->el[i1][j1]) != PROC_STR)  
	       InsPoint(dir_num,string,i1,j1);       
       else { /* for closed strings we want one node */
          i1=i;
          j1=j;
       }  
       node[dir_num] = ExistingOrNewNode(graph,nodePts,i1,j1);
	   }
	   InsEdgeAttGR(graph,node[0],node[1],string);
	  /* InsLastLL(allStrings,string); */
	 }   /* end of single edge processing */

   /* we must collect isolated points and JUNCT next to another JUNCT*/
   /* not on edges  of the image */
#define  IsJunction(di,dj)    ( GetState(l->el[i+di][j+dj]) == JUNCT)
   for(i=1; i<l->ub1 ; i++)
     for(j=1; j<l->ub2 ; j++)
       if( GetState(l->el[i][j]) == JUNCT)
	     {
         int ne,se,sw,nw;
         ne=se=sw=nw=0;

         if (IsJunction( 1,  0)) { InsEdge(graph,nodePts,i,j, 1, 0); ne=se=1;}
         if (IsJunction( 0,  1)) { InsEdge(graph,nodePts,i,j, 0, 1); se=sw=1;}
         if (IsJunction(-1,  0)) {  sw=nw=1;}
         if (IsJunction( 0, -1)) {  nw=ne=1;}

         if (se==0 && IsJunction( 1, 1)) InsEdge(graph,nodePts,i,j,  1,  1);
         if (ne==0 && IsJunction( 1,-1)) InsEdge(graph,nodePts,i,j,  1, -1);
         /*
         if (sw==0 && IsJunction(-1, 1)) InsEdge(graph,nodePts,i,j, -1,  1);
         if (nw==0 && IsJunction(-1,-1)) InsEdge(graph,nodePts,i,j, -1, -1);
         */
       }
  }


/* the upper six bit contain information about neigbouring edge direction*/
/* remove the information                                                */
  if(mask)
  {
    int i,j;
    for(i=0; i<l->ub1 + 1; i++)
      for(j=0; j<l->ub2 + 1; j++)
        l->el[i][j]&=TWO_BIT_MASK;
  }

  destAry(nodePts);
  return graph;
}
