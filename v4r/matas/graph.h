#ifndef GR_h
#define GR_h
/*--------------------------------------------------------------------------*/
/* Copyright 1994, George Matas.  

  
  sccs: "@(#)94/05/13 g.matas@ee.surrey.ac.uk 2.2 graph.h"
*/
/*--------------------------------------------------------------------------*/

#include "LL.h"

/*--------------------------------------------------------------------------*/
typedef enum s_typeGR {UNDIRECTED,DIRECTED} t_typeGR;

/*-allow for different attribute types. No attributes if t_att undef -------*/
#define  t_atNGR void *
#define  t_atEGR void *


/*--------------------------------------------------------------------------*/
typedef struct
{
  t_LL nodes;
  t_LL edges;
  t_typeGR type;
} t_graphGR;

/*--------------------------------------------------------------------------*/
typedef struct s_nodeGR
{
  char  * id;
  int   marker;
  t_LL  in;
  t_LL  out;

#ifdef t_atNGR
  t_atNGR att;
#endif
  /* more space (for LL heads ) allocated!!*/  
} * t_nodeGR;

/*--------------------------------------------------------------------------*/
struct s_edgeGR;

typedef struct s_edgeLink 
{
  t_nodeGR          node;
  struct s_edgeGR   *  me;
}*t_edgeLink;

typedef struct s_edgeGR
{
  char*    id;
  int      marker;
  t_linkLL          linkS;
  struct s_edgeLink source;
  t_linkLL          linkT;
  struct s_edgeLink target;   
#ifdef t_atEGR
  t_atNGR att;
#endif
}* t_edgeGR;

/*--------------- basic functions ------------------------------------------*/
t_graphGR ConsGenGR(t_typeGR type);
t_graphGR ConsDirGR(void);
t_graphGR ConsGR(void);

void DestGR(t_graphGR graph);

/*--------------- inserting/deleting nodes and edges ------------------------*/
t_nodeGR  InsNodeGR     (t_graphGR graph);
t_nodeGR  InsNodeIdGR   (t_graphGR graph, char * id);

t_edgeGR InsEdgeGR   (t_graphGR graph, t_nodeGR pN0, t_nodeGR pN1);
t_edgeGR InsEdgeIdGR (t_graphGR graph, t_nodeGR pN0, t_nodeGR pN1,char * id);

t_edgeGR DelEdgeGR(t_edgeGR pEdge);
t_nodeGR DelNodeGR(t_nodeGR pNode);

/*-------------- node info -------------------------------------------------*/
char *  NodeIdGR (t_nodeGR  pNode);
char *  EdgeIdGR (t_edgeGR pEdge);

/*------------- attributes -------------------------------------------------*/
#ifdef t_atNGR
t_nodeGR  InsNodeIdAttGR(t_graphGR graph, char * id, t_atNGR att);
t_nodeGR  InsNodeAttGR  (t_graphGR graph,  t_atNGR att);

t_atNGR   NodeAttGR(t_nodeGR  pNode);
t_atNGR * NodepAttGR(t_nodeGR  pNode);
#endif

#ifdef t_atEGR
t_edgeGR InsEdgeIdAttGR(t_graphGR graph, t_nodeGR pN0, t_nodeGR pN1,
			char * id, t_atEGR att);
t_edgeGR InsEdgeAttGR(t_graphGR graph, t_nodeGR pN0, t_nodeGR pN1, t_atEGR att);

t_atEGR   EdgeAttGR(t_edgeGR pEdge);
t_atEGR * EdgepAttGR(t_edgeGR pEdge);
#endif

void FullPrintGR(t_graphGR graph);
void PrintGR(t_graphGR graph, int nodes, int edges);

t_edgeGR FindEdgeGR(t_graphGR graph, char * id);
t_nodeGR FindNodeGR(t_graphGR graph, char * id);


/*
#define ForAdjEdgesGR_M(pN,pL,pAdj) \
   for(pL=FirstElmLL(pN->out),pAdj=(pL)->me; IsElmLL(pL);\
       pL=NextElmLL(pL),pAdj=(pL)->me)
*/
#define ForAdjEdgesGR_M(pN,pL,pAdj) \
 for(pL=FirstElmLL(pN->out); IsElmLL(pL) && (pAdj=(pL)->me); pL=NextElmLL(pL))

/*
  #define ForAdjNodesGR_M(pN,pL,pAdj) \
   for(pL=FirstElmLL(pN->out),pAdj=(pL)->node; IsElmLL(pL);\
       pL=NextElmLL(pL),pAdj=(pL)->node)
*/
#define ForAdjNodesGR_M(pN,pL,pAdj) \
 for(pL=FirstElmLL(pN->out); IsElmLL(pL) && (pAdj=(pL)->node); \
     pL=NextElmLL(pL))

#define OtherLinkGR_M(pL) \
      ( (&((pL)->me->source)==(pL)) ? &(pL)->me->target : &(pL)->me->source)

#endif
