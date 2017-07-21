#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <option.h> /* Michael Zillich, 2002-06-11: was optionGM.h */

#include "canny.h"
#include "cannyGradGlob.h"

#define RtoD 57.29578

/* ----------------- global variables ----------------------------------- */
/*   see option parsing for defaults and comments on meaning              */
int debug_general=0, debug_time=0;           /*see options for  defaults*/

char * grad_non_name=0;
char * derivative_name=0;
char * ef_name = 0;
char * direct_name = 0;
int derivatives_only=0;

int Phil_on=0;  
int four_con=0, mag_out=0, not_all_junct=0;
int fix_thresh=0;
static int mask = 0;

int RofI[4];        /* xs, ys, xe, ye of the Region of Interest */
int doRofI = 0;

/*-------- local vars  --------------------*/

/*------- load the image ---------------------------------------*/
FARY* ReadImage(char * in_file)
{
  ExitOnAryError();
  return Pgm2Fary(in_file);
}

/*----- filtering --------------------------------------------------*/

/*----- Thresholding and linking -----------------------------------*/
/*----- Output  -----------------------------------*/
void Output(BARY * edge_image, t_graphGR graph, 
            char* out_file,char* gf_name,char* ll_name)
{
  if (NULL !=ll_name)
  {
    t_edgeGR pEdge;
    t_LL edges = ConsLL();
    ForeachLL_M(graph.edges,pEdge)
    InsLastLL(edges,pEdge->att);

    WriteLev2LL(ll_name,edges);
  }

  if (NULL != out_file) Bary2P5pgm(edge_image,out_file);
}

 

/*----- Output  -----------------------------------*/
int Canny(int argc, char **argv, int options_only)  
{ 
  OptionInit(argv,&argc);          /* analyse the command line */

  {
  char *gf_name,*ll_name, *grad_name ;

  /*char *in_file = OptionStr("i",NULL, "input image (P5  pgm)");
  char * out_file = OptionStr("o",NULL, 
	     "output pgm with class. pixels (0-bck 1-string 2-junction)");*/
		    
  int   link        = OptionToggle("link",0,"link (no filter. and non-max)"); 
  /*float alpha       = OptionDouble("a",1.50, "filter width");
  float omega       = OptionDouble("O",0.001,"filter parameter");*/

  /*int stats   = OptionToggle("S",0,"don't update the stats file");*/
  fix_thresh  = OptionToggle("U",0,"don't use automatic thresh.");

  ef_name     = OptionStr("e",NULL,"output pre-link file");
  gf_name     = OptionStr("g",NULL,"output .gf file");
  ll_name     = OptionStr("L",NULL,"output .LL file");

  Phil_on     = OptionToggle("p",0,"output .gf in Phil Palmer format");
  direct_name = OptionStr("d",NULL,"output gradient direction in .gf");
  mag_out     = OptionToggle("m",0,"output gradient magnitude in .gf");
  not_all_junct = OptionToggle("nj",0,"don't output ends as juncts");

  debug_general= OptionToggle("I",0,"debug info on");
  debug_time   = OptionToggle("t",0,"timming info on");
  four_con     = OptionToggle("F",0,"allow four connectivity");

  mask           = OptionToggle("nomask",1,"do not mask relative directions");

  grad_non_name  =OptionStr("N",NULL,"output a file with non-max gradient");
  grad_name      =OptionStr("G",NULL,"output a file with gradient");
  derivative_name=OptionStr("D",NULL,"output a files with derivatives");
  derivatives_only=OptionToggle("Donly",0,"exit after computing derivatives");

  OptionIntArr("R",RofI,4,"Region of Interest Definition (corner-corner)");
  doRofI = OptionOnCommLine("R");

  if (options_only) return 1;


  OptionCompulsory("i");
  OptionDependXor("p d");
  OptionMultIf(!link,"a O l u U p d N G R","-link not specified");
  

   OptionCheck();
#ifdef NO_GF
	  fprintf(stderr, "Compiled with NO_GF; -d, -p, -g option ignored\n");
#endif

  /*-------------- processing ----------------------*/
  } /* of option parsing */
  return 0;
}
