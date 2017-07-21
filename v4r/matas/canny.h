#include "ary.h"
#include "LL.h"
#include "graph.h"

typedef struct {
		int x,y;
}t_i2D;


extern int debug_general;
extern int not_all_junct;

#include "cannyGradGlob.h" /*global variables set by Option handling used in FindGradient*/

int Canny(int arc, char **argv, int option_only);

/*------------ low level functions --------------------------*/
FARY* ReadImage(char * in_file);
FARY * Filter(FARY *in,float alpha, float omega,char* grad_name);
BARY* ThreshLink(FARY* grad_mag, t_graphGR *graph);
void Output(BARY * edge_image, t_graphGR graph, 
            char* out_file,char* gf_name,char* ll_name);

/*------------ low level functions --------------------------*/
BARY     *HysteresisThresh(FARY *grad, double up_thr, double down_thr);
FARY     *FindGradient(FARY *image, float alpha, float omega,
                       float *high_tresh, float *low_thresh);
FARY     *FindGradientDir(FARY *image, float alpha, float omega,FARY* dir,
                          float *high_tresh, float *low_thresh);
t_graphGR LinkEdges(BARY * edgeImage);
t_graphGR LinkEdgesMask(BARY * l, int mask);


void ResetSetHighLowThresh(void) ;
void SetHighLowThresh(float hight, float low);

#ifndef NO_GF
#include "gfLL.h"
void WriteStringsInGF(char * name,t_graphGR graph, FARY * grad_dir);
void WriteStringsSiInGF(char * name,t_graphGR graph, FARY * grad_dir, int si);
#endif


