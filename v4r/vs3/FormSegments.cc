/**
 * @file FormSegments.cc
 * @author Richtsfeld Andreas, Michael Zillich
 * @date 2007, 2010
 * @version 0.1
 * @brief Gestalt principle class for forming segments.
 **/

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

extern "C" {
#include "v4r/matas/canny.h"
#include "v4r/matas/vector2.h"
}
#include "VisionCore.hh"
#include "Color.hh"
#include "Draw.hh"
#include "Segment.hh"
#include "Line.hh"
#include "Arc.hh"
#include "FormSegments.hh"
#include "rosin_lines.hh"
#include "rosin_arcline.hh"

namespace Z
{

IdImage *FormSegments::edge_img = 0;
float *FormSegments::dir_img = 0;
int FormSegments::num_edgels = 0;

static int CmpSegments(const void *a, const void *b)
{
  if( (*(Segment**)a)->sig > (*(Segment**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

/**
 * @brief Constructor of class FormSegments.
 * @param vc Vision core.
 */
FormSegments::FormSegments(VisionCore *vc) : GestaltPrinciple(vc)
{
  done = false;
}

/**
 * @brief Destructor of class FormSegments.
 */
FormSegments::~FormSegments()
{
  delete edge_img;
  delete[] dir_img;
}

/**
 * @brief Reset class FormSegments.
 */
void FormSegments::Reset()
{
  done = false;
  if(core->HaveImage())
  {
    const IplImage *img = core->GetImage();
    if(edge_img != 0 &&
       !(edge_img->width == img->width &&
         edge_img->height == img->height))
    {
      delete edge_img;
      edge_img = 0;
      delete[] dir_img;
      dir_img = 0;
    }
    if(edge_img == 0)
    {
      edge_img = new IdImage(img->width, img->height);
      dir_img = new float[img->width*img->height];
    }
    else
    {
      edge_img->Clear();
      for(int i = 0; i < img->width*img->height; i++)
        dir_img[i] = 0.;
    }
  }
}

/**
 * @brief The first step after acquiring an image is to calculate Canny edges and save \n
 * all edge segments in a segment array.
 * @param incremental Calculate the segments incremental.
 */
void FormSegments::PreOperate()
{
  if(!done)
  {
    StartRunTime();
    Create();
    Rank();
    unsigned n = core->NumGestalts(Gestalt::SEGMENT);
    for(unsigned i = 0; i < n; i++)
      DrawToEdgeImage((Segment*)core->Gestalts(Gestalt::SEGMENT, i));
    done = true;
    StopRunTime();
  }
}

/**
 * @brief Create new segments
 */
void FormSegments::Create()
{
	CreateSegmentsMatas();		// create edges with matas-canny
// 	CreateSegmentsCEdge();			// create edges with openCV canny
  //SegmentArcsLines();
}


/**
 * @brief Segment arc lines
 * TODO obsolete
 */
void FormSegments::SegmentArcsLines()
{
  /* HACK: comment until Line.hh is changed
  const int MAX_PIXELS = 5000;
  const int MAX_LINES = 100;
  // note: list indexing goes from 1 (not 0!) to SIZE-1
  // original list of pixels
  float x_c[MAX_PIXELS];
  float y_c[MAX_PIXELS];
  float x_start[MAX_LINES], y_start[MAX_LINES];
  float x_end[MAX_LINES], y_end[MAX_LINES];
  float sigs[MAX_LINES];
  int line_start[MAX_LINES], line_end[MAX_LINES];
  int n_lines = MAX_LINES;
  LineSegmenter ls;
  ArcLineSegmenter als;

  for(unsigned rank = 0; rank < NumSegments(); rank++)
  {
    Segment *seg = core->RankedGestalts(Gestalt::SEGMENT, rank);
    for(unsigned i = 1; i <= seg->edgels.Size(); i++)
    {
      x_c[i] = seg->edgels[i - 1].p.x;
      y_c[i] = seg->edgels[i - 1].p.y;
    }
    n_lines = MAX_LINES;
    if(ls.segment_lines(x_c, y_c, seg->edgels.Size(),
        x_start, y_start, x_end, y_end, line_start, line_end, sigs, &n_lines))
    {
      for(int i = 0; i < n_lines; i++)
      {
        // note: lines of length 2 often cause problems
        if(line_end[i] - line_start[i] + 1 >= 3)
          NewGestalt(new Line(seg, line_start[i] - 1, line_end[i] - 1));
      }
      als.segment_arcs_lines(x_start, y_start, x_end, y_end, sigs, n_lines);
    }
  }*/
}

/**
 * @brief Create an image where each edge is drawn with its ID. \n
 * This image is later used for determining the colours left/right of lines.
 * @param seg Segment to draw.
 */
void FormSegments::DrawToEdgeImage(Segment *seg)
{
  for(unsigned i = 0; i < seg->edgels.Size(); i++)
  {
    int x = (int)seg->edgels[i].p.x;
    int y = (int)seg->edgels[i].p.y;
    edge_img->SetPixel(x, y, seg->ID());
    dir_img[edge_img->width*y + x] = seg->edgels[i].dir;
  }
}

/**
 * @brief Canny edge detection using code by Matas et al.
 */
void FormSegments::CreateSegmentsMatas()
{
//  float alpha = 1.00;  // filter width (these are "good" values) 1.00, 1.50
//   float alpha = 0.800;  // filter width (these are "good" values) 1.00, 1.50
//   float omega = 0.001; // filter parameter  0.001
  float high_thresh, low_thresh;
  const IplImage *iplimg = core->GetImage();
  FARY *grey_img = 0;
  FARY *grad_dir = 0;
  FARY *grad_mag = 0;
  BARY *edge_img = 0;
  t_graphGR edge_graph;
  void *void_edge;
  Array<Edgel> edgel_arr;
  double avg_len = 0.;

  if(iplimg == 0)
    throw std::runtime_error("FormSegments::CreateSegmentsMatas: Vision core does not have an image.\n");

  // calculate Canny edges
  grey_img = makFary(iplimg->height, iplimg->width);
  for(int y = 0; y < iplimg->height; y++)
    for(int x = 0; x < iplimg->width; x++)
    {
      RGBColor *pix = (RGBColor*)&iplimg->imageData[y*iplimg->widthStep +
          x*iplimg->nChannels];
      grey_img->el[y][x] = (float)(pix->r + pix->g + pix->b)/3.;
    }
  grad_dir = makFary(iplimg->height, iplimg->width);
  grad_mag = FindGradientDir(grey_img, alpha, omega, grad_dir,
                             &high_thresh, &low_thresh);
  edge_img = HysteresisThresh(grad_mag, high_thresh, low_thresh);
  edge_graph = LinkEdges(edge_img);

  // first calculate edgel probability, i.e. the probability that a pixel is an
  // edgel: total number of edgels by total number of pixels.
  // (needed for calculation of significances of newly created edges)
  num_edgels = 0;
  ForeachLL_M(edge_graph.edges, void_edge)
  {
    t_edgeGR edge = (t_edgeGR)void_edge;
    t_LL edgels = (t_LL)edge->att;
    void *void_edgel;
    ForeachLL_M(edgels, void_edgel)
      num_edgels++;
  }
  core->p_e = (double)num_edgels/(double)(iplimg->height*iplimg->width);

  // create segments from edge graph
  ForeachLL_M(edge_graph.edges, void_edge)
  {
    t_edgeGR edge = (t_edgeGR)void_edge;
    t_LL edgels = (t_LL)edge->att;
    void *void_edgel;
    t_V2i *edgel;
    unsigned i = 0;
    edgel_arr.Resize(SizeLL(edgels));
    ForeachLL_M(edgels, void_edgel)
    {
      edgel = (t_V2i*)void_edgel;
      // note that we swap x and y! (the Canny implementation uses a strange
      // co-ordinate system)
      edgel_arr[i].p.x = edgel->y;
      edgel_arr[i].p.y = edgel->x;
      edgel_arr[i].dir = ScaleAngle_0_2pi(grad_dir->el[edgel->x][edgel->y]);
      i++;
    }
    avg_len += edgel_arr.Size();
    core->NewGestalt(GestaltPrinciple::FORM_SEGMENTS, new Segment(core, edgel_arr));
  }
  // probability of an edgel, given that the previous pixel was an edgel
  avg_len /= core->NumGestalts(Gestalt::SEGMENT);
  core->p_ee = 1. - 1./avg_len;

  destAry((ARY*)grey_img);
  destAry((ARY*)grad_dir);
  destAry((ARY*)grad_mag);
  destAry((ARY*)edge_img);
  DestGR(edge_graph);
}

/**
 * @brief Canny edge detection using openCV library.
 * Johann Prankl, 20091111
 */
void FormSegments::CreateSegmentsCEdge()
{
  const IplImage *himg = core->GetImage();
  double avg_len = 0.;

  IplImage *edges = cvCreateImage(cvGetSize(himg), IPL_DEPTH_8U, 1 );
  IplImage *dx = cvCreateImage(cvGetSize(himg), IPL_DEPTH_16S, 1 );
  IplImage *dy = cvCreateImage(cvGetSize(himg), IPL_DEPTH_16S, 1 );

  Array<Array<Edgel> *> segs;

  computeEdges.Set(true, 3);														/// COLOR
  computeEdges.Sobel((IplImage*) himg, dx, dy);
  computeEdges.Canny(dx, dy, edges, 30, 140);						/// CANNY PARAMETER => good: 70, 140
  computeEdges.LinkEdge(edges,dx, dy, segs);
  
  for (unsigned i=0; i<segs.Size(); i++)
  {
    avg_len += (segs[i]->Size());
    core->NewGestalt(GestaltPrinciple::FORM_SEGMENTS, new Segment(core, *segs[i]));
  }

  // probability of an edgel, given that the previous pixel was an edgel
  avg_len /= NumSegments(core);
  core->p_ee = 1. - 1./avg_len;

  for (unsigned i=0; i<segs.Size(); i++)
    delete segs[i];
  segs.Clear();

  cvReleaseImage(&edges);
  cvReleaseImage(&dx);
  cvReleaseImage(&dy);
}


/**
 * @brief Rank the estimated segments.
 */
void FormSegments::Rank()
{
  RankGestalts(Gestalt::SEGMENT, CmpSegments);
}

/**
 * @brief Set the alpha and omega value for the edge detector. \n
 * @param a Alpha value
 * @param o Omega value
 */
void FormSegments::SetCanny(float a, float o)
{
	alpha = a;
	omega = o;
}
}

