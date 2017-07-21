#ifndef ROSIN_LINES_HH
#define ROSIN_LINES_HH

// #include "Namespace.hh"
#include "VisionCore.hh"
#include "Segment.hh"

namespace Z
{

/**
 * Class implementing the Rosin/West method to segment an curve into lines.
 *
 * For details of the curve approximation techniques see the paper:
 *   P. L. Rosin and G. A. W. West,
 *   "Non-Parametric Segmentation of Curves into Various Representations",
 *   IEEE Trans. PAMI, vol 17, pp. 1140-1153, 1995.
 *
 */
class LineSegmenter
{
private:
  VisionCore *core;

  static void segment(float x_c[], float y_c[], float x_c2[], float y_c2[],
      float sigs[], float sums[], int flags[],
      int start_in,int finish_in,float *sig_out,float *sum_out);
  static void transform(float x_c[], float y_c[], float x_c2[], float y_c2[],
      int start, int finish, int *end2);
  static void deviation(float y_c2[], int end2,
      int *pos,float *dev,int *ok,float *sum);
  static void improve_lines(float x_c[], float y_c[], float x_c2[], float y_c2[],
      float sigs[], int flags[], int number_pixels, int start_in,int finish_in);
  static void combine(float x_c[], float y_c[], float x_c2[], float y_c2[],
      float sigs[], int flags[], int number_pixels,
      int v1, int v2, int v3, int v4, int *flag);

public:
  LineSegmenter(VisionCore *vc) : core(vc) {}
  void RosinFitLinesToSegment(Segment *seg);
  bool segment_lines(float x_c[], float y_c[], int n_pixels,
      float x_start[], float y_start[], float x_end[], float y_end[],
      int i_start[], int i_end[], float sigs[], int *n_lines);
};

}

#endif

