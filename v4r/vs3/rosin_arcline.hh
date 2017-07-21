#ifndef ROSIN_ARCLINE_HH
#define ROSIN_ARCLINE_HH

namespace Z
{

/**
 * Class implementing the Rosin/West method to segment an curve into lines and
 * circular arcs. Uses the output of LineSegmenter (i.e. a list of lines) and
 * tries to replace some with arcs if it results in a better fit.
 *
 * For details of the curve approximation techniques see the paper:
 *   P. L. Rosin and G. A. W. West,
 *   "Non-Parametric Segmentation of Curves into Various Representations",
 *   IEEE Trans. PAMI, vol 17, pp. 1140-1153, 1995.
 *
 */
class ArcLineSegmenter
{
private:
  static const int NO_LINE_SEGS = 5000;   /* max number of lines per list */
  static const int NO_ARCS = 2000;        /* max number of arcs per list  */

  /* 1 list from x/y_start/end */
  short x_start2[NO_LINE_SEGS], y_start2[NO_LINE_SEGS];
  short x_end2[NO_LINE_SEGS], y_end2[NO_LINE_SEGS];
  float sigs2[NO_LINE_SEGS];
  /* is line ok, deleted or an arc? */
  short flags[NO_LINE_SEGS];
  short location[NO_LINE_SEGS];

  /* x/y_start2 transformed by determine_circle */
  float x_trans3[NO_LINE_SEGS],y_trans3[NO_LINE_SEGS];
  short arc_centre_x[NO_ARCS],arc_centre_y[NO_ARCS];
  short arc_start_x[NO_ARCS],arc_start_y[NO_ARCS];
  short arc_end_x[NO_ARCS],arc_end_y[NO_ARCS];
  int radii[NO_ARCS];
  short arc_dirs[NO_ARCS];
  float arc_sig[NO_ARCS];
  int next[NO_LINE_SEGS];
  int start[NO_LINE_SEGS];

  int number_arcs;
  int number_lists;
  int number_segments;
  int number_segments2;

  /* arc stuff */
  double radius,best_r;
  double error;
  int global_pos;
  int best_y;
  int direction;

  void improve_arcs(int start_in, int finish_in);
  void combine(int previous, int current, int *flag);
  void segment(int start_in, int finish_in, float *sig_out);
  void compute_error(double *error, int y_val);
  void compute_dev_pos(int y_val);
  void compute_dev(int arc_dir, double *max_dev);
  void compute_lgt(double *lgt, int arc_size);
  void compute_poly_lgt(double *lgt);
  double gradient(int position);
  void search(int start_y, int finish_y);
  void determine_circle(int st, int fi, int *final_radius,
      int *final_xc, int *final_yc,
      float *final_dev, int *final_lgt, int *arc_dir);

public:
  void segment_arcs_lines(float x_start[], float y_start[],
      float x_end[], float y_end[], float sigs[], int n_lines);
};

}

#endif

