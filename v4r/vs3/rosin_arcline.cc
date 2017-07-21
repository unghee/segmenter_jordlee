/*
    arcline.c

    A P.L. Rosin/G.A.W.West production. Copyright 1987/1988/1991.
    This version of arcs outputs super data that is a list of
    image features with their parameters.
    Works on one set of connected lines at a time to reduce
    memory space required.
    Now only reads data from a super data file.
    Super data includes sigs.
    This version has two passes.
    First pass does the classic arcs and lines segmentation.
    Second pass tries to grow arcs by combining neighbouring arcs and
    lines.
    There is a weight value that can be used to force the growing
    process. - GAWW August 1991.

*/

#include <stdio.h>
#include <math.h>
#include "rosin_arcline.hh"

#define LINE 1
#define ARC 2
#define IGNORE 3
#define SMALL 1             /* type of arc */
#define BIG 2
#define CLOCKWISE 1         /* direction of arc from start to finish */
#define ANTICLOCKWISE 2

#define DEBUG 0            /* compile debug output if 1 */

#define LISTS 1
#define LINES 2
#define ARCS 3
#define ENDL 4
#define ENDF 5


#define FIRST 1
#define SECOND 2
#define THIRD 3

#define FALSE 0
#define TRUE !FALSE

namespace Z
{

/* in improve_arcs, weights bias decision */
static const int USE_IMPROVE = TRUE;
/* if weight < 1.0 then favours bigger new arcs use_weight enables a user to
 * input a value */
static const float WEIGHT = 1.;

static int abs_i(int value)
{
    if (value < 0) value = -value;
    return(value);
}

static int round_i(double value)
{
    int i = (int)floor(value + 0.5);
    return(i);
}

static float angle(float x1, float y1, float x2, float y2)
{
    float angle_temp;
    float xx,yy;

    xx = x2 - x1;
    yy = y2 - y1;
    if (xx == 0.0)
    angle_temp = M_PI/2.0;
    else
    angle_temp = atan(fabs(yy/xx));
    if ((xx < 0.0) && (yy >= 0.0))
    angle_temp = M_PI - angle_temp;
    else if ((xx < 0.0) && (yy < 0.0))
    angle_temp = M_PI + angle_temp;
    else if ((xx >= 0.0) && ( yy < 0.0))
    angle_temp = M_PI*2.0 - angle_temp;
    return(angle_temp);
}

static double euclid(float x1, float y1, float x2, float y2)
{
    double temp1,temp2;
    double dist;

    temp1 = fabs(x1-x2);
    temp2 = fabs(y1-y2);
    dist = sqrt(temp1 * temp1 + temp2 * temp2);
    return(dist);
}

static void compute_minimum_diff(float x_cir, float y_cir, float x1, float y1,
    float x2, float y2, double *deviation)
{
    /* rotate polygon segment to lie along x axis,
       compute the minimum distance between the line segment and
       the circle.
       take perpendicular distance if x_cir between end points else
       take euclidian distance to nearest end point.
    */
    float angle1;
    float x_off,y_off,cosine,sine;
    float temp;
    float min1,min2;

    angle1 = angle(x1,y1,x2,y2);
    angle1 = -angle1;
    cosine = cos(angle1);
    sine = sin(angle1);
    x_off = x1;
    y_off = y1;
    /* offset points so x1,y1 at origin */
    x1 = 0;
    y1 = 0;
    x2 = x2 - x_off;
    y2 = y2 - y_off;
    x_cir = x_cir - x_off;
    y_cir = y_cir - y_off;
    /* rotate points with x2,y2 on x axis */
    temp = x2*cosine - y2*sine;
    y2 = x2*sine + y2*cosine;
    x2 = temp;
    temp = x_cir*cosine - y_cir*sine;
    y_cir = x_cir*sine + y_cir*cosine;
    x_cir = temp;

    min1 = euclid(x_cir,y_cir,x1,y1);
    min2 = euclid(x_cir,y_cir,x2,y2);
    if (x_cir < x1) {
        *deviation = min1;
    }else if (x_cir > x2) {
        *deviation = min2;
    }else{
        *deviation = fabs(y_cir);
    }
}

#if 0
static RosinFitArcLinesToSegment(unsigned seg)
{
  const int MAX_LINES = 100;
  /* note: list indexing goes from 1 (not 0!) to SIZE-1 */
  /* original list of pixels */
  float x_c[Segments(seg)->edgels.Size() + 1];
  float y_c[Segments(seg)->edgels.Size() + 1];
  /* sequence of ordered lists of line segments */
  float x_start[MAX_LINES], y_start[MAX_LINES];
  float x_end[MAX_LINES], y_end[MAX_LINES];
  float sigs[MAX_LINES];
  int line_start[MAX_LINES], line_end[MAX_LINES];
  int n_lines = MAX_LINES;

  for(unsigned i = 1; i <= Z::Segments(seg)->edgels.Size(); i++)
  {
    x_c[i] = Segments(seg)->edgels[i - 1].p.x;
    y_c[i] = Segments(seg)->edgels[i - 1].p.y;
  }
  if(segment_lines(x_c, y_c, Segments(seg)->edgels.Size(),
      x_start, y_start, x_end, y_end, line_start, line_end, sigs, &n_lines))
  {
    for(int i = 0; i < n_lines; i++)
    {
      // note: lines of length 2 often cause problems
      if(line_end[i] - line_start[i] + 1 >= 3)
        NewGestalt(new Line(seg, line_start[i] - 1, line_end[i] - 1));
    }
  }
}
#endif

void ArcLineSegmenter::segment_arcs_lines(float x_start[], float y_start[],
    float x_end[], float y_end[], float sigs[], int n_lines)
{
    int i,j;
    int index;
    float sig;    /* not really used - just dummy param for segment */

    // note:  LineSegmenter::segment_lines() returns arrays starting at 0,
    // the following code assumes arrays starting at 1
    for (i = 1; i <= n_lines;i++) {
        x_start2[i] = (short)x_start[i-1];
        y_start2[i] = (short)y_start[i-1];
        x_end2[i] = (short)x_end[i-1];
        y_end2[i] = (short)y_end[i-1];
        sigs2[i] = sigs[i-1];
    }
    number_segments = n_lines;
    number_arcs = 0;
    for (i = 1;i<= number_segments;i++)
        flags[i] = LINE;
#if DEBUG
    printf("INITIALLY calling segment\n");
#endif
    segment(1,number_segments,&sig);

    if (USE_IMPROVE == TRUE)
        improve_arcs(1,number_segments);

    /* write out data in super data format */
    for (j = 1;j <= number_segments; j++) {
        if (flags[j] == LINE) {
            printf("line: %f %d %d %d %d\n",
            sigs2[j],x_start2[j],y_start2[j],x_end2[j],y_end2[j]);
        }
        else if (flags[j] == ARC) {
            index = location[j];
            printf("arc: %f %d %d %d %d %d %d %d %d\n",
            arc_sig[index],
            arc_centre_x[index],arc_centre_y[index],
            arc_start_x[index],arc_start_y[index],
            arc_end_x[index],arc_end_y[index],
            radii[index],arc_dirs[index]);
        }
    }
}

/*
    For each arc in representation, tries to combine with each neighbour.
    Four choices:
        Neighbour on right
        Neighbour on left
        Both neighbours
        Neither neighbours
    Chooses best representation.
    Repeats until can do no more.
*/
void ArcLineSegmenter::improve_arcs(int start_in, int finish_in)
{
    int loop1;
    int cont,cont2;
    int previous,current;

    /* first segment will always be either ARC of LINE */
    if ((flags[1] != ARC) && (flags[1] != LINE)) {
        printf("ERROR - first segment should always ARC or LINE\n");
        return;
    }
    do {
        loop1 = start_in;
        previous = start_in;
        cont = TRUE;
        cont2 = FALSE;
        do {
            if ((flags[loop1] == ARC) || (flags[loop1] == LINE)) {
                current = loop1;
                if (flags[current] == ARC) {
                    if ((flags[previous] == LINE) || (flags[previous] == ARC)) {
                        combine(previous,current,&cont2);
                        /* print_reps(); */
                    }
                }
                previous = current;
            }
            loop1++;
        } while ((loop1 <= finish_in) && (cont == TRUE));
        if (cont2 == FALSE)
            cont = FALSE;
    } while (cont == TRUE);
}

void ArcLineSegmenter::combine(int previous, int current, int *flag)
{
    int radius1,centre_x1,centre_y1,arc_dir1;
    int radius2,centre_x2,centre_y2,arc_dir2;
    int radius3,centre_x3,centre_y3,arc_dir3;
    int arc_length;
    float max_dev,sig[4];
    int index1,index2;
    int next,next2;
    int best[4];
    float sigp,sigc,sign;
    int i,j;
    int replaced;

#if DEBUG
    printf("into combine\n");
#endif
    
    /*
       find next representation - if cant find one, set to current
       also used (if current == ARC) to determine last line segment to
       use for fitting arc to
    */
    
    next = current;
    do {
        next++;
    } while ((flags[next] != LINE) && (flags[next] != ARC) 
            && (next < number_segments));
    if (next == number_segments)
        next = current;

    /* combine with previous representation? */
    
    if (previous == current)
        sig[1] = 9999;
    else{
        /*
        printf("combining previous %d,%d current %d,%d\n",
                previous,flags[previous],current,flags[current]);
        */
        determine_circle(previous,next-1,
                &radius1,&centre_x1,&centre_y1,
                &max_dev,&arc_length,&arc_dir1);
#if DEBUG
        printf("circle? rad %d ctr %d %d dev %f lgt %d size %d\n",
            radius1,centre_x1,centre_y1,max_dev,arc_length,arc_dir1);
#endif
        if (max_dev > 0)
            sig[1] = (max_dev / (float)arc_length); /* try weighting arcs */
        else
            sig[1] = 9999;
        if (flags[previous] == ARC)
            sigp = arc_sig[location[previous]]; /* arc significance */
        else
            sigp = sigs2[previous];               /* line significance */
        sigc = arc_sig[location[current]];      /* arc significance */
    }
    
    /* combine with next representation? */
    
    if (next == current)
        sig[2] = 9999;
    else{
        if (flags[next] == ARC) {
            /* 
                find next representation to get last line segment to use 
                if line then it is just the next segment
                if arc then it is the next representation-1
                if next is last segment then it is number_segments
            */
            next2 = next;
            do {
                next2++;
            } while ((flags[next2] != LINE) && (flags[next2] != ARC) 
                    && (next2 < number_segments));
            if (next2 != number_segments)
                next2--;
        }
        else
            next2 = next;
#if DEBUG
        printf("combining current %d,%d next %d,%d\n",
                current,flags[current],next,flags[next]);
#endif
        determine_circle(current,next2,
                &radius2,&centre_x2,&centre_y2,
                &max_dev,&arc_length,&arc_dir2);
#if DEBUG
        printf("circle? rad %d ctr %d %d dev %f lgt %d size %d\n",
            radius2,centre_x2,centre_y2,max_dev,arc_length,arc_dir2);
#endif
        if (max_dev > 0)
            sig[2] = (max_dev/(float)arc_length); /* try weighting arcs */
        else
            sig[2] = 9999;
        if (flags[next] == ARC)
            sign = arc_sig[location[next]]; /* arc significance */
        else
            sign = sigs2[next];               /* line significance */
        sigc = arc_sig[location[current]];  /* arc significance */
    }

    /* combine with previous and next representation? */

    if ((sig[1] == 9999) || (sig[2] == 9999)) {
        sig[3] = 9999;
    }
    else{
#if DEBUG
        printf("combining previous %d,%d next %d,%d\n",
                previous,flags[previous],next,flags[next]);
#endif
        determine_circle(previous,next2,
                &radius3,&centre_x3,&centre_y3,
                &max_dev,&arc_length,&arc_dir3);
#if DEBUG
        printf("circle? rad %d ctr %d %d dev %f lgt %d size %d\n",
                radius3,centre_x3,centre_y3,max_dev,arc_length,arc_dir3);
#endif
        if (max_dev > 0)
            sig[3] = (max_dev/(float)arc_length); /* try weighting arcs */
        else
            sig[3] = 9999;
    }

#if DEBUG
    printf("old sigs previous: %f current %f next %f\n",
        sigp,sigc,sign);
    printf("new significances: first %f second %f third %f\n",
        sig[1],sig[2],sig[3]);
#endif

    
    /* rank significances */
    /* temporary set significance of THIRD to 9999 to stop it being used
    */
    best[1] = FIRST;
    best[2] = SECOND;
    best[3] = THIRD;
    for (j=1;j<=2;j++)
        for (i=1;i<=2;i++)
            if (sig[i+1] < sig[i]) {
                sig[0] = sig[i];
                sig[i] = sig[i+1];
                sig[i+1] = sig[0];
                best[0] = best[i];
                best[i] = best[i+1];
                best[i+1] = best[0];
            }


    /* 
       Replace original representation if this one better - 
       Complex decision rules:
        if the best sig is better than the old then 
            replace
        else 
            try the next best significance
        This is continued until run out of replacements.
    */
    i = 0;
    replaced = FALSE;
    do {
        i++;
        if (best[i] == THIRD) {
            if ((sig[i]*WEIGHT<sigp) && (sig[i]*WEIGHT<sigc) && (sig[i]*WEIGHT<sign)) {
                /* replace all three previous reps */
                /* replace previous, delete current and next */
                printf("replacing with THIRD\n");
                replaced = TRUE;
                /* delete previous and next, modify current */
                index1 = location[current];
                if (flags[previous] == LINE) {
                    /* delete previous line and modify current arc */
                    printf("deleting line\n");
                    flags[previous] = IGNORE;
                    arc_start_x[index1] = x_start2[previous];
                    arc_start_y[index1] = y_start2[previous];
                }
                else{
                    /* delete previous arc and modify current arc */
                    printf("deleting arc\n");
                    index2 = location[previous];
                    flags[previous] = IGNORE;
                    arc_start_x[index1] = arc_start_x[index2];
                    arc_start_y[index1] = arc_start_y[index2];
                }
                if (flags[next] == LINE) {
                    /* delete next line and modify current arc */
                    printf("deleting line\n");
                    flags[next] = IGNORE;
                    arc_end_x[index1] = x_end2[next];
                    arc_end_y[index1] = y_end2[next];
                }
                else{
                    /* delete next arc and modify current arc */
                    printf("deleting arc\n");
                    flags[next] = IGNORE;
                    index2 = location[next];
                    arc_end_x[index1] = arc_end_x[index2];
                    arc_end_y[index1] = arc_end_y[index2];
                }
                arc_sig[index1] = sig[i];
                arc_centre_x[index1] = centre_x3;
                arc_centre_y[index1] = centre_y3;
                arc_dirs[index1] = arc_dir3;
                radii[index1] = radius3;
            }
        }
        else if (best[i] == FIRST) {
            if ((sig[i]*WEIGHT<sigp) && (sig[i]*WEIGHT<sigc)) {
                /* replace previous and current */
                printf("replacing with FIRST\n");
                replaced = TRUE;
                /* delete previous and modify current */
                index1 = location[current];
                if (flags[previous] == LINE) {
                    /* delete previous line and modify current arc */
                    printf("deleting line\n");
                    flags[previous] = IGNORE;
                    arc_start_x[index1] = x_start2[previous];
                    arc_start_y[index1] = y_start2[previous];
                }
                else{
                    /* delete previous arc and modify current arc */
                    printf("deleting arc\n");
                    index2 = location[previous];
                    flags[previous] = IGNORE;
                    arc_start_x[index1] = arc_start_x[index2];
                    arc_start_y[index1] = arc_start_y[index2];
                }
                arc_sig[index1] = sig[i];
                arc_centre_x[index1] = centre_x1;
                arc_centre_y[index1] = centre_y1;
                arc_dirs[index1] = arc_dir1;
                radii[index1] = radius1;
            }
        }
        else{
            if ((sig[i]*WEIGHT<sigc) && (sig[i]*WEIGHT<sign)) {
                /* replace current and next */
                printf("replacing with SECOND\n");
                replaced = TRUE;
                /* delete next and modify current */
                index1 = location[current];
                if (flags[next] == LINE) {
                    /* delete next line and modify current arc */
                    printf("deleting line\n");
                    flags[next] = IGNORE;
                    arc_end_x[index1] = x_end2[next];
                    arc_end_y[index1] = y_end2[next];
                }
                else{
                    /* delete next arc and modify current arc */
                    printf("deleting arc\n");
                    flags[next] = IGNORE;
                    index2 = location[next];
                    arc_end_x[index1] = arc_end_x[index2];
                    arc_end_y[index1] = arc_end_y[index2];
                }
                arc_sig[index1] = sig[i];
                arc_centre_x[index1] = centre_x2;
                arc_centre_y[index1] = centre_y2;
                arc_dirs[index1] = arc_dir2;
                radii[index1] = radius2;
            }
        }
    } while ((replaced == FALSE) && (i<=3));
    *flag = replaced;
}

void ArcLineSegmenter::segment(int start_in, int finish_in, float *sig_out)
{
    int i;
    int pos;
    float max_dev;
    float sig1,sig2,sig3,best_sig;
    int centre_x, centre_y, radius;
    int arc_length,arc_dir;

#if DEBUG
    printf("entering segment with limits %d %d\n",start_in,finish_in);
#endif
    if ((finish_in - start_in) < 2) {        /* don't subdivide any more */
        best_sig=999999;
        for (i=start_in;i<=finish_in;i++)
            if (sigs2[i]<best_sig)
                best_sig=sigs2[i];
        *sig_out = best_sig;
#if DEBUG
        printf("line sig %f\n",best_sig);
#endif
    }
    else{
        determine_circle(start_in,finish_in,
            &radius,&centre_x,&centre_y,&max_dev,&arc_length,&arc_dir);

#if DEBUG
        printf("circle? rad %d ctr %d %d dev %f lgt %d size %d\n",
        radius,centre_x,centre_y,max_dev,arc_length,arc_dir);
#endif
        pos = global_pos + start_in - 1;
        if (max_dev > 0)
            sig1 = (max_dev / (float)arc_length); /* try weighting arcs */
        else
            sig1 = 9999;
#if DEBUG
        printf("circle sig %f\n",sig1);
#endif
        segment(start_in,pos-1,&sig2);
        segment(pos,finish_in,&sig3);
        if (sig2 < sig3)
            best_sig = sig2;
        else
            best_sig = sig3;
        if (best_sig < sig1)
            *sig_out = best_sig;
        else{
            *sig_out = sig1;
            number_arcs++;
            flags[start_in] = ARC;
            arc_sig[number_arcs] = *sig_out;
            arc_centre_x[number_arcs] = centre_x;
            arc_centre_y[number_arcs] = centre_y;
            arc_start_x[number_arcs] = x_start2[start_in];
            arc_start_y[number_arcs] = y_start2[start_in];
            arc_end_x[number_arcs] = x_end2[finish_in];
            arc_end_y[number_arcs] = y_end2[finish_in];
            arc_dirs[number_arcs] = arc_dir;
            radii[number_arcs] = radius;
            location[start_in] = number_arcs;
            for (i = start_in+1;i <= finish_in;i++)
                flags[i] = IGNORE;
        }
    }
}

void ArcLineSegmenter::compute_error(double *error, int y_val)
{
    int loop1;
    double temp1,temp2,temp3,temp4;

    temp4 = 0;
    temp1 = y_val;
    temp1 = temp1 * temp1;
    temp2 = x_trans3[1];
    temp2 = temp2 * temp2;
    temp3 = temp1 + temp2;
    radius = sqrt(temp3);
    for (loop1 = 2; loop1 <= number_segments2 - 1;loop1++) {
       temp1 = x_trans3[loop1];
       temp1 = temp1 * temp1;
       temp2 = y_val - y_trans3[loop1];
       temp2 = temp2 * temp2;
       temp3 = radius - sqrt(temp1+temp2);
       if (temp3 < 0) temp3 = -temp3;
       temp4 = temp4 + temp3;
    }
    *error = temp4;
}

void ArcLineSegmenter::compute_dev_pos(int y_val)
{
    int loop1;
    int pos1,pos2;
    double temp1,temp2,temp3;
    double dev,dev1,dev2;

    temp1 = y_val;
    temp1 = temp1 * temp1;
    temp2 = x_trans3[1];
    temp2 = temp2 * temp2;
    temp3 = temp1 + temp2;
    radius = sqrt(temp3);
    dev1 = 0;
    for (loop1 = 2;loop1 <= number_segments2 - 1;loop1++) {
        temp1 = x_trans3[loop1];
        temp1 = temp1 * temp1;
        temp2 = y_val - y_trans3[loop1];
        temp2 = temp2*temp2;
        dev = radius-sqrt(temp1+temp2);
        if (dev < 0)
            dev= -dev;
        if (dev > dev1) {
            dev1 = dev;
            pos1=loop1;
        }
    }
    dev2 = 0;
    for (loop1 = 1;loop1 <= number_segments2 - 1;loop1++) {
       temp1 = (x_trans3[loop1] + x_trans3[loop1+1]) / 2;
       temp1 = temp1 * temp1;
       temp2 = y_val - (y_trans3[loop1] + y_trans3[loop1+1]) / 2;
       temp2 = temp2 * temp2;
       dev = radius-sqrt(temp1+temp2);
       if (dev < 0) dev= -dev;
       if (dev > dev2) {dev2 = dev; pos2=loop1;}
    }
    global_pos = pos1;
#if DEBUG
    printf("breaking at %d\n",global_pos); 
#endif
}

/*
    ********* version 4.0 **********
    New version that walks around hypothesised circle
    finding minimum distance to polygon at each step.
    Nearest point is either the perpendicular distance,
    or the euclidian distance to the nearest end point.
    For all cases the nearest end point is taken as the
    possible break point.
    approximation, max_dev is returned deviation,
*/
void ArcLineSegmenter::compute_dev(int arc_dir, double *max_dev)
{
    int l2;
    float l1,step;
    float angle1,angle2;
    float x_cir,y_cir,x_cent,y_cent;
    double deviation;
    float min_dev;
    float temp1;

    *max_dev = 0.0;
    x_cent = 0.0;
    y_cent = best_y;
/*
    printf("number of vertices: %d\n",number_segments2);   
    printf("centre: (%f , %f)\n",x_cent,y_cent);
*/
#if DEBUG
    printf("radius: %f\n",best_r);
#endif
/* determine angles */
    angle1 = angle(x_cent,y_cent,x_trans3[1],y_trans3[1]);
    angle2 = angle(x_cent,y_cent,x_trans3[number_segments2],
                   y_trans3[number_segments2]);
#if DEBUG
    printf("%f %f\n",x_trans3[1],y_trans3[1]);
    printf("%f %f\n",x_trans3[number_segments2],y_trans3[number_segments2]);
    printf("angles: %f %f\n",angle1,angle2);
#endif
    if (arc_dir == CLOCKWISE) {
        temp1 = angle1;
        angle1 = angle2;
        angle2 = temp1;
    }
/* ang4 must be bigger than ang3 */
    if (angle2 <= angle1) {
        angle2 = angle2 + M_PI * 2.0;
    }
#if DEBUG
    printf("start angle: %f finish angle %f\n",angle1,angle2);
/* walk around circle from angle1 to angle2 */
    printf("\nwalking around circle\n");
#endif
    step = (angle2-angle1) / 10.0; /* use 11 steps presently */
    l1 = angle1;
    do {
        min_dev = 1000000;
        x_cir = cos(l1) * best_r + x_cent;
        y_cir = sin(l1) * best_r + y_cent;
/*    printf("\nl1: %f (x_cir,y_cir): %f %f\n",l1,x_cir,y_cir);  */
        l2 = 1;
        do {
/*        printf("for line %d\n",l2);   */
            compute_minimum_diff(x_cir,y_cir,
                x_trans3[l2],y_trans3[l2],
                x_trans3[l2+1],y_trans3[l2+1],
                &deviation);
            if (deviation < min_dev) {
                min_dev = deviation;
            }
        l2++;
        } while (l2 <= number_segments2-1);
/*    printf("at this angle minimum deviation: %f at vertex: %d\n",
        min_dev,pos1);  */
        if (min_dev > *max_dev) {
            *max_dev = min_dev;
        }
        l1 += step;
    } while (l1 <= angle2);
#if DEBUG
    printf("maximum deviation: %f\n",*max_dev);
#endif
}

void ArcLineSegmenter::compute_lgt(double *lgt, int arc_size)
{
    double temp1,temp2,temp3;
    double chord,angle;

    temp1 = x_trans3[1] - x_trans3[number_segments2];
    temp1 = temp1 * temp1;
    temp2 = y_trans3[1] - y_trans3[number_segments2];
    temp2 = temp2 * temp2;
    temp3 = temp1 + temp2;
    chord = sqrt(temp3);
    temp3 = (chord / 2.0) / radius;
    if (temp3 > 1.0) temp3 = 1;        /* eliminate rounding errors */
    angle = asin(temp3) * 2;
    if (arc_size == BIG) angle = 2 * M_PI - angle;
    *lgt = angle*radius;
}

void ArcLineSegmenter::compute_poly_lgt(double *lgt)
{
    int loop1;
    double dx,dy;
    double total,dist;

    total = 0.0;
    for (loop1 = 1;loop1 < number_segments2;loop1++) {
        dx = x_trans3[loop1] - x_trans3[loop1+1];
        dy = y_trans3[loop1] - y_trans3[loop1+1];
        dist = dx * dx + dy * dy;
        dist = sqrt(dist);
        total += dist;
    }
    *lgt = total;
}

double ArcLineSegmenter::gradient(int position)
{
    double error1,error2,diff;

    compute_error(&error1,position-direction);
    compute_error(&error2,position+direction);
    diff = error1 - error2;
    return(diff);
}

void ArcLineSegmenter::search(int start_y, int finish_y)
{
    double b;
    int diff,middle;

    diff = abs_i(finish_y-start_y);
    if (diff > 1) {
        middle=(start_y+finish_y)/2;
        b=gradient(middle);
        if (b<=0)
            search(start_y,middle);
        else
            search(middle,finish_y);
    }
    else{
        best_y=start_y;
        compute_error(&error,best_y);
        best_r=radius;
    }
}

void ArcLineSegmenter::determine_circle(int st, int fi, int *final_radius,
    int *final_xc, int *final_yc,
    float *final_dev, int *final_lgt, int *arc_dir)
{
    double x_org,y_org;
    double x_off,y_off;
    double xt,yt;
    double x_cent,y_cent;
    double a_c;
    double sine,cosine;
    double error_pos, error_neg;
    double max_dev,arc_length,poly_length;
    int loop1,loop2;
    double sum,ratio;
    int start_y,finish_y;
    double temp;
    int arc_size;
    loop2=0;
    for (loop1 = st;loop1 <= fi;loop1++) {
    loop2++;
    x_trans3[loop2] = x_start2[loop1];
    y_trans3[loop2] = y_start2[loop1];
    }
    loop2++;
    x_trans3[loop2] = x_end2[fi];
    y_trans3[loop2] = y_end2[fi];
    number_segments2=loop2;
/*
    for (loop2=1;loop2<=number_segments2;loop2++)
    printf("x_trans3: %f y_trans3: %f\n",x_trans3[loop2],y_trans3[loop2]);
*/
/*
    printf("determining circle from start of %d to finish of %d\n",st,fi);
*/
    /*
       take chord between points, rotate so that normal to chord is
       along y axis - rotate and translate all other points the same
    */
    x_org = (x_trans3[number_segments2]+x_trans3[1]) / 2;
    y_org = (y_trans3[number_segments2]+y_trans3[1]) / 2;
    yt = y_trans3[number_segments2]-y_trans3[1];
    xt = x_trans3[number_segments2]-x_trans3[1];
    if (xt != 0)
       a_c = -atan(yt/xt);
    else
       a_c = M_PI/2.0;      /* angle with x axis is -a_c */
    sine = sin(a_c);
    cosine = cos(a_c);
    x_off = x_org;
    y_off = y_org;
    /*
    put (x_org,y_org) at origin and offset all other points
        by the same amount
    */
    for (loop1 = 1;loop1 <= number_segments2;loop1++) {
        x_trans3[loop1] = x_trans3[loop1] - x_off;
        y_trans3[loop1] = y_trans3[loop1] - y_off;
    }
    x_org = 0.0;
    y_org = 0.0;

    /* align chord with x axis */
    for (loop1 = 1;loop1 <= number_segments2;loop1++) {
        xt = x_trans3[loop1];
        yt = y_trans3[loop1];
        temp = xt*cosine - yt*sine;
        x_trans3[loop1] = temp;
        temp = xt*sine + yt*cosine;
        y_trans3[loop1] = temp;
    }
/*
    for (loop2=1;loop2<=number_segments2;loop2++)
    printf("x_trans3: %f y_trans3: %f\n",x_trans3[loop2],y_trans3[loop2]);
*/
    /* determine error at start point */
    compute_error(&error,0);

    /* determine error on +ve side of start point */
    compute_error(&error_pos,-1);

    /* determine error on -ve side start point */
    compute_error(&error_neg,1);

    sum = 0;
    for (loop1 = 2;loop1 <= number_segments2 - 1;loop1++)
    sum=sum + y_trans3[loop1];
#if DEBUG
    printf("error_pos: %f error_neg: %f\n",error_pos,error_neg);
#endif
    temp = error_pos - error_neg;
    if (temp<0) temp = -temp;
    if (temp < 0.0000001) {    /* ensure arc_size will be SMALL */
       if (sum < 0)
      direction = 1;
       else
      direction = -1;
    }
    else if (error_pos < error_neg)
       direction = -1;
    else
       direction = 1;

    if (((direction == 1) && (sum < 0))
    ||((direction == -1) && (sum > 0))) {
    arc_size=SMALL;
    start_y=0;
    if (direction == 1) finish_y = 16384;
    else finish_y = -16384;
    search(start_y,finish_y);
    }
    else{
    arc_size = BIG;
    start_y = 0;
    finish_y = 0;
    for (loop1=2;loop1<=number_segments2 - 1;loop1++)
        if (direction == 1) {
            if (y_trans3[loop1]>finish_y) finish_y=y_trans3[loop1];
        }
        else
            if (y_trans3[loop1]<finish_y) finish_y=y_trans3[loop1];
    search(start_y,finish_y);
    }
    /*
    code to get correct mode for circle from start to finish nodes
        either anti or clockwise around centre
    */
    if (arc_size == SMALL) {
        if (x_trans3[1] < x_trans3[number_segments2]) {
        if (direction == -1) {
            *arc_dir = CLOCKWISE;
        }
        else{
            *arc_dir = ANTICLOCKWISE;
        }
        }
        else{
        if (direction == -1) {
            *arc_dir = ANTICLOCKWISE;
        }
            else{
            *arc_dir = CLOCKWISE;
        }
        }
    }
    else{
        if (x_trans3[1] < x_trans3[number_segments2]) {
        if (direction == -1) {
            *arc_dir = ANTICLOCKWISE;
        }
        else{
            *arc_dir = CLOCKWISE;
        }
        }
        else{
        if (direction == -1) {
            *arc_dir = CLOCKWISE;
            }
        else{
            *arc_dir = ANTICLOCKWISE;
        }
        }
    }
    x_cent = 0.0;
    y_cent = best_y;
    radius = best_r;
    compute_lgt(&arc_length,arc_size);
    compute_poly_lgt(&poly_length);
    ratio = arc_length / poly_length;
    if (ratio<1) ratio = poly_length / arc_length;
#if DEBUG
    printf("ratio: %f\n",ratio);
#endif
    /* !!!!!!!!! stopping if definitely not a circle ?????????? */
    /* if (ratio < 1.5) {  */
    compute_dev(*arc_dir,&max_dev);
    compute_dev_pos(best_y);
        a_c = -a_c;
        sine = sin(a_c);
        cosine = cos(a_c);
        for (loop1 = 1;loop1 <= number_segments2;loop1++) {
            xt = x_trans3[loop1];
            yt = y_trans3[loop1];
            x_trans3[loop1] = round_i(xt*cosine - yt*sine);
            y_trans3[loop1] = round_i(xt*sine + yt*cosine);
            /*
            x_trans3[loop1] = (xt*cosine - yt*sine);
            y_trans3[loop1] = (xt*sine + yt*cosine);
            */
        }
        xt = x_cent;
        yt = y_cent;
        x_cent = xt*cosine - yt*sine;
        y_cent = xt*sine + yt*cosine;
        for (loop1 = 1;loop1 <= number_segments2;loop1++) {
            x_trans3[loop1] = x_trans3[loop1] + x_off;
            y_trans3[loop1] = y_trans3[loop1] + y_off;
        }
        x_cent = x_cent + x_off;
        y_cent = y_cent + y_off;
        *final_radius = round_i(radius);
        *final_xc = round_i(x_cent);
        *final_yc = round_i(y_cent);
        *final_lgt = arc_length;
        *final_dev = max_dev * ratio;   /* temp to modify sig */
    /* } */
    /*
    else{
        *final_radius = 0;
        *final_xc = 0;
        *final_yc = 0;
        *final_lgt = 0;
        *final_dev = -1;
    global_pos = (fi - st + 2) / 2;
#if DEBUG
    printf("definitely not a circle - breaking at centre\n");
#endif
    }
    */
}

}

