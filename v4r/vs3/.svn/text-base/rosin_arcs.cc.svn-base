/*
    arc.c

    A  P.L. Rosin / G.A.W.West production. Copyright 1987 / 1988.

    New version 1-May-88 processes one connected set of pixels at
    a time to reduce memory requirements.
    Outputs data in super format (as for arcs) to allow extendability.
    Version that fits arcs to original data, not line data.

    Input: pixel data format.
    Output: super data format (only arcs).
*/

#include <stdio.h>
#include <math.h>
#include "Math.hh"
#include "VisionCore.hh"
#include "Segment.hh"
#include "Arc.hh"
#include "GestaltPrinciple.hh"

// large radius means arc is actually a line
static const int MAX_RADIUS = 10000;
// small arcs correspond to speckles, which we want to ignore
static const int MIN_RADIUS = 2;

#ifndef FALSE
# define FALSE 0
# define TRUE (!FALSE)
#endif

#define DEBUG 0            /* compile debug output if 1 */
#define CROSS 0            /* draw crosses at arc/line ends if 1 */

#define NO_PIXELS 10000
#define NO_LINE_SEGS 1000   /* max number of lines per list */
#define NO_ARCS 1000        /* max number of arcs per list  */

#define PI 3.141591
#define NINETY 1.570796327
#define SMALL 1             /* type of arc */
#define BIG 2
#define CLOCKWISE 1         /* direction of arc from start to finish */
#define ANTICLOCKWISE 2
#define KEEP 0    /* KEEP and REJECT used to tell if arcs to be kept */
#define REJECT 1

/* one list of pixels */
static short x_c[NO_PIXELS],y_c[NO_PIXELS];

/* x/y_c transformed by determine_circle */
static float x_trans[NO_PIXELS],y_trans[NO_PIXELS];

static short arc_centre_x[NO_ARCS],arc_centre_y[NO_ARCS];
static short arc_start_x[NO_ARCS],arc_start_y[NO_ARCS];
static short arc_end_x[NO_ARCS],arc_end_y[NO_ARCS];
static int radii[NO_ARCS];
static short arc_dirs[NO_ARCS];
static float arc_sig[NO_ARCS];
static short arc_start[NO_ARCS],arc_finish[NO_ARCS];
static short arc_status[NO_ARCS];
static short arc_line[NO_ARCS];

static int number_pixels;
static int number_arcs;
static int number_segments2;

static double radius,best_r;
static double error;
static int global_pos;
static int best_y;
static int direction;


int abs_i(int value)
{
    if (value < 0) value = -value;
    return(value);
}

int round_i(double value)
{
    return (int)floor(value + 0.5);
}

void compute_error(double *error, int y_val)
{
    int loop1;
    double temp1,temp2,temp3,temp4;

    temp4 = 0;
    temp1 = y_val;
    temp1 = temp1 * temp1;
    temp2 = x_trans[1];
    temp2 = temp2 * temp2;
    temp3 = temp1 + temp2;
    radius = sqrt(temp3);
    for (loop1 = 2; loop1 <= number_segments2 - 1;loop1++) {
       temp1 = x_trans[loop1];
       temp1 = temp1 * temp1;
       temp2 = y_val - y_trans[loop1];
       temp2 = temp2 * temp2;
       temp3 = radius - sqrt(temp1+temp2);
       if (temp3 < 0) temp3 = -temp3;
       temp4 = temp4 + temp3;
    }
    *error = temp4;
}

float angle(float x1, float y1, float x2, float y2)
{

    float angle_temp;
    float xx,yy;

    xx = x2 - x1;
    yy = y2 - y1;
    if (xx == 0.0)
    angle_temp = PI/2.0;
    else
    angle_temp = atan(fabs(yy/xx));
    if ((xx < 0.0) && (yy >= 0.0))
    angle_temp = PI - angle_temp;
    else if ((xx < 0.0) && (yy < 0.0))
    angle_temp = PI + angle_temp;
    else if ((xx >= 0.0) && ( yy < 0.0))
    angle_temp = PI*2.0 - angle_temp;
    return(angle_temp);
}

double euclid(float x1, float y1, float x2, float y2)
{
    double temp1,temp2;
    double dist;

    temp1 = fabs(x1-x2);
    temp2 = fabs(y1-y2);
    dist = sqrt(temp1 * temp1 + temp2 * temp2);
    return(dist);
}

void compute_minimum_diff(float x_cir, float y_cir,
                float x1, float y1, float x2, float y2,
                double *deviation)
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

/*
    printf("entered compute_minimum_diff\n");
    printf("x_cir: %f y_cir %f\n",x_cir,y_cir);
    printf("x1: %f y1 %f x2 %f y2 %f\n",x1,y1,x2,y2);
*/
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
/*
    printf("x_cir: %f y_cir %f\n",x_cir,y_cir);
    printf("x1: %f y1 %f x2 %f y2 %f\n",x1,y1,x2,y2);
*/
    min1 = euclid(x_cir,y_cir,x1,y1);
    min2 = euclid(x_cir,y_cir,x2,y2);
    if (x_cir < x1) {
        *deviation = min1;
    }else if (x_cir > x2) {
        *deviation = min2;
    }else{
        *deviation = fabs(y_cir);
    }
/*    printf("deviation: %f\n",*deviation,); */
}

void compute_dev_pos(int y_val)
{
    int loop1;
    int pos1=0,pos2;
    double temp1,temp2,temp3;
    double dev,dev1,dev2;

    temp1 = y_val;
    temp1 = temp1 * temp1;
    temp2 = x_trans[1];
    temp2 = temp2 * temp2;
    temp3 = temp1 + temp2;
    radius = sqrt(temp3);
    dev1 = 0;
    for (loop1 = 2;loop1 <= number_segments2 - 1;loop1++) {
           temp1 = x_trans[loop1];
        temp1 = temp1 * temp1;
           temp2 = y_val - y_trans[loop1];
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
       temp1 = (x_trans[loop1] + x_trans[loop1+1]) / 2;
       temp1 = temp1 * temp1;
       temp2 = y_val - (y_trans[loop1] + y_trans[loop1+1]) / 2;
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

void compute_dev(int arc_dir, double *max_dev)
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
/*    printf("number of vertices: %d\n",number_segments2);
    printf("centre: (%f , %f)\n",x_cent,y_cent);
*/
#if DEBUG
    printf("radius: %f\n",best_r);
#endif
/* determine angles */
    angle1 = angle(x_cent,y_cent,x_trans[1],y_trans[1]);
    angle2 = angle(x_cent,y_cent,x_trans[number_segments2],
                            y_trans[number_segments2]);
#if DEBUG
    printf("%f %f\n",x_trans[1],y_trans[1]);
    printf("%f %f\n",x_trans[number_segments2],y_trans[number_segments2]);
    printf("angles: %f %f\n",angle1,angle2);
#endif
    if (arc_dir == CLOCKWISE) {
    temp1 = angle1;
    angle1 = angle2;
    angle2 = temp1;
    }
/* ang4 must be bigger than ang3 */
    if (angle2 < angle1) {
    angle2 = angle2 + PI * 2.0;
    }
#if DEBUG
    printf("start angle: %f finish angle %f\n",angle1,angle2);
/* walk around circle from angle1 to angle2 */
    printf("\nwalking around circle\n");
#endif
    step = (angle2-angle1) / 10.0; /* use 11 steps presently */
    l1 = angle1;
    do{
    min_dev = 1000000;
    x_cir = cos(l1) * best_r + x_cent;
    y_cir = sin(l1) * best_r + y_cent;
/*    printf("\nl1: %f (x_cir,y_cir): %f %f\n",l1,x_cir,y_cir);  */
    l2 = 1;
    do{
/*        printf("for line %d\n",l2);   */
        compute_minimum_diff(x_cir,y_cir,
                x_trans[l2],y_trans[l2],
                x_trans[l2+1],y_trans[l2+1],
                &deviation);
        if (deviation < min_dev) {
        min_dev = deviation;
        }
        l2++;
    }while (l2 <= number_segments2-1);
/*    printf("at this angle minimum deviation: %f at vertex: %d\n",
        min_dev,pos1);  */
        if (min_dev > *max_dev) {
            *max_dev = min_dev;
    }
    l1 += step;
    }while (l1 <= angle2);
#if DEBUG
    printf("maximum deviation: %f\n",*max_dev);
#endif
}

void compute_lgt(double *lgt, int arc_size)
{
    double temp1,temp2,temp3;
    double chord,angle;

    temp1 = x_trans[1] - x_trans[number_segments2];
    temp1 = temp1 * temp1;
    temp2 = y_trans[1] - y_trans[number_segments2];
    temp2 = temp2 * temp2;
    temp3 = temp1 + temp2;
    chord = sqrt(temp3);
    temp3 = (chord / 2.0) / radius;
    if (temp3 > 1.0) temp3 = 1;        /* eliminate rounding errors */
    angle = asin(temp3) * 2;
    if (arc_size == BIG) angle = 2 * PI - angle;
    *lgt = angle*radius;
}

void compute_poly_lgt(double *lgt)
{
    int loop1;
    double dx,dy;
    double total,dist;

    total = 0.0;
    for (loop1 = 1;loop1 < number_segments2;loop1++) {
        dx = x_trans[loop1] - x_trans[loop1+1];
        dy = y_trans[loop1] - y_trans[loop1+1];
        dist = dx * dx + dy * dy;
        dist = sqrt(dist);
        total += dist;
    }
    *lgt = total;
}

double gradient(int position)
{
    double error1,error2,diff;

    compute_error(&error1,position-direction);
    compute_error(&error2,position+direction);
    diff = error1 - error2;
    return(diff);
}

void search(int start_y, int finish_y)
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

void determine_circle(int st, int fi, int *final_radius, int *final_xc,
    int *final_yc, float *final_dev, int *final_lgt, int *arc_dir)
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
    int x1,y1;
    int xflag = TRUE;
    int yflag = TRUE;

    loop2=0;
    x1 = x_c[st];
    y1 = y_c[st];
    for (loop1 = st;loop1 <= fi;loop1++) {
        loop2++;
        x_trans[loop2] = x_c[loop1];
        y_trans[loop2] = y_c[loop1];
        if (x1 != x_c[loop1]) xflag = FALSE;
        if (y1 != y_c[loop1]) yflag = FALSE;
    }
    number_segments2=loop2;

    if (xflag || yflag) {
        *final_radius = 0;
        *final_xc = 0;
        *final_yc = 0;
        *final_lgt = 0;
        *final_dev = -1;
        global_pos = (fi - st + 2) / 2;
        return;
    }

/*
    for (loop2=1;loop2<=number_segments2;loop2++)
    printf("x_trans: %f y_trans: %f\n",x_trans[loop2],y_trans[loop2]);
*/
/*
    printf("determining circle from start of %d to finish of %d\n",st,fi);
*/
    /*
       take chord between points, rotate so that normal to chord is
       along y axis - rotate and translate all other points the same
    */
    x_org = (x_trans[number_segments2]+x_trans[1]) / 2;
    y_org = (y_trans[number_segments2]+y_trans[1]) / 2;
    yt = y_trans[number_segments2]-y_trans[1];
    xt = x_trans[number_segments2]-x_trans[1];
    if (xt != 0)
       a_c = -atan(yt/xt);
    else
       a_c = PI/2.0;      /* angle with x axis is -a_c */
    sine = sin(a_c);
    cosine = cos(a_c);
    x_off = x_org;
    y_off = y_org;
    /*
    put (x_org,y_org) at origin and offset all other points
        by the same amount
    */
    for (loop1 = 1;loop1 <= number_segments2;loop1++) {
        x_trans[loop1] = x_trans[loop1] - x_off;
        y_trans[loop1] = y_trans[loop1] - y_off;
    }
    x_org = 0.0;
    y_org = 0.0;

    /* align chord with x axis */
    for (loop1 = 1;loop1 <= number_segments2;loop1++) {
        xt = x_trans[loop1];
        yt = y_trans[loop1];
        temp = xt*cosine - yt*sine;
        x_trans[loop1] = temp;
        temp = xt*sine + yt*cosine;
        y_trans[loop1] = temp;
    }
/*
    for (loop2=1;loop2<=number_segments2;loop2++)
    printf("x_trans: %f y_trans: %f\n",x_trans[loop2],y_trans[loop2]);
*/
    /* determine error at start point */
    compute_error(&error,0);

    /* determine error on +ve side of start point */
    compute_error(&error_pos,-1);

    /* determine error on -ve side start point */
    compute_error(&error_neg,1);

    sum = 0;
    for (loop1 = 2;loop1 <= number_segments2 - 1;loop1++)
    sum=sum + y_trans[loop1];
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
            if (y_trans[loop1]>finish_y) finish_y=(int)y_trans[loop1];
        }
        else
            if (y_trans[loop1]<finish_y) finish_y=(int)y_trans[loop1];
    search(start_y,finish_y);
    }
    /*
    code to get correct mode for circle from start to finish nodes
        either anti or clockwise around centre
    */
    if (arc_size == SMALL) {
        if (x_trans[1] < x_trans[number_segments2]) {
            if (direction == -1) *arc_dir = CLOCKWISE;
            else *arc_dir = ANTICLOCKWISE;
        }
        else{
            if (direction == -1) *arc_dir = ANTICLOCKWISE;
            else *arc_dir = CLOCKWISE;
        }
    }
    else{
        if (x_trans[1] < x_trans[number_segments2]) {
            if (direction == -1) *arc_dir = ANTICLOCKWISE;
            else *arc_dir = CLOCKWISE;
        }
        else{
            if (direction == -1) *arc_dir = CLOCKWISE;
            else *arc_dir = ANTICLOCKWISE;
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
            xt = x_trans[loop1];
            yt = y_trans[loop1];
            x_trans[loop1] = round_i(xt*cosine - yt*sine);
            y_trans[loop1] = round_i(xt*sine + yt*cosine);
            /*
            x_trans[loop1] = (xt*cosine - yt*sine);
            y_trans[loop1] = (xt*sine + yt*cosine);
            */
        }
        xt = x_cent;
        yt = y_cent;
        x_cent = xt*cosine - yt*sine;
        y_cent = xt*sine + yt*cosine;
        for (loop1 = 1;loop1 <= number_segments2;loop1++) {
            x_trans[loop1] = x_trans[loop1] + x_off;
            y_trans[loop1] = y_trans[loop1] + y_off;
        }
        x_cent = x_cent + x_off;
        y_cent = y_cent + y_off;
        *final_radius = round_i(radius);
        *final_xc = round_i(x_cent);
        *final_yc = round_i(y_cent);
        *final_lgt = (int)arc_length;
        *final_dev = max_dev * ratio;   /* temp to modify significance */
#if DEBUG
        printf("st: %d fi: %d\n",st,fi);
#endif
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

void segment(int start_in, int finish_in, float *sig_out)
{
    int i;
    int pos;
    float max_dev;
    float sig1,sig2,sig3,max_sig;
    int centre_x, centre_y, radius;
    int arc_length,arc_dir;

    /* compute significance at this level */

    determine_circle(start_in,finish_in,&radius,&centre_x,&centre_y,
                    &max_dev,&arc_length,&arc_dir);
#if DEBUG
    printf("circle? rad %d ctr %d %d dev %d lgt %d size %d\n",
            radius,centre_x,centre_y,max_dev,arc_length,arc_dir);
#endif
    pos = global_pos + start_in - 1;
    if (max_dev > 0)
        sig1 = max_dev / (float)arc_length;
    else
        sig1 = 9999;
    number_arcs++;
    arc_line[number_arcs] = (max_dev <= 0);
    arc_sig[number_arcs] = sig1;
    arc_centre_x[number_arcs] = centre_x;
    arc_centre_y[number_arcs] = centre_y;
    arc_start_x[number_arcs] = x_c[start_in];
    arc_start_y[number_arcs] = y_c[start_in];
    arc_end_x[number_arcs] = x_c[finish_in];
    arc_end_y[number_arcs] = y_c[finish_in];
    arc_dirs[number_arcs] = arc_dir;
    radii[number_arcs] = radius;
    arc_start[number_arcs] = start_in;
    arc_finish[number_arcs] = finish_in;
    arc_status[number_arcs] = KEEP;
    if (((finish_in - start_in) < 3) || (max_dev < 3)) {
        /* at lowest level of recursion */
        /* save arc match at this lowest of levels */
        /* and significance */
        *sig_out = sig1;
    }
    else{
        /* recurse to next level down */
        segment(start_in,pos,&sig2);
        segment(pos,finish_in,&sig3);
        /* get best significance from lower level */
        if (sig2 < sig3)
            max_sig = sig2;
        else
            max_sig = sig3;
        if (max_sig < sig1) {
            /* return best significance, keep lower level description */
            *sig_out = max_sig;
        /* remove the single arc from start_in to finish_in */
        for (i = 0;i <= number_arcs;i++)
            if ((arc_start[i] == start_in) &&
                (arc_finish[i] == finish_in))
            arc_status[i] = REJECT;
        }
        else{
            /* line at this level is more significant */
            /* remove arcs at lower levels */
            /* i.e between start_in and finish_in */
            /* DOES IT EVER GET HERE ???? */
            /* YES IT DOES AND THE PRINTF'S ARE ANNOYING
            printf("JUST REPLACED LOWER LEVEL ARCS\n");*/
            *sig_out = sig1;
            for (i = 0;i <= number_arcs;i++) {
                if ((arc_start[i] == start_in) &&
                    (arc_finish[i] == finish_in)) /* leave it */ {}
                else if ((arc_start[i] >= start_in)
                        && (arc_finish[i] <= finish_in))
                    arc_status[i] = REJECT;
            }
        }
    }
}

void FitArcsToSegment(Z::VisionCore *vcore, Z::Segment *seg)
{
    int j;
    float sig;    /* not really used - just dummy param for segment */
    Z::Arc *new_arc, *prev_arc = 0;

    number_pixels = 0;
    for(unsigned i = 0; i < seg->edgels.Size(); i++)
    {
      // note Rosin/West start arrays at 1, I start at 0
      number_pixels++;
      x_c[number_pixels] = (int)seg->edgels[i].p.x;
      y_c[number_pixels] = (int)seg->edgels[i].p.y;
    }
    /* avoid duplicated endpoints for closed curves */
    if ((x_c[1] == x_c[number_pixels]) && (y_c[1] == y_c[number_pixels]))
        number_pixels--;

    number_arcs = 0;
    segment(1,number_pixels,&sig);

    /* create arcs */
    for (j = 1;j <= number_arcs;j++)
        if (arc_status[j] == KEEP) {
            if (!arc_line[j]) 
            {
              if(arc_finish[j] - arc_start[j] + 1 >= 7 &&
                  radii[j] > MIN_RADIUS && radii[j] < MAX_RADIUS)
              {
                VEC::Vector2 cent(arc_centre_x[j], arc_centre_y[j]);
                // note Rosin/West start arrays at 1, I start at 0
                new_arc = new Z::Arc(vcore, seg,
                        (unsigned)arc_start[j] - 1,
                        (unsigned)(arc_start[j] - 1 + arc_finish[j] - 1)/2,
                        (unsigned)arc_finish[j] - 1,
                        cent, (double)radii[j]);
                vcore->NewGestalt(Z::GestaltPrinciple::FORM_ARCS, new_arc);
                if(prev_arc != 0)
                {
                  new_arc->prev_on_seg = prev_arc;
                  prev_arc->next_on_seg = new_arc;
                }
              }
            }
        }
}

