/*
    LINES.C

    A  P.L. Rosin / G.A.W.West production. Copyright 1987 / 1988.

    May 1988:-
    Processes one connected set of pixels at a time to reduce
    memory requirements.
    Outputs data in super format (as for arcs) to allow extendability.

    November 1988:-
    Changed dev threshold to 2

    March 1989:-
    Now uses euclidean length of straight line, not length of list for
    significance calculation

    April 1989:-
    Compiler option to disable tail recursion in segment routine, to
    investigate affect of comparing significances - option RECURSION

    February 1990:-
    Horrible heuristic! - if deviation is 0 make it 1 - compiler option
    MAKE_ONE

    August 1990:-
    Outputs soft (non-break) points
    pixel flags stored in "flags" instead of x_c

    December 1990:-
    Outputs 0->4 soft break-points

    October 1991:-
    Added second stage - improve_lines
    Geoff

    July 1992:-
    Deals with floating point and integer input/output data.
    If integer pixel values generates integer line output
    If float pixel values generates float line output
    Geoff

    October 1993:-
    discard pixels lists with less than 2 pixels - avoid crashes!
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Segment.hh"
#include "Line.hh"
#include "rosin_lines.hh"

#define RECURSION 1     /* compile tail recursion if 1 in function segment */
#define MAKE_ONE 1      /* compile heuristic if 1 in function deviation */

#define MIN_DEV 1.9   /* minimum allowable deviation  ad hoc value! */

#define OK 0
#define NULL_BRKPT -3
#define NULL_SIG -9999
#define ALL_SBREAK 0      /* all non-endpoints as soft breakpoints */
#define SBREAK_1 1        /* 1 soft breakpoint at middle of curve */
#define SBREAK_2 2        /* 2 soft breakpoints in curve */
#define SBREAK_3 3        /* 2 soft breakpoints + middle breakpoint */
#define SBREAK_4 4        /* 0 up to 4 soft breakpoints */
#define NO_SBREAK 5       /* no soft breakpoints */

#define FALSE 0
#define TRUE !FALSE

#define FIRST 1           /* three constants used in improve_lines */
#define SECOND 2
#define THIRD 3

#define sqr(x) ((x)*(x))
/* our version of abs for all number representations */
#define Abs(x) (x)<0?(-(x)):(x)

namespace Z
{

/* use second stage */
static const int USE_IMPROVE = TRUE;
/* if weight = 1.0, no effect, choose best sig < 1.0, favours new longer lines
 */
static const float WEIGHT = 1.0;

void LineSegmenter::RosinFitLinesToSegment(Segment *seg)
{
  const int MAX_LINES = 100;
  /* note: list indexing goes from 1 (not 0!) to SIZE-1 */
  /* original list of pixels */
  float x_c[seg->edgels.Size() + 1];
  float y_c[seg->edgels.Size() + 1];
  float x_start[MAX_LINES], y_start[MAX_LINES];
  float x_end[MAX_LINES], y_end[MAX_LINES];
  float sigs[MAX_LINES];
  int line_start[MAX_LINES], line_end[MAX_LINES];
  int n_lines = MAX_LINES;

  for(unsigned i = 1; i <= seg->edgels.Size(); i++)
  {
    x_c[i] = seg->edgels[i - 1].p.x;
    y_c[i] = seg->edgels[i - 1].p.y;
  }
  if(segment_lines(x_c, y_c, seg->edgels.Size(),
      x_start, y_start, x_end, y_end, line_start, line_end, sigs, &n_lines))
  {
    for(int i = 0; i < n_lines; i++)
    {
      // note: lines of length 2 often cause problems
      if(line_end[i] - line_start[i] + 1 >= 3)
        core->NewGestalt(GestaltPrinciple::FORM_LINES,
            new VisibleLine(core, seg, line_start[i] - 1, line_end[i] - 1));
    }
  }
}

/**
 * Notes: 
 * Indexing of pixels on the segment goes from 1 to n_pixels (not 0 to
 * n_pixels - 1). Indexing of all other arrays is as normal (starting from 0).
 * Output arrays must be allocated beforehand by caller, big enough to hold
 * the maximum number of segmentable lines per segment (which can never be
 * larger than n_pixels/2). If the arrays are too small, the function will fail
 * and return false.
 * Segments shorter than 2 pixels are ignored and false is returned.
 *
 * @param x_c  array of size n_pixels + 1 of x-values of pixels of the original
 *             segment
 * @param x_y  array of size n_pixels + 1 of y-values of pixels of the original
 *             segment
 * @param n_pixels  number of pixels of the original segment
 * @param x_start  output, x-values of line start points
 * @param y_start  output, y-values of line start points
 * @param x_end  output, x-values of line end points
 * @param y_end  output, y-values of line end points
 * @param i_start  output, start indices of lines
 * @param i_end    output, end indices of lines
 * @param sigs  output, line signatures 
 * @param n_lines  on function call must hold the size of the output arrays, on
 *                 return holds the number of segmented lines
 * @return  true, if all went well, false if there was any error
 */
bool LineSegmenter::segment_lines(float x_c[], float y_c[], int n_pixels,
    float x_start[], float y_start[], float x_end[], float y_end[],
    int i_start[], int i_end[], float line_sigs[], int *n_lines)
{
  float sig;  /* not really used - just dummy param for segment */
  float sum;  /* as for sig above */
  int line_start, line_end;
  /* pixels transformed and rotated (to current line position/orientation) */
  float x_c2[n_pixels + 1], y_c2[n_pixels + 1];
  float sigs[n_pixels + 1];
  float sums[n_pixels + 1];
  int flags[n_pixels + 1];
  int max_lines = *n_lines;

  /* avoid duplicated endpoints for closed curves */
  if ((x_c[1] == x_c[n_pixels]) && (y_c[1] == y_c[n_pixels]))
    n_pixels--;
  /* this is regarded noise */
  if (n_pixels < 2) {
    return false;
  }
  for(int i = 1; i <= n_pixels; i++) {
    sigs[i] = NULL_SIG;
    flags[i] = OK;
  }
  segment(x_c, y_c, x_c2, y_c2, sigs, sums, flags, 1, n_pixels, &sig, &sum);
  if (USE_IMPROVE == TRUE)
      improve_lines(x_c, y_c, x_c2, y_c2, sigs, flags, n_pixels,
          1, n_pixels);
  /* iterate over point list and create lines */
  line_start = 1;
  *n_lines = 0;
  for(int j = 2; j <= n_pixels - 1; j++)
    if(flags[j] != NULL_BRKPT)
    {
      line_end = j;
      if(*n_lines < max_lines)
      {
        x_start[*n_lines] = x_c[line_start];
        y_start[*n_lines] = y_c[line_start];
        x_end[*n_lines] = x_c[line_end];
        y_end[*n_lines] = y_c[line_end];
        i_start[*n_lines] = line_start;
        i_end[*n_lines] = line_end;
        line_sigs[*n_lines] = sigs[line_start];
        (*n_lines)++;
      }
      // if no more room in output array to store further lines
      else
        return false;
      line_start = line_end;
    }
  line_end = n_pixels;
  if(*n_lines < max_lines)
  {
    x_start[*n_lines] = x_c[line_start];
    y_start[*n_lines] = y_c[line_start];
    x_end[*n_lines] = x_c[line_end];
    y_end[*n_lines] = y_c[line_end];
    i_start[*n_lines] = line_start;
    i_end[*n_lines] = line_end;
    line_sigs[*n_lines] = sigs[line_start];
    (*n_lines)++;
  }
  else
    return false;
  return true;
}

void LineSegmenter::segment(float x_c[], float y_c[], float x_c2[],
    float y_c2[], float sigs[],
    float sums[], int flags[],
    int start_in, int finish_in, float *sig_out, float *sum_out)
{
    int i;
    int pos;
    float dev;
    float sig1,sig2,sig3,max_sig,sum1,sum2,sum3,best_sum;
    int ok;
    float length;
    int end2;

    /* compute significance at this level */

    transform(x_c, y_c, x_c2, y_c2, start_in, finish_in, &end2);
    deviation(y_c2, end2, &pos,&dev,&ok,&sum1);
#if MAKE_ONE
    if (dev == 0)
       dev = MIN_DEV;
#endif
    pos = pos + start_in - 1;
    /* euclidean length */
    length = sqr(x_c2[1]-x_c2[end2]);
    length += sqr(y_c2[1]-y_c2[end2]);
    length = sqrt(length);
    sig1 = dev / length;
    sum1 = sum1 / length;
    if (((finish_in - start_in) < 3) || (dev < MIN_DEV) || (ok == FALSE)) {
        /* save line match at this lowest of levels */
        /* modify x_c,y_c data to delete unused coordinates */
        /* save significance */
        sigs[start_in] = sig1;
        sums[start_in] = sum1;
        *sig_out = sig1;
        *sum_out = sum1;
        /* delete breakpoints between end points */
        if ((finish_in - start_in) >= 2)
            for (i = start_in + 1; i < finish_in;i++)
                flags[i] = NULL_BRKPT;
    }
    else{
        /* recurse to next level down */
        segment(x_c, y_c, x_c2, y_c2, sigs, sums, flags,
            start_in, pos, &sig2, &sum2);
        segment(x_c, y_c, x_c2, y_c2, sigs, sums, flags,
            pos, finish_in, &sig3, &sum3);
#if RECURSION
        /* get best significance from lower level */
        if (sig2 < sig3) {
            max_sig = sig2;
            best_sum = sum2;
        }
        else{
            max_sig = sig3;
            best_sum = sum3;
        }
        if (max_sig < sig1) {
            /* return best significance, keep lower level description */
            *sig_out = max_sig;
            *sum_out = best_sum;
        }
        else{
            /* line at this level is more significant so remove coords
               at lower levels */
            *sig_out = sig1;
            *sum_out = sum1;
            sigs[start_in] = *sig_out;
            sums[start_in] = *sum_out;
            if ((finish_in - start_in) >= 2)
                for (i = start_in + 1;i < finish_in;i++)
                    flags[i] = NULL_BRKPT;
        }
#endif
    }
}

void LineSegmenter::transform(float x_c[], float y_c[],
    float x_c2[], float y_c2[],
    int start, int finish, int *end2)
{
    int i,j;
    float x_offset,y_offset,x_end,y_end;
    double angle,sine,cosine;
    double temp;

    x_offset = x_c[start];
    y_offset = y_c[start];
    x_end = x_c[finish];
    y_end = y_c[finish];
    if ((x_end - x_offset) == 0.0) {
        if ((y_end - y_offset) > 0.0)
            angle = -M_PI_2;
        else
            angle = M_PI_2;
    }
    else{
        temp = ((float)(y_end-y_offset) / (float)(x_end-x_offset));
        angle = -atan(temp);
    }
    cosine = cos(angle);
    sine = sin(angle);
    j = 0;
    for (i=start;i<=finish;i++) {
        j++;
        x_c2[j] = x_c[i] - x_offset;
        y_c2[j] = y_c[i] - y_offset;
        temp = (float)(cosine * x_c2[j]) - (float)(sine * y_c2[j]);
        y_c2[j] = (float)(sine * x_c2[j]) + (float)(cosine * y_c2[j]);
        x_c2[j] = temp;
    }
    *end2 = j;
}

void LineSegmenter::deviation(float y_c2[], int end2, int *pos,float *dev,
    int *ok,float *sum)
{
    int i;
    int pos1;
    float max1,temp;

    pos1 = 0;
    max1 = 0.0;
    *sum = 0.0;
    for (i = 1;i <= end2;i++) {
       temp = Abs(y_c2[i]);
       if (temp > max1) {
          max1 = temp;
          pos1 = i;
       }
       *sum += temp;
    }
    /* if no peak found - signal with ok */
    if (max1 == 0.0)
       *ok = FALSE;
    else
        *ok = TRUE;
    *pos = pos1;
    *dev = max1;
}

/* second stage after initial segmentation
    For each line in representation, tries to combine with each neighbour.
    Four choices:
        Neighbour on right
        Neighbour on left
        Both neighbours
        Neither neighbours
    Chooses best representation.
    Repeats until can do no more.
*/
void LineSegmenter::improve_lines(float x_c[], float y_c[],
    float x_c2[], float y_c2[],
    float sigs[], int flags[], int number_pixels,
    int start_in,int finish_in)
{
    int loop1,loop2,loop3,loop4;
    int cont,flag;

    /* 
       repeatedly scan the list of vertices until cannot alter anymore.
       for each scan generate all combinations of adjacent vertices, 
       i.e. find four vertices that form three lines in the data 
       (1) find a vertex that is flagged OK - start of first line
       (2) find next vertex flagged as OK - finish of first line/start of second
       (3) find next vertex flagged as OK - finish of second line/start of third
       (4) find next vertex flagged as OK - finish of third line
       Then pass info to combine.
    */
    do {
        cont = FALSE;
        loop1 = start_in;
        do {
            while ((loop1 < finish_in) && (flags[loop1] != OK)) {
                loop1++;
            }
            if(loop1 < finish_in)
            {
              loop2 = loop1 + 1;
              while ((loop2 < finish_in) && (flags[loop2] != OK))
                loop2++;
            }
            else
              loop2 = finish_in;
            if(loop2 < finish_in)
            {
              loop3 = loop2 + 1;
              while ((loop3 < finish_in) && (flags[loop3] != OK))
                loop3++;
            }
            else
              loop3 = finish_in;
            if(loop3 < finish_in)
            {
              loop4 = loop3 + 1;
              while ((loop4 < finish_in) && (flags[loop4] != OK))
                loop4++;
            }
            else
              loop4 = finish_in;
            /* 
               check to make sure the four vertices are different 
               some will be identical if reach end of list before all four
               vertices determined - call combine if they are different
            */
            if ((loop2 > loop1) && (loop3 > loop2) && (loop4 > loop3)) { 
                combine(x_c, y_c, x_c2, y_c2, sigs, flags, number_pixels,
                    loop1, loop2, loop3, loop4, &flag);
                if (cont == FALSE)
                    if (flag == TRUE)
                        cont = TRUE;
            }
            loop1++;
        } while (loop4 < finish_in);
    } while (cont == TRUE);
}

void LineSegmenter::combine(float x_c[], float y_c[],
    float x_c2[], float y_c2[], float sigs[],
    int flags[], int number_pixels, int v1, int v2, int v3, int v4, int *flag)
{
    float sig[4];
    int best[4];
    float sigp,sigc,sign;
    int i,j;
    int replaced;
    int ok,pos;
    float dev,sum;
    int end2;

    sigp = sigs[v1];
    sigc = sigs[v2];
    sign = sigs[v3];

    /* determine significances of combinations */
    /* only need the significance values from deviation() */
    
    /* combine with previous representation? */
    transform(x_c, y_c, x_c2, y_c2, v1, v3, &end2);
    deviation(y_c2, end2, &pos,&dev,&ok,&sum);
    sig[1] = dev / (float)end2;
    
    /* combine with next representation? */
    if (v4 > number_pixels)
        sig[2] = 9999;
    else{
        transform(x_c, y_c, x_c2, y_c2, v2, v4, &end2);
        deviation(y_c2, end2, &pos,&dev,&ok,&sum);
        sig[2] = dev / (float)end2;
    }
    
    /* combine with previous and next representation? */
    if (v4 > number_pixels)
        sig[3] = 9999;
    else{
        transform(x_c, y_c, x_c2, y_c2, v1, v4, &end2);
        deviation(y_c2, end2, &pos,&dev,&ok,&sum);
        sig[3] = dev / (float)end2;
    }

    /* rank significances */
    /* temporary set significance of THIRD to 9999 to stop it being used */
    
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
                replaced = TRUE;
                /* delete current and next, modify previous */
                flags[v2] = NULL_BRKPT;
                flags[v3] = NULL_BRKPT;
                sigs[v1] = sig[i];
            }
        }
        else if (best[i] == FIRST) {
            if ((sig[i]*WEIGHT<sigp) && (sig[i]*WEIGHT<sigc)) {
                /* replace previous and current */
                replaced = TRUE;
                /* delete current and modify prvious */
                flags[v2] = NULL_BRKPT;
                sigs[v1] = sig[i];
            }
        }
        else{
            if ((sig[i]*WEIGHT<sigc) && (sig[i]*WEIGHT<sign)) {
                /* replace current and next */
                replaced = TRUE;
                /* delete next and modify current */
                flags[v3] = NULL_BRKPT;
                sigs[v2] = sig[i];
            }
        }
    } while ((replaced == FALSE) && (i<3));
    *flag = replaced;
}

}

