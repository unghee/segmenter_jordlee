/*
 * This software is copyrighted as noted below.  It may be freely copied,
 * modified, and redistributed, provided that the copyright notice is 
 * preserved on all copies.
 * 
 * There is no warranty or other guarantee of fitness for this software,
 * it is provided solely "as is".  Bug reports or fixes may be sent
 * to the author, who may or may not act on them as he desires.
 *
 * You may not include this software in a program or other software product
 * without supplying the source, or without informing the end-user that the 
 * source is available for no extra charge.
 *
 * If you modify this software, you should include a notice giving the
 * name of the person performing the modification, the date of modification,
 * and the reason for such modification.
 */
/*
 * colorquant.c
 *
 * Perform variance-based color quantization on a "full color" image.
 * Author:	Craig Kolb
 *		Department of Mathematics
 *		Yale University
 *		kolb@yale.edu
 * Date:	Tue Aug 22 1989
 * Copyright (C) 1989 Craig E. Kolb
 * $Id: colorquant.c,v 1.5 1995/11/13 13:18:00 ees2gm Exp $
 *
 * $Log: colorquant.c,v $
 * Revision 1.5  1995/11/13  13:18:00  ees2gm
 * malloc.h header removed
 *
 * Revision 1.4  1995/11/13  12:23:26  ees2gm
 * *** empty log message ***
 *
 * Revision 1.3  1995/09/19  13:08:24  ees1rm
 * In function `makenearest'   the warning: `which' might be used uninitialized removed.
 *
 * Revision 1.2  1995/09/19  08:26:32  ees1rm
 * The warning produced by gcc2.7.0 removed.
 *
 * Revision 1.1  1995/09/07  08:40:57  ees1rm
 * Initial revision
 *
 * Revision 1.3  89/12/03  18:27:16  craig
 * Removed bogus integer casts in distance calculation in makenearest().
 * 
 * Revision 1.2  89/12/03  18:13:12  craig
 * FindCutpoint now returns FALSE if the given box cannot be cut.  This
 * to avoid overflow problems in CutBox.
 * "whichbox" in GreatestVariance() is now initialized to 0.
 * 
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h> /*memset()*/
#include "colorquant.h"
/*
 * Maximum number of colormap entries.  To make larger than 2^8, the rgbmap
 * type will have to be changed from unsigned chars to something larger.
 */
#define H_HUGE                1e10
#define MAXCOLORS		256
/*
 * Value corresponding to full intensity in colormap.  The values placed
 * in the colormap are scaled to be between zero and this number.  Note
 * that anything larger than 255 is going to lead to problems, as the
 * colormap is declared as an unsigned char.
 */
#define FULLINTENSITY		255
#define MAX(x,y)	((x) > (y) ? (x) : (y))

/*
 * Readability constants.
 */
#define REDI		0	
#define GREENI		1
#define BLUEI		2	
#define TRUE		1
#define FALSE		0

typedef struct {
	float		weightedvar,		/* weighted variance */
			mean[3];		/* centroid */
	unsigned long 	weight,			/* # of pixels in box */
			freq[3][MAXCOLORS];	/* Projected frequencies */
	int 		low[3], high[3];	/* Box extent */
} Box;

unsigned long		*Histogram,		/* image histogram */
			NPixels;		/* total # of pixels */
unsigned int		Colors,			/* Maximum colormap entries */
			Bits,			/* # significant input bits */
			ColormaxI;		/* # of colors, 2^Bits */

/* Declaration of local functions. */
static void QuantHistogram(register unsigned char *r,
                           register unsigned char *g,
                           register unsigned char * b, Box * box);
static int  CutBoxes(Box	*boxes, int	colors); 
static int  GreatestVariance(Box *boxes, int n);
static void BoxStats(register Box *box);
static int  CutBox(Box *box, Box *newbox);
static int  FindCutpoint(Box *box, int color, Box * newbox1, Box *newbox2);
static void UpdateFrequencies(register Box *box1, register Box *box2);
static void ComputeRGBMap(Box *boxes, int colors, unsigned char *rgbmap,
                          int bits, unsigned char *colormap[3], int fast);
static void SetRGBmap(int boxnum, Box *box, unsigned char *rgbmap, int bits);
static void find_colors(Box *boxes, int colors, unsigned char *rgbmap,
                        int bits, unsigned char *colormap[3]);
static int  getneighbors(Box *boxes, int num, int * neighbors, int colors, 
                         unsigned char *colormap[3]);
static void makenearest(Box *boxes, int boxnum, int nneighbors,
                        int * neighbors, unsigned char *rgbmap, 
                        int bits, unsigned char *colormap[3]);

/*
 * Perform variance-based color quantization on a 24-bit image.
 *
 * Input consists of:
 *	red, green, blue	Arrays of red, green and blue pixel
 *				intensities stored as unsigned characters.
 *				The color of the ith pixel is given
 *				by red[i] green[i] and blue[i].  0 indicates
 *				zero intensity, 255 full intensity.
 *	pixels			The length of the red, green and blue arrays
 *				in bytes, stored as an unsigned long int.
 *	colormap		Points to the colormap.  The colormap
 *				consists of red, green and blue arrays.
 *				The red/green/blue values of the ith
 *				colormap entry are given respectively by
 *				colormap[0][i], colormap[1][i] and
 *				colormap[2][i].  Each entry is an unsigned char.
 *	colors			The number of colormap entries, stored
 *				as an integer.	
 *	bits			The number of significant bits in each entry
 *				of the red, green and blue arrays. An integer.
 *	rgbmap			An array of unsigned chars of size (2^bits)^3.
 *				This array is used to map from pixels to
 *				colormap entries.  The 'prequantized' red,
 *				green and blue components of a pixel
 *				are used as an index into rgbmap to retrieve
 *				the index which should be used into the colormap
 *				to represent the pixel.  In short:
 *				index = rgbmap[(((r << bits) | g) << bits) | b];
 * 	fast			If non-zero, the rgbmap will be constructed
 *				quickly.  If zero, the rgbmap will be built
 *				much slower, but more accurately.  In most
 *				cases, fast should be non-zero, as the error
 *				introduced by the approximation is usually
 *				small.  'Fast' is stored as an integer.
 *
 * colorquant returns the number of colors to which the image was
 * quantized.
 */
int
colorquant(unsigned char * red, unsigned char * green, unsigned char *blue,
           unsigned long pixels, unsigned char * colormap[3],
           int colors, int bits, unsigned char *rgbmap, int fast)
{
	Box	*Boxes;				/* Array of color boxes. */
	int	i,				/* Counter */
		OutColors,			/* # of entries computed */
		Colormax;			/* quantized full-intensity */ 
	float	Cfactor;			/* Conversion factor */

	ColormaxI = 1 << bits;			/* 2 ^ Bits */
	Colormax = ColormaxI - 1;
	Bits = bits;
	NPixels = pixels;
	Cfactor = (float)FULLINTENSITY / Colormax;

	Histogram = (unsigned long *)calloc(ColormaxI*ColormaxI*ColormaxI, 
						sizeof(long));
	Boxes = (Box *)malloc(colors * sizeof(Box));
	QuantHistogram(red, green, blue, &Boxes[0]);
	OutColors = CutBoxes(Boxes, colors);
	/*
	 * We now know the set of representative colors.  We now
	 * must fill in the colormap and convert the representatives
	 * from their 'prequantized' range to 0-FULLINTENSITY.
	 */
	for (i = 0; i < OutColors; i++) {
		colormap[0][i] =
			(unsigned char)(Boxes[i].mean[REDI] * Cfactor + 0.5);
		colormap[1][i] =
			(unsigned char)(Boxes[i].mean[GREENI] * Cfactor + 0.5);
		colormap[2][i] =
			(unsigned char)(Boxes[i].mean[BLUEI] * Cfactor + 0.5);
	}

	ComputeRGBMap(Boxes, OutColors, rgbmap, bits, colormap, fast);
	free((char *)Histogram);
	free((char *)Boxes);

	return OutColors;		/* Return # of colormap entries */
}

/*
 * Compute the histogram of the image as well as the projected frequency
 * arrays for the first world-encompassing box.
 */
void
QuantHistogram(register unsigned char *r, register unsigned char *g,
               register unsigned char * b, Box * box)
{
	unsigned long *rf, *gf, *bf, i;

	rf = box->freq[0];
	gf = box->freq[1];
	bf = box->freq[2];

	/*
	 * Zero-out the projected frequency arrays of the largest box.
	 * We compute both the histogram and the proj. frequencies of
	 * the first box at the same time to save a pass through the
	 * entire image. 
	 */
	memset(rf, 0, ColormaxI * sizeof(unsigned long));
	memset(gf, 0, ColormaxI * sizeof(unsigned long));
	memset(bf, 0, ColormaxI * sizeof(unsigned long));

	for (i = 0; i < NPixels; i++) {
		rf[*r]++;
		gf[*g]++;
		bf[*b]++;
		Histogram[((((*r++)<<Bits)|(*g++))<<Bits)|(*b++)]++;
	}
}

/*
 * Iteratively cut the boxes.
 */
int
CutBoxes(Box	*boxes, int	colors) 
{
	int curbox;

	boxes[0].low[REDI] = boxes[0].low[GREENI] = boxes[0].low[BLUEI] = 0;
	boxes[0].high[REDI] = boxes[0].high[GREENI] =
			      boxes[0].high[BLUEI] = ColormaxI;
	boxes[0].weight = NPixels;

	BoxStats(&boxes[0]);

	for (curbox = 1; curbox < colors; curbox++) {
		if (CutBox(&boxes[GreatestVariance(boxes, curbox)],
			   &boxes[curbox]) == FALSE)
				break;
	}

	return curbox;
}

/*
 * Return the number of the box in 'boxes' with the greatest variance.
 * Restrict the search to those boxes with indices between 0 and n-1.
 */
int GreatestVariance(Box *boxes, int n)
{
	register int i, whichbox = 0;
	float max;

	max = -1;
	for (i = 0; i < n; i++) {
		if (boxes[i].weightedvar > max) {
			max = boxes[i].weightedvar;
			whichbox = i;
		}
	}
	return whichbox;
}

/*
 * Compute mean and weighted variance of the given box.
 */
void BoxStats(register Box *box)
{
	register int i, color;
	unsigned long *freq;
	float mean, var;

	if(box->weight == 0) {
		box->weightedvar = 0;
		return;
	}

	box->weightedvar = 0.;
	for (color = 0; color < 3; color++) {
		var = mean = 0;
		i = box->low[color];
		freq = &box->freq[color][i];
		for (; i < box->high[color]; i++, freq++) {
			mean += i * *freq;
			var += i*i* *freq;
		}
		box->mean[color] = mean / (float)box->weight;
		box->weightedvar += var - box->mean[color]*box->mean[color]*
					(float)box->weight;
	}
	box->weightedvar /= NPixels;
}

/*
 * Cut the given box.  Returns TRUE if the box could be cut, FALSE otherwise.
 */
int CutBox(Box *box, Box *newbox)
{
	int i;
	float totalvar[3];
	Box newboxes[3][2];

	if (box->weightedvar == 0. || box->weight == 0)
		/*
		 * Can't cut this box.
		 */
		return FALSE;

	/*
	 * Find 'optimal' cutpoint along each of the red, green and blue
	 * axes.  Sum the variances of the two boxes which would result
	 * by making each cut and store the resultant boxes for 
	 * (possible) later use.
	 */
	for (i = 0; i < 3; i++) {
		if (FindCutpoint(box, i, &newboxes[i][0], &newboxes[i][1]))
			totalvar[i] = newboxes[i][0].weightedvar +
				newboxes[i][1].weightedvar;
		else
			totalvar[i] = H_HUGE;
	}

	/*
	 * Find which of the three cuts minimized the total variance
	 * and make that the 'real' cut.
	 */
	if (totalvar[REDI] <= totalvar[GREENI] &&
	    totalvar[REDI] <= totalvar[BLUEI]) {
		*box = newboxes[REDI][0];
		*newbox = newboxes[REDI][1];
	} else if (totalvar[GREENI] <= totalvar[REDI] &&
		 totalvar[GREENI] <= totalvar[BLUEI]) {
		*box = newboxes[GREENI][0];
		*newbox = newboxes[GREENI][1];
	} else {
		*box = newboxes[BLUEI][0];
		*newbox = newboxes[BLUEI][1];
	}

	return TRUE;
}

/*
 * Compute the 'optimal' cutpoint for the given box along the axis
 * indicated by 'color'.  Store the boxes which result from the cut
 * in newbox1 and newbox2.
 */
int FindCutpoint(Box *box, int color, Box * newbox1, Box *newbox2)
{
	float u, v, max;
	int i, maxindex, minindex, cutpoint;
	unsigned long optweight, curweight;

	if (box->low[color] + 1 == box->high[color])
		return FALSE;	/* Cannot be cut. */
	minindex = (int)((box->low[color] + box->mean[color]) * 0.5);
	maxindex = (int)((box->mean[color] + box->high[color]) * 0.5);

	cutpoint = minindex;
	optweight = box->weight;

	curweight = 0.;
	for (i = box->low[color] ; i < minindex ; i++)
		curweight += box->freq[color][i];
	u = 0.;
	max = -1;
	for (i = minindex; i <= maxindex ; i++) {
		curweight += box->freq[color][i];
		if (curweight == box->weight)
			break;
		u += (float)(i * box->freq[color][i]) /
					(float)box->weight;
		v = ((float)curweight / (float)(box->weight-curweight)) *
				(box->mean[color]-u)*(box->mean[color]-u);
		if (v > max) {
			max = v;
			cutpoint = i;
			optweight = curweight;
		}
	}
	cutpoint++;
	*newbox1 = *newbox2 = *box;
	newbox1->weight = optweight;
	newbox2->weight -= optweight;
	newbox1->high[color] = cutpoint;
	newbox2->low[color] = cutpoint;
	UpdateFrequencies(newbox1, newbox2);
	BoxStats(newbox1);
	BoxStats(newbox2);

	return TRUE;	/* Found cutpoint. */
}

/*
 * Update projected frequency arrays for two boxes which used to be
 * a single box.
 */
void UpdateFrequencies(register Box *box1, register Box *box2)
{
	register unsigned long myfreq, *h;
	register int b, g, r;
	int roff;

	memset(box1->freq[0], 0, ColormaxI * sizeof(unsigned long));
	memset(box1->freq[1], 0, ColormaxI * sizeof(unsigned long));
	memset(box1->freq[2], 0, ColormaxI * sizeof(unsigned long)); 

	for (r = box1->low[0]; r < box1->high[0]; r++) {
		roff = r << Bits;
		for (g = box1->low[1];g < box1->high[1]; g++) {
			b = box1->low[2];
			h = Histogram + (((roff | g) << Bits) | b);
			for (; b < box1->high[2]; b++) {
				if ((myfreq = *h++) == 0)
					continue;
				box1->freq[0][r] += myfreq;
				box1->freq[1][g] += myfreq;
				box1->freq[2][b] += myfreq;
				box2->freq[0][r] -= myfreq;
				box2->freq[1][g] -= myfreq;
				box2->freq[2][b] -= myfreq;
			}
		}
	}
}

/*
 * Compute RGB to colormap index map.
 */
void ComputeRGBMap(Box *boxes, int colors, unsigned char *rgbmap,
                    int bits, unsigned char *colormap[3], int fast)
{
	register int i;

	if (fast) {
		/*
		 * The centroid of each box serves as the representative
		 * for each color in the box.
		 */
		for (i = 0; i < colors; i++)
			SetRGBmap(i, &boxes[i], rgbmap, bits);
	} else
		/*
		 * Find the 'nearest' representative for each
		 * pixel.
		 */
		find_colors(boxes, colors, rgbmap, bits, colormap);
}

/*
 * Make the centroid of "boxnum" serve as the representative for
 * each color in the box.
 */
void SetRGBmap(int boxnum, Box *box, unsigned char *rgbmap, int bits)
{
	register int r, g, b;
	
	for (r = box->low[REDI]; r < box->high[REDI]; r++) {
		for (g = box->low[GREENI]; g < box->high[GREENI]; g++) {
			for (b = box->low[BLUEI]; b < box->high[BLUEI]; b++) {
				rgbmap[(((r<<bits)|g)<<bits)|b]=(char)boxnum;
			}
		}
	}
}
/*
 * Form colormap and NearestColor arrays.
 */
void find_colors(Box *boxes, int colors, unsigned char *rgbmap,
                 int bits, unsigned char *colormap[3])
{
	register int i;
	int num, *neighbors;

	neighbors = (int *)malloc(colors * sizeof(int));

	/*
	 * Form map of representative (nearest) colors.
	 */
	for (i = 0; i < colors; i++) {
		/*
		 * Create list of candidate neighbors and
		 * find closest representative for each
		 * color in the box.
		 */
		num = getneighbors(boxes, i, neighbors, colors, colormap);
		makenearest(boxes, i, num, neighbors, rgbmap, bits, colormap);
	}
	free((char *)neighbors);
}

/*
 * In order to minimize our search for 'best representative', we form the
 * 'neighbors' array.  This array contains the number of the boxes whose
 * centroids *might* be used as a representative for some color in the
 * current box.  We need only consider those boxes whose centroids are closer
 * to one or more of the current box's corners than is the centroid of the
 * current box. 'Closeness' is measured by Euclidean distance.
 */
int getneighbors(Box *boxes, int num, int * neighbors, int colors, 
                  unsigned char *colormap[3])
{
	register int i, j;
	Box *bp;
	float dist, LowR, LowG, LowB, HighR, HighG, HighB, ldiff, hdiff;

	bp = &boxes[num];

	ldiff = bp->low[REDI] - bp->mean[REDI];
	ldiff *= ldiff;
	hdiff = bp->high[REDI] - bp->mean[REDI];
	hdiff *= hdiff;
	dist = MAX(ldiff, hdiff);

	ldiff = bp->low[GREENI] - bp->mean[GREENI];
	ldiff *= ldiff;
	hdiff = bp->high[GREENI] - bp->mean[GREENI];
	hdiff *= hdiff;
	dist += MAX(ldiff, hdiff);

	ldiff = bp->low[BLUEI] - bp->mean[BLUEI];
	ldiff *= ldiff;
	hdiff = bp->high[BLUEI] - bp->mean[BLUEI];
	hdiff *= hdiff;
	dist += MAX(ldiff, hdiff);

#ifdef IRIS
	dist = fsqrt(dist);
#else
	dist = (float)sqrt((double)dist);
#endif

	/*
	 * Loop over all colors in the colormap, the ith entry of which
	 * corresponds to the ith box.
	 *
	 * If the centroid of a box is as close to any corner of the
	 * current box as is the centroid of the current box, add that
	 * box to the list of "neighbors" of the current box.
	 */
	HighR = (float)bp->high[REDI] + dist;
	HighG = (float)bp->high[GREENI] + dist;
	HighB = (float)bp->high[BLUEI] + dist;
	LowR = (float)bp->low[REDI] - dist;
	LowG = (float)bp->low[GREENI] - dist;
	LowB = (float)bp->low[BLUEI] - dist;
	for (i = j = 0, bp = boxes; i < colors; i++, bp++) {
		if (LowR <= bp->mean[REDI] && HighR >= bp->mean[REDI] &&
		    LowG <= bp->mean[GREENI] && HighG >= bp->mean[GREENI] &&
		    LowB <= bp->mean[BLUEI] && HighB >= bp->mean[BLUEI])
			neighbors[j++] = i;
	}

	return j;	/* Return the number of neighbors found. */
}

/*
 * Assign representative colors to every pixel in a given box through
 * the construction of the NearestColor array.  For each color in the
 * given box, we look at the list of neighbors passed to find the
 * one whose centroid is closest to the current color.
 */
void makenearest(Box *boxes, int boxnum, int nneighbors, int * neighbors,
                 unsigned char *rgbmap, int bits, unsigned char *colormap[3])
{
	register int n, b, g, r;
	float rdist, gdist, bdist, dist, mindist;
	int which = -1, *np;
	Box *box;
	extern unsigned long *Histogram;

	box = &boxes[boxnum];

	for (r = box->low[REDI]; r < box->high[REDI]; r++) {
		for (g = box->low[GREENI]; g < box->high[GREENI]; g++) {
			for (b = box->low[BLUEI]; b < box->high[BLUEI]; b++) {
/*
 * The following two lines should be commented out if the RGBmap is going
 * to be used for images other than the one given.
 */
				if (Histogram[(((r<<bits)|g)<<bits)|b] == 0)
					continue;
				mindist = H_HUGE;
				/*
				 * Find the colormap entry which is
				 * closest to the current color.
				 */
				np = neighbors;
				for (n = 0; n < nneighbors; n++, np++) {
					rdist = r-boxes[*np].mean[REDI];
					gdist = g-boxes[*np].mean[GREENI];
					bdist = b-boxes[*np].mean[BLUEI];
					dist = rdist*rdist + gdist*gdist + bdist*bdist;
					if (dist < mindist) {
						mindist = dist;
						which = *np; 
					}
				}
				/*
				 * The colormap entry closest to this
				 * color is used as a representative.
				 */
				rgbmap[(((r<<bits)|g)<<bits)|b] = which;
			}
		}
	}
}
