/**
* $Id: fitellipse.c,v 1.3 2006/07/19 21:22:46 mz Exp mxz $
*
* This code is a C++ conversion of Maurizio Pilu's Java implementation (see
* the header of the original file below).
* 
* Calculating the inverse of a matrix now uses LU decomposition instead of
* Gauss-Jordan method (as recommended by Numerical Recipes).
* 
* Uses normalisation of data to values around 1 for improved numerical
* stability (see Fitzgibbons website for explanations
* http://research.microsoft.com/%7Eawf/ellipse).
*/

//**************************************************************************
// This java code is  an interactive demo of the first ellipse-specific
// direct fitting method presented in the papers:

//    M. Pilu, A. Fitzgibbon, R.Fisher ``Ellipse-specific Direct
//    least-square Fitting '' , IEEE International Conference on Image
//    Processing, Lausanne, September 1996. (poscript) (HTML)

//    A. Fitzgibbon, M. Pilu , R.Fisher ``Direct least-square fitting of
//    Ellipses '' , International Conference on Pattern Recognition, Vienna,
//    August 1996. (poscript) - Extended version available as DAI Research
//    Paper #794

// The demo can be tried out at
//   http://www.dai.ed.ac.uk/students/maurizp/ElliFitDemo/demo.html

// The code was written by Maurizio Pilu , University of Edinburgh.  The
// applet's graphic interface was much inspired by the Curve Applet
// written by Michael Heinrichs at SFU, Vancouver:
// http://fas.sfu.ca:80/1/cs/people/GradStudents/heinrica/personal/curve.html

// Some math routines are from the "Numerical Recipes in C" by
// Press/Teukolsky/Vettering/Flannery, Cambridge Uiniversity Press,
// Second Edition (1988). PLEASE READ COPYRIGHT ISSUES ON THE NUMERICAL
// RECIPES BOOK.

// NOTE: Some parts of the program are rather scruffy. The author
//       will tidy it up whan he has some spare time.

// DISCLAIMER: The authors and the department assume no responsabilities
//             whatsoever for any wrong use of this code.

// COPYRIGHT: Any commercial use of the code and the method is forbidden
//            without written authorization from the authors.
//**************************************************************************

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "nrutil.h"

static const double TINY  = 1.0e-20;

static double mean(double *v, int nl, int nh)
{
  double sum = 0.;
  int i;
  for(i = nl; i <= nh; i++)
    sum += v[i];
  return sum/(double)(nh - nl+ 1);
}

static double min(double *v, int nl, int nh)
{
  double min = HUGE;
  int i;
  for(i = nl; i <= nh; i++)
    if(v[i] < min)
      min = v[i];
  return min;
}

static double max(double *v, int nl, int nh)
{
  double max = -HUGE;
  int i;
  for(i = nl; i <= nh; i++)
    if(v[i] > max)
      max = v[i];
  return max;
}

static inline double det_2x2(double a11, double a12, double a21, double a22)
{
  return a11*a22 - a12*a21;
}

static double det_3x3(double **a)
{
  return a[1][1]*det_2x2(a[2][2], a[2][3], a[3][2], a[3][3]) -
    a[1][2]*det_2x2(a[2][1], a[2][3], a[3][1], a[3][3]) +
    a[1][3]*det_2x2(a[2][1], a[2][2], a[3][1], a[3][2]);
}

static void inverse_3x3(double **a, double **b)
{
  double det = det_3x3(a);
  if(fabs(det) < TINY)
    nrerror("singular matrix in routine inverse_3x3");
  b[1][1] = det_2x2(a[2][2], a[2][3], a[3][2], a[3][3])/det;
  b[1][2] = det_2x2(a[1][3], a[1][2], a[3][3], a[3][2])/det;
  b[1][3] = det_2x2(a[1][2], a[1][3], a[2][2], a[2][3])/det;
  b[2][1] = det_2x2(a[2][3], a[2][1], a[3][3], a[3][1])/det;
  b[2][2] = det_2x2(a[1][1], a[1][3], a[3][1], a[3][3])/det;
  b[2][3] = det_2x2(a[1][3], a[1][1], a[2][3], a[2][1])/det;
  b[3][1] = det_2x2(a[2][1], a[2][2], a[3][1], a[3][2])/det;
  b[3][2] = det_2x2(a[1][2], a[1][1], a[3][2], a[3][1])/det;
  b[3][3] = det_2x2(a[1][1], a[1][2], a[2][1], a[2][2])/det;
}

static void AperB(double **A, double **B, double **res,
           int rowA, int colA, int rowB, int colB)
{
  int p,q,l;
  for (p=1;p<=rowA;p++)
    for (q=1;q<=colB;q++)
    {
      res[p][q]=0.0;
      for (l=1;l<=colA;l++)
        res[p][q]=res[p][q]+A[p][l]*B[l][q];
    }
}

static void A_TperB(double **A, double **B, double **res,
       int rowA, int colA, int rowB, int colB)
{
  int p,q,l;
  for (p=1;p<=colA;p++)
    for (q=1;q<=colB;q++)
    {
      res[p][q]=0.0;
      for (l=1;l<=rowA;l++)
        res[p][q]=res[p][q]+A[l][p]*B[l][q];
    }
}

static void AperB_T(double **A, double **B, double **res,
       int rowA, int colA, int rowB, int colB)
{
  int p,q,l;
  for (p=1;p<=colA;p++)
    for (q=1;q<=colB;q++)
    {
      res[p][q]=0.0;
      for (l=1;l<=rowA;l++)
        res[p][q]=res[p][q]+A[p][l]*B[q][l];
    }
}

static void ludcmp(double **a, int n, int *indx, double *d)
{
	int i,imax=-1,j,k;
	double big,dum,sum,temp;
	double *vv;

	vv=dvector(1,n);
	*d=1.0;
	for (i=1;i<=n;i++) {
		big=0.0;
		for (j=1;j<=n;j++)
			if ((temp=fabs(a[i][j])) > big) big=temp;
		if (big == 0.0) nrerror("Singular matrix in routine ludcmp");
		vv[i]=1.0/big;
	}
	for (j=1;j<=n;j++) {
		for (i=1;i<j;i++) {
			sum=a[i][j];
			for (k=1;k<i;k++) sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
		}
		big=0.0;
		for (i=j;i<=n;i++) {
			sum=a[i][j];
			for (k=1;k<j;k++)
				sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
			if ( (dum=vv[i]*fabs(sum)) >= big) {
				big=dum;
				imax=i;
			}
		}
		if (j != imax) {
			for (k=1;k<=n;k++) {
				dum=a[imax][k];
				a[imax][k]=a[j][k];
				a[j][k]=dum;
			}
			*d = -(*d);
			vv[imax]=vv[j];
		}
		indx[j]=imax;
		if (a[j][j] == 0.0) a[j][j]=TINY;
		if (j != n) {
			dum=1.0/(a[j][j]);
			for (i=j+1;i<=n;i++) a[i][j] *= dum;
		}
	}
	free_dvector(vv,1,n);
}

static void lubksb(double **a, int n, int *indx, double b[])
{
	int i,ii=0,ip,j;
	double sum;

	for (i=1;i<=n;i++) {
		ip=indx[i];
		sum=b[ip];
		b[ip]=b[i];
		if (ii)
			for (j=ii;j<=i-1;j++) sum -= a[i][j]*b[j];
		else if (sum) ii=i;
		b[i]=sum;
	}
	for (i=n;i>=1;i--) {
		sum=b[i];
		for (j=i+1;j<=n;j++) sum -= a[i][j]*b[j];
		b[i]=sum/a[i][i];
	}
}

/*
 * Calculate the inverse of a matrix using the LU decomposition.
 * On exit b contains the inverse of the original matrix a, which will be
 * destroyed.
 */
static void inverse(double **a, double **b, int n)
{
  int i, j;
  double d;
  int *indx = ivector(1, n);
  double *col = dvector(1, n);

  ludcmp(a, n, indx, &d);
  for(j = 1; j <= n; j++)
  {
    for(i = 1; i <= n; i++)
      col[i] = 0.;
    col[j] = 1.;
    lubksb(a, n, indx, col);
    for(i = 1; i <= n; i++)
      b[i][j] = col[i];
  }
  free_dvector(col, 1, n);
  free_ivector(indx, 1, n);
}

static void ROTATE(double **a, int i, int j, int k, int l, double tau,
    double s)
{
  double g,h;
  g=a[i][j];h=a[k][l];a[i][j]=g-s*(h+g*tau);
  a[k][l]=h+s*(g-h*tau);
}

static void jacobi(double **a, int n, double *d, double **v, int nrot)
{
  int j,iq,ip,i;
  double tresh,theta,tau,t,sm,s,h,g,c;

  double *b = dvector(1, n);
  double *z = dvector(1, n);

  for (ip=1;ip<=n;ip++) {
    for (iq=1;iq<=n;iq++) v[ip][iq]=0.0;
    v[ip][ip]=1.0;
  }
  for (ip=1;ip<=n;ip++) {
    b[ip]=d[ip]=a[ip][ip];
    z[ip]=0.0;
  }
  nrot=0;
  for (i=1;i<=50;i++) {
    sm=0.0;
    for (ip=1;ip<=n-1;ip++) {
      for (iq=ip+1;iq<=n;iq++)
        sm += fabs(a[ip][iq]);
    }
    if (sm == 0.0) {
      free_dvector(z, 1, n);
      free_dvector(b, 1, n);
      return;
    }
    if (i < 4)
      tresh=0.2*sm/(n*n);
    else
      tresh=0.0;
    for (ip=1;ip<=n-1;ip++) {
      for (iq=ip+1;iq<=n;iq++) {
        g=100.0*fabs(a[ip][iq]);
        if (i > 4 && fabs(d[ip])+g == fabs(d[ip])
      && fabs(d[iq])+g == fabs(d[iq]))
    a[ip][iq]=0.0;
        else if (fabs(a[ip][iq]) > tresh) {
    h=d[iq]-d[ip];
    if (fabs(h)+g == fabs(h))
      t=(a[ip][iq])/h;
    else {
      theta=0.5*h/(a[ip][iq]);
      t=1.0/(fabs(theta)+sqrt(1.0+theta*theta));
      if (theta < 0.0) t = -t;
    }
    c=1.0/sqrt(1+t*t);
    s=t*c;
    tau=s/(1.0+c);
    h=t*a[ip][iq];
    z[ip] -= h;
    z[iq] += h;
    d[ip] -= h;
    d[iq] += h;
    a[ip][iq]=0.0;
    for (j=1;j<=ip-1;j++) {
      ROTATE(a,j,ip,j,iq,tau,s);
      }
    for (j=ip+1;j<=iq-1;j++) {
      ROTATE(a,ip,j,j,iq,tau,s);
      }
    for (j=iq+1;j<=n;j++) {
      ROTATE(a,ip,j,iq,j,tau,s);
      }
    for (j=1;j<=n;j++) {
      ROTATE(v,j,ip,j,iq,tau,s);
      }
    ++nrot;
        }
      }
    }
    for (ip=1;ip<=n;ip++) {
      b[ip] += z[ip];
      d[ip]=b[ip];
      z[ip]=0.0;
    }
  }
  free_dvector(z, 1, n);
  free_dvector(b, 1, n);
  nrerror("Too many iterations in routine JACOBI");
}

// Perform the Cholesky decomposition
// Return the lower triangular L  such that L*L'=A
static void choldc(double **a, int n, double **l)
{
  int i,j,k;
  double sum;
  double *p = dvector(1, n);

  for (i=1; i<=n; i++)  {
    for (j=i; j<=n; j++)  {
      for (sum=a[i][j],k=i-1;k>=1;k--) sum -= a[i][k]*a[j][k];
      if (i == j) {
        if (sum<=0.0)
        {
          free_dvector(p, 1, n);
          nrerror("Cholesky decomposition: A is not poitive definite");
        }
        p[i]=sqrt(sum);
      }
      else
        a[j][i]=sum/p[i];
    }
  }
  for (i=1; i<=n; i++)
    for (j=i; j<=n; j++)
      if (i==j)
        l[i][i] = p[i];
      else
      {
        l[j][i]=a[j][i];
        l[i][j]=0.0;
      }
  free_dvector(p, 1, n);
}

/**
 * Transform an ellipse from the general quadratic form
 *   a11 x^2 + 2 a12 x y + a22 y^2 + b1 x + b2 y + c = 0
 * to the center/angle form given by center point (x, y), semiaxes lengths
 * (a,b) and angle between major axis and x-axis (phi).
 *
 * From 
 * www.magic-software.com/Documentation/Information/InformationAboutEllipses.pdf
 */
static void EllipseGenQuad2CentAng(double a11, double two_a12, double a22,
    double b1, double b2, double c,
    double *x, double *y, double *a, double *b, double *phi)
{
  double t, m, m11, m12, m22, l;
  double a12 = two_a12/2.;
  if(4*a11*a22-DSQR(a12) < TINY)
  {
    printf("* 4ac - b^2 = %.12f (should be > 0)\n",
        4*a11*a22 - two_a12*two_a12);
    printf("* parameters: %f %f %f %f %f %f\n", a11, two_a12, a22, b1, b2, c);
  }
  t = 2.*(DSQR(a12) - a11*a22);
  assert(t != 0.);
  *x = (a22*b1 - a12*b2)/t;
  *y = (a11*b2 - a12*b1)/t;
  t = a11*DSQR(*x) + 2*a12*(*x)*(*y) + a22*DSQR(*y) - c;
  assert(t != 0.);
  m = 1./t;
  m11 = m*a11;
  m12 = m*a12;
  m22 = m*a22;
  t = sqrt(DSQR(m11 - m22) + 4.*DSQR(m12));
  l = (m11 + m22 + t)/2.;
  assert(l != 0.);
  *b = 1./sqrt(l);
  l = (m11 + m22 - t)/2.;
  assert(l != 0.);
  *a = 1./sqrt(l);
  assert(a22 - a11 != 0.);
  *phi = atan(-2.*a12/(a22 - a11))/2.;
}

/**
 * Fits ellipse to points using B2AC algorithm.
 * Number of points must be >= 6
 * Returns true if fitting was successful, false otherwise.
 */
int fit_ellipse(double x[], double y[], int n,
    double *par_x, double *par_y, double *par_a, double *par_b, double *par_phi)
{
  int i, j;
  double tx,ty, mx, my, sx, sy;
  double zero=10e-20;
  int solind=0;
  int nrot=0;
  int rtc = 0;
  double *xn = dvector(1, n);
  double *yn = dvector(1, n);
  double **D = dmatrix(1, n, 1, 6);
  double **S = dmatrix(1, 6, 1, 6);
  double **Const = dmatrix(1, 6, 1, 6);
  double **temp = dmatrix(1, 6, 1, 6);
  double **L = dmatrix(1, 6, 1, 6);
  double **C = dmatrix(1, 6, 1, 6);
  double **invL = dmatrix(1, 6, 1, 6);
  double *d = dvector(1, 6);
  double **V = dmatrix(1, 6, 1, 6);
  double **sol = dmatrix(1, 6, 1, 6);
  double *pvec = dvector(1, 6);
  double *par = dvector(1, 6);

  if(n < 6)
    return rtc;

  // for normalising data to values around 1 (shift and scale)
  // Note: x and y are C-arrays, starting at 0!
  mx = mean(x, 0, n-1);
  my = mean(y, 0, n-1);
  sx = (max(x, 0, n-1) - min(x, 0, n-1))/2;
  sy = (max(y, 0, n-1) - min(y, 0, n-1))/2; 

  // Build nx6 design matrix
  // Note: x and y are C-arrays, starting at 0!
  for(i = 1; i <= n; i++)
  {
    // use normalised x and y
    tx = (x[i-1] - mx)/sx;
    ty = (y[i-1] - my)/sy;
    D[i][1] = tx*tx;
    D[i][2] = tx*ty;
    D[i][3] = ty*ty;
    D[i][4] = tx;
    D[i][5] = ty;
    D[i][6] = 1.0;
  }

  // Build 6x6 scatter matrix
  A_TperB(D,D,S,n,6,n,6);

  // Build 6x6 constraint matrix
  Const[1][3] = -2;
  Const[2][2] = 1;
  Const[3][1] = -2;

  choldc(S,6,L);

  inverse(L,invL,6);

  AperB_T(Const,invL,temp,6,6,6,6);
  AperB(invL,temp,C,6,6,6,6);

  jacobi(C,6,d,V,nrot);

  A_TperB(invL,V,sol,6,6,6,6);

  // Now normalize them
  for (j=1;j<=6;j++)  // Scan columns
  {
    double mod = 0.0;
    for (i=1;i<=6;i++)
      mod += sol[i][j]*sol[i][j];
    for (i=1;i<=6;i++)
      sol[i][j] /=  sqrt(mod);
  }

  // for the FWF (ellipse specific) case:
  for (i=1; i<=6; i++)
    if (d[i]<0 && fabs(d[i])>zero)
      solind = i;

  // Now fetch the right solution
  for (j=1;j<=6;j++)
    pvec[j] = sol[j][solind];

  // if 4ac - b^2 > 0
  if(4*pvec[1]*pvec[3] - DSQR(pvec[2]) > TINY)
  {
    // Un-normalise (shift and scale)
    par[1] = pvec[1]*sy*sy;
    par[2] = pvec[2]*sx*sy;
    par[3] = pvec[3]*sx*sx;
    par[4] = -2*pvec[1]*sy*sy*mx - pvec[2]*sx*sy*my + pvec[4]*sx*sy*sy;
    par[5] = -pvec[2]*sx*sy*mx - 2*pvec[3]*sx*sx*my + pvec[5]*sx*sx*sy;
    par[6] = pvec[1]*sy*sy*mx*mx + pvec[2]*sx*sy*mx*my + pvec[3]*sx*sx*my*my
        - pvec[4]*sx*sy*sy*mx - pvec[5]*sx*sx*sy*my
        + pvec[6]*sx*sx*sy*sy;
    // transform to center/angle form
    EllipseGenQuad2CentAng(par[1], par[2], par[3], par[4], par[5], par[6],
        par_x, par_y, par_a, par_b, par_phi);
    rtc = 1;
  }

  free_dvector(xn, 1, n);
  free_dvector(yn, 1, n);
  free_dmatrix(D, 1, n, 1, 6);
  free_dmatrix(S, 1, 6, 1, 6);
  free_dmatrix(Const, 1, 6, 1, 6);
  free_dmatrix(temp, 1, 6, 1, 6);
  free_dmatrix(L, 1, 6, 1, 6);
  free_dmatrix(C, 1, 6, 1, 6);
  free_dmatrix(invL, 1, 6, 1, 6);
  free_dvector(d, 1, 6);
  free_dmatrix(V, 1, 6, 1, 6);
  free_dmatrix(sol, 1, 6, 1, 6);
  free_dvector(pvec, 1, 6);
  free_dvector(par, 1, 6);

  return rtc;

  /* This is an improved method (see again Fitzgibbons website)
   * note: this is numerically more stable, but requires eigenvalues of a
   * non-symmetric matrix, and i don't know how to do that.

  double **tmpA, **tmpB, **tmpC, **tmpD;
  double **tmpC_i = d_matrix(1, 3, 1, 3);
  double **tmpD_i = d_matrix(1, 3, 1, 3);
  double **tmpE = d_matrix(1, 3, 1, 3);
  double **tmpF = d_matrix(1, 3, 1, 3);
  double **tmpG = d_matrix(1, 3, 1, 3);
  double **tmpH = d_matrix(1, 3, 1, 3);

  // Solve eigensystem (new way, numerically stabler in C);
  // Break problem into blocks
  tmpA = submatrix_of(S, 1, 6, 1, 6, 1, 3, 1, 3);
  tmpB = submatrix_of(S, 1, 6, 1, 6, 1, 3, 4, 6);
  tmpC = submatrix_of(S, 1, 6, 1, 6, 4, 6, 4, 6);
  tmpD = submatrix_of(C, 1, 6, 1, 6, 1, 3, 1, 3);
  inverse(tmpC, tmpC_i, 3);
  // E = C_i*B'
  AperB_T(tmpC_i, tmpB, tmpE, 3, 3, 3, 3);
  // F = B*E
  AperB(tmpB, tmpE, tmpF, 3, 3, 3, 3);
  // G = A - F
  AminusB(tmpA, tmpF, tmpG, 3, 3);
  inverse(tmpD, tmpD_i, 3);
  // H = D_i*G
  AperB(tmpD_i, tmpG, tmpH, 3, 3, 3, 3);
  // Now solve eigenproblem for H (which is non-symmetric)
  eig(tmpH);???
 
  free_dmatrix(tmpA, 1, 3, 1, 3);
  free_dmatrix(tmpB, 1, 3, 1, 3);
  free_dmatrix(tmpC, 1, 3, 1, 3);
  free_dmatrix(tmpD, 1, 3, 1, 3);
  free_dmatrix(tmpC_i, 1, 3, 1, 3);
  free_dmatrix(tmpD_i, 1, 3, 1, 3);
  free_dmatrix(tmpE, 1, 3, 1, 3);
  free_dmatrix(tmpF, 1, 3, 1, 3);
  free_dmatrix(tmpG, 1, 3, 1, 3);
  free_dmatrix(tmpH, 1, 3, 1, 3);
  */
}

