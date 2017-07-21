/*---------------------------------------------------------------------*/
/*
  G. Matas, 17-Jun-93
      created with the help of Homam Dabis

   Fary3_2RGBarya, Fary3_2RGBaryb, RGBary2Fary3a, RGBary2Fary3b by 
      Rupert Young
*/
/*---------------------------------------------------------------------*/
static char sccsid[]="@(#)convertAry.c	2.10	95/05/23 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include "ary.h"

/* convert FARY to BARY by setting all values bove  255 to 255 , */
/* all negative  values to 0 and rest is 'floored' (not rounded!)*/ 
BARY *  Fary2Bary(FARY *fary)
{
   int i,j;
   BARY * bary = (BARY *)
	 makary(fary->lb1,fary->ub1,fary->lb2,fary->ub2,BYTEMATRIX);

   for(i=fary->lb1; i<=fary->ub1; i++)
    for(j=fary->lb2; j<=fary->ub2; j++)
      if      (fary->el[i][j] > 255.0) bary->el[i][j] = 255;
      else if (fary->el[i][j] < 0.0)   bary->el[i][j] = 0;
      else     bary->el[i][j] = (unsigned char) fary->el[i][j];
				   
   return bary;
}
BARY *  Iary2Bary(IARY *in)
{
   int i,j;
   BARY * bary = (BARY *)
	 makary(in->lb1,in->ub1,in->lb2,in->ub2,BYTEMATRIX);

   for(i=in->lb1; i<=in->ub1; i++)
    for(j=in->lb2; j<=in->ub2; j++)
      if      (in->el[i][j] > 255.0) bary->el[i][j] = 255;
      else if (in->el[i][j] < 0.0)   bary->el[i][j] = 0;
      else     bary->el[i][j] = (unsigned char) in->el[i][j];
				   
   return bary;
}

IARY *  Bary2Iary(BARY *in)
{
   int i,j;
   IARY * iary = (IARY *)
	 makary(in->lb1,in->ub1,in->lb2,in->ub2,INTEGERMATRIX);

   for(i=in->lb1; i<=in->ub1; i++)
    for(j=in->lb2; j<=in->ub2; j++)
      iary->el[i][j] = 256 * (int) in->el[i][j];
				   
   return iary;
}
/*---------------------------------------------------------------------*/
/* scale bary to fary, maximum of FARY becomes 255 */
/* IT IS ASSUMED THAT ALL VALUES IN FARY ARE POSITIVE !! */
BARY *  ScaleFary2Bary(FARY *fary)
{
   int i,j;
   BARY * bary = (BARY *)
	 makary(fary->lb1,fary->ub1,fary->lb2,fary->ub2,BYTEMATRIX);

   float max = 0.0;

   for(i=fary->lb1; i<=fary->ub1; i++)
    for(j=fary->lb2; j<=fary->ub2; j++)
      if (fary->el[i][j]>max) max = fary->el[i][j]; 
				   
   for(i=fary->lb1; i<=fary->ub1; i++)
    for(j=fary->lb2; j<=fary->ub2; j++)
      bary->el[i][j] = (unsigned char) 255.0*fary->el[i][j]/max;
				   
   return bary;
}
BARY *  ScaleMinMaxFary2Bary(FARY *fary)
{
   int i,j;
   BARY * bary = (BARY *)
	 makary(fary->lb1,fary->ub1,fary->lb2,fary->ub2,BYTEMATRIX);

   float max = 0.0;
   float min =256.0;

   for(i=fary->lb1; i<=fary->ub1; i++)
    for(j=fary->lb2; j<=fary->ub2; j++)
    {
      if      (fary->el[i][j]>max) max = fary->el[i][j]; 
      else if (fary->el[i][j]<min) min = fary->el[i][j]; 
    }
				   
   if(max==min) {InitialiseBary(bary,128); return bary;}

   for(i=fary->lb1; i<=fary->ub1; i++)
    for(j=fary->lb2; j<=fary->ub2; j++)
      bary->el[i][j] = (unsigned char) 255.0*(fary->el[i][j]-min)/(max-min);
				   
   return bary;
}
/*---------------------------------------------------------------------*/
RGBARY *  XRGBary2RGBary(XRGBARY *xrgb)
{
   int i,j;
   RGBARY * rgb = 
     consAry1(xrgb->lb1,xrgb->ub1,xrgb->lb2, xrgb->ub2,sizeof(t_rgb),NULL);


   for(i=xrgb->lb1; i<=xrgb->ub1; i++)
    for(j=xrgb->lb2; j<=xrgb->ub2; j++)
    {
      rgb->el[i][j].st.r = xrgb->el[i][j].st.r; 
      rgb->el[i][j].st.g = xrgb->el[i][j].st.g; 
      rgb->el[i][j].st.b = xrgb->el[i][j].st.b; 
    }
				   
   return rgb;
}
RGBARY *  XBGRary2RGBary(XRGBARY *xrgb)
{
   int i,j;
   RGBARY * rgb =  (RGBARY*)
     makary(xrgb->lb1,xrgb->ub1,xrgb->lb2, xrgb->ub2,RGBMATRIX);

 
   for(i=xrgb->lb1; i<=xrgb->ub1; i++)
    for(j=xrgb->lb2; j<=xrgb->ub2; j++)
    {
      rgb->el[i][j].st.r = xrgb->el[i][j].st.b;
      rgb->el[i][j].st.g = xrgb->el[i][j].st.g;
      rgb->el[i][j].st.b = xrgb->el[i][j].st.r;
    }
 
   return rgb;
}
 
RGBARY *  RGBXary2RGBary(RGBXARY *rgbx)
{
   int i,j;
   RGBARY * rgb =  (RGBARY*)
     makary(rgbx->lb1,rgbx->ub1,rgbx->lb2, rgbx->ub2,RGBMATRIX);


   for(i=rgbx->lb1; i<=rgbx->ub1; i++)
    for(j=rgbx->lb2; j<=rgbx->ub2; j++)
    {
      rgb->el[i][j].st.r = rgbx->el[i][j].st.r; 
      rgb->el[i][j].st.g = rgbx->el[i][j].st.g; 
      rgb->el[i][j].st.b = rgbx->el[i][j].st.b; 
    }
				   
   return rgb;
}
void RGBXaryToRGBary(RGBXARY *rgbx,RGBARY *rgb)
{
   int i,j;

   for(i=rgbx->lb1; i<=rgbx->ub1; i++)
    for(j=rgbx->lb2; j<=rgbx->ub2; j++)
    {
      rgb->el[i][j].st.r = rgbx->el[i][j].st.r; 
      rgb->el[i][j].st.g = rgbx->el[i][j].st.g; 
      rgb->el[i][j].st.b = rgbx->el[i][j].st.b; 
    }
}

/*---------------------------------------------------------------------*/
SARY *  Bary2Sary(BARY *bary)
{
   int i,j;
   SARY * sary = (SARY *)
	 makary(bary->lb1,bary->ub1,bary->lb2,bary->ub2,SHORTINTMATRIX);

   for(i=bary->lb1; i<=bary->ub1; i++)
    for(j=bary->lb2; j<=bary->ub2; j++)
      sary->el[i][j] = bary->el[i][j];

    return sary;
}

BARY *  Sary2Bary(SARY *sary)
{
   int i,j;
   BARY * bary = (BARY *)
	 makary(sary->lb1,sary->ub1,sary->lb2,sary->ub2,BYTEMATRIX);

   for(i=sary->lb1; i<=sary->ub1; i++)
    for(j=sary->lb2; j<=sary->ub2; j++)
      bary->el[i][j] = sary->el[i][j];

    return bary;
}

BARY *  USary2Bary(USARY *usary)
{
   int i,j;
   BARY * bary = (BARY *)
	 makary(usary->lb1,usary->ub1,usary->lb2,usary->ub2,BYTEMATRIX);

   for(i=usary->lb1; i<=usary->ub1; i++)
    for(j=usary->lb2; j<=usary->ub2; j++)
      bary->el[i][j] = usary->el[i][j];

    return bary;
}
void  USaryToBary(USARY *usary, BARY* bary)
{
   int i,j;

   for(i=usary->lb1; i<=usary->ub1; i++)
    for(j=usary->lb2; j<=usary->ub2; j++)
      bary->el[i][j] = usary->el[i][j];

}



/*--------------------------------------------------------------------*/
#define cut(x) (x<0?0:x>255?255:(x))
t_frgb Rgb2Frgb(t_rgb rgb)
{
  t_frgb frgb;

  frgb.st.r= rgb.st.r;
  frgb.st.g= rgb.st.g;
  frgb.st.b= rgb.st.b;

  return frgb;
}

t_rgb Frgb2Rgb(t_frgb frgb)
{
  t_rgb rgb;

  rgb.st.r= cut(frgb.st.r);
  rgb.st.g= cut(frgb.st.g);
  rgb.st.b= cut(frgb.st.b);

  return rgb;
}

static int offY = 0;   /* no offset */
void SetYoff(int off) { offY = off;}

t_frgb  Yuv2Frgb(t_yuv p)
{
   t_frgb frgb;

   p.y     -= offY;
   frgb.st.r= 1.176   * p.y                 + 1.27635  * p.v - 0.0290891;
   frgb.st.g= 1.17589 * p.y - 0.3127  * p.u - 0.635371 * p.v - 0.105016 ;
   frgb.st.b= 1.17604 * p.y + 1.67181 * p.u                - 0.0511733;

   return frgb;
}

/*
t_frgb  Yuv2FrgbStd(t_yuv p)
{
   t_frgb frgb;

   frgb.st.r= p.y                + 1.140  * p.v ;
   frgb.st.g= p.y - 0.395  * p.u - 0.581 * p.v ;
   frgb.st.b= p.y + 2.032  * p.u                ;

   return frgb;
}
*/

t_rgb  Yuv2Rgb(t_yuv p) { return Frgb2Rgb(Yuv2Frgb(p)); }

t_rg Frgb2Rg(t_frgb frgb) 
{
  t_rg rg;
  float sum  = frgb.st.r+frgb.st.g+frgb.st.b;

  if(sum!=0.0)
  {
    rg.st.r = frgb.st.r/sum;
    rg.st.g = frgb.st.g/sum;
  }
  else
  {
    rg.st.r = 0.333;
    rg.st.g = 0.333;
  }

  return rg;
}


t_rgI Frgb2Rgi(t_frgb frgb) 
{
  t_rgI rgI;
  float sum  = frgb.st.r+frgb.st.g+frgb.st.b;

  if(sum!=0.0)
  {
    rgI.st.r = frgb.st.r/sum;
    rgI.st.g = frgb.st.g/sum;
  }
  else
  {
    rgI.st.r = 0.333;
    rgI.st.g = 0.333;
  }
  rgI.st.I = sum;

  return rgI;
}

t_rgI Rgb2Rgi(t_rgb rgb) 
{
  t_rgI rgI;
  float sum  = (int)rgb.st.r+(int)rgb.st.g+(int)rgb.st.b;

  if(sum!=0.0)
  {
    rgI.st.r = rgb.st.r/sum;
    rgI.st.g = rgb.st.g/sum;
  }
  else
  {
    rgI.st.r = 0.333;
    rgI.st.g = 0.333;
  }
  rgI.st.I = sum;

  return rgI;
}

t_rgI Yuv2Rgi(t_yuv yuv) { return Frgb2Rgi(Yuv2Frgb(yuv)); }

RGBARY*  YUVary2RGBary(YUVARY * yuv) 
{
  int i,j;
  RGBARY * rgb = (RGBARY*)makary(yuv->lb1,yuv->ub1,yuv->lb2,yuv->ub2,RGBMATRIX);

  for(i=yuv->lb1;i<=yuv->ub1;i++)
    for(j=yuv->lb2;j<=yuv->ub2;j++)
       rgb->el[i][j]=Yuv2Rgb(yuv->el[i][j]);

  return rgb;
}

FRGBARY* YUVary2FRGBary(YUVARY * yuv) 
{
  int i,j;

  FRGBARY *frgb=
    (FRGBARY*)makary(yuv->lb1,yuv->ub1,yuv->lb2,yuv->ub2,FRGBMATRIX);

  for(i=yuv->lb1;i<=yuv->ub1;i++)
    for(j=yuv->lb2;j<=yuv->ub2;j++)
     frgb->el[i][j] = Yuv2Frgb(yuv->el[i][j]);

  return frgb;
}


RGIARY*  YUVary2RGIary(YUVARY * yuv) 
{
  int i,j;

  RGIARY * rgI = 
    (RGIARY*)makary(yuv->lb1,yuv->ub1,yuv->lb2,yuv->ub2,RGIMATRIX);

  for(i=yuv->lb1;i<=yuv->ub1;i++)
    for(j=yuv->lb2;j<=yuv->ub2;j++)
      rgI->el[i][j]=Yuv2Rgi(yuv->el[i][j]);

  return rgI;
}
/*--------------- Functions Bellow by Rupert Young --------------*/
/*
 * RGBary2Fary3: Converts a colour array to three arrays,
 * each representing one of the planes of the primary 
 * colours.
 *
 * Input: A colour array (RGBARY).
 *
 * Output: An array (FARY) of three pointers to the plane
 * arrays.
 *
 */

void RGBary2Fary3 (RGBARY * col_image, FARY * out[3])
{
   int i,j,k;

   for(k = 0; k <= 3; k++)
   {
      out[k] = (FARY *)makary(col_image->lb1, col_image->ub1, col_image->lb2, col_image->ub2, FLOATMATRIX); 
   }

   for(i=col_image->lb1; i<=col_image->ub1; i++) 
      for(j=col_image->lb2; j<=col_image->ub2; j++)
	  for(k=0;k<3;k++)
	     out[k]->el[i][j] = col_image->el[i][j].arr[k];
}

/*
 * RGBary2FaryArr3 Converts a colour array to three arrays,
 * each representing one of the planes of the primary 
 * colours.
 *
 * Input: A colour array (RGBARY).
 *
 * Output: Three arrays (FARY) corresponding to the three colour planes.
 *
 */

void RGBary2FaryArr3(RGBARY * col_image, FARY * red, FARY * green, FARY * blue)
{
   FARY * outa[3];
	
   RGBary2Fary3(col_image, outa);

   *red = *outa[0];
   *green = *outa[1];
   *blue = *outa[2];
}	

/*
 * Fary3_2RGBary: Builds a colour array from three 
 * grey-level arrays representing one of the primary 
 * colour planes.
 *
 * Input: Array (FARY)of pointers to the three grey-level 
 * arrays.
 *
 * Output: A colour array (RGBARY).
 *
 */

RGBARY * Fary3_2RGBary(FARY * out[3])
{ 
   int i,j,k;
   BARY * outbary[3];
   RGBARY * col_image = (RGBARY *)makary(out[1]->lb1, out[1]->ub1, out[1]->lb2, out[1]->ub2, RGBMATRIX); 
   
   for(i = 0; i<= 2; i++)
   {
      outbary[i] = ScaleFary2Bary(out[i]);
   }


   for(i=col_image->lb1; i<=col_image->ub1; i++)    
      for(j=col_image->lb2; j<=col_image->ub2; j++)
	  for(k=0;k<3;k++)
	     col_image->el[i][j].arr[k] =outbary[k]->el[i][j];
	 
   return col_image;
}

/*
 * FaryArr3_2RGBary: Builds a colour array from three 
 * grey-level arrays representing one of the primary 
 * colour planes.
 *
 * Input: The three grey-level arrays (FARY).
 *
 * Output: A colour array (RGBARY).
 *
 */

RGBARY * FaryArr3_2RGBary(FARY * red, FARY * green, FARY * blue)
{ 
   RGBARY * col_image;
   FARY * out[3];

   out[0] = red;
   out[1] = green;
   out[2] = blue;

   col_image = Fary3_2RGBary(out);
   return col_image;
}


/*
 * RGBary2Y: Converts a colour (RGBARY) image to a 
 * grey-level(FARY) image. Takes the average intensity of
 * the three colours as the grey-level value.
 *
 * Input: A colour array.
 *
 * Output: A grey-level array. 
 *
 */

FARY * RGBary2Y(RGBARY * image)
{
   FARY * glimage = (FARY *)makary(image->lb1, image->ub1, image->lb2, image->ub2, FLOATMATRIX);
   int i,j;

   for(i=image->lb1; i<=image->ub1; i++) 
   {
      for(j=image->lb2; j<=image->ub2; j++)
      {
	  glimage->el[i][j] =
	  (image->el[i][j].arr[0] * 0.299 + 	
	  image->el[i][j].arr[1] * 0.587 + 
	  image->el[i][j].arr[2] * 0.114)/3; 	 
      }    
   }
  return glimage;
}


