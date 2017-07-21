#ifndef _TG_IMAGE_PROCESSOR_H_
#define _TG_IMAGE_PROCESSOR_H_

#include "headers.h"

#include "tgShader.h"
#include "tgTexture.h"
#include "tgCamera.h"

namespace TomGine {

/** @brief GPU accelerated image processing using OpenGL. */
class tgImageProcessor
{
private:
  unsigned m_width; ///< Image width in pixels
  unsigned m_height; ///< Image height in pixels

  tgShader* m_shadeGauss; ///< Fragment shader for blurring image using gaussian filter
  tgShader* m_shadeSobel; ///< Fragment shader for edge detection with sobel algorithm
  tgShader* m_shadeSobelNormal; ///< Fragment shader for edge detection with sobel algorithm
  tgShader* m_shadeThinning; ///< Fragment shader for thinning edges
  tgShader* m_shadeSpreading; ///< Fragment shader for spreading edges
  tgShader* m_shadeParam2Polar; ///< Fragment shader for edge conversion
  tgShader* m_shadeRGB2HSV; ///< Fragment shader for color conversion
  tgShader* m_shadeAdd; ///< Fragment shader to add two textures
  tgShader* m_shadeInvert; ///< Fragment shader to invert a texture;
  tgShader* m_shadeMultiply; ///< Fragment shader to multiply two textures
  tgShader* m_shadeThreshold; ///< Fragment shader to threshold a texture
  tgShader* m_shadeDilate;    ///< Fragment shader to delate a gradient texture

  TomGine::tgCamera m_cam_ortho, m_cam_ortho_fbo; ///< orthographic cameras

  bool m_use_fbo;
  GLuint m_fbo_id;
  //	GLuint fbo_tex;
  //	GLuint fbo_tex_depth;
  //	int fbo_res;
  //	int fbo_stage;
  //	bool m_avg_init;

  void drawQuadUpsideDown(float w, float h);
  void drawQuad(float w, float h);
  void drawQuad(float x, float y, float w, float h);
  bool transform(float i, float j, float *ix, float *iy);

  void init(unsigned width, unsigned height);
  void initShader();

  void applyOperation(tgShader* shader, const tgTexture2D& src, tgTexture2D& result);

public:
  /** @brief Create the image processors with the shaders.
   * 	@param widht,height		Size of the frame buffer object. */

  /**	@brief Initialize orthographic camera and shaders.
   *  @param w,h	Size of the orthographic camera of the image processor in pixel.	 */
  tgImageProcessor(unsigned w, unsigned h);

  /** @brief Destroy orthographic camera and shaders. */
  ~tgImageProcessor();

  /** @brief enables/disables usage of frame buffer object
   *  @param val true = render to texture, false = render to screen	 */
  void setFBO(bool val)
  {
    m_use_fbo = val;
  }
  /** @brief returns flag for usage of frame buffer object. */
  bool getFBO()
  {
    return m_use_fbo;
  }
  /** @brief Apply camera for orthographic projection. */
  void setCamOrtho();
  /** @brief Get the width of the orthographic camera. */
  int getWidth()
  {
    return m_width;
  }
  /** @brief Get the height of the orthographic camera. */
  int getHeight()
  {
    return m_height;
  }

  /** @brief Flip an image upside down.
   *  @param source	Input image.
   *  @param result	Output image. */
  void flipUpsideDown(const tgTexture2D& source, tgTexture2D& result);

  /** @brief Copy the content of an image (on GPU).
   * 	@param source	Source image.
   * 	@param result	Destination image.	 */
  void copy(const tgTexture2D& source, tgTexture2D& result);

  /** @brief Copy the content of an image within an rectangle (on GPU).
   * 	@param source	Source image.
   * 	@param result	Destination image.
   * 	@param x,y		Lower left corner of the rectangle.
   * 	@param w,h		Width height of the rectangle. */
  void copy(const tgTexture2D& source, tgTexture2D& result, int x, int y, unsigned w, unsigned h);

  /** @brief thresholds a texture (on GPU)
   *  @param source	input texture
   *  @param result	output texture	 */
  void threshold(const tgTexture2D& source, tgTexture2D& result, float th);

  /** @brief Add two textures (on GPU).
   * 	@param src1,src2	Source image 1.
   * 	@param result		Destination image. */
  void add(const tgTexture2D& src1, const tgTexture2D& src2, tgTexture2D& result, float k1 = 1.0f, float k2 = 1.0f);

  /** @brief Multiply two textures (on GPU).
   * 	@param src1,src2	Source image 1.
   * 	@param result		Destination image.
   * 	@param src1grad,src2grad true if src1 or src2 is a gradient image respectively */
  void multiply(const tgTexture2D& src1, const tgTexture2D& src2, tgTexture2D& result, bool src1grad=false, bool src2grad=false);

  void multiply(const tgTexture2D& src, float k1, tgTexture2D &result);

  /** @brief Invert an image per color channel.
   *  @param source Source image
   *  @param result Result image	 */
  void invert(const tgTexture2D& source, tgTexture2D& result);

  /** @brief Calculating average on GPU using mip-mapping */
  float average(std::vector<float> &data);

  /** @brief Gaussian smoothing of an image.
   *  @param source	Input color image.
   *  @param result	Output color image.	 */
  void gauss(const tgTexture2D& source, tgTexture2D& result);

  /** @brief Sobel edge detection calculating the gradients of an color image.
   * 	@brief The gradient is scaled to the range [0..1] and can be transformed back with:
   * 			grad = (vec2(R,G) - vec2(0.5,0.5)) * 2
   *  		where R and G are the red and green component of a gradient image pixel.
   * 	@param source	Input color image.
   * 	@param result	Output gradient image.
   * 	@param threshold	Defines the minimum magnitude a gradient must have to be treated as edge (otherwise it is zero).
   * 	@param normalise	Normalize the gradients (RG values).
   * 	@param binary	Does not return gradients but 1 (white) if the magnitude of a gradient exceeds the threshold, 0 (black) otherwise. */
  void sobel(const tgTexture2D& source, tgTexture2D& result, float threshold = 0.01, bool normalise = false,
      bool binary = false);

  /** @brief Masked sobel edge detection.
   * 	@param mask		Mask defining pixels that are not treated.
   * 			i.e. if pixel (u,v) of mask is black the pixel (u,v) of the source image will be included for edge detection, otherwise the output gradient at this location will be set to 0.  */
  void sobel(const tgTexture2D& source, tgTexture2D& result, tgTexture2D& mask, float threshold = 0.01,
      bool normalise = false, bool binary = false);

  void sobelNormal(const tgTexture2D &source, tgTexture2D& result, float threshold = 0.01);

  /** @brief Computes edges using depth and curvature information.
   *  @param color		RGB image
   *  @param depth		depth values
   *  @param curvature	curvature of a dense point-cloud
   *  @param result		output
   *  @param k1,k2,k3		weighting of edges	 */
  void rgbdEdges(const tgTexture2D& color, const tgTexture2D& depth, const tgTexture2D& curvature, tgTexture2D& result,
      float k1, float k2, float k3);

  /** @brief Thinning edges in gradient images (RG = xy-gradient direction, B = magnitude of gradient).
   * 	@param source	Input gradient image.
   * 	@param result	Output gradient image.	 */
  void thinning(const tgTexture2D& source, tgTexture2D& result, bool normalise = false, bool binary = false);
  //	void thinning(const tgTexture2D& source, tgTexture2D& result, tgTexture2D& mask);

  /** @brief Convert gradients into polar coordinates
   * 	@param source	Input gradient image.
   * 	@param result	Output polar image.	 */
  void param2polar(const tgTexture2D& source, tgTexture2D& result);

  /** @brief Converts a rgb to hsv.
   * 	@param source	Input rgb image.
   * 	@param result	Output hsv image.	 */
  void rgb2hsv(const tgTexture2D& source, tgTexture2D& result);

  /** @brief Spreading (widen) edges in an gradient image  (RG = xy-gradient direction [scaled to 0-1], B = magnitude of gradient).
   * 	@param source	Input gradient image.
   * 	@param result	Output gradient image.	 */
  void spreading(const tgTexture2D& source, tgTexture2D& result, bool binary = false, float fDistScale = 0.5f);

  /** @brief Dilate (widen) an image an gradient image (RG = xy-gradient direction [scaled to 0-1], B = magnitude of gradient).
   *  @brief Overwrites pixel with pixel of highest magnitude in 4-neighbourhood.
   *  @param source Input gradient image.
   *  @param result Output gradient image.
   *  @param scale  Distance fall of for pixel*/
  void dilate(const tgTexture2D& source, tgTexture2D& result, float scale=1.0f);

  /** @brief Render an image.
   *  @param	tex		Image to render.	 */
  void render(const tgTexture2D& tex);

  //	void render(const tgTexture2D& tex, int x, int y, unsigned w, unsigned h);

  //	// average computations
  //	void avgActivate();
  //	void avgGet(float *avg, int lvl=0);
  //	void avgDeactivate();
  //	inline int avgGetResolution(){ return fbo_res; }
};

} // namespace TomGine

#endif
