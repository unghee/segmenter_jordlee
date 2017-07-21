/**
 * @file tgTextureModel.h
 * @author Thomas Mörwald
 * @date February 2012
 * @version 0.1
 * @brief Defining a textured model for rendering.
 */

#ifndef TG_TEXTURE_MODEL
#define TG_TEXTURE_MODEL

#include "tgRenderModel.h"
#include "tgTexture.h"
#include <opencv2/core/core.hpp>

namespace TomGine {


class tgTextureModel: public tgRenderModel
{
private:
  bool sync;
  std::vector<tgTexture2D*> m_tex_gl;
  void syncTextures();
  void clearGLTextures();

public:
  std::vector<cv::Mat3b> m_tex_cv;          ///< Textures of the model as cv::Mat3b
  std::vector<unsigned> m_face_tex_id;      ///< index of tgTexture2D in m_textures for each face in tgModel::m_face

  /** @brief constructors and copy-constructors */
  tgTextureModel();
  tgTextureModel(const tgTextureModel &m);
  tgTextureModel(const tgRenderModel &m);
  tgTextureModel(const tgModel &m);
  ~tgTextureModel();

  tgTextureModel& operator=(const tgTextureModel &m);

  /** @brief Calls DrawTexFaces if a texture is available, otherwise use tgRenderModel::DrawFaces */
  virtual void Draw();

  /** @brief Synchronizes textures with OpenGL if necessary and draws textured faces. */
  virtual void DrawFaces() const;

  /** @brief Allows user to force synchronization of textures with OpenGL */
  inline void Sync(){ sync = false; }

  /** @brief Crop textures to minimum area necessary
   *  @param vertices Vertices used for this texture. texCoords are used to calculate bounding box
   *  @param tex  Original texture (usually bigger than necessary)
   *  @param tex_opt Optimized (cropped down) texture */
  static void OptimizeTexture(std::vector<tgVertex> &vertices, const cv::Mat3b &tex, cv::Mat3b &tex_opt);

};

}

#endif
