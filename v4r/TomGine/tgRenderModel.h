/**
 * @file tgRenderModel.h
 * @author Thomas Mörwald
 * @date January 2010
 * @version 0.1
 * @brief Advanced model for rendering including pose, lighting, material, speed up techniques and bounding primitive (sphere).
 */

#ifndef TG_RENDER_MODEL
#define TG_RENDER_MODEL

#include <stdio.h>
#include <vector>

#include "tgMathlib.h"
#include "tgPose.h"
#include "tgModel.h"
#include "tgMaterial.h"

#include <GL/glu.h>

namespace TomGine {

/** @brief Advanced model for rendering including pose, lighting, material, speed up techniques and bounding primitive (sphere).  */
class tgRenderModel: public tgModel
{
protected:
  GLuint m_displaylist;
  bool m_displaylist_initialized;

  GLuint m_vertexVBO;
  GLuint m_triangleIBO;
  GLuint m_quadIBO;
  std::vector<unsigned> m_triangleIDX;
  std::vector<unsigned> m_quadIDX;
  bool m_bufferobject_initialized;

public:
  /** @brief Defines how the object is rendered. */
  enum RenderMode
  {
    RENDERNORMAL, ///< render object normal (glBegin(), glVertex()).
    DISPLAYLIST, ///< render object using display lists. glCalllist().
    BUFFEROBJECT
  ///< render object using buffer objects.
  };
  tgPose m_pose; ///< The pose of the object (see tgPose).
  tgMaterial m_material; ///< Material of the object (see tgMaterial)
  tgModel* m_bsmodel; ///< bounding sphere of the geometry.

  /** @brief Creates empty model with random material */
  tgRenderModel();
  /** @brief Creates model from tgModel with random material. */
  tgRenderModel(const tgModel& model);
  /** @brief Destroys bounding sphere (m_bsmodel), display lists and buffer objects. */
  ~tgRenderModel();

  /** @brief Applies material of this render model to OpenGL and enables lighting. */
  void ApplyMaterial();
  /** @brief Applies color of this render model to OpenGL. */
  void ApplyColor();

  /** @brief Set color of this render model */
  void SetColor(float r, float g, float b, float a=1.0f)
  {
    m_material.Color(r, g, b, a);
  }

  /** @brief Set color of this render model */
  void SetColor(int r, int g, int b, int a=255)
  {
    m_material.Color(float(r) / 255.0f, float(g) / 255.0f, float(b) / 255.0f, float(a) / 255.0f);
  }

  /** @brief Generates a display list from the faces of the model. */
  void GenDisplayList();
  /** @brief Generates a buffer object from the faces of the model. */
  void GenBufferObject();
  /** @brief Draws the model as buffer object. */
  void DrawBufferObject();

  /** @brief Draw all data in model. */
  virtual void Draw();

  /** @brief Draws the faces of the render model. Applies pose, material and lighting.
   *  @param lighting		Enable/Disable OpenGL lighting calculations.
   *  @param rmode		Rendering mode (NORMAL, DISPLAYLIST, BUFFEROBJECT; see RenderMode) */
  void DrawFaces(bool lighting = true, RenderMode rmode = RENDERNORMAL);

  /** @brief Draws vertex normals of the model.
   *  @param length The length of the normal vector. */
  virtual void DrawNormals(float length);

  /** @brief Draws bounding sphere of model. At first call it calculates the bounding sphere and stores it (m_bsmodel). */
  virtual void DrawBoundingSphere();
};

} // namespace TomGine

#endif
