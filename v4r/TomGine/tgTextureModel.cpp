/**
 * @file tgTextureModel.h
 * @author Thomas Mörwald
 * @date February 2012
 * @version 0.1
 * @brief Defining a textured model for rendering.
 */

#include "tgTextureModel.h"
#include "tgError.h"

using namespace TomGine;

void tgTextureModel::clearGLTextures()
{
  for (unsigned i = 0; i < m_tex_gl.size(); i++)
    delete (m_tex_gl[i]);
  m_tex_gl.clear();
}

void tgTextureModel::syncTextures()
{
  clearGLTextures();

  for (unsigned i = 0; i < m_tex_cv.size(); i++) {
    if (!m_tex_cv[i].isContinuous())
      printf("[tgTextureModel::syncTextures] Warning, data of texture not memory aligned\n");

    tgTexture2D *tex = new tgTexture2D();
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    tex->Load(m_tex_cv[i].data, m_tex_cv[i].cols, m_tex_cv[i].rows, GL_RGB, GL_BGR, GL_UNSIGNED_BYTE);
    m_tex_gl.push_back(tex);
  }
  sync = true;
}

tgTextureModel::tgTextureModel() :
  tgRenderModel::tgRenderModel(), sync(false)
{
}

tgTextureModel::tgTextureModel(const tgTextureModel &m) :
  tgRenderModel::tgRenderModel(m), sync(false)
{
  if (!m.m_tex_gl.empty())
    printf(
        "[tgTextureModel::tgTextureModel(const tgTextureModel &)] Warning, cannot copy tgTexture2D in tgTextureModel::m_tex_gl\n");

  this->m_tex_cv.assign(m.m_tex_cv.size(), cv::Mat3b());
  for (unsigned i = 0; i < m.m_tex_cv.size(); i++)
    m.m_tex_cv[i].copyTo(this->m_tex_cv[i]);

  this->m_face_tex_id = m.m_face_tex_id;
}

tgTextureModel::tgTextureModel(const tgRenderModel &m) :
  tgRenderModel::tgRenderModel(m), sync(false)
{
}

tgTextureModel::tgTextureModel(const tgModel &m) :
  tgRenderModel::tgRenderModel(m), sync(false)
{
}

tgTextureModel::~tgTextureModel()
{
  clearGLTextures();
}

tgTextureModel& tgTextureModel::operator=(const tgTextureModel &m)
{
  tgRenderModel::operator=(m);
  this->sync = false;

  if (!m.m_tex_gl.empty())
    printf(
        "[tgTextureModel::tgTextureModel(const tgTextureModel &)] Warning, cannot copy tgTexture2D in tgTextureModel::m_tex_gl\n");

  this->m_tex_cv.assign(m.m_tex_cv.size(), cv::Mat3b());
  for (unsigned i = 0; i < m.m_tex_cv.size(); i++)
    m.m_tex_cv[i].copyTo(this->m_tex_cv[i]);

  this->m_face_tex_id = m.m_face_tex_id;

  return *this;
}

void tgTextureModel::Draw()
{
  if ((!sync) || (m_tex_cv.size() != m_tex_gl.size()))
    syncTextures();

  if (sync && m_face_tex_id.size() == m_faces.size()) {
    this->DrawFaces();
  } else {
    tgRenderModel::DrawFaces();
  }

  DrawLines();
  DrawPoints();
  DrawColorPoints();
}

void tgTextureModel::DrawFaces() const
{
  unsigned v;
  m_material.Activate();
  m_pose.Activate();
  unsigned nfaces = m_faces.size();

  for (unsigned i = 0; i < nfaces; i++) {

    m_tex_gl[m_face_tex_id[i]]->Bind();

    if (m_faces[i].v.size() == 3)
      glBegin(GL_TRIANGLES);
    else if (m_faces[i].v.size() == 4)
      glBegin(GL_QUADS);
    else {
      printf("[tgTextureModel::DrawTexFaces()] Warning, no suitable face format\n");
      printf("[tgTextureModel::DrawTexFaces()] Face has %d vertices (supported: 3 or 4)\n", (int) m_faces[i].v.size());
      return;
    }
    for (unsigned j = 0; j < m_faces[i].v.size(); j++) {
      v = m_faces[i].v[j];
      glTexCoord2f(m_vertices[v].texCoord.x, m_vertices[v].texCoord.y);
      glNormal3f(m_vertices[v].normal.x, m_vertices[v].normal.y, m_vertices[v].normal.z);
      glVertex3f(m_vertices[v].pos.x, m_vertices[v].pos.y, m_vertices[v].pos.z);
    }
    glEnd();
  }

  glDisable(GL_TEXTURE_2D);

  m_pose.Deactivate();
}

void tgTextureModel::OptimizeTexture(std::vector<tgVertex> &vertices, const cv::Mat3b &tex, cv::Mat3b &tex_opt)
{
  if (vertices.size() <= 3) {
    printf("[tgTextureModel::OptimizeTexture] Warning too few vertices: %d\n", (unsigned) vertices.size());
    return;
  }
  vec2 tc_min(FLT_MAX, FLT_MAX);
  vec2 tc_max(0.0f, 0.0f);
  int width = tex.cols;
  int height = tex.rows;
  float dwidth = 1.0f / width;
  float dheight = 1.0f / height;

  // get bounding box of texture coordinates
  for (unsigned i = 0; i < vertices.size(); i++) {
    vec2 &tc = vertices[i].texCoord;

    if (tc.x < tc_min.x)
      tc_min.x = tc.x;
    if (tc.y < tc_min.y)
      tc_min.y = tc.y;
    if (tc.x > tc_max.x)
      tc_max.x = tc.x;
    if (tc.y > tc_max.y)
      tc_max.y = tc.y;

  }

  // if tex is already optimal, just copy it
  if (tc_min.x < dwidth && tc_min.y < dheight && tc_max.x > (1.0f - dwidth) && tc_max.y > (1.0f - dheight)) {
    tex.copyTo(tex_opt);
    return;
  }

  // bounding box in pixel coordinates
  int i_min_x = int(floor(tc_min.x * width));
  int i_min_y = int(floor(tc_min.y * height));
  int i_max_x = int(ceil(tc_max.x * width));
  int i_max_y = int(ceil(tc_max.y * height));
  int i_width = i_max_x - i_min_x;
  int i_height = i_max_y - i_min_y;

  // adjust texture width and height to be divisible by 8 (otherwise OpenGL Textures cause problems)
  if (i_width % 8)
    i_width = (1 + i_width / 8) * 8;
  if (i_min_x + i_width >= tex.cols)
    i_min_x -= (i_min_x + i_width - tex.cols);
  if (i_height % 8)
    i_height = (1 + i_height / 8) * 8;
  if (i_min_y + i_height >= tex.rows)
    i_min_y -= (i_min_y + i_height - tex.rows);

  if (i_min_x < 0)
    i_min_x = 0;
  if (i_min_y < 0)
    i_min_y = 0;
  if (i_width > tex.cols)
    i_width = tex.cols;
  if (i_height > tex.rows)
    i_height = tex.rows;

  // copy sub texture
  cv::Rect rect(i_min_x, i_min_y, i_width, i_height);

  float r_width = float(width) / i_width;
  float r_height = float(height) / i_height;

  cv::Mat3b tex_tmp(tex, rect);
  tex_tmp.copyTo(tex_opt);

  // adjust texture coordinats
  float d_min_x = r_width * dwidth * i_min_x;
  float d_min_y = r_height * dheight * i_min_y;

  for (unsigned i = 0; i < vertices.size(); i++) {
    vec2 &tc = vertices[i].texCoord;
    tc.x = tc.x * r_width - d_min_x;
    tc.y = tc.y * r_height - d_min_y;
  }
}
