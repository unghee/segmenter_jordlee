/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Thomas Mörwald nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * @author thomas.moerwald
 *
 */

#include "Texturing.h"
#include "v4r/PCLAddOns/PCLUtils.h"

using namespace objectmodeling;

std::vector<int> getLabels(const surface::View &view)
{
  std::vector<int> labels;

  if (view.surfaces.empty())
    return labels;

  labels.push_back(view.surfaces[0]->label);

  for (unsigned i = 1; i < view.surfaces.size(); i++) {
    if (std::find(labels.begin(), labels.end(), view.surfaces[i]->label) == labels.end())
      labels.push_back(view.surfaces[i]->label);
  }

  return labels;
}

unsigned getIndex(const std::vector<int> &labels, const int &label)
{
  unsigned idx;
  for (idx = 0; idx < labels.size(); idx++) {
    if (labels[idx] == label)
      break;
  }
  return idx;
}

void Texturing::CreateColorFromLabels(surface::View &view, float brightness)
{
  // Get number of labels (= number of models)
  std::vector<int> labels = getLabels(view);

  // create materials
  std::vector<TomGine::tgMaterial> material;
  for (unsigned i = 0; i < labels.size(); i++) {
    TomGine::tgMaterial mat;
    mat.Random(brightness);
    material.push_back(mat);
  }

  // Assign colors
  for (unsigned i = 0; i < view.surfaces.size(); i++) {
    int idx = getIndex(labels, view.surfaces[i]->label);
    view.surfaces[i]->mesh.m_material = material[idx];
  }
}

void Texturing::CreateTexture(TomGine::tgModel &mesh, TomGine::tgTextureModel &texmesh,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                              Eigen::Matrix3d intrinsic)
{
  if(mesh.m_vertices.empty()){
    printf("[Texturing::CreateTexture] Warning, no vertices in mesh model.\n");
    return;
  }

  // copy texture from point-cloud to cv::Mat
  cv::Mat3b texture;
  pclA::ConvertPCLCloud2Image(cloud, texture);

  float dw = 1.0f / cloud->width;
  float dh = 1.0f / cloud->height;

  texmesh.Merge(mesh);

  // Project vertices into image plane to get texture coordinates
  for (unsigned j = 0; j < texmesh.m_vertices.size(); j++) {
    TomGine::tgVertex &p = texmesh.m_vertices[j];
    Eigen::Vector3d v = intrinsic * Eigen::Vector3d(p.pos.x, p.pos.y, p.pos.z);
    if (v(2) != 0.0) {
      p.texCoord.x = dw * v(0) / v(2);
      p.texCoord.y = dh * v(1) / v(2);
    }
  }

  texmesh.m_tex_cv.assign(1, cv::Mat3b());
  TomGine::tgTextureModel::OptimizeTexture(texmesh.m_vertices, texture, texmesh.m_tex_cv[0]);

  while (texmesh.m_face_tex_id.size() < texmesh.m_faces.size())
    texmesh.m_face_tex_id.push_back(0);

  texmesh.m_material.Color(0.2, 0.2, 0.2, 1.0,
                           0.8, 0.8, 0.8, 1.0,
                           0.4, 0.4, 0.4, 1.0,
                           30);
}

void Texturing::CreateTextureModels(surface::View &view, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    std::vector<TomGine::tgTextureModel> &models)
{
  // copy texture from point-cloud to cv::Mat
  cv::Mat3b texture;
  pclA::ConvertPCLCloud2Image(cloud, texture);

  // Get number of labels (= number of models)
  std::vector<int> labels = getLabels(view);

  // create models and materials
  models.clear();
  std::vector<TomGine::tgMaterial> material;
  for (unsigned i = 0; i < labels.size(); i++) {
    TomGine::tgMaterial mat;
    //      mat.Random(1.0);
    mat.Color(1.0, 1.0, 1.0);
    material.push_back(mat);
    models.push_back(TomGine::tgTextureModel());
  }

  float dw = 1.0f / cloud->width;
  float dh = 1.0f / cloud->height;

  // Project vertices into image plane to get texture coordinates
  for (unsigned i = 0; i < view.surfaces.size(); i++) {
    int idx = getIndex(labels, view.surfaces[i]->label);
    if (view.surfaces[i]->mesh.m_vertices.empty())
      printf("[Texturing::CreateTextureModels] Warning, mesh[%d] is empty (1).\n", idx);
    else
      models[idx].Merge(view.surfaces[i]->mesh);
  }

  for (unsigned i = 0; i < models.size(); i++) {
    TomGine::tgTextureModel &model = models[i];

    if (model.m_vertices.empty()) {
      printf("[Texturing::CreateTextureModels] Warning, mesh[%d] is empty (2).\n", i);
      continue;
    }

    // Project vertices into image plane to get texture coordinates
    for (unsigned j = 0; j < model.m_vertices.size(); j++) {
      TomGine::tgVertex &p = model.m_vertices[j];
      Eigen::Vector3d v = view.intrinsic * Eigen::Vector3d(p.pos.x, p.pos.y, p.pos.z);
      if (v(2) != 0.0) {
        p.texCoord.x = dw * v(0) / v(2);
        p.texCoord.y = dh * v(1) / v(2);
      }
    }

    // copy texture
    model.m_tex_cv.push_back(cv::Mat3b());
    unsigned tex_id = model.m_tex_cv.size() - 1;
    TomGine::tgTextureModel::OptimizeTexture(model.m_vertices, texture, model.m_tex_cv[tex_id]);

    // add new texture id to all faces
    while (model.m_face_tex_id.size() < model.m_faces.size()) {
      model.m_face_tex_id.push_back(tex_id);
    }
    model.m_material = material[i];
    model.ComputeBoundingSphere();
  }
}

