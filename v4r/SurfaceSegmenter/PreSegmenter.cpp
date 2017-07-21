/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienn, Austria
 *    ari(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @file PreSegmenter.cpp
 * @author Andreas Richtsfeld
 * @date September 2012
 * @version 0.1
 * @brief Pre-segment images to planar patches.
 */


#include "PreSegmenter.h"

namespace segment
{
  
/* --------------- PreSegmenter --------------- */

PreSegmenter::PreSegmenter()
{}

PreSegmenter::~PreSegmenter()
{}

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr
PreSegmenter::processPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals;
  std::vector<surface::SurfaceModel::Ptr > surfaces;
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::copyPointCloud(*pcl_cloud, *result);

  // calcuate normals
  normals.reset(new pcl::PointCloud<pcl::Normal>);
  pclA::ZAdaptiveNormals::Parameter za_param;
  za_param.adaptive = true;
  pclA::ZAdaptiveNormals nor(za_param);
  nor.setInputCloud(pcl_cloud);
  nor.compute();
  nor.getNormals(normals);
  
  // adaptive clustering
  surface::ClusterNormalsToPlanes::Parameter param;
  param.adaptive = true;
  surface::ClusterNormalsToPlanes clusterNormals(param);
  clusterNormals.setInputCloud(pcl_cloud);
  clusterNormals.setInputNormals(normals);
  clusterNormals.setPixelCheck(true, 5);
  clusterNormals.compute();
  clusterNormals.getSurfaceModels(surfaces);
  
  for(unsigned i=0; i<surfaces.size(); i++) {
    for(unsigned j=0; j<surfaces[i]->indices.size(); j++) {
      result->points[surfaces[i]->indices[j]].label = surfaces[i]->label;
    }
  }
  return result;
}

} // end segment



