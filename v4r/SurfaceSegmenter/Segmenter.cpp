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
 * @file Segmenter.cpp
 * @author Andreas Richtsfeld
 * @date July 2012
 * @version 0.1
 * @brief Segment images
 */


#include "Segmenter.h"

namespace segment
{

unsigned WhichGraphCutGroup(unsigned modelID, std::vector< std::vector<unsigned> > _graphCutGroups)
{
  for(unsigned i=0; i<_graphCutGroups.size(); i++)
    for(unsigned j=0; j<_graphCutGroups[i].size(); j++)
      if(_graphCutGroups[i][j] == modelID)
        return i;
  return 0;
}


/* --------------- Segmenter --------------- */

Segmenter::Segmenter(std::string _model)
{
  model_path = _model;
  useStructuralLevel = true;
  useAssemblyLevel = false;
}

Segmenter::~Segmenter()
{
}

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr
Segmenter::processPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
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
  
  // model abstraction
  pcl::on_nurbs::SequentialFitter::Parameter nurbsParams;
  nurbsParams.order = 3;
  nurbsParams.refinement = 0;
  nurbsParams.iterationsQuad = 0;
  nurbsParams.iterationsBoundary = 0;
  nurbsParams.iterationsAdjust = 0;
  nurbsParams.iterationsInterior = 3;
  nurbsParams.forceBoundary = 100.0;
  nurbsParams.forceBoundaryInside = 300.0;
  nurbsParams.forceInterior = 1.0;
  nurbsParams.stiffnessBoundary = 0.1;
  nurbsParams.stiffnessInterior = 0.1;
  nurbsParams.resolution = 16; 
  surface::SurfaceModeling::Parameter sfmParams;
  sfmParams.nurbsParams = nurbsParams;
  sfmParams.sigmaError = 0.003;
  sfmParams.kappa1 = 0.008;
  sfmParams.kappa2 = 1.0;
  sfmParams.planePointsFixation = 8000;
  sfmParams.z_max = 0.01;
  surface::SurfaceModeling surfModeling(sfmParams);
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  surfModeling.setIntrinsic(525., 525., 320., 240.);
  surfModeling.setExtrinsic(pose);
  surfModeling.setInputCloud(pcl_cloud);
  surfModeling.setInputPatches(surfaces);
  surfModeling.compute();
  surfModeling.getSurfaceModels(surfaces);
  
  // svm-predictor
  std::string svmStructuralModel = model_path + "PP-Trainingsset.txt.scaled.model";
  std::string svmStructuralScaling = model_path + "param.txt";
  std::string svmAssemblyModel = model_path + "PP2-Trainingsset.txt.scaled.model";
  std::string svmAssemblyScaling = model_path + "param2.txt";
  svm::SVMPredictorSingle svm1st(svmStructuralModel.c_str());
  svm::SVMPredictorSingle svm2nd(svmAssemblyModel.c_str());
  svm1st.setScaling(true, svmStructuralScaling.c_str());
  svm2nd.setScaling(true, svmAssemblyScaling.c_str());

  // patch relations
  std::vector<surface::Relation> relation_vector;
  surface::PatchRelations patchRelations;
  patchRelations.setStructuralLevel(useStructuralLevel);
  patchRelations.setAssemblyLevel(useAssemblyLevel);
  patchRelations.setInputCloud(pcl_cloud);
  patchRelations.setSurfaceModels(surfaces);
  patchRelations.setOptimalPatchModels(true);
  patchRelations.computeSegmentRelations();
  patchRelations.getRelations(relation_vector);
  
  for(unsigned i=0; i<relation_vector.size(); i++) {
    if(relation_vector[i].type == 1)
      relation_vector[i].prediction = svm1st.getResult(relation_vector[i].type, 
                                                       relation_vector[i].rel_value, 
                                                       relation_vector[i].rel_probability);
    if(relation_vector[i].type == 2)
      relation_vector[i].prediction = svm2nd.getResult(relation_vector[i].type, 
                                                       relation_vector[i].rel_value, 
                                                       relation_vector[i].rel_probability);
  }
  
  // graph cutter
  std::vector< std::vector<unsigned> > graphCutGroups;
  gc::GraphCut graphCut;
  graphCut.init(surfaces.size(), relation_vector);
  graphCut.process();
  graphCut.getResults(surfaces.size(), graphCutGroups);
  graphCut.printResults();
  
  for(unsigned i=0; i<graphCutGroups.size(); i++)
    for(unsigned j=0; j<graphCutGroups[i].size(); j++)
  
  for(unsigned i=0; i<surfaces.size(); i++) {
    for(unsigned j=0; j<surfaces[i]->indices.size(); j++) {
      result->points[surfaces[i]->indices[j]].label = surfaces[i]->label;
    }
  }
  return result;
}

std::vector<pcl::PointIndices> 
Segmenter::processPointCloudV(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
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
  
  // model abstraction
  pcl::on_nurbs::SequentialFitter::Parameter nurbsParams;
  nurbsParams.order = 3;
  nurbsParams.refinement = 0;
  nurbsParams.iterationsQuad = 0;
  nurbsParams.iterationsBoundary = 0;
  nurbsParams.iterationsAdjust = 0;
  nurbsParams.iterationsInterior = 3;
  nurbsParams.forceBoundary = 100.0;
  nurbsParams.forceBoundaryInside = 300.0;
  nurbsParams.forceInterior = 1.0;
  nurbsParams.stiffnessBoundary = 0.1;
  nurbsParams.stiffnessInterior = 0.1;
  nurbsParams.resolution = 16; 
  surface::SurfaceModeling::Parameter sfmParams;
  sfmParams.nurbsParams = nurbsParams;
  sfmParams.sigmaError = 0.003;
  sfmParams.kappa1 = 0.008;
  sfmParams.kappa2 = 1.0;
  sfmParams.planePointsFixation = 8000;
  sfmParams.z_max = 0.01;
  surface::SurfaceModeling surfModeling(sfmParams);
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  surfModeling.setIntrinsic(525., 525., 320., 240.);
  surfModeling.setExtrinsic(pose);
  surfModeling.setInputCloud(pcl_cloud);
  surfModeling.setInputPatches(surfaces);
  surfModeling.compute();
  surfModeling.getSurfaceModels(surfaces);
  
  // svm-predictor
  std::string svmStructuralModel = model_path + "PP-Trainingsset.txt.scaled.model";
  std::string svmStructuralScaling = model_path + "param.txt";
  std::string svmAssemblyModel = model_path + "PP2-Trainingsset.txt.scaled.model";
  std::string svmAssemblyScaling = model_path + "param2.txt";
  svm::SVMPredictorSingle svm1st(svmStructuralModel.c_str());
  svm::SVMPredictorSingle svm2nd(svmAssemblyModel.c_str());
  svm1st.setScaling(true, svmStructuralScaling.c_str());
  svm2nd.setScaling(true, svmAssemblyScaling.c_str());

  // patch relations
  std::vector<surface::Relation> relation_vector;
  surface::PatchRelations patchRelations;
  patchRelations.setStructuralLevel(useStructuralLevel);
  patchRelations.setAssemblyLevel(useAssemblyLevel);
  patchRelations.setInputCloud(pcl_cloud);
  patchRelations.setSurfaceModels(surfaces);
  patchRelations.setOptimalPatchModels(true);
  patchRelations.computeSegmentRelations();
  patchRelations.getRelations(relation_vector);
  
  for(unsigned i=0; i<relation_vector.size(); i++) {
    if(relation_vector[i].type == 1)
      relation_vector[i].prediction = svm1st.getResult(relation_vector[i].type, 
                                                       relation_vector[i].rel_value, 
                                                       relation_vector[i].rel_probability);
    if(relation_vector[i].type == 2)
      relation_vector[i].prediction = svm2nd.getResult(relation_vector[i].type, 
                                                       relation_vector[i].rel_value, 
                                                       relation_vector[i].rel_probability);
  }
  
  // graph cutter
  std::vector< std::vector<unsigned> > graphCutGroups;
  gc::GraphCut graphCut;
  graphCut.init(surfaces.size(), relation_vector);
  graphCut.process();
  graphCut.getResults(surfaces.size(), graphCutGroups);
  graphCut.printResults();
  
  std::vector<pcl::PointIndices> results;
  results.resize(graphCutGroups.size());
  for(unsigned i=0; i<graphCutGroups.size(); i++)
    for(unsigned j=0; j<graphCutGroups[i].size(); j++)
      for(unsigned k=0; k<surfaces[graphCutGroups[i][j]]->indices.size(); k++)
        results[i].indices.push_back(surfaces[graphCutGroups[i][j]]->indices[k]);
  return results;
}


} // end segment
