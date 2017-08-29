/**
 * This software provides a ROS & libsvm wrapper for the segmentation released by Richtsfeld et al.
 * The software is under Copyrights described below.
*/
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
 * Chih-Chung Chang and Chih-Jen Lin, LIBSVM : a library for support vector machines.
 * ACM Transactions on Intelligent Systems and Technology, 2:27:1--27:27, 2011. Software available at http://www.csie.ntu.edu.tw/~cjlin/libsvm
*/
/**
 * @file Segmenter.h
 * @author Jordan Lee
 * @date July 2017
 * @version 0.1
 */


#ifndef SEGMENT_SEGMENTER_H
#define SEGMENT_SEGMENTER_H

#include <stdio.h>

#include "v4r/KinectInterface/KinectData.h"
#include "v4r/PCLAddOns/PCLUtils.h"
#include "v4r/SurfaceUtils/SurfaceModel.hpp"
#include "v4r/SurfaceClustering/ClusterNormalsToPlanes.hh"
#include "v4r/SurfaceClustering/ZAdaptiveNormals.hh"
#include "v4r/SurfaceModeling/SurfaceModeling.hh"
#include "v4r/SurfaceRelations/PatchRelations.h"
#include "v4r/svm/SVMPredictorSingle.h"
#include "v4r/svm/SVMFileCreator.h"
#include "v4r/GraphCut/GraphCut.h"
#include "v4r/ObjectModeling/ContourRefinement.h"
#include "v4r/ObjectModeling/CreateMeshModel.hh"

#include <pcl/io/pcd_io.h>

namespace segment
{
  
/**
 * @class Segmenter
 */
class Segmenter
{
private:
  
  bool useStructuralLevel;                                ///< Use structural level svm
  bool useAssemblyLevel;                                  ///< Use assembly
  
  double z_min, z_max;                                    ///< Minimum and maximum z-values

  std::string database_path;                              ///< database path
  std::string rgbd_filename;                              ///< depth image filename
  std::string kinect_config;                              ///< kinect configuration file
  std::string model_path;                                 ///< path to the svm model and scaling files

  unsigned startIdx, endIdx;                              ///< Start and end index of images
  bool data_live;                                         ///< load data live from Kinect
    
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;       ///< original pcl point cloud
  pcl::PointCloud<pcl::Normal>::Ptr normals;              ///< Normals of the point cloud
  std::vector<surface::SurfaceModel::Ptr > surfaces;      ///< Surfaces container (for Planes, NURBS)
  std::vector< std::vector<unsigned> > graphCutGroups;    ///< Graph cut groups of surface patch models

  KinectData *kinect;                                     ///< Load kinect data from file or live from Kinect
  surface::ClusterNormalsToPlanes *clusterNormals;        ///< Cluster normals to planes
  surface::SurfaceModeling *surfModeling;                 ///< Surface modeling (plane, NURBS, model selection)
  surface::PatchRelations *patchRelations;                ///< Calculatie the relations between surface patches
  svm::SVMPredictorSingle *svm1st;                        ///< SVM-predictor for structural level
  svm::SVMPredictorSingle *svm2nd;                        ///< SVM-predictor for assembly level
  gc::GraphCut *graphCut;                                 ///< Graph cut
  surface::SaveFileSequence *resultSaver;                 ///< Save segmented object models to sfv file


    svm::svm_problem svmProblem;                          ///< svm definition
    svm::svm_model *svmModel;
    std::vector<svm::svm_node *> svmProblem_vector_x ;
    std::vector<int> svmProblem_vector_y;
    svm::svm_parameter svmParameter;


    int indexing;
    int sizeOfVector;

public:

private:
  void init();
  void process();
  
public:
  Segmenter(std::string _db = "/media/Daten/OSD-0.2/",
            std::string _rgbd = "pcd/test%1d.pcd",
            std::string _model = "model/",
            bool _live = false);
  ~Segmenter();
  
  /** Use assembly level for processing **/
  void setAssemblyLevel(bool _on) {useAssemblyLevel = _on;}
  
  /** Set minimum and maximum depth values **/
  void setMinMaxDepth(double _z_min, double _z_max) {z_min = _z_min; z_max = _z_max;}
  
  /** Process a point cloud and return labeled cloud **/
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr
    processPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud);

  /** Process a point cloud and return vector of segment indices **/
  std::vector<pcl::PointIndices> 
    processPointCloudV(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud);

  /** Run the segmenter **/
  void run(std::string _rgbd_filename, 
           std::string _kinect_config,
           std::string _model_path,
           int _startIdx, int _endIdx, 
           bool _live, bool _useAssemblyLevel);


    /** svm **/
  void svm_model_create(std::vector<surface::Relation> &relation_vector, std::vector< std::vector<int> > anno);
  void annotator(std::vector<surface::Relation> &relation_vector );
};

}

#endif
