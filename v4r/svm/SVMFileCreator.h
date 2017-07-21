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
 * @file SVMFileCreator.h
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Store relations between features in a file for svm-training.
 */

#ifndef SVM_SVM_FILE_CREATOR_H
#define SVM_SVM_FILE_CREATOR_H

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include "v4r/SurfaceRelations/Relation.h"

namespace svm
{

/**
 * @brief Class SVMFileCreator: 
 */
class SVMFileCreator
{
public:
  
private:

  bool analyze;
  bool testset;
  bool have_relations;
  std::vector<surface::Relation> relations;

public:
  SVMFileCreator();
  ~SVMFileCreator();
  
  /** Set relation vector for structural and assembly level svm **/
  void setRelations(std::vector<surface::Relation> &_relations);
  void setRelations2(std::vector<surface::Relation> &_relations);
  
  /** Print aditional positive/negative file for analysation. **/
  void setAnalyzeOutput(bool _analyze) {analyze = _analyze;}
  
  /** Set output for testset **/
  void setTestSet(bool _testset) {testset = _testset;}
  
  /** Print relations for both levels to file (append) **/
  void process();
};

}

#endif

