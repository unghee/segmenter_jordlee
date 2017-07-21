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
 * @file SVMFileCreator.cpp
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Store relations between features in a file for svm-training.
 */

#include "SVMFileCreator.h"

namespace svm
{

/**
 * @brief Constructor of SVMFileCreator
 */
SVMFileCreator::SVMFileCreator()
{
  analyze = false;
  testset = false;
  have_relations = false;
}


/**
 * @brief Destructor of SVMFileCreator
 */
SVMFileCreator::~SVMFileCreator()
{}


void SVMFileCreator::setRelations(std::vector<surface::Relation> &_relations)
{
  relations = _relations;
  have_relations = true;
}


/**
 * @brief Process the relation extraction algorithm
 */
void SVMFileCreator::process()
{
  if(!have_relations) {
    std::printf("[SVMFileCreator::process] Error: No relations available.\n");
    return;
  }

  if(!testset)
  {
    FILE *PPfile = std::fopen("./PP-Trainingsset.txt", "a");         // first level results
    FILE *PP2file = std::fopen("./PP2-Trainingsset.txt", "a");       // second level results
    FILE *PP3file = std::fopen("./PP-Trainingsset.pos.txt", "a");    // first level positiv results
    FILE *PP4file = std::fopen("./PP-Trainingsset.neg.txt", "a");    // first level negative results

    for(unsigned i=0; i<relations.size(); i++)
    {
      if(relations[i].type == 1)
      {
        std::fprintf(PPfile,"%u ", relations[i].groundTruth);
        for(unsigned j=0; j<relations[i].rel_value.size(); j++) {
          if(relations[i].rel_value[j] != relations[i].rel_value[j])
            printf("[SVMFileCreator::process] Warning: Found NAN in relation vector (%u-%u).\n", i, j);
          std::fprintf(PPfile,"%u:%6.5f ", j+1, relations[i].rel_value[j]);
        }
        std::fprintf(PPfile,"\n");
        
        if(relations[i].groundTruth) {
          std::fprintf(PP3file,"%u ", relations[i].groundTruth);
          for(unsigned j=0; j<relations[i].rel_value.size(); j++) {
            if(relations[i].rel_value[j] != relations[i].rel_value[j])
              printf("[SVMFileCreator::process] Warning: Found NAN in relation vector (%u-%u).\n", i, j);
            std::fprintf(PP3file,"%u:%6.5f ", j+1, relations[i].rel_value[j]);
          }
          std::fprintf(PP3file,"\n");
        }
        else {
          std::fprintf(PP4file,"%u ", relations[i].groundTruth);
          for(unsigned j=0; j<relations[i].rel_value.size(); j++) {
            if(relations[i].rel_value[j] != relations[i].rel_value[j])
              printf("[SVMFileCreator::process] Warning: Found NAN in relation vector (%u-%u).\n", i, j);
            std::fprintf(PP4file,"%u:%6.5f ", j+1, relations[i].rel_value[j]);
          }
          std::fprintf(PP4file,"\n");
        }    }
      else if(relations[i].type == 2)
      {
        fprintf(PP2file,"%u ", relations[i].groundTruth);
        for(unsigned j=0; j<relations[i].rel_value.size(); j++) {
          if(relations[i].rel_value[j] != relations[i].rel_value[j])
            printf("[SVMFileCreator::process] Warning: Found NAN in relation vector (%u-%u).\n", i, j);
          fprintf(PP2file,"%u:%6.5f ", j+1, relations[i].rel_value[j]);
        }
        fprintf(PP2file,"\n");
      }
      else
        printf("[SVMFileCreator::process] Warning: Unknown type of relation received.\n");
    }
    std::fclose(PPfile);
    std::fclose(PP2file);
    std::fclose(PP3file);
    std::fclose(PP4file);

    
    if(analyze)
    {
      static int counter = 0;
      
      FILE *PPfileAna = std::fopen("./PP-Trainingsset.txt.ana", "a");
      FILE *PPfilePos = std::fopen("./PP-Trainingsset.txt.pos", "a");
      FILE *PPfileNeg = std::fopen("./PP-Trainingsset.txt.neg", "a");
      FILE *PP2fileAna = std::fopen("./PP2-Trainingsset.txt.ana", "a");
      FILE *PP2filePos = std::fopen("./PP2-Trainingsset.txt.pos", "a");
      FILE *PP2fileNeg = std::fopen("./PP2-Trainingsset.txt.neg", "a");
      
      FILE *PPfileAnaRel = std::fopen("./PP-Trainingsset.txt.rel.ana", "a");
      FILE *PP2fileAnaRel = std::fopen("./PP2-Trainingsset.txt.rel.ana", "a");
      
      for(unsigned i=0; i<relations.size(); i++)
      {
        if(relations[i].type == 1)
        {
          std::fprintf(PPfileAna,"%u, ", relations[i].groundTruth);
          for(unsigned j=0; j<relations[i].rel_value.size(); j++)
            std::fprintf(PPfileAna,"%6.5f, ", relations[i].rel_value[j]);
          std::fprintf(PPfileAna,"\n");
          
          std::fprintf(PPfileAnaRel,"%u: [%u][%u] (%u), ", counter, relations[i].id_0, relations[i].id_1, relations[i].groundTruth);
          for(unsigned j=0; j<relations[i].rel_value.size(); j++)
            std::fprintf(PPfileAnaRel,"%6.5f, ", relations[i].rel_value[j]);
          std::fprintf(PPfileAnaRel,"\n");          
          
          if(relations[i].groundTruth) {
            for(unsigned j=0; j<relations[i].rel_value.size(); j++)
              std::fprintf(PPfilePos,"%6.5f, ", relations[i].rel_value[j]);
            std::fprintf(PPfilePos,"\n");
          }
          else {
            for(unsigned j=0; j<relations[i].rel_value.size(); j++)
              std::fprintf(PPfileNeg,"%6.5f, ", relations[i].rel_value[j]);
            std::fprintf(PPfileNeg,"\n");
          }
        }
        if(relations[i].type == 2)
        {
          std::fprintf(PP2fileAna,"%u, ", relations[i].groundTruth);
          for(unsigned j=0; j<relations[i].rel_value.size(); j++)
            std::fprintf(PP2fileAna,"%6.5f, ", relations[i].rel_value[j]);
          std::fprintf(PP2fileAna,"\n");
  
          std::fprintf(PP2fileAnaRel,"%u: [%u][%u] (%u), ", counter, relations[i].id_0, relations[i].id_1, relations[i].groundTruth);
          for(unsigned j=0; j<relations[i].rel_value.size(); j++)
            std::fprintf(PP2fileAnaRel,"%6.5f, ", relations[i].rel_value[j]);
          std::fprintf(PP2fileAnaRel,"\n");   
          
          if(relations[i].groundTruth) {
            for(unsigned j=0; j<relations[i].rel_value.size(); j++)
              std::fprintf(PP2filePos,"%6.5f, ", relations[i].rel_value[j]);
            std::fprintf(PP2filePos,"\n");
          }
          else {
            for(unsigned j=0; j<relations[i].rel_value.size(); j++)
              std::fprintf(PP2fileNeg,"%6.5f, ", relations[i].rel_value[j]);
            std::fprintf(PP2fileNeg,"\n");
          }
        }
      }
      std::fclose(PPfileAna);
      std::fclose(PPfilePos);
      std::fclose(PPfileNeg);
      std::fclose(PP2fileAna);
      std::fclose(PP2filePos);
      std::fclose(PP2fileNeg);
      std::fclose(PPfileAnaRel);
      std::fclose(PP2fileAnaRel);
      counter++;
    } 
  }
  
  /// TESTSET OUTPUT
  /// The output of the testset is not complete (write only relations, if they are correct or not)
  else
  {
    FILE *PPfile = std::fopen("./PP-Testset.txt", "a");         // structural level results
    FILE *PP2file = std::fopen("./PP2-Testset.txt", "a");       // assembly level results

    for(unsigned i=0; i<relations.size(); i++)
    {
      if(relations[i].groundTruth == 0  || relations[i].groundTruth == 1)   // write out, only relations with known groundTruth
      {
        if(relations[i].type == 1)
        {
          std::fprintf(PPfile,"%u ", relations[i].groundTruth);
          for(unsigned j=0; j<relations[i].rel_value.size(); j++) {
            if(relations[i].rel_value[j] != relations[i].rel_value[j])
              printf("[SVMFileCreator::process] Warning: Found NAN in relation vector (%u-%u).\n", i, j);
            std::fprintf(PPfile,"%u:%6.5f ", j+1, relations[i].rel_value[j]);
          }
          std::fprintf(PPfile,"\n");
        }
        else if(relations[i].type == 2)
        {
          fprintf(PP2file,"%u ", relations[i].groundTruth);
          for(unsigned j=0; j<relations[i].rel_value.size(); j++) {
            if(relations[i].rel_value[j] != relations[i].rel_value[j])
              printf("[SVMFileCreator::process] Warning: Found NAN in relation vector (%u-%u).\n", i, j);
            fprintf(PP2file,"%u:%6.5f ", j+1, relations[i].rel_value[j]);
          }
          fprintf(PP2file,"\n");
        }
        else
          printf("[SVMFileCreator::process] Warning: Unknown type of relation received.\n");
      }
    }
    std::fclose(PPfile);
    std::fclose(PP2file);

    if(analyze)
    {
      static int counter = 0;
      
      FILE *PPfileAna = std::fopen("./PP-Testset.txt.ana", "a");                           // All relations (with unknown ones)
      FILE *PPfilePos = std::fopen("./PP-Testset.txt.pos", "a");
      FILE *PPfileNeg = std::fopen("./PP-Testset.txt.neg", "a");
      FILE *PP2fileAna = std::fopen("./PP2-Testset.txt.ana", "a");
      FILE *PP2filePos = std::fopen("./PP2-Testset.txt.pos", "a");
      FILE *PP2fileNeg = std::fopen("./PP2-Testset.txt.neg", "a");
      FILE *PPfileAnaRel = std::fopen("./PP-Testset.txt.rel.ana", "a");                    // All relations (with unknown ones)
      FILE *PP2fileAnaRel = std::fopen("./PP2-Testset.txt.rel.ana", "a");                  // All relations (with unknown ones)
      
      for(unsigned i=0; i<relations.size(); i++)
      {
        if(relations[i].type == 1)
        {
          std::fprintf(PPfileAna,"%u, ", relations[i].groundTruth);
          for(unsigned j=0; j<relations[i].rel_value.size(); j++)
            std::fprintf(PPfileAna,"%6.5f, ", relations[i].rel_value[j]);
          std::fprintf(PPfileAna,"\n");
          
          std::fprintf(PPfileAnaRel,"%u: [%u][%u] (%u), ", counter, relations[i].id_0, relations[i].id_1, relations[i].groundTruth);
          for(unsigned j=0; j<relations[i].rel_value.size(); j++)
            std::fprintf(PPfileAnaRel,"%6.5f, ", relations[i].rel_value[j]);
          std::fprintf(PPfileAnaRel,"\n");          
          
          if(relations[i].groundTruth == 1) {
            for(unsigned j=0; j<relations[i].rel_value.size(); j++)
              std::fprintf(PPfilePos,"%6.5f, ", relations[i].rel_value[j]);
            std::fprintf(PPfilePos,"\n");
          }
          else if(relations[i].groundTruth == 0) {
            for(unsigned j=0; j<relations[i].rel_value.size(); j++)
              std::fprintf(PPfileNeg,"%6.5f, ", relations[i].rel_value[j]);
            std::fprintf(PPfileNeg,"\n");
          }
        }
        if(relations[i].type == 2)
        {
          std::fprintf(PP2fileAna,"%u, ", relations[i].groundTruth);
          for(unsigned j=0; j<relations[i].rel_value.size(); j++)
            std::fprintf(PP2fileAna,"%6.5f, ", relations[i].rel_value[j]);
          std::fprintf(PP2fileAna,"\n");
  
          std::fprintf(PP2fileAnaRel,"%u: [%u][%u] (%u), ", counter, relations[i].id_0, relations[i].id_1, relations[i].groundTruth);
          for(unsigned j=0; j<relations[i].rel_value.size(); j++)
            std::fprintf(PP2fileAnaRel,"%6.5f, ", relations[i].rel_value[j]);
          std::fprintf(PP2fileAnaRel,"\n");   
          
          if(relations[i].groundTruth == 1) {
            for(unsigned j=0; j<relations[i].rel_value.size(); j++)
              std::fprintf(PP2filePos,"%6.5f, ", relations[i].rel_value[j]);
            std::fprintf(PP2filePos,"\n");
          }
          else if(relations[i].groundTruth == 0) {
            for(unsigned j=0; j<relations[i].rel_value.size(); j++)
              std::fprintf(PP2fileNeg,"%6.5f, ", relations[i].rel_value[j]);
            std::fprintf(PP2fileNeg,"\n");
          }
        }
      }
      std::fclose(PPfileAna);
      std::fclose(PPfilePos);
      std::fclose(PPfileNeg);
      std::fclose(PP2fileAna);
      std::fclose(PP2filePos);
      std::fclose(PP2fileNeg);
      std::fclose(PPfileAnaRel);
      std::fclose(PP2fileAnaRel);
      counter++;
    } 
  } 
}

} 











