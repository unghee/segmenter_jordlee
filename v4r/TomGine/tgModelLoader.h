 /**
 * @file tgModelLoader.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Loading geometrical models from file
 * @namespace TomGine
 */
 
#ifndef TG_MODELLOADER
#define TG_MODELLOADER

#include <stdlib.h>
#include <stddef.h>
#include <string>

#include "ply.h"
#include "PlyStructure.h"
#include "tgModel.h"
#include "tgRenderModel.h"


namespace TomGine{

/** @brief Loading geometrical models from file. */
class tgModelLoader
{
private:

	static bool propertyIsInList(PlyProperty* prop, PlyProperty* list, int n, int* index);

public:
	
	/** @brief Loads PLY files (Polygon File Format, Stanford Triangle Format)
	*		@param model	Model to load file to.
	*		@param filename	Path and filename of file to load.
	*		@return True on success, false on failure. */
	static bool LoadPly(tgModel &model, const char* filename);
	
	/** @brief Saves PLY files (Polygon File Format, Stanford Triangle Format)
	*	@param model Model to save.
	*	@param filename Path and filename of file to save to.
	*	@return True on success, false on failure. */
	static bool SavePly(tgModel &model, const char* filename);
	   
};

} // namespace TomGine

#endif
