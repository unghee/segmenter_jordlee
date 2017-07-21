 /**
 * @file tgLight.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Light settings, representing a light. See glLight in the OpenGL spec. for more information.
 */

#ifndef TG_LIGHTING
#define TG_LIGHTING

#include "headers.h"
#include "tgMathlib.h"

namespace TomGine{

/** @brief Light settings, representing a light. See glLight in the OpenGL spec. for more information. */
class tgLight
{
public:
	vec4 ambient;	///< ambient RGBA intensity of light.
	vec4 diffuse;	///< diffuse RGBA intensity of light.
	vec4 specular;	///< specular RGBA intensity of light.
	vec4 position;	///< position of the light in homogeneous coordinates.

	/** @brief Create white light. */
	tgLight();

	/** @brief Enables lighting and applies colors and position of the light.
	 *  @param id	OpenGL light id ranging from 0 to GL_MAX_LIGHTS-1 */
	void Activate(int id=0) const;

	/** @brief Creates light with a specific color intensity RGBA */
	void Color(float r, float g, float b, float a=1.0f);

	/** @brief Create light with random color intensity RGBA. */
	void Random();
};

} // namespace TomGine

#endif
