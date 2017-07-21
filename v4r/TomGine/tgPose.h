 /**
 * @file tgPose.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Defining the position and orientation of an object in 3D.
 */
 
#ifndef TG_POSE
#define TG_POSE

#include "tgMathlib.h"
#include "tgQuaternion.h"

namespace TomGine{

/** @brief Represents a pose in 3D space with position and orientation.
 *  See Special Euclidean Space SE(3).
 *  Orientation is implemented using quaternions. */
class tgPose{
public:
	vec3 t;				///< Position of the pose. (or translation)
	tgQuaternion q;		///< Orientation of the pose. (or rotation)
	
	/** @brief	Compares two poses for equality. */
	bool operator==(const tgPose &p) const;
	/** @brief	Transform pose p into this coordinate frame. */
	tgPose operator*(const tgPose& p) const;
	/** @brief	Transform vector t into this coordinate frame. */
	vec3 operator*(const vec3& t) const;
	/** @brief	Add poses. */
	tgPose operator+(const tgPose &p) const;
	/** @brief	Subtract poses. */
	tgPose operator-(const tgPose &p) const;
	/** @brief	Calculate the transpose (inverse for similarity poses). */
	tgPose Transpose() const;
	
	/** @brief	Prints the components of the position and orientation to the console. */
	void Print() const;

	/** @brief	Pushes the pose as transformation matrix into the OpenGL matrix stack. */
	void Activate() const;	
	/** @brief	Pops (removes) the pose from the OpenGL matrix stack. */
	void Deactivate() const;
	/**	@brief Draws a simple coordinate frame at this pose */
	void DrawCoordinates(float linelength = 1.0f, float linewidth = 1.0f) const;
	
	/** @brief	Set the pose as rotation r and translation p. */
	void SetPose(mat3 r, vec3 p);	
	/** @brief	Gets the pose as rotation r and translation p. */
	void GetPose(mat3 &r, vec3 &p) const;
	
	/** @brief	Rotate the pose using Euler angles.
	 *  @param	x,y,z	Rotation about x,y,z axis respectively. */
	void Rotate(float x, float y, float z);	
	/** @brief	Rotate pose through length(r) in radians about axis given by r. */
	void RotateAxis(vec3 r);
	/** @brief	Rotate the pose using Euler angles.
	 *  @param	r	Rotation about x,y,z axis respectively. */
	void RotateEuler(vec3 r);
	/** @brief	Translation in x,y,z direction. */
	void Translate(float x, float y, float z);	
	/** @brief	Translation along the vector t by the length of t. */
	void Translate(vec3 t);	
};

} // namespace TomGine

#endif
