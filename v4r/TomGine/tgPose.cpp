
#include "tgPose.h"

#include <stdio.h>

#ifdef WIN32
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>
#endif

using namespace TomGine;

bool tgPose::operator==(const tgPose &p) const{
	return t==p.t && q==p.q;
}

tgPose tgPose::operator*(const tgPose& p) const{
	tgPose ret;
	mat3 R, pR, rR;
	vec3 t, pt, rt;
	
	this->GetPose(R,t);
	p.GetPose(pR, pt);
	
	rR = R * pR;
	rt = (R * pt) + t;
	
	ret.SetPose(rR, rt);
	return ret;
}

vec3 tgPose::operator*(const vec3& pt) const{
	vec3 ret;
	mat3 R;
	vec3 t;
	
	this->GetPose(R,t);
	
	ret = (R * pt) + t;
	
	return ret;
}

tgPose tgPose::operator+(const tgPose &p) const{
	tgPose res;
	res.t = t - p.t;
	res.q = q - p.q;
	return res;
}

tgPose tgPose::operator-(const tgPose &p) const{
	tgPose res;
	res.t = t - p.t;
	res.q = q - p.q;
	return res;
}

tgPose tgPose::Transpose() const{
	tgPose ret;
	mat3 R;
	vec3 t;
	GetPose(R,t);
	
	R = R.transpose();
	t = t;
	
	ret.SetPose(R,t);
	return ret;
}

void tgPose::Activate() const {
	glPushMatrix();
		glTranslatef(t.x, t.y, t.z);
		glMultMatrixf(q.getMatrix4());
}

void tgPose::Deactivate() const {
	glPopMatrix();
}

void tgPose::DrawCoordinates(float linelength, float linewidth) const{
	this->Activate();

	float l1 = linelength;
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_TEXTURE_2D);
	glLineWidth(linewidth);

	glBegin(GL_LINES);
		glColor3f(1.0f,0.0f,0.0f);
		glVertex3f(0.0f,0.0f,0.0f); glVertex3f(l1, 0.0f, 0.0f);
		glColor3f(0.0f,1.0f,0.0f);
		glVertex3f(0.0f,0.0f,0.0f); glVertex3f(0.0f, l1, 0.0f);
		glColor3f(0.0f,0.0f,1.0f);
		glVertex3f(0.0f,0.0f,0.0f); glVertex3f(0.0f, 0.0f, l1);
	glEnd();

	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);

	this->Deactivate();
}

void tgPose::Print() const {
	printf("%f %f %f %f %f %f %f\n", t.x, t.y, t.z, q.x, q.y, q.z, q.w);
}

void tgPose::SetPose(mat3 rot, vec3 pos){
	q.fromMatrix(rot);
	q.normalise();
		
	t.x = pos.x;
	t.y = pos.y;
	t.z = pos.z;
}

void tgPose::GetPose(mat3 &rot, vec3 &pos) const {
	
	rot = q.getMatrix3();
	
	pos.x = t.x;
	pos.y = t.y;
	pos.z = t.z;
}

void tgPose::Rotate(float x, float y, float z){
	if(x==0.0 && y==0.0 && z ==0.0)
		return;
	
	tgQuaternion q2;
	
	q2.fromEuler(x,y,z);
	q = q2 * q;
	q.normalise();
}

void tgPose::RotateAxis(vec3 r){
// 	tgQuaternion q2;
// 	q2.fromEuler(rot.x,rot.y,rot.z);
// 	q = q2 * q;
// 	q.normalise();
	tgQuaternion q2;
	float a = r.length();
	r.normalize();
	q2.fromAxis(r,a);
	q = q2 * q;
	q.normalise(); 
}

void tgPose::RotateEuler(vec3 r){
	tgQuaternion q2;
	q2.fromEuler(r.x,r.y,r.z);
	q = q2 * q;
	q.normalise();
}

void tgPose::Translate(float x, float y, float z){
	t.x = t.x + x;
	t.y = t.y + y;
	t.z = t.z + z;
}

void tgPose::Translate(vec3 trans){
	t.x = t.x + trans.x;
	t.y = t.y + trans.y;
	t.z = t.z + trans.z;
}
