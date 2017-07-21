
#include "tgLight.h"

using namespace TomGine;

tgLight::tgLight(){
	ambient  = vec4(0.0, 0.0, 0.0, 1.0);
	diffuse  = vec4(1.0, 1.0, 1.0, 1.0);
	specular = vec4(1.0, 1.0, 1.0, 1.0);
	position = vec4(0.0, 0.0, 1.0, 0.0);
}

void tgLight::Activate(int id) const{
	glEnable(GL_LIGHTING);
	glLightfv(GL_LIGHT0+id, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0+id, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0+id, GL_SPECULAR, specular);
	glLightfv(GL_LIGHT0+id, GL_POSITION, position);
	glEnable(GL_LIGHT0+id);
}

void tgLight::Color(float r, float g, float b, float a){
	ambient = vec4(r,g,b,a) * 0.5f;
	diffuse = vec4(0.2f,0.2f,0.2f,a) + vec4(r,g,b,0.0f) * 0.8f;
	specular = vec4(0.3f,0.3f,0.3f,a);
	position = vec4(0.0f, 0.0f, 0.0f, 1.0f);
}

void tgLight::Random(){
	vec4 c;
	c.random();
	Color(c.x, c.y, c.z);
}

