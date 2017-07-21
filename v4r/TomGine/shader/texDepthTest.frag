varying vec4 vertex_model;

uniform sampler2D tex_depth_map;

uniform mat4 modelviewprojection;

uniform float dist_off;
uniform float dist_max;

void main(){
	vec4 texcoords_model;
	vec4 texcoords_frame;
	float depth;
	float error;
	
	// get edge information of model
	texcoords_model = modelviewprojection * vertex_model;
	texcoords_model.x = (texcoords_model.x / texcoords_model.w + 1.0) * 0.5;
	texcoords_model.y = (texcoords_model.y / texcoords_model.w + 1.0) * 0.5;
	depth = texture2D(tex_depth_map, texcoords_model.xy).x;
	
	depth += dist_off;
	depth *= dist_max;
	
	error = abs(texcoords_model.z - depth);
	
	gl_FragColor = vec4(error, 1.0-error, 0.0, 0.0);
}



