uniform sampler2D frame;

const float epsilon = 1e-6f;

const float M_PI = 3.14159265358979323846;
const float M_PI_2 = 1.57079632679489661923;
const float M_1_PI = 0.31830988618379067154;

float param2polar(vec2 v)
{
	float r;

	if(v.x > epsilon){

		if(v.y > epsilon)
			r = atan(v.y/v.x);
		else if(v.y < -epsilon)
			r = 2.0f * M_PI + atan(v.y/v.x);
		else // v.y == 0.0
			r = 0.0;

	}else if(v.x < -epsilon){

		if(v.y > epsilon)
			r = M_PI + atan(v.y/v.x);
		else if(v.y < -epsilon)
			r = M_PI + atan(v.y/v.x);
		else // v.y == 0.0
			r = M_PI;

	}else{ // v.x == 0

		if(v.y > epsilon)
			r = M_PI_2;
		else if(v.y < -epsilon)
			r = 3.0f * M_PI_2;
		else
			r = 0.0f;

	}
	return r;
}


void main()
{
	vec3 param = texture2D( frame, gl_TexCoord[0] ).rgb;
	vec2 vG = (param.xy - 0.5) * 2.0;

	if(param.z<epsilon){
		gl_FragColor = vec4( 0.0, 0.0, 0.0, 0.0 );
	}else{
		float vP = param2polar(vG) * 0.5 * M_1_PI;
		gl_FragColor = vec4( vP, param.z, 0.0, 0.0 );
	}

}



