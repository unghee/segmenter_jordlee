varying vec4 color;

varying vec3 eye_normal;

uniform vec4 ucolor;
uniform bool user_color=false;


void main()
{

//	eye_normal = normalize(eye_normal);
//	if( dot(eye_normal, vec3(0.0f, 0.0f, 1.0f)) >= 0.0f)
//		discard;

	if( user_color )
		gl_FragColor = ucolor;
	else
		gl_FragColor = color;

}

