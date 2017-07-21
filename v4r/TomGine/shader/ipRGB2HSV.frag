uniform sampler2D frame;

const float ftol = 0.01;
const float d360 = 0.002777778;

vec3 rgb2hsv(vec3 rgb)
{
	float h,s,v;
	
	float fMax = rgb.r;
	float fMin = rgb.r;
	if(rgb.g > fMax) fMax = rgb.g;
	if(rgb.b > fMax) fMax = rgb.b;
	if(rgb.g < fMin) fMin = rgb.g;
	if(rgb.b < fMin) fMin = rgb.b;
	
	if(fMax-fMin < ftol) h = 0.0;
	else if(fMax == rgb.r) h = 60.0 * (0.0 + (rgb.g-rgb.b)/(fMax-fMin));
	else if(fMax == rgb.g) h = 60.0 * (2.0 + (rgb.b-rgb.r)/(fMax-fMin));
	else if(fMax == rgb.b) h = 60.0 * (4.0 + (rgb.r-rgb.g)/(fMax-fMin));
	
	if(h < 0.0)
		h = h + 360.0;
	h = h * d360;
	
	if(fMax < ftol)
		s = 0.0;
	else
		s = (fMax-fMin)/fMax;
	
	v = fMax;
		
	vec3 hsv = vec3(h, s, v);

	return hsv;
}


void main()
{
	vec3 rgb = texture2D( frame, gl_TexCoord[0] ).rgb;
	gl_FragColor = vec4( rgb2hsv(rgb), 0.0 );
}



