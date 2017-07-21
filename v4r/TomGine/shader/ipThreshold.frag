
uniform sampler2D tex1;

uniform float th;

void main(){
	vec4 col1 = texture2D(tex1, gl_TexCoord[0].st);
	vec4 col2 = vec4(0.0,0.0,0.0,0.0);
	
	if(col1.r > th) col2.r = 1.0;
	if(col1.g > th) col2.g = 1.0;
	if(col1.b > th) col2.b = 1.0;
	if(col1.a > th) col2.a = 1.0;	

	gl_FragColor = col2;
}
