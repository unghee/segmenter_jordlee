
uniform sampler2D tex1;
uniform sampler2D tex2;

uniform float k1;
uniform float k2;

void main(){
	vec4 vColor1 = texture2D(tex1, gl_TexCoord[0].st);
	vec4 vColor2 = texture2D(tex2, gl_TexCoord[0].st);
	
	gl_FragColor = (vColor1*k1 + vColor2*k2);
}
