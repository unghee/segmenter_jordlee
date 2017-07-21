
// handle to texture
uniform sampler2D frame;

const float kernel[25] = {	2,  4,  5,  4, 2,
							4,  9, 12,  9, 4,
							5, 12, 15, 12, 5,
							4,  9, 12,  9, 4,
							2,  4,  5,  4, 2 };
    						
vec4 gauss(){
	vec4 vColor = vec4(0.0, 0.0, 0.0, 0.0);
	    						
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2(-2,-2)) * kernel[0];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2(-2,-1)) * kernel[1];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2(-2, 0)) * kernel[2];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2(-2, 1)) * kernel[3];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2(-2, 2)) * kernel[4];

	vColor += textureOffset(frame, gl_TexCoord[0], ivec2(-1,-2)) * kernel[5];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2(-1,-1)) * kernel[6];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2(-1, 0)) * kernel[7];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2(-1, 1)) * kernel[8];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2(-1, 2)) * kernel[9];

	vColor += textureOffset(frame, gl_TexCoord[0], ivec2( 0,-2)) * kernel[10];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2( 0,-1)) * kernel[11];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2( 0, 0)) * kernel[12];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2( 0, 1)) * kernel[13];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2( 0, 2)) * kernel[14];

	vColor += textureOffset(frame, gl_TexCoord[0], ivec2( 1,-2)) * kernel[15];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2( 1,-1)) * kernel[16];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2( 1, 0)) * kernel[17];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2( 1, 1)) * kernel[18];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2( 1, 2)) * kernel[19];

	vColor += textureOffset(frame, gl_TexCoord[0], ivec2( 2,-2)) * kernel[20];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2( 2,-1)) * kernel[21];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2( 2, 0)) * kernel[22];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2( 2, 1)) * kernel[23];
	vColor += textureOffset(frame, gl_TexCoord[0], ivec2( 2, 2)) * kernel[24];
	
	vColor = vColor / 159.0;
	
	return vColor;
}

void main(){
	gl_FragColor = gauss();
}

