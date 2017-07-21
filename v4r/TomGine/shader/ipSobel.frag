
// handle to texture
uniform sampler2D frame;
uniform sampler2D mask;

// kernels for edge detection (to load from program)
uniform mat3 mSobelX;
uniform mat3 mSobelY;

// Threshold for removing noise
uniform float fThreshold;

uniform bool norm;
uniform bool masked;
uniform bool binary;

vec4 sobel(){
	vec4 vColor, vCenterColor, vMask;
	float fGx, fGy, fGm;
	vec3 vGr, vGg, vGb, vGm;
	vec2 vG;
	vec4 vNull;
	vec4 vResult;

	if(binary)
		vNull = vec4(0.0,0.0,0.0,0.0);
	else
		vNull = vec4(0.5,0.5,0.0,0.0);

//	if(masked){
//		vMask = texture2D(mask, gl_TexCoord[0].st);
//		if( vMask.z != 0.0 )
//			return vNull;
//		vCenterColor = texture2D(frame, gl_TexCoord[0].st);
//	}
	
	// convolute sobel kernel for each color channel
	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++){
		  if(masked){
		    vMask = textureOffset(mask, gl_TexCoord[0].st, ivec2((i-1),(j-1)));
		    if( vMask.z > 0.0 )
		      vColor = vCenterColor;
		    else
		      vColor = textureOffset(frame, gl_TexCoord[0].st, ivec2((i-1),(j-1)));
		  }else{
		    vColor = textureOffset(frame, gl_TexCoord[0].st, ivec2((i-1),(j-1)));
		  }
			vGr.x += mSobelX[i][j] * vColor.r;
			vGr.y += mSobelY[i][j] * vColor.r;
			vGr.z = abs(vGr.x + vGr.y);
			vGg.x += mSobelX[i][j] * vColor.g;
			vGg.y += mSobelY[i][j] * vColor.g;
			vGg.z = abs(vGg.x + vGg.y);
			vGb.x += mSobelX[i][j] * vColor.b;
			vGb.y += mSobelY[i][j] * vColor.b;
			vGb.z = abs(vGb.x + vGb.y);
		}
	}

	// determine channel with maximum gradient
	vGm = vGr;
	if(vGg.z > vGm.z) vGm = vGg;
	if(vGb.z > vGm.z) vGm = vGb;
	
	vG = vec2(vGm.x,vGm.y);
	
	// magnitude of edge
	fGm = length(vG);
	
	// Threshold for removing background noise
	if(fGm <= fThreshold){
		vResult = vNull;
	}else{
	
    if(binary){
      if(norm)
        vResult = vec4(1.0f, 1.0f, 1.0f, 0.0);
      else
        vResult = vec4(fGm, fGm, fGm, 0.0);
    }else{

      if(norm)
        vG = normalize(vG);

      // scale gradient to range = [0 ... 1]
      vG = vG * 0.5 + 0.5;
      vResult = vec4(vG, fGm, 0.0);

    }
	}
	
	return vResult;
}

void main(){
	gl_FragColor = sobel();
}
