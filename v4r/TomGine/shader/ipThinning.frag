
// handle to texture
uniform sampler2D frame;
uniform sampler2D mask;

// enable thinning, or just normalize
uniform bool thinning_enabled;

uniform bool masked;
uniform bool binary;
uniform bool norm;

const vec4 red = vec4(1,0,0,0);
const vec4 green = vec4(0,1,0,0);
const vec4 blue = vec4(0,0,1,0);
const float pi = 3.141592;

vec4 thinning(){
	vec2 vG;
	vec4 edge_color;
	//vec2 vN;
	float fNm;
	float fGm;
	float f_len_1, f_len_2;
	vec4 vNull;
	vec4 vResult;

	if(binary)
		vNull = vec4(0.0,0.0,0.0,0.0);
	else
		vNull = vec4(0.5,0.5,0.0,0.0);
	
//     if(masked){
//     	vec4 vColor;
// 		vColor = texture2D(mask, gl_TexCoord[0].st);
// 		if( vColor.z != 0.0 )
// 			return vNull;
// 	}
	
	// get edge gradient from texture
	edge_color = texture2D(frame, gl_TexCoord[0].st);
	fGm = edge_color.z;	// magnitude
	vG = (edge_color.xy - 0.5) * 2.0;  // scale to range [-1 ... 1]
	//vG = normalize(vG);

	bool x_aligned = abs(vG.x) > abs(vG.y);

	// get color of neighbouring pixel
	if(x_aligned)
		fNm = textureOffset(frame, gl_TexCoord[0].xy, ivec2(1,0)).z;
	else
		fNm = textureOffset(frame, gl_TexCoord[0].xy, ivec2(0,1)).z;

	f_len_1 = fNm;
	if(fGm < f_len_1){
		vResult = vNull;
	}else{

    // get color for negative gradient direction
    if(x_aligned)
      fNm = textureOffset(frame, gl_TexCoord[0].xy, ivec2(-1,0)).z;
    else
      fNm = textureOffset(frame, gl_TexCoord[0].xy, ivec2(0,-1)).z;

    f_len_2 = fNm;
    if(fGm < f_len_2){
      vResult = vNull;
    }else if( (f_len_1 < 0.001) && (f_len_2 < 0.001) ){
      // remove this pixel if neighbouring pixels are no edges
      vResult = vNull;
    }else{

      // return gradient (xy) and magnitude (z) and safe it as color
      if(binary){

        if(norm)
          vResult = vec4(1.0f, 1.0f, 1.0f, 0.0f);
        else
          vResult = vec4(fGm, fGm, fGm, 0.0f);

      }else{

        // set magnitude to 1
        if(norm)
          vG = normalize(vG);

        // scale to range [0 ... 1]
        vG = vG * 0.5 + 0.5;
        vResult = vec4(vG,fGm,0.0);

      }
    }
	}
	return vResult;
}

void main(){
    gl_FragColor = thinning();
}
