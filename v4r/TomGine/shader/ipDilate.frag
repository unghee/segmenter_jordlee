
uniform sampler2D frame;

uniform float fDistScale;

vec4 spreading(){
	vec4 vColor;
	vec4 vMax;
	vec4 vNeighbour;
	
	// get gradient and magnitude of pixel stored in color
	vColor = texture2D(frame, gl_TexCoord[0]);
	vMax = vColor;
	
	// parse through neighbouring pixels
	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++){
			if( (i==1 || j==1) && (i!=j) ){
				// Erode filter:
				// - * -
				// * - *
				// - * -
			
				vNeighbour = textureOffset(frame, gl_TexCoord[0].st, ivec2((i-1),(j-1)));
				
				// if magnitude of adjacent pixel is bigger than a threshold
				// set magnitude of this pixel to that of the adjacent pixel considering distance
				// only pass gradient and magnitude of neighbour with maximum magnitude
				if(vNeighbour.z > vMax.z){
					vMax = vNeighbour;					
				}
			}
		}
	}
	
	if(vMax.z > vColor.z)
		vColor = vMax * fDistScale;	
	
	return vColor;
}

void main(){
    gl_FragColor = spreading();
}
