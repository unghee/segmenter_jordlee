
// includes 'cogxdeboor.c' as header

uniform sampler1D knotsU;
uniform sampler1D knotsV;
uniform sampler1D knotsW;
uniform sampler3D cps;
uniform int orderU;
uniform int orderV;
uniform int orderW;
uniform int nknotsU;
uniform int nknotsV;
uniform int nknotsW;

varying vec4 color;

vec4 getCP(int i, int j, int k){
	return texelFetch(cps, ivec3(i,j,k), 0);
}

vec3 EvaluateSurface(float u, float v, float w){
	
	// HACK
	if(u==getKnot(nknotsU-1, knotsU))
		u = u - 0.000001;
	if(v==getKnot(nknotsV-1, knotsV))
		v = v - 0.000001;
	if(w==getKnot(nknotsW-1, knotsW))
		w = w - 0.000001;

	// get knot span
	int spanU = getSpan(u, knotsU, nknotsU, orderU);
	int spanV = getSpan(v, knotsV, nknotsV, orderV);
	int spanW = getSpan(w, knotsW, nknotsW, orderW);
	if(spanU==-1 || spanV==-1 || spanW==-1){
		color = vec4(1.0,0.2,0.0,0.0);
		return gl_Vertex.xyz;
	}

	// evaluate B-Spline basis functions using cox-de-boor algorithm
	NurbsBasis basisU = cox(u, spanU, knotsU, nknotsU, orderU);
	NurbsBasis basisV = cox(v, spanV, knotsV, nknotsV, orderV);
	NurbsBasis basisW = cox(w, spanW, knotsW, nknotsW, orderW);

	// evaluate NURBS basis, divisor in equation (2.29) in [1]
	float W = 0.0;
	for(int su=0; su<(orderU+1); su++){
		for(int sv=0; sv<(orderV+1); sv++){
			for(int sw=0; sw<(orderW+1); sw++){
				vec4 cp = getCP(spanU-orderU+su, spanV-orderV+sv, spanW-orderW+sw);
				W = W + cp.w * basisU.N[su + orderU*(orderU+1)] * basisV.N[sv + orderV*(orderV+1)] * basisW.N[sw + orderW*(orderW+1)];
			}
		}
	}
	float dW = 1.0/W;

	// evaluate NURBS surface, equation (2.29) in [1]
	vec3 result = vec3(0.0,0.0,0.0);
	for(int su=0; su<(orderU+1); su++){
		for(int sv=0; sv<(orderV+1); sv++){
			for(int sw=0; sw<(orderW+1); sw++){
				vec4 B = getCP(spanU-orderU+su, spanV-orderV+sv, spanW-orderW+sw);
				float R = basisU.N[su + orderU*(orderU+1)] * basisV.N[sv + orderV*(orderV+1)] * basisW.N[sw + orderW*(orderW+1)] * B.w * dW;
				result = result + B.xyz * R;
			}
		}
	}
	return result;
}

void main(){

	vec4 vertex = gl_Vertex;

	// use texture coordinates as parameter space
	float u = vertex.x;
	float v = vertex.y;
	float w = vertex.z;

	vec3 result = EvaluateSurface(u, v, w);
	vertex = vec4(result, 1.0);

	color = vec4(gl_Vertex.x, gl_Vertex.y, gl_Vertex.z, 0.0);

	gl_Position = gl_ModelViewProjectionMatrix * vertex;

}
