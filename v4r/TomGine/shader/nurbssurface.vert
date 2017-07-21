
// includes 'cogxdeboor.c' as header

varying vec4 color;
varying float depth;

varying vec3 eye_normal;

uniform sampler1D knotsU;
uniform sampler1D knotsV;
uniform sampler2D cps;
uniform int orderU;
uniform int orderV;
uniform int nknotsU;
uniform int nknotsV;

vec4 getCP(int i, int j){
	return texelFetch(cps, ivec2(i,j), 0);
}

struct Vertex{
	vec3 pos;
	vec3 tangentU;
	vec3 tangentV;
};

Vertex EvaluateSurface(float xi, float eta){
	
	Vertex result;
	result.pos = gl_Vertex.xyz;

	// HACK
	if(xi==getKnot(nknotsU-1, knotsU))
		xi = xi - 0.000001;
	if(eta==getKnot(nknotsV-1, knotsV))
		eta = eta - 0.000001;

	// get knot span
	int spanU = getSpan(xi, knotsU, nknotsU, orderU);
	int spanV = getSpan(eta, knotsV, nknotsV, orderV);
	if(spanU==-1 || spanV==-1){
		return result;
	}

	// evaluate B-Spline basis functions using cox-de-boor algorithm
	NurbsBasis basisU = cox(xi, spanU, knotsU, nknotsU, orderU);
	NurbsBasis basisV = cox(eta, spanV, knotsV, nknotsV, orderV);

	// evaluate NURBS basis, divisor in equation (2.29) in [1]
	float W = 0.0;
	for(int su=0; su<(orderU+1); su++){
		for(int sv=0; sv<(orderV+1); sv++){
			vec4 cp = getCP(spanU-orderU+su, spanV-orderV+sv);
			W = W + cp.w * basisU.N[su + orderU*(orderU+1)] * basisV.N[sv + orderV*(orderV+1)];
			// TODO evaluate W for derivatives
		}
	}
	float dW = 1.0/W;

	// evaluate NURBS surface, equation (2.29) in [1]
	result.pos = vec3(0.0,0.0,0.0);
	result.tangentU = vec3(0.0,0.0,0.0);
	result.tangentV = vec3(0.0,0.0,0.0);
	for(int su=0; su<(orderU+1); su++){
		for(int sv=0; sv<(orderV+1); sv++){
			vec4 B = getCP(spanU-orderU+su, spanV-orderV+sv);
			float R = basisU.N[su + orderU*(orderU+1)] * basisV.N[sv + orderV*(orderV+1)] * B.w * dW;
			result.pos = result.pos + B.xyz * R;
			if(sv==0){
				float D = basisU.Nd[su + orderU*(orderU+1)];
				result.tangentU = result.tangentU + B.xyz * D;
			}
			if(su==0){
				float D = basisV.Nd[sv + orderV*(orderV+1)];
				result.tangentV = result.tangentV + B.xyz * D;
			}
		}
	}
	return result;
}

void main(){

	vec4 vertex = gl_Vertex;

	// use texture coordinates as parameter space
	float xi = vertex.x;
	float eta = vertex.y;

	Vertex result = EvaluateSurface(xi, eta);
	vertex = vec4(result.pos, 1.0);

	color = vec4(gl_Vertex.x, gl_Vertex.y, gl_Vertex.z, 0.0);

	vec3 normal = normalize(cross(result.tangentU, result.tangentV));
	eye_normal = gl_NormalMatrix * normal;

	gl_Position = gl_ModelViewProjectionMatrix * vertex;
	depth = gl_Position.z;
}
