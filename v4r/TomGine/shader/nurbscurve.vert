
// includes 'cogxdeboor.c' as header

varying vec4 color;
varying float depth;

uniform sampler1D g_knots;
uniform sampler1D g_cps;
uniform int g_nknots;
uniform int g_order;

vec4 getCP(int i){
	return texelFetch(g_cps, i, 0);
}

vec3 EvaluateCurve(float xi){

	// HACK
	if(xi==getKnot(g_nknots-1, g_knots))
		xi = xi - 0.000001;

	// get knot span
	int span = getSpan(xi, g_knots, g_nknots, g_order);
	if(span==-1){
		return gl_Vertex.xyz;
	}

	// evaluate B-Spline basis functions using cox-de-boor algorithm, equation (2.1, 2.2) in [1]
	NurbsBasis basis = cox(xi, span, g_knots, g_nknots, g_order);

	// evaluate NURBS basis, divisor in equation (2.27) in [1]
	float W = 0.0;
	for(int id=0; id<(g_order+1); id++){
		vec4 cp = getCP(span-g_order+id);
		W = W + cp.w * basis.N[id + g_order*(g_order+1)];
	}
	float dW = 1.0/W;

	// evaluate NURBS curve, equation (2.27, 2.28) in [1]
	vec3 result = vec3(0.0,0.0,0.0);
	for(int si=0; si<(g_order+1); si++){
		vec4 B = getCP(span-g_order+si);
		float R = basis.N[si + g_order*(g_order+1)] * B.w * dW;
		result = result + B.xyz * R;
	}
	return result;
}


void main(){

	vec4 vertex = gl_Vertex;
	
	// use texture coordinates as parameter space
	float xi = vertex.x;

	vec3 result = EvaluateCurve(xi);
	vertex = vec4(result, 1.0);

	color = vec4(gl_Vertex.x, gl_Vertex.y, gl_Vertex.z, 0.0);

	gl_Position = gl_ModelViewProjectionMatrix * vertex;
	depth = gl_Position.z;
}
