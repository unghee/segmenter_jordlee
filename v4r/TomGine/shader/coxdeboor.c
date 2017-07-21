
// Cox-de-Boor Algorithm
// [1] Isogeometric Analysis, Toward Integration of CAD and FEA; J.Austin Cottrell, Thomas J.R. Hughes, Yuri Bazilevs

float getKnot(int i, sampler1D knots){
	return texelFetch(knots, i, 0).r;
}

struct NurbsBasis{
	float N[25]; // (order<=4) -> (4+1)*(4+1)=25
	float Nd[25];
};

int getSpan(float xi, sampler1D knots, int nknots, int order)
{
	int knotSpan = -1;
	for(int s=0; s<nknots; s++){
		if( xi>=getKnot(s, knots) && xi<getKnot(s+1, knots) ){
			knotSpan = s;
			break;
		}else if(xi==getKnot(nknots-1, knots)){
			knotSpan = nknots-order-1;
		}
	}
	return knotSpan;
}

NurbsBasis cox(float xi, int knotSpan, sampler1D knots, int nknots, int order)
{
	NurbsBasis basis;
	int nsupKnot = 2*order+1;
	float supKnot[9];	// (order<=4) -> max(nsupKnot) = 9

	// get supporting knots
	for(int s=0; s<nsupKnot; s++){
		supKnot[s] = getKnot(knotSpan-order+s, knots);
	}

	for(int p=0; p<(order+1); p++){	// loop from lower order to higher -> unwrapped recursion

		for(int s=0; s<(order+1); s++){	// evaluate the basis N for each knotspan i

			if(p==0){
				// Equation (2.1) in [1]
				if(xi>=supKnot[s] && xi<supKnot[s+1])
					basis.N[s] = 1.0;
				else if(s==order && xi==supKnot[s+1])
					basis.N[s] = 1.0;
				else
					basis.N[s] = 0.0;

			}else{
				// Equation (2.2) in [1]
				float A=0.0;
				float B=0.0;
				if( (xi-supKnot[s])!=0.0 && (supKnot[s+p]-supKnot[s])!=0.0 )
					A = (xi-supKnot[s]) / (supKnot[s+p]-supKnot[s]);

				if( (supKnot[s+p+1]-xi)!=0.0 && (supKnot[s+p+1]-supKnot[s+1])!=0.0 && s<order ) // (s<order) because N(s+i,p-1) does not support
					B = (supKnot[s+p+1]-xi) / (supKnot[s+p+1]-supKnot[s+1]);

				basis.N[s + p*(order+1)] = A * basis.N[s + (p-1)*(order+1)] + B * basis.N[(s+1) + (p-1)*(order+1)];

			}
		}
	}

	int p = order;
	for( int s = 0; s < (order + 1); s++ )
	{ // evaluate the basis N for each knotspan s
		// Equation (2.7)
		float Ad = 0.0f;
		float Bd = 0.0f;
		if( (supKnot[s + p] - supKnot[s]) != 0.0 )
			Ad = p / (supKnot[s + p] - supKnot[s]);

		if( (supKnot[s + p + 1] - supKnot[s + 1]) != 0.0 && s < order ) // (s<order) because N(s+i,p-1) does not support
			Bd = p / (supKnot[s + p + 1] - supKnot[s + 1]);

		basis.Nd[s + p * (order + 1)] = Ad * basis.N[s + (p - 1) * (order + 1)] - Bd * basis.N[(s + 1) + (p - 1) * (order + 1)];
	}

	return basis;
}
