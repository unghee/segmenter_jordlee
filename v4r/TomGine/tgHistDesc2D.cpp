
#include "tgHistDesc2D.h"

using namespace TomGine;

//bool tgHistDesc2D::ispow2(unsigned a)
//{
//	unsigned i=1;
//	while(a>pow(2,i))
//		i++;
//
//	return a==pow(2,i);
//}

unsigned tgHistDesc2D::global2local(	unsigned idx,
										const TomGine::tgRect2Di &bb,
										const unsigned &subsizex, const unsigned &subsizey,
										const unsigned &nsegx,
										unsigned &subidx)
{
	unsigned I = idx % bb.w;
	unsigned J = idx / bb.w;

	unsigned i = I % subsizex;
	unsigned j = J % subsizey;

	unsigned segx = I / subsizex;
	unsigned segy = J / subsizey;

	subidx = segy*nsegx + segx;

	return (j*subsizex + i);
}

//unsigned tgHistDesc2D::local2global(	unsigned idx,
//										const TomGine::tgRect2Di &bb,
//										const unsigned &subsizex, const unsigned &subsizey,
//										const unsigned &segx, const unsigned &segy)
//{
//	unsigned i = idx % subsizex;
//	unsigned j = idx / subsizex;
//
//	unsigned I = subsizex * segx + i;
//	unsigned J = subsizey * segy + j;
//
//	return (J*bb.w + I);
//}

void tgHistDesc2D::Preprocess( const TomGine::tgRect2Di &bb )
{
	if(bb.w%m_nsegx){
		printf("[tgHistDesc2D::Preprocess] Error bounding-box width not divideable (%d / %d; %d).\n", bb.w, m_nsegx, m_bb.w);
		return;
	}
	if(bb.h%m_nsegy){
		printf("[tgHistDesc2D::Preprocess] Error bounding-box height not divideable (%d / %d; %d).\n", bb.h, m_nsegy, m_bb.h);
		return;
	}

	m_bb = bb;

	m_subsizex = m_bb.w / m_nsegx;
	m_subsizey = m_bb.h / m_nsegy;

//	printf("[tgHistDesc2D::Init2] %d, %d, %d %d, %d %d\n", m_nsegx, m_nsegy, m_subsizex, m_subsizey, m_bb.w, m_bb.h);
}

tgHistDesc2D::tgHistDesc2D(unsigned segx, unsigned segy, unsigned hist_bins_x, unsigned hist_bins_y)
{
	m_nsegx = segx;
	m_nsegy = segy;

	unsigned size = m_nsegx*m_nsegy;

	m_histlist.assign( size, TomGine::tgHistogram2D(hist_bins_x,hist_bins_y) );

	m_subdata.assign( size, std::vector<vec2>() );

	m_maskratio.assign( size, 0.0f );
}

void tgHistDesc2D::Process(	const std::vector<vec2> &data,
								const TomGine::tgRect2Di &bb,
								vec2 min_val, vec2 max_val)
{
	Preprocess(bb);

	if((int)data.size() != m_bb.w*m_bb.h){
		printf("[tgHistDesc2D::Preprocess] Warning size of data does not match size of bounding-box [%d != %dx%d].\n", (int)data.size(), m_bb.w, m_bb.h);
		return;
	}

	for(unsigned i=0; i<m_subdata.size(); i++)
		m_subdata[i].clear();

	unsigned subidx=0;
	for(unsigned IDX=0; IDX<data.size(); IDX++)
	{
		vec2 d = data[IDX];
		global2local(IDX, m_bb, m_subsizex, m_subsizey, m_nsegx, subidx);
		if(d.y > 0.001f)
			m_subdata[subidx].push_back(d);
	}
	for( unsigned i=0; i<m_subdata.size(); i++ ){
		m_histlist[i].Reset();
		m_histlist[i].EvaluatePolar(m_subdata[i],min_val,max_val);
	}
}

void tgHistDesc2D::Process( const std::vector<vec2> &data,
							const TomGine::tgRect2Di &bb,
							vec2 min_val, vec2 max_val,
							const std::vector<bool> &mask, double vis_ratio)
{
	Preprocess(bb);

	if((int)data.size() != m_bb.w*m_bb.h){
		printf("[tgHistDesc2D::Preprocess] Warning size of data does not match size of bounding-box [%d != %dx%d].\n", (int)data.size(), m_bb.w, m_bb.h);
		return;
	}
	if((int)mask.size() != m_bb.w*m_bb.h){
		printf("[tgHistDesc2D::Preprocess] Warning size of mask does not match size of bounding-box [%d != %dx%d].\n", (int)mask.size(), m_bb.w, m_bb.h);
		return;
	}

	for(unsigned i=0; i<m_subdata.size(); i++)
		m_subdata[i].clear();

	m_maskratio.assign(m_maskratio.size(), 0.0);

	double maskdiv = 1.0/(m_subsizex*m_subsizey);
	unsigned subidx=0;
	for(unsigned IDX=0; IDX<data.size(); IDX++)
	{
		vec2 d = data[IDX];
		global2local(IDX, m_bb, m_subsizex, m_subsizey, m_nsegx, subidx);
		if(mask[IDX]){
			if(d.y > 0.001f)
				m_subdata[subidx].push_back(d);
			m_maskratio[subidx] += maskdiv;
		}
	}
	for( unsigned i=0; i<m_subdata.size(); i++ ){
		m_histlist[i].Reset();
		if(this->m_maskratio[i] >= vis_ratio)
			m_histlist[i].EvaluatePolar(m_subdata[i],min_val,max_val);
	}
}

std::vector<double> tgHistDesc2D::operator*(const tgHistDesc2D& h) const
{
	std::vector<double> result;
	if( this->m_histlist.size() != h.m_histlist.size() )	{
		printf("[tgHistDesc2D::operator*] Number of descriptor histograms do not match.\n");
		return result;
	}

	for( unsigned i=0; i<m_subdata.size(); i++ ){
		result.push_back( this->m_histlist[i] * h.m_histlist[i] );
	}
	return result;
}

std::vector<double> tgHistDesc2D::Euclidean(const tgHistDesc2D& h, double vis_ratio) const
{
	std::vector<double> result;
	if( this->m_histlist.size() != h.m_histlist.size() )	{
		printf("[tgHistDesc2D::Intersect] Number of descriptor histograms do not match.\n");
		return result;
	}

	for( unsigned i=0; i<m_subdata.size(); i++ ){
		if(this->m_maskratio[i] >= vis_ratio){
			result.push_back( this->m_histlist[i].Euclidean(h.m_histlist[i]) );
		}
	}
	return result;
}

std::vector<double> tgHistDesc2D::Intersect(const tgHistDesc2D& h, double vis_ratio) const
{
	std::vector<double> result;
	if( this->m_histlist.size() != h.m_histlist.size() )	{
		printf("[tgHistDesc2D::Intersect] Number of descriptor histograms do not match.\n");
		return result;
	}

	for( unsigned i=0; i<m_subdata.size(); i++ ){
		if(this->m_maskratio[i] >= vis_ratio){
			result.push_back( this->m_histlist[i].Intersect(h.m_histlist[i]) );
		}else{
		  result.push_back( 0.0 );
		}
	}
	return result;
}

std::vector<double> tgHistDesc2D::Fidelity(const tgHistDesc2D& h) const
{
	std::vector<double> result;
	if( this->m_histlist.size() != h.m_histlist.size() )	{
		printf("[tgHistDesc2D::Fidelity] Number of descriptor histograms do not match.\n");
		return result;
	}

	for( unsigned i=0; i<m_subdata.size(); i++ ){
		result.push_back( this->m_histlist[i].Fidelity(h.m_histlist[i]) );
	}
	return result;
}
