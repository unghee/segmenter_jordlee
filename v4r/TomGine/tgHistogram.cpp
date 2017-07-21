
#include "tgHistogram.h"
#include <iostream>

using namespace TomGine;
using namespace std;



tgHistogram::tgHistogram(unsigned size)
{
	m_size = size;
	m_counter.assign(m_size, 0);
	m_pdf.assign(m_size, 0.0);
	m_max=0.0;
	m_sum=0;
}

void tgHistogram::Reset()
{
	m_counter.assign(m_size, 0);
	m_pdf.assign(m_size, 0.0);
	m_sum=0;
	m_max=0.0;
}

double tgHistogram::operator*(const tgHistogram& h) const
{
	if(this->m_size != h.m_size)
		printf("[tgHistogram operator*] Warning size does not match.\n");

	double d=0.0;

	for(unsigned i=0; i<m_size; i++)
	{
		d += this->m_pdf[i] * h.m_pdf[i];
	}

	return d;
}

double tgHistogram::Euclidean(const tgHistogram& h) const
{
	if(this->m_size != h.m_size)
		printf("[tgHistogram operator*] Warning size does not match.\n");

	double d=0.0;

	for(unsigned i=0; i<m_size; i++)
	{
		d += pow(this->m_pdf[i] - h.m_pdf[i],2);
	}

	return sqrt(d);
}

double tgHistogram::Intersect(const tgHistogram& h) const
{
	if(this->m_size != h.m_size)
		printf("[tgHistogram operator*] Warning size does not match.\n");

	double d=0.0;
	for(unsigned i=0; i<m_size; i++)
	{
		d += min(this->m_pdf[i], h.m_pdf[i]);
	}

	return d;
}

double tgHistogram::Fidelity(const tgHistogram& h) const
{
	if(this->m_size != h.m_size)
		printf("[tgHistogram operator*] Warning size does not match.\n");

	double d=0.0;

	for(unsigned i=0; i<m_size; i++)
	{
		d += sqrt(this->m_pdf[i] * h.m_pdf[i]);
	}

	return d;
}

void tgHistogram::Evaluate(const std::vector<double> &data, const double &min, const double &max)
{
	for(unsigned i=0; i<data.size(); i++)
	{
		if(data[i]<min || data[i]>max){
			cout << "[tgHistogram::Evaluate] Warning value " << data[i] << " out of bounds: [" << min << "-" << max << "]." << endl;
			continue;
		}

		double clmp = (data[i]-min) / (max-min);	// clamp to [0..1]
		unsigned bin = (unsigned)round( clmp*(m_counter.size()-1) );

		m_counter[ bin ] ++;
		m_sum++;

	}

	for(unsigned i=0; i<m_counter.size(); i++)
	{
		m_pdf[i] = double(m_counter[i]) / m_sum;	// Normalize
		if(m_pdf[i]>m_max)
			m_max=m_pdf[i];
	}
}

void tgHistogram::Print() const
{
	for(unsigned i=0; i<m_counter.size(); i++)
		printf("%d ", m_counter[i]);
	printf("\n");
}

void tgHistogram2D::EvaluatePolar(	const std::vector<vec2> &data,
										const vec2 &min, const vec2 &max)
{
	vec2 tm_max = min;
	vec2 tm_min = max;
	for(unsigned i=0; i<data.size(); i++)
	{
		vec2 d = data[i];
		if(d.x<min.x || d.x>max.x){
			cout << "[tgHistogram::EvaluatePolar] Warning x-value " << data[i].x << " out of bounds: [" << min.x << "-" << max.x << "]." << endl;
			continue;
		}
		if(d.y<min.y || d.y>max.y){
			cout << "[tgHistogram::EvaluatePolar] Warning y-value " << data[i].y << " out of bounds: [" << min.y << "-" << max.y << "]." << endl;
			continue;
		}

		if(d.x > tm_max.x)
			tm_max.x = d.x;
		if(d.x < tm_min.x)
			tm_min.x = d.x;
		if(d.y > tm_max.y)
			tm_max.y = d.y;
		if(d.y < tm_min.y)
			tm_min.y = d.y;

		vec2 clmp;
		clmp.x = (d.x-min.x) / (max.x-min.x);	// clamp to [0..1]
		clmp.y = (d.y-min.y) / (max.y-min.y);	// clamp to [0..1]

		unsigned bin_x = (unsigned)floor( clmp.x*m_size_x );
		unsigned bin_y = (unsigned)floor( clmp.y*m_size_y );

		if(d.x == max.x)
			bin_x = m_size_x - 1;
		if(d.y == max.y)
			bin_y = m_size_y - 1;

//		printf("%d %d, %f %f\n", bin_x, bin_y, d.x*180.0*M_1_PI, d.y);

		unsigned bin = bin_y*m_size_x + bin_x;

		if(bin >= m_counter.size()){
			printf("[tgHistogram::EvaluatePolar] bin out of bounds: %d %d\n", bin, (int)m_counter.size());
		}

		m_counter[ bin ] ++;
		m_sum++;

	}

//	printf("max: %f %f, min: %f %f\n", tm_max.x*180.0*M_1_PI, tm_max.y, tm_min.x*180.0*M_1_PI, tm_min.y);

	double dsum;
	if(m_sum==0)
		dsum = 1.0/m_counter.size();
	else
		dsum = 1.0/m_sum;

	for(unsigned i=0; i<m_counter.size(); i++)
	{
		m_pdf[i] = double(m_counter[i]) * dsum;	// Normalize

		if(m_pdf[i]>m_max)
			m_max=m_pdf[i];
	}
}

