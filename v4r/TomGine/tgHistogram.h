 /**
 * @file tgHistogram
 * @author Thomas Mörwald
 * @date August 2011
 * @version 0.1
 * @brief Evaluate and draws histogram of data.
 */
 
#ifndef _TG_HISTOGRAM_H_
#define _TG_HISTOGRAM_H_

#include <vector>
#include <math.h>
#include <stdio.h>
#include <GL/gl.h>

#include "tgMathlib.h"

namespace TomGine{

/** @brief Evaluate histogram of data. */
class tgHistogram
{
protected:
	std::vector<unsigned> m_counter;
	std::vector<double> m_pdf;
	double m_max;
	unsigned m_sum;
	unsigned m_size;

public:
	/** @brief Creates the Histogram
	 *  @param size	number of bins used. */
	tgHistogram(unsigned size);

	/** @brief Sets the counter of the histogram bins to 0. */
	void Reset();

	/** @brief Inner product of two histograms. */
	double operator*(const tgHistogram& h) const;

	/** @brief Euclidean distance of two histograms. */
	double Euclidean(const tgHistogram& h) const;

	/** @brief Intersection of two histograms. */
	double Intersect(const tgHistogram& h) const;

	/** @brief Fidelity similarity of two histograms. */
	double Fidelity(const tgHistogram& h) const;

	/** @brief Evaluates histogram of data given by clamping to the range [0,1],
	 * 		multiplying by the size of the histogram, and rounding to the nearest integer.
	 * 	@param data	The data to calculate the histogram from
	 * 	@param min	The minimum of the histogram range
	 * 	@param max	The maximum of the histogram range	 */
	void Evaluate(const std::vector<double> &data, const double &min, const double &max);

	/** @brief Returns the histogram counters. */
	std::vector<unsigned> GetCounter() const{ return m_counter; }

	/** @brief Returns the histogram normalized to its sum. */
	std::vector<double> GetNormalized() const{ return m_pdf; }

	/** @brief Get maximum number in histogram. */
	double GetMax() const{ return m_max; }

	/** @brief Get sum of histogram. */
	unsigned GetSum() const{ return m_sum; }

	/** @brief Returns the number of bins of the histogram. */
	unsigned Size() const{ return m_size; }

	/** @brief Prints the histogram counter to console. */
	void Print() const;

};

/** @brief Evaluate histogram with data given in polar coordinates (angle, magnitude). */
class tgHistogram2D : public tgHistogram
{
protected:
	unsigned m_size_x, m_size_y;
public:
	/** @brief Creates the Histogram
	 *  @param size	number of bins used. */
	tgHistogram2D(unsigned size_x, unsigned size_y) : tgHistogram(size_x * size_y)
	{
		m_size_x = size_x;
		m_size_y = size_y;
	}

	/** @brief Evaluates histogram of 2d-vectors, where a vector is given in polar coordinates,
	 * 			i.e. by its angle (x-component) and magnitude (y-component).
	 * 	@param data	The data to calculate the histogram from
	 * 	@param min	The minimum of the histogram range
	 * 	@param max	The maximum of the histogram range	 */
	void EvaluatePolar(const std::vector<vec2> &data, const vec2 &min, const vec2 &max);

};

}

#endif
