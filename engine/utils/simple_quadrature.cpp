#include "simple_quadrature.h"

#include <algorithm>
#include <numeric>

std::vector<vec3> simple_quadrature::sample_points_;
float simple_quadrature::volume_ = 0.0;


float simple_quadrature::integrate(simple_quadrature::Integrand_t integrand)
{
	float res = 0.0;
 	for (unsigned int i = 0; i < sample_points_.size(); i++)
	{
		res += volume_ * integrand(sample_points_[i]);
	}
    return res;
}

void simple_quadrature::determineSamplePointsInSphere(const float radius, unsigned int p)
{
	if (p < 1)
		p = 1;

	sample_points_.clear();
	sample_points_.reserve(p*p*p);
	const float radius2 = radius*radius;
	const float step_size = 2.0f * radius / (float)p;
	const float start = -radius + 0.5f*step_size;
	volume_ = step_size*step_size*step_size;

	vec3 pos;
	pos.x = start;
	for (unsigned int i = 0; i < p; i++)
	{
		pos.y = start;
		for (unsigned int j = 0; j < p; j++)
		{
			pos.z = start;
			for (unsigned int k = 0; k < p; k++)
			{
				// test if sample point is in support radius and if it is not the origin
				const float pn = lengthSqr(pos);
				if (pn < radius2)
				{
					sample_points_.push_back(pos);
				}
				pos.z += step_size;
			}
			pos.y += step_size;
		}
		pos.x += step_size;
	}
}

void simple_quadrature::determineSamplePointsInCircle(const float radius, unsigned int p)
{
	if (p < 1)
		p = 1;

	sample_points_.clear();
	sample_points_.reserve(p*p);
	const float radius2 = radius*radius;
	const float step_size = 2.0f * radius / (float)p;
	const float start = -radius + 0.5f*step_size;
	volume_ = step_size*step_size;

	vec3 pos;
	pos.x = start;
	for (unsigned int i = 0; i < p; i++)
	{
		pos.y = start;
		for (unsigned int j = 0; j < p; j++)
		{
			pos.z = 0.0f;

			// test if sample point is in support radius and if it is not the origin
			const float pn = lengthSqr(pos);
			if (pn < radius2) {
				sample_points_.push_back(pos);
			}
			pos.y += step_size;
		}
		pos.x += step_size;
	}
}
