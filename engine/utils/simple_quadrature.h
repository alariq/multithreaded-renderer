#pragma once

#include "vec.h"
#include <vector>
#include <functional>

class simple_quadrature {
  public:
	using Integrand_t = std::function<float(vec3 const &)>;
	static std::vector<vec3> sample_points_;
	static float volume_;

	static void determineSamplePointsInSphere(const float radius, unsigned int p);
	static void determineSamplePointsInCircle(const float radius, unsigned int p);
	static float integrate(Integrand_t integrand);
};
