#pragma once

#include "utils/vec.h"

class CubicKernel {
  protected:
	static float m_radius;
	static float m_k;
	static float m_l;
	static float m_W_zero;

  public:
	static float getRadius() { return m_radius; }
	static void setRadius(float val) {
		m_radius = val;
		const float pi = static_cast<float>(M_PI);

		const float h3 = m_radius * m_radius * m_radius;
		m_k = 8.0f / (pi * h3);
		m_l = 48.0f / (pi * h3);
		m_W_zero = W(vec3(0));
	}

  public:
	static float W(const float r) {
		float res = 0.0;
		const float q = r / m_radius;
		if (q <= 1.0) {
			if (q <= 0.5) {
				const float q2 = q * q;
				const float q3 = q2 * q;
				res = m_k * (6.0f * q3 - 6.0f * q2 + 1.0f);
			} else {
				res = m_k * (2.0f * powf(1.0f - q, 3.0f));
			}
		}
		return res;
	}

	static float W(const vec3 &r) { return W(length(r)); }

	static vec3 gradW(const vec3 &r) {
		vec3 res;
		const float rl = length(r);
		const float q = rl / m_radius;
		if ((rl > 1.0e-5) && (q <= 1.0)) {
			const vec3 gradq = r * (static_cast<float>(1.0) / (rl * m_radius));
			if (q <= 0.5) {
				res = m_l * q * (3.0f * q - 2.0f) * gradq;
			} else {
				const float factor = 1.0f - q;
				res = m_l * (-factor * factor) * gradq;
			}
		} else
			res = vec3(0);

		return res;
	}

	static float W_zero() { return m_W_zero; }
};

class CubicKernel2D {
  protected:
	static float m_radius;
	static float m_k;
	static float m_l;
	static float m_W_zero;

  public:
	static float getRadius() { return m_radius; }
	static void setRadius(float val) {
		m_radius = val;
		const float pi = static_cast<float>(M_PI);

		const float h2 = m_radius * m_radius;
		m_k = 40.0 / (7.0 * (pi * h2));
		m_l = 240.0 / (7.0 * (pi * h2));

		m_W_zero = W(vec3(0));
	}

  public:
	static float W(const float r) {
		float res = 0.0;
		const float q = r / m_radius;
		if (q <= 1.0) {
			if (q <= 0.5) {
				const float q2 = q * q;
				const float q3 = q2 * q;
				res = m_k * (static_cast<float>(6.0) * q3 - static_cast<float>(6.0) * q2 +
							 static_cast<float>(1.0));
			} else {
				res =
					m_k * (static_cast<float>(2.0) * pow(static_cast<float>(1.0) - q, 3));
			}
		}
		return res;
	}

	static float W(const vec3 &r) { return W(length(r)); }

	static vec3 gradW(const vec3 &r) {
		vec3 res(0);
		const float rl = length(r);
		const float q = rl / m_radius;
		if (q <= 1.0) {
			if (rl > 1.0e-5) {
				const vec3 gradq = r * (1.0f / (rl * m_radius));
				if (q <= 0.5) {
					res = m_l * q * (3.0f * q - 2.0f) * gradq;
				} else {
					const float factor = 1.0f - q;
					res = m_l * (-factor * factor) * gradq;
				}
			}
		} else
			res = vec3(0);

		return res;
	}

	static float W_zero() { return m_W_zero; }
};
