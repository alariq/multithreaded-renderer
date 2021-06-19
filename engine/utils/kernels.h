#pragma once

#include "vec.h"

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
		m_k = 40.0f / (7.0f * (pi * h2));
		m_l = 240.0f / (7.0f * (pi * h2));

		m_W_zero = W(vec3(0));
	}

  public:
	static float W(const float r) {
		float res = 0.0;
		const float q = r / m_radius;
		if (q <= 1.0f) {
			if (q <= 0.5f) {
				const float q2 = q * q;
				const float q3 = q2 * q;
				res = m_k * (6.0f * q3 - 6.0f * q2 + 1.0f);
			} else {
				res = m_k * (2.0f * powf(1.0f - q, 3));
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

class Poly6Kernel {
  protected:
	static float m_radius;
	static float m_k;
	static float m_l;
	static float m_m;
	static float m_W_zero;

  public:
	static float getRadius() { return m_radius; }
	static void setRadius(float val) {
		m_radius = val;
		const float pi = (float)M_PI;
		m_k = 315.0f / (64.0f * pi * powf(m_radius, 9.0f));
		m_l = -945.0f / (32.0f * pi * powf(m_radius, 9.0f));
		m_m = m_l;
		m_W_zero = W(vec3(0.0f));
	}

  public:
	/**
	 * W(r,h) = (315/(64 pi h^9))(h^2-|r|^2)^3
	 *        = (315/(64 pi h^9))(h^2-r*r)^3
	 */
	static float W(const float r) {
		float res = 0.0;
		const float r2 = r * r;
		const float radius2 = m_radius * m_radius;
		if (r2 <= radius2) {
			res = pow(radius2 - r2, 3) * m_k;
		}
		return res;
	}

	static float W(const vec3& r) {
		float res = 0.0;
		const float r2 = lengthSqr(r);
		const float radius2 = m_radius * m_radius;
		if (r2 <= radius2) {
			res = pow(radius2 - r2, 3) * m_k;
		}
		return res;
	}

	/**
	 * grad(W(r,h)) = r(-945/(32 pi h^9))(h^2-|r|^2)^2
	 *              = r(-945/(32 pi h^9))(h^2-r*r)^2
	 */
	static vec3 gradW(const vec3& r) {
		vec3 res;
		const float r2 = lengthSqr(r);
		const float radius2 = m_radius * m_radius;
		if (r2 <= radius2) {
			float tmp = radius2 - r2;
			res = m_l * tmp * tmp * r;
		} else
			res = vec3(0.0f);

		return res;
	}

	/**
	 * laplacian(W(r,h)) = (-945/(32 pi h^9))(h^2-|r|^2)(-7|r|^2+3h^2)
	 *                   = (-945/(32 pi h^9))(h^2-r*r)(3 h^2-7 r*r)
	 */
	static float laplacianW(const vec3& r) {
		float res;
		const float r2 = lengthSqr(r);
		const float radius2 = m_radius * m_radius;
		if (r2 <= radius2) {
			float tmp = radius2 - r2;
			float tmp2 = 3 * radius2 - 7 * r2;
			res = m_m * tmp * tmp2;
		} else
			res = (float)0.0f;

		return res;
	}

	static float W_zero() { return m_W_zero; }
};

class Poly6Kernel2D {
  protected:
	static float m_radius;
	static float m_k;
	static float m_l;
	static float m_m;
	static float m_W_zero;
	static float m_r_sq;

  public:
	static float getRadius() { return m_radius; }
	static void setRadius(float val) {
		m_radius = val;
        m_r_sq = val * val;
		const float pi = (float)M_PI;
		m_k = 4.0f / (pi * powf(m_radius, 8.0f));
		m_l = 12.0f / (pi * powf(m_radius, 8.0f));
		m_m = m_l;
		m_W_zero = W(vec3(0.0f));
	}

  public:
	static float W(const float r) {
		float res = 0.0;
		const float r2 = r * r;
		if (r2 <= m_r_sq) {
			res = pow(m_r_sq - r2, 3) * m_k;
		}
		return res;
	}

	static float W(const vec3& r) {
		float res = 0.0;
		const float r2 = lengthSqr(r);
		if (r2 <= m_r_sq) {
			res = pow(m_r_sq - r2, 3) * m_k;
		}
		return res;
	}

	static vec3 gradW(const vec3& r) {
		vec3 res;
		const float r2 = lengthSqr(r);
		if (r2 <= m_r_sq) {
			float tmp = m_r_sq - r2;
			res = m_l * tmp * tmp * (-2.0f * sqrtf(r2)) * (r2>0?normalize(r):vec3(1,1,1));
		} else
			res = vec3(0.0f);

		return res;
	}

	static float W_zero() { return m_W_zero; }
};

class SpikyKernel {
  protected:
	static float m_radius;
	static float m_k;
	static float m_l;
	static float m_W_zero;

  public:
	static float getRadius() { return m_radius; }
	static void setRadius(float val) {
		m_radius = val;
		const float radius6 = powf(m_radius, 6.0f);
		const float pi = static_cast<float>(M_PI);
		m_k = static_cast<float>(15.0f) / (pi * radius6);
		m_l = -static_cast<float>(45.0f) / (pi * radius6);
		m_W_zero = W(vec3(0));
	}

  public:
	/**
	 * W(r,h) = 15/(pi*h^6) * (h-r)^3
	 */
	static float W(const float r) {
		float res = 0.0;
		const float r2 = r * r;
		const float radius2 = m_radius * m_radius;
		if (r2 <= radius2) {
			const float hr3 = pow(m_radius - r, 3);
			res = m_k * hr3;
		}
		return res;
	}

	static float W(const vec3& r) {
		float res = 0.0;
		const float r2 = lengthSqr(r);
		const float radius2 = m_radius * m_radius;
		if (r2 <= radius2) {
			const float hr3 = powf(m_radius - sqrtf(r2), 3.0f);
			res = m_k * hr3;
		}
		return res;
	}

	/**
	 * grad(W(r,h)) = -r(45/(pi*h^6) * (h-r)^2)
	 */
	static vec3 gradW(const vec3& r) {
		vec3 res;
		const float r2 = lengthSqr(r);
		const float radius2 = m_radius * m_radius;
		if (r2 <= radius2) {
			const float r_l = sqrtf(r2);
			const float hr = m_radius - r_l;
			const float hr2 = hr * hr;
			res = m_l * hr2 * r * (1.0f / r_l);
		} else
			res = vec3(0);

		return res;
	}

	static float W_zero() { return m_W_zero; }
};

class SpikyKernel2D {
  protected:
	static float m_radius;
	static float m_k;
	static float m_l;
	static float m_W_zero;

  public:
	static float getRadius() { return m_radius; }
	static void setRadius(float val) {
		m_radius = val;
		const float radius5 = powf(m_radius, 5.0f);
		const float pi = static_cast<float>(M_PI);
		m_k = static_cast<float>(10.0f) / (pi * radius5);
		m_l = -static_cast<float>(30.0f) / (pi * radius5);
		m_W_zero = W(vec3(0));
	}

  public:
    // so, square of a circle of "h" is: integral 2pi*1*x*dx, x [0, h]
    // in our case we put (h-x)^3 for 1. So interal becomes: pi*h^5 / 10
    // thus we have this normalization component
    // https://www.wolframalpha.com/input/?i=integrate++2*pi*%28%28a+-+x%29%5E3%29+*%28x%29
	/**
	 * W(r,h) = 10/(pi*h^5) * (h-r)^3
	 */
	static float W(const float r) {
		float res = 0.0;
		const float r2 = r * r;
		const float radius2 = m_radius * m_radius;
		if (r2 <= radius2) {
			const float hr3 = powf(m_radius - r, 3);
			res = m_k * hr3;
		}
		return res;
	}

	static float W(const vec3& r) {
		float res = 0.0;
		const float r2 = lengthSqr(r);
		const float radius2 = m_radius * m_radius;
		if (r2 <= radius2) {
			const float hr3 = powf(m_radius - sqrtf(r2), 3.0f);
			res = m_k * hr3;
		}
		return res;
	}

	/**
	 * grad(W(r,h)) = -(30/(pi*h^5) * (h-r)^2)
	 */
	static vec3 gradW(const vec3& r) {
		vec3 res;
		const float r2 = lengthSqr(r);
		const float radius2 = m_radius * m_radius;
		if (r2 <= radius2) {
			const float r_l = sqrtf(r2);
			const float hr = m_radius - r_l;
			res = m_l * hr * hr * r * (1.0f / r_l);
		} else
			res = vec3(0);

		return res;
	}

	static float W_zero() { return m_W_zero; }
};

