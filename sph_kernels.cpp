#include "sph_kernels.h"

float CubicKernel::m_radius = 0.0f;
float CubicKernel::m_k = 0.0f;
float CubicKernel::m_l = 0.0f;
float CubicKernel::m_W_zero = 0.0f;

float CubicKernel2D::m_radius = 0.0f;
float CubicKernel2D::m_k = 0.0f;
float CubicKernel2D::m_l = 0.0f;
float CubicKernel2D::m_W_zero = 0.0f;

float Poly6Kernel::m_radius = 0.0f;
float Poly6Kernel::m_k = 0.0f;
float Poly6Kernel::m_l = 0.0f;
float Poly6Kernel::m_W_zero = 0.0f;
float Poly6Kernel::m_m = 0.0f;

float Poly6Kernel2D::m_radius = 0.0f;
float Poly6Kernel2D::m_r_sq = 0.0f;
float Poly6Kernel2D::m_k = 0.0f;
float Poly6Kernel2D::m_l = 0.0f;
float Poly6Kernel2D::m_W_zero = 0.0f;
float Poly6Kernel2D::m_m = 0.0f;
