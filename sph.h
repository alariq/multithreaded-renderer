#pragma once

#include "utils/vec.h"

struct SPHParticle2D {
	vec2 pos;
	vec2 vel;
	vec2 force;
	float density;
	float pressure;
};

void sph_update(SPHParticle2D *particles, int count, vec2 view_dim);

