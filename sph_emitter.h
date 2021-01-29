#pragma once
#include <vector>
#include "utils/vec.h"

struct SPHEmitter {

	float rate_;
	vec3 size_;
	vec2 initial_vel_;

	struct SPHFluidModel* fluid_model_;
	
	// transient params
	vec2 pos_;
	vec2 dir_;
	float time_since_last_emit_; // seconds

};

class SPHEmitterSystem {

	std::vector<SPHEmitter*> emitters_;


	void step(float dt, float cur_time);
};
