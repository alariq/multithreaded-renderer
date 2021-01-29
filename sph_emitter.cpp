#include "sph_emitter.h"
#include "sph.h"

#include <vector>
#include "utils/vec.h"

void SPHEmitterSystem::step(float dt, float cur_time) {
	for (SPHEmitter* e : emitters_) {
		float num_to_emit = std::floorf((e->time_since_last_emit_ + dt) / e->rate_);
		e->time_since_last_emit_ -= num_to_emit * e->rate_;
		assert(e->time_since_last_emit_ >= 0.0f);

		int idx = e->fluid_model_->add((int)num_to_emit);
		for (int i = 0, count = (int)num_to_emit; i < count; ++i) {
			e->fluid_model_->particles_[idx].density = 0;
			e->fluid_model_->particles_[idx].pos = e->pos_;
			e->fluid_model_->particles_[idx].vel = e->initial_vel_;
			e->fluid_model_->particles_[idx].force = vec2(0,0);
			e->fluid_model_->particles_[idx].pressure = 0;
			e->fluid_model_->particles_[idx].flags = kSPHFlagActive;
		}
	}
}


