#include "sph_emitter.h"
#include "sph.h"
#include "sph_boundary.h"
#include "utils/vec.h"

#include <vector>
#include <cassert>

SPHEmitter *SPHEmitterSystem::createrEmitter()
{
    SPHEmitter* e = new SPHEmitter();
    e->time_since_last_emit_ = 0.0f;
    e->fluid_model_ = nullptr;
    sph_get_emitter_system()->add(e);
    return e;
}
       
void SPHEmitterSystem::destroyEmitter(SPHEmitter* e)
{
    sph_get_emitter_system()->remove(e);
    delete e;
}

void SPHEmitterSystem::add(SPHEmitter* emitter) {
    emitters_.push_back(emitter);
}

void SPHEmitterSystem::remove(SPHEmitter* emitter) {
    auto b = std::begin(emitters_);
    auto e = std::end(emitters_);
    auto it = std::remove(b, e, emitter);
    assert(it!=emitters_.end());
    emitters_.erase(it, e);
}

void SPHEmitterSystem::step(float dt) {
	for (SPHEmitter* e : emitters_) {
		float num_to_emit = std::floor((e->time_since_last_emit_ + dt) / e->rate_);
		e->time_since_last_emit_ += dt - num_to_emit * e->rate_;
		assert(e->time_since_last_emit_ >= 0.0f);

		int idx = e->fluid_model_->add((int)num_to_emit);
		for (int i = 0, count = (int)num_to_emit; i < count; ++i) {
			e->fluid_model_->particles_[idx + i].pos = e->pos_;
			e->fluid_model_->particles_[idx + i].vel = e->initial_vel_;

			e->fluid_model_->particles_[idx + i].density = 0;
			e->fluid_model_->particles_[idx + i].force = vec2(0,0);
			e->fluid_model_->particles_[idx + i].pressure = 0;

			e->fluid_model_->particles_[idx + i].flags = kSPHFlagPending;
		}
	}
}



