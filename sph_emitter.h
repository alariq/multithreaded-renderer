#pragma once

#include "utils/vec.h"
#include <vector>

struct SPHEmitter {

	float rate_;
	vec3 size_;
	vec2 initial_vel_;

	struct SPHFluidModel* fluid_model_;

	// transient params
	vec2 pos_;
	vec2 dir_;
	float time_since_last_emit_; // seconds
    bool enabled_;


  private:
    friend class SPHEmitterSystem;
    ~SPHEmitter() = default;
    SPHEmitter() = default;
};

class SPHEmitterSystem {

	std::vector<SPHEmitter *> emitters_;
public:

	void step(float dt);

	static SPHEmitter *createrEmitter();
	static void destroyEmitter(SPHEmitter *);

    void add(SPHEmitter* emitter);
    void remove(SPHEmitter* emitter);
};
