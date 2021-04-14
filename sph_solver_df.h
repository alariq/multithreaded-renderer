#pragma once

// TODO: make struct with function pointers

void SPH_DFTimestepTick(class SPHSimulation* sim, float dt);
struct SPHSimData* SPH_DFCreateSimData();

