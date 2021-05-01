#pragma once

const struct SPHSolverInterface* Get_SPH_PBD_SolverInterface();
void SPH_PBDDebugDrawSimData(const struct SPHFluidModel* fm, class RenderList* rl);
