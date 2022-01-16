#include "pbd_test.h"
#include "pbd/pbd_particles.h"
#include "pbd/pbd_particles_collision.h"
#include "res_man.h"
#include "utils/vec.h"
#include "utils/camera_utils.h" // screen2world_vec
#include "utils/kernels.h"
#include "utils/math_utils.h"

#include "engine/profiler/profiler.h"

template<int N>
struct RandArray {
	float random_floats[N];
	unsigned int i;
    
    RandArray(float mmin, float mmax):i(0) {
        for(int ii=0;ii<N;ii++) {
            random_floats[ii] = random(mmin, mmax);
        }
    }

    float operator[](int idx) const { return random_floats[idx%N]; }
    
    void reset() { i = 0; }
	float get_next() { return random_floats[i++%N]; }
	vec3 get_next_vec() {
		vec3 r;
		r.x = get_next();
		r.y = get_next();
		r.z = get_next();
		return r;
	}
};

static RandArray<1024> r_rotations(0, 1);
static RandArray<1024> r_jitters(-0.01f, 0.01f);
static RandArray<1024> r_offsets(-0.01f, 0.01f);

// TODO: move to input utils?
static vec3 get_ws_mouse_pos(RenderFrameContext* rfc);

void initialize_particle_positions(struct PBDUnifiedSimulation* sim,
								   const ivec2& row_column, const vec2 offset,
								   float density0);

void initialize_fluid_particle_positions(struct PBDUnifiedSimulation* sim,
										 const vec2& dim, const vec2& offset,
										 int row_size, int count, int fluid_model_idx);

void initialize_particle_positions2(struct PBDUnifiedSimulation* sim, const vec2& offset,
									int count, float density0, float mu_s, float mu_k,
									float e);

void pbd_unified_sim_debug_draw_world(const struct PBDUnifiedSimulation* sim,
									  struct RenderFrameContext* rfc, PBDTestObject::DDFlags flags);

void collision_debug_draw(const struct CollisionWorld* cworld, RenderList* rl);

void scene_soft_body(PBDUnifiedSimulation* sim) {

	const float r = pbd_unified_sim_get_particle_radius(sim);
	vec2 world_size = pbd_unified_sim_get_world_bounds(sim);
	vec2 ppos = vec2(0.45f * world_size.x, 0.2f*world_size.y);
    int w = 1;
    float stiffness = 0.125f;
	pbd_unified_sim_add_box_soft_body(sim, 8, 2, ppos, 0.0f, 1000, w, stiffness);
    vec2 off(0,10*r);
	pbd_unified_sim_add_box_rigid_body(sim, 2, 2, ppos + off + 1*vec2(r,0), 0.0f, 1000);
	pbd_unified_sim_add_box_rigid_body(sim, 2, 2, ppos + off*2, 0.0f, 1000);

	CollisionWorld* cworld = collision_create_world();
	pbd_unified_sim_set_collision_world(sim, cworld);

    {
    SDFBoxCollision box;
    box.pos = vec2(world_size.x * 0.27f, r * 3.0f);
    box.rot = identity2();
    box.size = vec2(3.0f * r, 3.0f * r);
    collision_add_box(cworld, box);
    }

    SDFBoxCollision box;
    box.pos = vec2(world_size.x * 0.63f, r * 3.0f);
    box.rot = identity2();
    box.size = vec2(3.0f * r, 3.0f * r);
    collision_add_box(cworld, box);
}

void scene_soft_body_from_lattice(PBDUnifiedSimulation* sim) {

	const float r = pbd_unified_sim_get_particle_radius(sim);
	vec2 world_size = pbd_unified_sim_get_world_bounds(sim);
	vec2 ppos = vec2(0.45f * world_size.x, 0.2f*world_size.y);
    int w = 1;
    float stiffness = 0.125f;
#if 0
    const constexpr uint32_t sx = 7;
    const constexpr uint32_t sy = 4;
	uint8_t bp[sy][sx] = {
		{1, 1, 1, 1, 1, 1, 1},
		{1, 0, 0, 1, 0, 0, 1},
		{1, 0, 0, 1, 0, 0, 1},
		{1, 1, 1, 1, 1, 1, 1},
	};
#else
    const constexpr uint32_t sx = 6;
    const constexpr uint32_t sy = 5;
	uint8_t bp[sy][sx] = {
		{0, 0, 1, 1, 0, 0},
		{0, 1, 0, 0, 1, 0},
		{1, 0, 0, 0, 0, 1},
		{0, 1, 0, 0, 1, 0},
		{0, 0, 1, 1, 0, 0},
	};
#endif

	pbd_unified_sim_add_soft_body(sim, sx, sy, &bp[0][0], ppos, 1000, w, stiffness);

    vec2 off(0,10*r);
	pbd_unified_sim_add_box_rigid_body(sim, 2, 2, ppos + off + 1*vec2(r,0), 0.0f, 100);
}


void scene_initial_penetration(PBDUnifiedSimulation* sim) {
	vec2 world_size = pbd_unified_sim_get_world_bounds(sim);
	vec2 ppos = vec2(0.55f * world_size.x, 0.0f);

	int idx = pbd_unified_sim_add_particle(sim, ppos, 1000);
	const float e = 0.75f;
	pbd_unified_sim_particle_set_params(sim, idx, 0.0f, 0.0f, e);
}

void scene_restitution_test(PBDUnifiedSimulation* sim) {
	const float r = pbd_unified_sim_get_particle_radius(sim);
	vec2 world_size = pbd_unified_sim_get_world_bounds(sim);

	const int num_particles = 10;
	float e = 1.0f;
	vec2 dp = vec2(4.0f * r, 0.0f);
	vec2 pos = vec2(3.0f * r, world_size.y * 0.5f);
	for (int i = 0; i < num_particles; ++i) {
		int idx = pbd_unified_sim_add_particle(sim, pos, 1000);
		pbd_unified_sim_particle_set_params(sim, idx, 0.0f, 0.0f, e);
		pos += dp;
		e -= 1.0f / num_particles;
	}
}

void scene_static_particle_friction_test(PBDUnifiedSimulation* sim) {
	const float r = pbd_unified_sim_get_particle_radius(sim);
	vec2 world_size = pbd_unified_sim_get_world_bounds(sim);
	CollisionWorld* cworld = collision_create_world();
	pbd_unified_sim_set_collision_world(sim, cworld);

	vec2 ppos = vec2(0.35f * world_size.x, 0.3f * world_size.y);
	int idx0 = pbd_unified_sim_add_particle(sim, ppos, 1000);
	int idx1 = pbd_unified_sim_add_particle(sim, ppos + vec2(30*r,0), 1000);
	pbd_unified_sim_particle_set_params(sim, idx0, 0.3f, 0.09f, 0.0);
	pbd_unified_sim_particle_set_params(sim, idx1, 0.7f, 0.09f, 0.0);

	{
		SDFBoxCollision box;
		box.pos = vec2(world_size.x * 0.3f, r * 3);
		box.rot = rotate2(5.0f * M_PI / 180.0f);
		box.size = vec2(15 * r, 3 * r);
		collision_add_box(cworld, box);
	}

	if (1) {
		SDFBoxCollision box;
		box.pos = vec2(world_size.x * 0.3f + 30.0f*r, r * 3);
		box.rot = rotate2(5.0f * M_PI / 180.0f);
		box.size = vec2(15 * r, 3 * r);
		collision_add_box(cworld, box);
	}
}


void scene_dynamic_particle_friction_test(PBDUnifiedSimulation* sim) {
	const float r = pbd_unified_sim_get_particle_radius(sim);
	vec2 world_size = pbd_unified_sim_get_world_bounds(sim);
	CollisionWorld* cworld = collision_create_world();
	pbd_unified_sim_set_collision_world(sim, cworld);

	vec2 ppos = vec2(0.35f * world_size.x, 0.3f * world_size.y);
	int idx0 = pbd_unified_sim_add_particle(sim, ppos, 1000);
	int idx1 = pbd_unified_sim_add_particle(sim, ppos + vec2(30*r,0), 1000);
    // first particle should reach end first because of less dynamic friction
	pbd_unified_sim_particle_set_params(sim, idx0, 0.1f, 0.0f, 0.0);
	pbd_unified_sim_particle_set_params(sim, idx1, 0.1f, 0.09f, 0.0);

	{
		SDFBoxCollision box;
		box.pos = vec2(world_size.x * 0.3f, r * 3);
		box.rot = rotate2(5.0f * M_PI / 180.0f);
		box.size = vec2(15 * r, 3 * r);
		collision_add_box(cworld, box);
	}

	if (1) {
		SDFBoxCollision box;
		box.pos = vec2(world_size.x * 0.3f + 30.0f*r, r * 3);
		box.rot = rotate2(5.0f * M_PI / 180.0f);
		box.size = vec2(15 * r, 3 * r);
		collision_add_box(cworld, box);
	}
}

// !NB: for exact effect need to remove sleeping functionality
void scene_restitution_chain_of_bodies(PBDUnifiedSimulation* sim) {
	const float r = pbd_unified_sim_get_particle_radius(sim);
	vec2 world_size = pbd_unified_sim_get_world_bounds(sim);

	// striker
	vec2 pos0 = vec2(world_size.x * 0.2f, 1.0f * r);
	// still particles
	vec2 pos1 = vec2(world_size.x * 0.5f, 1.0f * r);
	vec2 pos2 = vec2(world_size.x * 0.5f + 2.0f * r, 1.0f * r);
	vec2 pos3 = vec2(world_size.x * 0.5f + 4.0f * r, 1.0f * r);

	int idx0 = pbd_unified_sim_add_particle(sim, pos0, 1000);
	int idx1 = pbd_unified_sim_add_particle(sim, pos1, 1000);
	int idx2 = pbd_unified_sim_add_particle(sim, pos2, 1000);
	int idx3 = pbd_unified_sim_add_particle(sim, pos3, 1000);

	const float e = 1.0f; // try 1.0f and 0.0f
	pbd_unified_sim_particle_set_params(sim, idx0, 0.0f, 0.0f, e);
	pbd_unified_sim_particle_set_params(sim, idx1, 0.0f, 0.0f, e);
	pbd_unified_sim_particle_set_params(sim, idx2, 0.0f, 0.0f, e);
	pbd_unified_sim_particle_set_params(sim, idx3, 0.0f, 0.0f, e);

	// shoot
	pbd_unified_sim_particle_add_velocity(sim, idx0, vec2(3.0f, 0.0f));
}
#if 1
void scene_rigid_body_restitution_test(PBDUnifiedSimulation* sim) {
    const float r = pbd_unified_sim_get_particle_radius(sim);
    vec2 world_size = pbd_unified_sim_get_world_bounds(sim);

    const int num_rb = 4;
    float e = 1.0f;
    vec2 dp = vec2(5.0f*r, 0.0f);
	vec2 pos = vec2(13.0f * r, world_size.y * 0.5f);
	for (int i = 0; i < num_rb; ++i) {
		/*int idx = */pbd_unified_sim_add_box_rigid_body(sim, 3, 2, pos, 0, 1000);
		//pbd_unified_sim_particle_set_params(sim, idx, 0.0f, 0.0f, e);
        pos += dp;
		e -= 1.0f / num_rb;
	}
}
#endif

void scene_stacking_particles(PBDUnifiedSimulation* sim) {

    const float radius = pbd_unified_sim_get_particle_radius(sim);
	vec2 offset = vec2(1.2f, 1.0f * radius+ 1);
    initialize_particle_positions(sim, ivec2(10, 1), offset, 1000);
}

void scene_stacking_particles_and_box_above(PBDUnifiedSimulation* sim) {

    const float radius = pbd_unified_sim_get_particle_radius(sim);

	pbd_unified_sim_add_box_rigid_body(sim, 5, 4, vec2(1.0, 4), 0, 1000);

	vec2 offset = vec2(1.2f, 1.0f * radius+ 1);
	initialize_particle_positions(sim, ivec2(10, 1), offset, 1000);
}

void scene_friction_test(PBDUnifiedSimulation* sim) {

    const float density0 = 1000;
    const float radius = pbd_unified_sim_get_particle_radius(sim);
	vec2 offset = vec2(1.2f, 1.0f * radius);
    const float column_size = 4;
    const float row_size = 10;

	pbd_unified_sim_add_particle(sim, vec2(0.1f, 1.5f), vec2(7,3.0f), density0);
    
	for (int y = 0; y < row_size; ++y) {
		for (int x = 0; x < column_size; ++x) {

			float jitter = 0;
			vec2 pos = offset + vec2(x * 2.0f * radius + jitter, y * 2.0f * radius);

			int pidx = pbd_unified_sim_add_particle(sim, pos, density0 - y*100);
            pbd_unified_sim_particle_set_params(sim, pidx, .4f, 0.1f, 0.0f);
		}
	}
}

void scene_single_rb_friction_test(PBDUnifiedSimulation* sim) {

    const float density0 = 1000;
    // TODO: add functin to set material params (as for particles)
    int rb2_idx = pbd_unified_sim_add_box_rigid_body(sim, 7, 3, vec2(2,0.3f), 0*30.0f* 3.1415f/180.0f, density0);
    pbd_unified_sim_rb_add_velocity(sim, rb2_idx, vec2(10, 0));
}

void scene_rb_friction_test(PBDUnifiedSimulation* sim) {

    const float density0 = 1000;

	pbd_unified_sim_add_particle(sim, vec2(0.1f, 1.5f), vec2(7,3.0f), density0);

    //vec2 rb_pos = vec2(0.2f, 3.2f);
    //int rb_idx = pbd_unified_sim_add_box_rigid_body(sim, 5, 4, rb_pos, 1*30.0f* 3.1415f/180.0f, density0);
    //pbd_unified_sim_rb_add_velocity(sim, rb_idx, vec2(3, 0));
    
    int rb2_idx = pbd_unified_sim_add_box_rigid_body(sim, 7, 3, vec2(2,0.3f), 0*30.0f* 3.1415f/180.0f, density0);
    pbd_unified_sim_rb_add_velocity(sim, rb2_idx, vec2(10, 0));

    int rb3_idx = pbd_unified_sim_add_box_rigid_body(sim, 7, 3, vec2(2,2.0f), 0*30.0f* 3.1415f/180.0f, density0);
    pbd_unified_sim_rb_add_velocity(sim, rb3_idx, vec2(5, 0));


    pbd_unified_sim_add_box_rigid_body(sim, 7, 3, vec2(0.6f, 0.7f), -65.0f* 3.1415f/180.0f, density0);
}

void scene_rb_friction_test2(PBDUnifiedSimulation* sim) {

    const float density0 = 1000;
    const float radius = pbd_unified_sim_get_particle_radius(sim);

	//pbd_unified_sim_add_particle(sim, vec2(0.1f, 1.5f), vec2(7,3.0f), density0);

    int rb_dim_x = 4;
    int rb_dim_y = 4;
    vec2 rb_size = 2 * vec2(rb_dim_x*radius, rb_dim_y*radius);

    constexpr const int column_size = 4;
    constexpr const int row_size = 4;

	vec2 offset = vec2(0.0f, 0.0f) + 1.0f * vec2(rb_size.x, rb_size.y);
	for (int y = 0; y < row_size; ++y) {
		for (int x = 0; x < column_size; ++x) {

			float jitter = r_jitters[y * column_size + x];
			float rot = r_rotations[y * column_size + x];
			vec2 pos = offset + vec2(x * 1.5f * rb_size.x + jitter, y * 1.5f * rb_size.y);

			pbd_unified_sim_add_box_rigid_body(sim, rb_dim_x, rb_dim_y, pos, rot * 2 * 3.1415f, density0);
		}
	}
}

void scene_particle_box_collision_test(PBDUnifiedSimulation* sim) {
    const float density0 = 1000;
    pbd_unified_sim_add_box_rigid_body(sim, 5, 4, vec2(4.0, 0.4f), 0, density0);

	pbd_unified_sim_add_particle(sim, vec2(0.1f, 0.5f), vec2(7,3.0f), density0);
}

void scene_complex(PBDUnifiedSimulation* sim) {
	const float radius = pbd_unified_sim_get_particle_radius(sim);
	vec2 offset = vec2(1.2f, 1.0f * radius+ 1);
    initialize_particle_positions(sim, ivec2(10, 0), offset, 1000);

    vec2 rb_pos = vec2(0.2f, 3.2f);
    pbd_unified_sim_add_box_rigid_body(sim, 3, 1, rb_pos, 0*45.0f* 3.1415f/180.0f, 1000);
    pbd_unified_sim_add_box_rigid_body(sim, 3, 1, rb_pos - vec2(0,-0.6f), 0*-45.0f* 3.1415f/180.0f, 1000);
    pbd_unified_sim_add_box_rigid_body(sim, 3, 1, rb_pos - vec2(0,-2*0.6f),0* 45.0f* 3.1415f/180.0f, 1000);
    pbd_unified_sim_add_box_rigid_body(sim, 3, 1, rb_pos - vec2(0,-3*0.6f),0* -45.0f* 3.1415f/180.0f, 1000);
    pbd_unified_sim_add_box_rigid_body(sim, 3, 1, rb_pos - vec2(0,-4*0.6f),0* 45.0f* 3.1415f/180.0f, 1000);

    for(int i=0; i< 2; i++) {
        vec2 pos = vec2(1.6f, 4.5f - i*1.4f);
        float r = 0;//(i%2) ? -15.0f* 3.1415f/180.0f : 15.0f* 3.1415f/180.0f;
        pbd_unified_sim_add_box_rigid_body(sim, 4 + i, 4, pos, r, 1000);
    }
}

void scene_fluid_simple_expose_bug(PBDUnifiedSimulation* sim) {

    const vec2 world_size = pbd_unified_sim_get_world_bounds(sim);
	const float radius = pbd_unified_sim_get_particle_radius(sim);

    float desired_density0 = 100;

    int fm_idx = pbd_unified_sim_add_fluid_model(sim, 0, desired_density0);
	vec2 offset = vec2(0.5f, 1.0f * radius);
	int row_size = 16;
    initialize_fluid_particle_positions(sim, world_size, offset, row_size, 200, fm_idx);

    vec2 rb_pos = vec2(world_size.x*0.500f, world_size.y - 12*radius-1);
    pbd_unified_sim_add_box_rigid_body(sim, 3, 3, rb_pos, 0*45.0f* 3.1415f/180.0f, 50);

    vec2 rb_pos2 = vec2(world_size.x*0.5f + 0*(3*2*radius + 0.5f), world_size.y - 3*radius-1);
    pbd_unified_sim_add_box_rigid_body(sim, 3, 3, rb_pos2, 0*45.0f* 3.1415f/180.0f, 250);
    
    vec2 rb_pos3 = vec2(world_size.x*0.1f + 3*2*radius + 0.5f, world_size.y - 3*radius-1);
    pbd_unified_sim_add_box_rigid_body(sim, 1, 1, rb_pos3, 0*45.0f* 3.1415f/180.0f, 30);
}


int g_rb = 0;

void scene_fluid_simple(PBDUnifiedSimulation* sim) {

    const vec2 world_size = pbd_unified_sim_get_world_bounds(sim);
	const float radius = pbd_unified_sim_get_particle_radius(sim);

    float desired_density0 = 100;//m/(radius*radius*2*2);

    int fm_idx = pbd_unified_sim_add_fluid_model(sim, 0, desired_density0);
	vec2 offset = vec2(0.5f, 1.0f * radius);
	int row_size = 16;
    int num_fluid_particles = 200;
    initialize_fluid_particle_positions(sim, world_size, offset, row_size, num_fluid_particles, fm_idx);
#if 1
    vec2 rb_pos = vec2(world_size.x*0.550f, world_size.y - 13*radius-1);
    g_rb = pbd_unified_sim_add_box_rigid_body(sim, 3, 3, rb_pos, 0*45.0f* 3.1415f/180.0f, 50);

    //vec2 rb_pos2 = vec2(world_size.x*0.5f + 0*(3*2*radius + 0.5f), world_size.y - 3*radius-1);
    //pbd_unified_sim_add_box_rigid_body(sim, 3, 3, rb_pos2, 0*45.0f* 3.1415f/180.0f, 250);
    
    //vec2 rb_pos3 = vec2(world_size.x*0.1f + 3*2*radius + 0.5f, world_size.y - 3*radius-1);
    vec2 rb_pos3 = vec2(world_size.x*0.7f + 3*2*radius + 0.5f, 23*radius-1);
    pbd_unified_sim_add_box_rigid_body(sim, 3, 2, rb_pos3, 0*45.0f* 3.1415f/180.0f, 130);

    vec2 rb_pos4 = vec2(world_size.x*0.7f + 3*2*radius + 0.5f, world_size.y - 3*radius-1);
    pbd_unified_sim_add_box_rigid_body(sim, 1, 1, rb_pos4, 0*45.0f* 3.1415f/180.0f, 80);

#endif
#if 0

    for(int i=0; i< 2; i++) {
        vec2 pos = vec2(1.6f, 4.5f - i*1.4f);
        float r = 0;//(i%2) ? -15.0f* 3.1415f/180.0f : 15.0f* 3.1415f/180.0f;
        if(i==0)
            pbd_unified_sim_add_box_rigid_body(sim, 4 + i, 4, pos, r, 1000);
        else
            pbd_unified_sim_add_box_rigid_body(sim, 4 + i, 4, pos, r, 1000);
    }
#endif
}

void scene_fluid_and_solids(PBDUnifiedSimulation* sim) {

    const vec2 world_size = pbd_unified_sim_get_world_bounds(sim);
	const float radius = pbd_unified_sim_get_particle_radius(sim);

    float desired_density0 = 100;//m/(radius*radius*2*2);

    int fm_idx = pbd_unified_sim_add_fluid_model(sim, 0, desired_density0);
	vec2 offset = vec2(0.5f, 1.0f * radius);
	int row_size = 16;
    int num_fluid_particles = 200;
    initialize_fluid_particle_positions(sim, world_size, offset, row_size, num_fluid_particles, fm_idx);

    vec2 rb_pos = vec2(world_size.x*0.550f, world_size.y - 13*radius-1);
    pbd_unified_sim_add_particle(sim,rb_pos, desired_density0/2.0f);
    pbd_unified_sim_add_particle(sim, rb_pos + vec2(5*radius, 0.0f), desired_density0);
    pbd_unified_sim_add_particle(sim, rb_pos + vec2(10*radius, 0.0f), desired_density0*2.0f);
}


void scene_two_fluids(PBDUnifiedSimulation* sim) {

    const vec2 world_size = pbd_unified_sim_get_world_bounds(sim);
	const float radius = pbd_unified_sim_get_particle_radius(sim);

    float desired_density0_heavy = 200;//m/(radius*radius*2*2);
    float desired_density0_light = 50;//m/(radius*radius*2*2);

	//vec2 offset_h = vec2(0.5f, 1.0f * radius);
	//vec2 offset_l = vec2(1.5f, 12.0f * radius);
    
	vec2 offset_h = vec2(radius, 1.0f * radius);
	vec2 offset_l = vec2(radius, 15.0f * radius);
	int row_size = int(world_size.x/(2*radius));
    int num_heavy = 100;
    int num_light = 100;

    int fm_heavy_idx = pbd_unified_sim_add_fluid_model(sim, 0, desired_density0_heavy);
    initialize_fluid_particle_positions(sim, world_size, offset_l, row_size, num_heavy, fm_heavy_idx);

    int fm_light_idx = pbd_unified_sim_add_fluid_model(sim, 0, desired_density0_light);
    initialize_fluid_particle_positions(sim, world_size, offset_h, row_size, num_light, fm_light_idx);
}

void scene_distant_constraint_simple(PBDUnifiedSimulation* sim) {

    const vec2 world_size = pbd_unified_sim_get_world_bounds(sim);
	const float radius = pbd_unified_sim_get_particle_radius(sim);

    const float density = 100;

    const vec2 pos_a = 0.5f * world_size;
    const vec2 pos_b = pos_a + vec2(3.0f*radius, 10.0f*radius);

    const int idx_a = pbd_unified_sim_add_particle(sim, pos_a, density);
    const int idx_b = pbd_unified_sim_add_particle(sim, pos_b, density);

    pbd_unified_sim_particle_set_params(sim, idx_a, 0.0f, 0.0f, 0.0f);
    pbd_unified_sim_particle_set_params(sim, idx_b, 0.0f, 0.0f, 0.0f);
    
    const float len_ab = length(pos_a - pos_b);
    pbd_unified_sim_add_distance_constraint(sim, idx_a, idx_b, len_ab);

}

int g_anchor1_c = 0;
int g_anchor2_c = 0;
void scene_distant_constraint_two_ropes(PBDUnifiedSimulation* sim) {

    const vec2 world_size = pbd_unified_sim_get_world_bounds(sim);
	const float radius = pbd_unified_sim_get_particle_radius(sim);



    {
    float density = 100;
    const vec2 pos_a = vec2(0.45f * world_size.x, 0.9f*world_size.x);
    const int anchor_idx = pbd_unified_sim_add_particle(sim, pos_a, density);
    g_anchor1_c = pbd_unified_sim_add_distance2_constraint(sim, anchor_idx, pos_a, 0.0f*radius);

    constexpr const int num_links = 18;
    //int chain[num_links];
    int prev_link_idx = anchor_idx;
    vec2 prev_link_pos = pos_a;
    for(int i=0; i<num_links; ++i) {
        //if(i==num_links-1)
            //density -= 9;
        const vec2 pos = prev_link_pos + vec2(2.0f*radius, 0.0f);
        const int idx = pbd_unified_sim_add_particle(sim, pos, density);
        const float len_ab = length(pos - prev_link_pos);
        /*const int c_a2b_idx = */pbd_unified_sim_add_distance_constraint(sim, idx, prev_link_idx, len_ab);
        prev_link_idx = idx;
        prev_link_pos = pos;
    }
    }

    if(1)
    {
    const float density = 1000;
    const vec2 pos_a = vec2(0.40f * world_size.x - 5*radius, 0.9f*world_size.x);
    const int anchor_idx = pbd_unified_sim_add_particle(sim, pos_a, density);
    g_anchor2_c = pbd_unified_sim_add_distance2_constraint(sim, anchor_idx, pos_a, 0.0f*radius);

    constexpr const int num_links = 10;
    //int chain[num_links];
    int prev_link_idx = anchor_idx;
    vec2 prev_link_pos = pos_a;
    for(int i=0; i<num_links; ++i) {
        const vec2 pos = prev_link_pos - vec2(2.0f*radius, 0.0f);
        const int idx = pbd_unified_sim_add_particle(sim, pos, density);
        const float len_ab = length(pos - prev_link_pos);
        /*const int c_a2b_idx = */pbd_unified_sim_add_distance_constraint(sim, idx, prev_link_idx, len_ab);
        prev_link_idx = idx;
        prev_link_pos = pos;
    }
    }
}

void scene_rope_and_rigid_body(PBDUnifiedSimulation* sim) {

	const vec2 world_size = pbd_unified_sim_get_world_bounds(sim);
	const float radius = pbd_unified_sim_get_particle_radius(sim);

    int rb_particle_idx = -1;

	{
		float density = 100;
		const vec2 pos_a = vec2(0.45f * world_size.x, 0.9f * world_size.x);
		const int anchor_idx = pbd_unified_sim_add_particle(sim, pos_a, density);
		g_anchor1_c = pbd_unified_sim_add_distance2_constraint(sim, anchor_idx, pos_a,
															   0.0f * radius);

		constexpr const int num_links = 8;
		// int chain[num_links];
		int prev_link_idx = anchor_idx;
		vec2 prev_link_pos = pos_a;
		for (int i = 0; i < num_links; ++i) {
			// if(i==num_links-1)
			// density -= 9;
			const vec2 pos = prev_link_pos + vec2(0.0f, -2.0f * radius);
			const int idx = pbd_unified_sim_add_particle(sim, pos, density);
			const float len_ab = length(pos - prev_link_pos);
			/*const int c_a2b_idx = */ pbd_unified_sim_add_distance_constraint(
				sim, idx, prev_link_idx, len_ab);
			prev_link_idx = idx;
			prev_link_pos = pos;
		}

		vec2 rb_pos = vec2(prev_link_pos.x, prev_link_pos.y - 3 * radius);
		const float len_rb_rope = length(rb_pos - prev_link_pos);
		const int rb_idx = pbd_unified_sim_add_box_rigid_body(
			sim, 2, 2, rb_pos, 0 * 45.0f * 3.1415f / 180.0f, 80);

		const PBDRigidBody* rb = pbd_unified_sim_get_rigid_bodies(sim);
		const int rb_part_data_start_idx = rb[rb_idx].start_pdata_idx;
		const int rb_part_data_count = rb[rb_idx].num_part;
		const PBDRigidBodyParticleData* part_data =
			pbd_unified_sim_get_rb_particle_data(sim);

		for (int i = 0; i < rb_part_data_count; ++i) {
			rb_particle_idx = part_data[i + rb_part_data_start_idx].index;
			pbd_unified_sim_add_distance_constraint(sim, rb_particle_idx, prev_link_idx,
													len_rb_rope);
			// add constraint to only one particle in a rigid body
			break;
		}
	}

	if (1) {
		const float density = 1000;
		const vec2 pos_a = vec2(0.40f * world_size.x - 5 * radius, 0.9f * world_size.x);
		const int anchor_idx = pbd_unified_sim_add_particle(sim, pos_a, density);
		g_anchor2_c = pbd_unified_sim_add_distance2_constraint(sim, anchor_idx, pos_a,
															   0.0f * radius);

		constexpr const int num_links = 10;
		// int chain[num_links];
		int prev_link_idx = anchor_idx;
		vec2 prev_link_pos = pos_a;
		for (int i = 0; i < num_links; ++i) {
			const vec2 pos = prev_link_pos - vec2(0.0f, 2.0f * radius);
			const int idx = pbd_unified_sim_add_particle(sim, pos, density);
			const float len_ab = length(pos - prev_link_pos);
			/*const int c_a2b_idx = */ pbd_unified_sim_add_distance_constraint(
				sim, idx, prev_link_idx, len_ab);
			prev_link_idx = idx;
			prev_link_pos = pos;
		}

		pbd_unified_sim_add_distance_constraint(sim, rb_particle_idx, prev_link_idx,
												2.0f * radius);
	}
}

void scene_rb_static_collision(PBDUnifiedSimulation* sim) {

	const vec2 world_size = pbd_unified_sim_get_world_bounds(sim);
	const float radius = pbd_unified_sim_get_particle_radius(sim);

    CollisionWorld* cworld = collision_create_world();
    pbd_unified_sim_set_collision_world(sim, cworld);
    
    SDFBoxCollision box;
    box.pos = vec2(world_size.x*0.9, radius*3);
    box.rot = identity2();
    box.size = vec2(5*radius, 3*radius);
    //int cbox_idx = 
      collision_add_box(cworld, box);

    const float density0 = 1000;
    int rb2_idx = pbd_unified_sim_add_box_rigid_body(sim, 5, 2, vec2(25*radius, world_size.y*0.5f), -0*65.0f* 3.1415f/180.0f, density0);
    pbd_unified_sim_rb_add_velocity(sim, rb2_idx, vec2(2, 0));
    
    //pbd_unified_sim_add_box_rigid_body(sim, 5, 1, vec2(10*radius, world_size.y*0.5f), 0*30.0f* 3.1415f/180.0f, density0);
    
    vec2 ppos = vec2(2*radius, radius*30);
	pbd_unified_sim_add_particle(sim, ppos, density0);
}

typedef void(*phys_scene_constructor_fptr)(struct PBDUnifiedSimulation* );

int g_cur_phys_scene_index = 0;
phys_scene_constructor_fptr phys_scenes[] = {
    scene_soft_body_from_lattice,
    scene_soft_body,
    scene_initial_penetration,
    scene_restitution_test,
    scene_static_particle_friction_test,
    scene_dynamic_particle_friction_test,
    scene_restitution_chain_of_bodies,
    scene_stacking_particles,
    scene_stacking_particles_and_box_above,
    scene_complex,
    scene_particle_box_collision_test,
    scene_friction_test,
    scene_single_rb_friction_test,
    scene_rb_friction_test,
    scene_rb_friction_test2,
    scene_fluid_simple,
    scene_fluid_and_solids,
    scene_two_fluids,
    scene_distant_constraint_simple,
    scene_distant_constraint_two_ropes,
    scene_rope_and_rigid_body,
    scene_rb_static_collision,
    scene_rigid_body_restitution_test
};



PBDTestObject* PBDTestObject::Create() {

    PBDTestObject* o = new PBDTestObject;
    o->sim_dim_ = vec2(5,5);
    o->sim_origin_ = vec2(0,0);
    o->sim_ = pbd_unified_sim_create(o->sim_dim_);
    o->dbg_flags_.contacts = false;
    o->dbg_flags_.friction = true;
    
    (phys_scenes[g_cur_phys_scene_index])(o->sim_);

	return o;
}

PBDTestObject::~PBDTestObject() {
   pbd_unified_sim_destroy(sim_); 
}

struct PBDParticleVDecl {
	vec2 pos;
	vec2 vel;
	vec2 force;
	float density;
	float pressure;
    uint32_t flags;
};

static HGOSVERTEXDECLARATION get_pbd_vdecl() {

	static gosVERTEX_FORMAT_RECORD pbd_vdecl[] = {
		// SVD
		{0, 3, false, sizeof(SVD), 0, gosVERTEX_ATTRIB_TYPE::kFLOAT, 0},
		{1, 2, false, sizeof(SVD), offsetof(SVD, uv), gosVERTEX_ATTRIB_TYPE::kFLOAT, 0},
		{2, 3, false, sizeof(SVD), offsetof(SVD, normal), gosVERTEX_ATTRIB_TYPE::kFLOAT, 0},

		// instance data: stream 1
		{3, 2, false, sizeof(PBDParticleVDecl), 0, gosVERTEX_ATTRIB_TYPE::kFLOAT, 1},
		{4, 2, false, sizeof(PBDParticleVDecl), offsetof(PBDParticleVDecl,vel), gosVERTEX_ATTRIB_TYPE::kFLOAT, 1},
		{5, 2, false, sizeof(PBDParticleVDecl), offsetof(PBDParticleVDecl,force), gosVERTEX_ATTRIB_TYPE::kFLOAT, 1},
		{6, 1, false, sizeof(PBDParticleVDecl), offsetof(PBDParticleVDecl,density), gosVERTEX_ATTRIB_TYPE::kFLOAT, 1},
		{7, 1, false, sizeof(PBDParticleVDecl), offsetof(PBDParticleVDecl,pressure), gosVERTEX_ATTRIB_TYPE::kFLOAT, 1},
		{8, 1, false, sizeof(PBDParticleVDecl), offsetof(PBDParticleVDecl,flags), gosVERTEX_ATTRIB_TYPE::kUNSIGNED_INT, 1},
	};

    static auto vdecl = gos_CreateVertexDeclaration(
        pbd_vdecl, sizeof(pbd_vdecl) / sizeof(gosVERTEX_FORMAT_RECORD));
    return vdecl;
}


extern int RendererGetNumBufferedFrames();
void PBDTestObject::InitRenderResources() {
	sphere_mesh_ = res_man_load_mesh("sphere");

	gos_AddRenderMaterial("deferred_pbd_particle");

    int num_buffers = RendererGetNumBufferedFrames() + 1;
    inst_vb_.resize(num_buffers);
	for (int32_t i = 0; i < num_buffers; ++i) {
		inst_vb_[i] =
			gos_CreateBuffer(gosBUFFER_TYPE::VERTEX, gosBUFFER_USAGE::DYNAMIC_DRAW,
							 sizeof(PBDParticleVDecl), 1000, nullptr);
	}

    vdecl_ = get_pbd_vdecl();
	mat_ = gos_getRenderMaterial("deferred_pbd_particle");

    b_initalized_rendering_resources = true;

}

void PBDTestObject::DeinitRenderResources() {
    b_initalized_rendering_resources = false;

    delete sphere_mesh_;
    sphere_mesh_ = nullptr;
    for (auto& buf : inst_vb_) {
        gos_DestroyBuffer(buf);
    }
    inst_vb_.clear();
    gos_DestroyVertexDeclaration(vdecl_);
    vdecl_ = nullptr;
}

void PBDTestObject::Update(float dt) {

    static float density = 50;
	if (g_rb >= 0) {
		if (gos_GetKeyStatus(KEY_V) == KEY_PRESSED) {
			density += 10;
			pbd_unified_sim_rb_set_density(sim_, g_rb, density);
			// add particle
		}
		if (gos_GetKeyStatus(KEY_B) == KEY_PRESSED) {
			density -= 10;
			if (density < 5) density = 5;
			pbd_unified_sim_rb_set_density(sim_, g_rb, density);
		}
	}

    bool b_change_scene = false;
	if (gos_GetKeyStatus(KEY_PERIOD) == KEY_PRESSED) {
        g_cur_phys_scene_index = (g_cur_phys_scene_index + 1) % (sizeof(phys_scenes)/sizeof(phys_scenes[0]));
        b_change_scene = true;
    }
	if (gos_GetKeyStatus(KEY_COMMA) == KEY_PRESSED) {
        size_t num_scenes = sizeof(phys_scenes)/sizeof(phys_scenes[0]);
        g_cur_phys_scene_index = (g_cur_phys_scene_index + num_scenes - 1) % num_scenes;
        b_change_scene = true;
    }
    if(b_change_scene) {
        CollisionWorld* cw = pbd_unified_sim_get_collision_world(sim_);
        collision_destroy_world(cw);
        pbd_unified_sim_reset(sim_);
        phys_scenes[g_cur_phys_scene_index](sim_);
    }

// disabled until remove function implemented
#if 0
	if (g_anchor1_c >= 0 && g_anchor2_c>=0) {
		if (gos_GetKeyStatus(KEY_R) == KEY_PRESSED) {
            pbd_unified_sim_remove_distance2_constraint(sim_, 0);//g_anchor1_c);
			// drop one at a time
            //pbd_unified_sim_remove_distance2_constraint(sim_, 0);//g_anchor2_c);
        }
    }
#endif

	{
		SCOPED_ZONE_N(pbd_unified_timestep, 0);
		pbd_unified_timestep(sim_, dt);
	}
}

void select_and_draw_debug_particle(const PBDUnifiedSimulation* sim,
									RenderFrameContext* rfc) {

	const int particles_count = pbd_unified_sim_get_particle_count(sim);
	const float particles_r = pbd_unified_sim_get_particle_radius(sim);
	const PBDParticle* particles = pbd_unified_sim_get_particles(sim);

	static int debug_particle = 104;
	if (gos_GetKeyStatus(KEY_MINUS) == KEY_PRESSED) {

        const vec3 pos = get_ws_mouse_pos(rfc);

		for (int i = 0; i < particles_count; ++i) {
			if (lengthSqr(pos.xy() - particles[i].x) < particles_r * particles_r) {
				debug_particle = i;
				break;
			}
		}
	}

	if (debug_particle >= 0 && debug_particle < particles_count) {
		vec2 deb_pos = particles[debug_particle].x;
		vec3 pos(deb_pos.x, deb_pos.y, -0.1f);
		rfc->rl_->addDebugPoints(&pos, 1, vec4(1, 1, 0.5f, 1), 4, true);
	}
}

void PBDTestObject::AddRenderPackets(struct RenderFrameContext *rfc) const {

    SCOPED_ZONE_N(PBDTestObject_AddRenderPackets, 0);

	if (!b_initalized_rendering_resources) return;

    pbd_unified_sim_debug_draw_world(sim_, rfc, dbg_flags_);

    select_and_draw_debug_particle(sim_, rfc);
    collision_debug_draw(pbd_unified_sim_get_collision_world(sim_), rfc->rl_);

	// update instancing buffer
	cur_inst_vb_ = (cur_inst_vb_ + 1) % ((int)inst_vb_.size());

    char buf[128];
    sprintf(buf, "particle buf idx: %d", cur_inst_vb_);
    BEGIN_ZONE_DYNAMIC_N(part_draw, buf, 0);

    const int cur_part_buf_idx = cur_inst_vb_;
	const int num_particles = pbd_unified_sim_get_particle_count(sim_);
	HGOSBUFFER inst_vb = inst_vb_[cur_inst_vb_];
    const PBDParticle* particles = pbd_unified_sim_get_particles(sim_);

    // TODO: reuse buffers
    PBDParticle* temp_buf = new PBDParticle[num_particles];
    memcpy(temp_buf, particles, sizeof(PBDParticle)*num_particles);
    // HACK: using phase here to store debug color for fluids
    for(int i=0;i<num_particles;++i) {
        if (particles[i].flags & PBDParticleFlags::kFluid) {
            const PBDFluidModel* fm = pbd_unified_sim_get_fluid_particle_model(sim_, particles[i].phase);
            temp_buf[i].phase = fm->debug_color_;
        } else {
            temp_buf[i].phase = 0xffffffff;
        }
    }

	if (num_particles) {
		ScheduleRenderCommand(rfc, [num_particles, inst_vb, temp_buf, cur_part_buf_idx ]() {
        
            char render_buf[128];
            sprintf(render_buf, "particle buf idx: %d", cur_part_buf_idx );
            BEGIN_ZONE_DYNAMIC_N(render_part_draw, render_buf, 0);

			const size_t bufsize = num_particles * sizeof(PBDParticleVDecl);
			// TODO: think about typed buffer wrapper
			int inst_buf_num_part =
				(int)(gos_GetBufferSizeBytes(inst_vb) / sizeof(PBDParticleVDecl));
			if (inst_buf_num_part < num_particles) {
				gos_ResizeBuffer(inst_vb, (uint32_t)(num_particles * 1.5f));
			}

			PBDParticleVDecl* part_data = (PBDParticleVDecl*)gos_MapBuffer(
				inst_vb, 0, bufsize, gosBUFFER_ACCESS::WRITE);
			for (int i = 0; i < num_particles; ++i) {
				PBDParticleVDecl& p = part_data[i];

				p.pos = temp_buf[i].x;
				p.vel = temp_buf[i].v;
				p.force = vec2(0);
				//p.density = 1;
				p.density = *reinterpret_cast<float*>(&temp_buf[i].phase);
				p.pressure = 1;
				p.flags = temp_buf[i].flags;
			}

            delete[] temp_buf;

			gos_UnmapBuffer(inst_vb);

#ifdef DEBUG_DRAW_PARTICLE_OUTLINE
			for (int i = 0; i < num_particles; ++i) {
				vec3 p = vec3(particles[i].pos.x, particles[i].pos.y, 0);
				// ceneter
				gos_AddPoints(&p, 1, vec4(1, 1, 1, 1), 4);
				for (int j = 0; j < 36; ++j) {
					vec3 dp = p + 0.1 * vec3(sin((float)j * 10 * M_PI / 180.0f),
											 cos((float)j * 10 * M_PI / 180.0f), 0.0f);
					gos_AddPoints(&dp, 1, vec4(1, 1, 1, 1), 4);
				}
			}
#endif
            END_ZONE(render_part_draw);
		});
	}

	class RenderList* rl = rfc->rl_;
	RenderPacket *rp = rl->AddPacket();
	memset(rp, 0, sizeof(RenderPacket));
	rp->id_ = GetId();
	rp->is_opaque_pass = 1;
	// NOTE: should we render it into selection pass? maybe.. but only when it will support
	// instaced vert shader
	rp->is_selection_pass = 0;
    rp->m_ = mat4::identity();
	rp->mesh_ = *sphere_mesh_;
	rp->mesh_.mat_ = mat_;
	rp->mesh_.inst_vb_ = inst_vb_[cur_inst_vb_];
    rp->mesh_.vdecl_ = vdecl_;
    rp->mesh_.num_instances = num_particles;

    END_ZONE(part_draw);
}

void initialize_particle_positions(struct PBDUnifiedSimulation* sim,
								   const ivec2& row_column, const vec2 offset,
								   float density0) {

	const float radius = pbd_unified_sim_get_particle_radius(sim);
	int row_size = row_column.x;
	int column_size = row_column.y;
	for (int y = 0; y < row_size; ++y) {
		for (int x = 0; x < column_size; ++x) {

			float jitter = 0;
			vec2 pos = offset + vec2(x * 2.0f * radius + jitter, y * 2.0f * radius);

			pbd_unified_sim_add_particle(sim, pos, density0);
		}
	}
}

void initialize_particle_positions2(struct PBDUnifiedSimulation* sim, const vec2& offset, int count,
								   float density0, float mu_s, float mu_k, float e) {

	const float radius = pbd_unified_sim_get_particle_radius(sim);
	int row_size = (int)(sqrtf((float)count) + 1.5f);
	int column_size = (count + row_size - 1) / row_size;
	for (int y = 0; y < column_size; ++y) {
		for (int x = 0; x < row_size; ++x) {

			int idx = y * row_size + x;
			if (idx >= count) break;

			float jitter = 0;
			vec2 pos = offset + vec2(x * 2.0f * radius + jitter, y * 2.0f * radius);

			int p_idx = pbd_unified_sim_add_particle(sim, pos, density0);
            pbd_unified_sim_particle_set_params(sim, p_idx, 0.0f, 0.0f, e);
		}
	}
}

void initialize_fluid_particle_positions(struct PBDUnifiedSimulation* sim,
										 const vec2& dim, const vec2& offset, int row_size, int count,
										 int fluid_model_idx) {

	const float radius = pbd_unified_sim_get_particle_radius(sim);
	int column_size = (count + row_size - 1) / row_size;
	for (int y = 0; y < column_size; ++y) {
		for (int x = 0; x < row_size; ++x) {

			int idx = y * row_size + x;
			if (idx >= count) break;

			float jitter = 0;
			vec2 pos = offset + vec2(x * 2.0f * radius + jitter, y * 2.0f * radius);

			pbd_unified_sim_add_fluid_particle(sim, pos, fluid_model_idx);
		}
	}
}

void collision_debug_draw(const struct CollisionWorld* cworld, RenderList* rl) {

    if(!cworld)
        return;

    const float z = -0.1f;
    int count;
    const SDFBoxCollision* boxes = collision_world_get_box(cworld, &count);
    for(int i=0; i<count;++i) {
        const SDFBoxCollision& box = boxes[i];
        vec2 s = vec2(box.size.x, box.size.y);
        vec2 rt = vec2(s.x, s.y);
        vec2 lt = vec2(-s.x, s.y);

        vec2 rb = vec2(s.x, -s.y);
        vec2 lb = vec2(-s.x, -s.y);

        rt = box.rot*rt + box.pos;
        lt = box.rot*lt + box.pos;
        rb = box.rot*rb + box.pos;
        lb = box.rot*lb + box.pos;
	
        rl->addDebugLine(vec3(rt.x, rt.y, z), vec3(lt.x, lt.y, z), vec4(1));
        rl->addDebugLine(vec3(lt.x, lt.y, z), vec3(lb.x, lb.y, z), vec4(1));
        rl->addDebugLine(vec3(lb.x, lb.y, z), vec3(rb.x, rb.y, z), vec4(1));
        rl->addDebugLine(vec3(rb.x, rb.y, z), vec3(rt.x, rt.y, z), vec4(1));
    }

}

void pbd_unified_sim_debug_draw_world(const struct PBDUnifiedSimulation* sim, RenderFrameContext* rfc, PBDTestObject::DDFlags draw_flags) {

    RenderList* rl = rfc->rl_;

    const PBDRigidBodyParticleData* data = pbd_unified_sim_get_rb_particle_data(sim);
    const PBDRigidBody* rb = pbd_unified_sim_get_rigid_bodies(sim);

    const int count = pbd_unified_sim_get_particle_count(sim);
    const PBDParticle* particles = pbd_unified_sim_get_particles(sim);

    const float z = -0.1f;

	for (int i = 0; i < count; ++i) {
		const PBDParticle& p = particles[i];
		if (p.flags & PBDParticleFlags::kRigidBody) {
			const PBDRigidBodyParticleData& data_i = data[p.rb_data_idx];
			const PBDRigidBody& rb_i = rb[p.phase];
			vec2 grad = rotate2(rb_i.angle) * data_i.sdf_grad;
			rl->addDebugLine(vec3(p.x, 0), vec3(p.x, 0) + 0.15f * vec3(grad, 0),
							 vec4(1, 1, 0.5f, 1));
		}
	}

	vec2 world_bounds = pbd_unified_sim_get_world_bounds(sim);
	rl->addDebugLine(vec3(0, 0, z), vec3(world_bounds.x, 0, z), vec4(1));
	rl->addDebugLine(vec3(world_bounds.x, 0, z), vec3(world_bounds, z), vec4(1));
	rl->addDebugLine(vec3(world_bounds, z), vec3(0, world_bounds.y, z), vec4(1));
	rl->addDebugLine(vec3(0, world_bounds.y, z), vec3(0, 0, z), vec4(1));

	//for(auto rb: sim->rigid_bodies_) {
    //}

	const float r = pbd_unified_sim_get_particle_radius(sim);

	if (draw_flags.contacts) {
		int num_contacts = 0;
		const DbgContactInfo* pcontacts =
			pbd_unified_sim_get_dbg_contacts(sim, &num_contacts);
		for (int i = 0; i < num_contacts; ++i) {
			const DbgContactInfo& c = pcontacts[i];
			vec3 p0(c.p0.x, c.p0.y, z);
			vec3 p1(c.p1.x, c.p1.y, z);
			rl->addDebugPoints(&p0, 1, vec4(1, 1, 0, 1), 1.0f, false, nullptr);
			rl->addDebugPoints(&p1, 1, vec4(1, 1, 0, 1), 1.0f, false, nullptr);

			vec3 e0 = p0 + vec3(c.dp0, 0);
			vec3 e1 = p1 - vec3(c.dp1, 0);
			rl->addDebugLine(p0, e0, vec4(1));
			rl->addDebugLine(p1, e1, vec4(1));

			e0 = p0 + 0.8f * r * vec3(c.n, 0);
			e1 = p1 - 0.8f * r * vec3(c.n, 0);
			rl->addDebugLine(p0, e0, vec4(0.3f, 0.3f, 1.0f, 1.0f));
			rl->addDebugLine(p1, e1, vec4(0.3f, 0.3f, 1.0f, 1.0f));
		}
	}

	if (draw_flags.friction) {
		static float scale = 100.0f; // can autonormalize scale depending on path len
		int num_contacts = 0;
        vec4 red = vec4(1, 0, 0, 1);
        vec4 blue = vec4(0, 0, 1, 1);
        vec4 green = vec4(0, 1, 1, 1);

		const DbgFrictionInfo* pcontacts =
			pbd_unified_sim_get_dbg_friction(sim, &num_contacts);
		for (int i = 0; i < num_contacts; ++i) {
			const DbgFrictionInfo& c = pcontacts[i];
            // path
			vec3 p0(c.s_path.x, c.s_path.y, z);
            vec3 path_len = scale*(vec3(c.d_path.x, c.d_path.y, z) - p0);
			vec3 p1 = p0 + path_len;
			rl->addDebugLine(p0, p1, vec4(1));

            // dx
			vec3 dx = scale*vec3(c.dx.x, c.dx.y, 0);
			rl->addDebugLine(p1, p1 + dx, red);
			rl->addDebugLine(p1, p1 + c.mu_s * dx, blue);

            // dxp
            vec2 path = c.d_path + c.dx - c.s_path;
            float dxn = dot(path, c.n);
            vec2 dxp = path - dxn * c.n;
			vec3 dxp_pt = p0 + scale*vec3(dxp.x, dxp.y, 0);
			rl->addDebugLine(p0, dxp_pt, green);
		}
	}

	vec2 mouse = get_ws_mouse_pos(rfc).xy();
    vec3 mp = vec3(mouse, z);
    rl->addDebugPoints(&mp,1, vec4(1), 3.0f, false, nullptr);

#if TEST_REFLECTION 

    static vec3 g_n = vec3(0.1, 0.7, 0);
    static vec3 g_v = vec3(1,0, 0);

    g_n = vec3(normalize(g_n.xy()), 0);
    vec3 c = vec3(3,1,z);
    g_v = vec3(mp.x - c.x, mp.y - c.y, 0);
    rl->addDebugPoints(&c,1, vec4(0,1,0,1), 2.0f, false, nullptr);
	rl->addDebugLine(c, c + g_n, vec4(0.3f, 0.3f, 1.0f, 1.0f));
	rl->addDebugLine(c, c + g_v, vec4(0.3f, 0.1f, 0.3f, 1.0f));
    vec3 refl = vec3(reflect(g_v.xy(), g_n.xy()), 0);
	rl->addDebugLine(c, c + refl, vec4(1.0f, 0, 0, 1));
#endif
    
}

vec3 get_ws_mouse_pos(RenderFrameContext* rfc) {
	int XDelta, YDelta, WheelDelta;
	float XPos, YPos;
	DWORD buttonsPressed;
	gos_GetMouseInfo(&XPos, &YPos, &XDelta, &YDelta, &WheelDelta, &buttonsPressed);

	const vec2 mouse_screen_pos = 2 * vec2(XPos, 1 - YPos) - vec2(1);
	const vec3 dir = screen2world_vec(rfc->inv_view_, rfc->inv_proj_, mouse_screen_pos);

	const vec3 ws_cam_pos = (rfc->inv_view_ * vec4(0, 0, 0, 1)).xyz();
	const vec3 pos =
		ray_plane_intersect(dir, ws_cam_pos, make_plane(vec3(0, 0, 1), vec3(0, 0, 0)));
	return pos;
}
