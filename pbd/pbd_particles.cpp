// Unified Particle Physics for Real-Time Applications

#include "pbd_particles.h"
#include "pbd_particles_collision.h"
#include "engine/utils/kernels.h"
#include "engine/profiler/profiler.h"
#include "engine/utils/math_utils.h"
#include "engine/utils/adt.h"

#include <cmath>
#include <vector>
#include <cassert>
#include <queue>

// just for convenience, less typing
typedef uint32_t u32;
typedef int32_t i32;
typedef uint16_t u16;
typedef int16_t i16;
typedef uint8_t u8;
typedef int8_t i8;

#define INCLUDE_DIAG_NEIGHBOURS 0

// flags for particle regions calculation
static const u16 kL = 0x1;
static const u16 kR = 0x2;
static const u16 kT = 0x4;
static const u16 kB = 0x8;
#if INCLUDE_DIAG_NEIGHBOURS
static const u16 kLT = 0x10;
static const u16 kLB = 0x20;
static const u16 kRT = 0x40;
static const u16 kRB = 0x80;
#endif
static const u16 kExists = 0x100;
static const u16 kVisited = 0x200;

u32 get_debug_colour() {
	static const u32 debug_colours[4] = {0xFFAA00FF, 0x55AA00FF, 0x007700FF, 0xAABB33FF};
    static u8 i = 0;
    return debug_colours[i++%4];
}

#define MAKE_COLLISIONS_GREAT_AGAIN 1
// this prefers normal straight along collision line, which works better just because
// corner particles in rigid bodies have gradients which are not always "correct"
// Maybe makes sense to query gradient of J RB from position of I particle

// investigate why particles go nuts and enable after that, because enabling this, will
// hide the problem
#define ENABLE_CFL 1
#define VELOCITY_UPDATE 1
#define GAUSS_SEIDEL_UPDATE() 1
#define GAUSS_SEIDEL_STATIC_COLLISION() (1 || GAUSS_SEIDEL_UPDATE())

struct Pair {
	int start;
	int num;
};

struct NeighbourData {
	std::vector<int> neigh_idx_;
	std::vector<Pair> neighbour_info_;
};

struct ShapeMatchingConstraint {
    int rb_index;
};

struct SoftShapeMatchingConstraint {
    int sb_index;
};

struct RigidBodyCollisionConstraint {
    int idx0, idx1;
    vec2 x_ij;
};

struct ParticleRigidBodyCollisionConstraint {
    // idx0 - rigid body particle
    // idx1 - ordinary particle
    int idx0, idx1;
    // xi - xj, xi <-------- xj
    vec2 x_ij;
};

struct SolidParticlesCollisionConstraint {
    int idx0, idx1;
    // xi - xj, xi <-------- xj
    vec2 x_ij;
};

struct BoxBoundaryConstraint {
    //bool b_invert;
    vec2 p_min;
    vec2 p_max;
    float mu_s, mu_k;
};

// idx - particle index or -1 if static boundary
// TODO: have a union with obj type and treat idx depending on it, or just directly here
// store all necessary info
struct ContactInfo {
    //vec2 r1;
    vec2 v1_pre;
    int idx1;

    //vec2 r2;
    vec2 v2_pre;
    int idx2; 

    // kinetic friction (a.k.a. dynamic friction)
	float mu_d;
	float e;
	// always r2's (i.e. normal of body 2)
	vec2 n;
	// delta lambda (in normal direction) which for 1 positin solve iteration is same as just lambda
	float d_lambda_n;
};

// internal debug functionality 
static void add_dbg_info(int p0_idx, int p1_idx, vec2 norm, vec2 dp0, vec2 dp1, vec2 p0, vec2 p1, PBDUnifiedSimulation* sim);
static void add_dbg_friction_info(int i, vec2 n, vec2 dx, float mu_s, float mu_k, vec2 s_path, vec2 d_path, PBDUnifiedSimulation* sim);
//

void findNeighboursNaiive(const vec2* x, const int size, float radius,
						  PBDParticle* particles, NeighbourData* nd) {
    SCOPED_ZONE_N(findNeighboursNaiive, 0);

	nd->neigh_idx_.resize(0);

	int offset = 0;
	const float r_sq = radius * radius;

	for (int i = 0; i < size; i++) {
		vec2 x_i = x[i];
		int start = offset;
		for (int j = 0; j < size; j++) {

			if (i == j) {
                continue;
            }

			// skip particles of same rigid body, maybe leave "j" in neighbours but just use
			// this condition when checking constraints?
			bool b_is_same_rb = particles[i].phase == particles[j].phase &&
							  (particles[i].flags & PBDParticleFlags::kRigidBody) &&
                              // this condition is redundant
							  (particles[j].flags & PBDParticleFlags::kRigidBody);
			bool b_phase_cond = !b_is_same_rb;
			if (b_phase_cond && lengthSqr(x_i - x[j]) < r_sq) {
				nd->neigh_idx_.push_back(j);
				offset++;
			}
		}
		nd->neighbour_info_[i].start = start;
		nd->neighbour_info_[i].num = offset - start;
	}
}

struct PBDUnifiedSimulation {

    static constexpr const float particle_r_ = 0.1f;
    static constexpr const float support_r_ = particle_r_ * 4.0f;

    vec2 world_size_ = vec2(5,5);

    PBDSettings settings;

    struct CollisionWorld* collision_world_;

	FreeList<DistanceConstraint> distance_c_;
	std::vector<int> distance_c_idxs_;

	FreeList<DistanceConstraint2> distance2_c_;
	std::vector<int> distance2_c_idxs_;

	std::vector<SolidParticlesCollisionConstraint> solid_collision_c_;
	std::vector<SolidParticlesCollisionConstraint> solid_fluid_c_;
    // maybe just iterate over rigid / soft bodies instead of having separate array for those?
	std::vector<ShapeMatchingConstraint> shape_matching_c_;
	std::vector<SoftShapeMatchingConstraint> soft_shape_matching_c_;
	std::vector<i32> break_checks_;

	std::vector<RigidBodyCollisionConstraint> rb_collision_c_;
	std::vector<ParticleRigidBodyCollisionConstraint> particle_rb_collision_c_;
	std::vector<BoxBoundaryConstraint> box_boundary_c_;

	std::vector<PBDParticle> particles_;
	std::vector<int> fluid_particle_idxs_;
	std::vector<PBDRigidBodyParticleData> rb_particles_data_;
	std::vector<PBDSoftBodyParticleData> sb_particles_data_;
	std::vector<PBDFluidParicleRuntimeData> fluid_particles_data_;  // do we need this?
	std::vector<PBDRigidBody> rigid_bodies_;
	FreeList<PBDSoftBody> soft_bodies_;
	std::vector<u32> sb_idxs_; // indices into soft_bodies_ free list
	std::vector<PBDRegion> sb_regions_;
	std::vector<uint32_t> sb_particles_per_region_;
	std::vector<uint32_t> sb_regions_per_particle_;
	FreeList<PBDBreakableSoftBody> breakable_bodies_;

	std::vector<u32> free_breakable_bodies_;
	std::vector<PBDBodyLatticeData> bb_lattice_;
	std::vector<PBDFluidModel> fluid_models_;
	std::vector<float> inv_scaled_mass_;
	std::vector<vec2> dp_;
	std::vector<int> num_constraints_;
	std::vector<vec2> x_pred_;
    NeighbourData neigh_data_;

	std::vector<ContactInfo> contacts_info_;

	std::vector<DbgContactInfo> dbg_contacts_;
	std::vector<DbgFrictionInfo> dbg_friction_;

    // relaxation parameter, see eq. 10 or 11 in the fluid paper
	inline static const float e_ = 0.0001f;//1.0e-6f;

    float W_zero_;
	float (*kernel_fptr_)(const vec3 &);
	vec3 (*grad_kernel_fptr_)(const vec3& r);

    float W(const vec2& v) const {
        return kernel_fptr_(vec3(v.x, v.y, 0.0f));
    }

    vec2 gradW(const vec2& v) const {
        vec3 grad = grad_kernel_fptr_(vec3(v.x, v.y, 0.0f));
        return vec2(grad.x, grad.y);
    }

    float Wzero() const { return W_zero_; }

};

struct PBDUnifiedSimulation* pbd_unified_sim_create(const vec2& sim_dim) {
    PBDUnifiedSimulation* sim = new PBDUnifiedSimulation();
    sim->world_size_ = sim_dim;

    float fluid_support_r = sim->particle_r_ * 3.0f;
	CubicKernel::setRadius(fluid_support_r);
	CubicKernel2D::setRadius(fluid_support_r);
	Poly6Kernel::setRadius(fluid_support_r);
	Poly6Kernel2D::setRadius(fluid_support_r);
    SpikyKernel::setRadius(fluid_support_r);
    SpikyKernel2D::setRadius(fluid_support_r);

    sim->kernel_fptr_ = Poly6Kernel2D::W;
	sim->W_zero_ = Poly6Kernel2D::W_zero();
	//sim->grad_kernel_fptr_ = SpikyKernel2D::gradW;
	sim->grad_kernel_fptr_ = Poly6Kernel2D::gradW;

	sim->box_boundary_c_.push_back(BoxBoundaryConstraint{
		//.b_invert = false,
		.p_min = vec2(0),
		.p_max = sim_dim,
        .mu_s = 1.0f,
        .mu_k = 1.0f,
	});

    sim->settings.world_bounds = sim->world_size_;
    sim->settings.supports_diag_links = !!INCLUDE_DIAG_NEIGHBOURS;

	return sim;
}

void pbd_unified_sim_destroy(struct PBDUnifiedSimulation* sim) {
    delete sim;
}

void pbd_unified_sim_set_collision_world(struct PBDUnifiedSimulation* sim, struct CollisionWorld* cworld) {
    assert(!sim->collision_world_);
    sim->collision_world_ = cworld;
}

struct CollisionWorld* pbd_unified_sim_get_collision_world(const struct PBDUnifiedSimulation* sim) {
    return sim->collision_world_;
}

int pbd_unified_sim_add_particle(struct PBDUnifiedSimulation* sim, vec2 pos,
								  float density) {

    float inv_mass = 1.0f / (2 * sim->particle_r_ * 2 * sim->particle_r_ * density);
	sim->particles_.emplace_back(PBDParticle{
		.x = pos,
		.v = vec2(0),
		.inv_mass = inv_mass,
        .flags = PBDParticleFlags::kSolid,
        .phase = -1,
        .rb_data_idx = -1,
        .mu_s = 0.0f,
        .mu_k = 0.0f,
        .e = 0.0f
	});

	sim->inv_scaled_mass_.push_back(inv_mass);
	sim->dp_.push_back(vec2(0));
	sim->num_constraints_.push_back(0);
	sim->x_pred_.push_back(vec2(0));

    sim->neigh_data_.neighbour_info_.push_back(Pair{0,0});

    return (int)(sim->particles_.size() - 1);
}

int pbd_unified_sim_add_particle(struct PBDUnifiedSimulation* sim, vec2 pos,
								 vec2 init_vel, float density) {

	int idx = pbd_unified_sim_add_particle(sim, pos, density);
    sim->particles_[idx].v = init_vel;
    return idx;
}

void pbd_unified_sim_particle_add_velocity(struct PBDUnifiedSimulation* sim, int idx, const vec2& vel) {
    assert((int)sim->particles_.size() > idx);
    sim->particles_[idx].v += vel;
}

void pbd_unified_sim_rb_add_velocity(struct PBDUnifiedSimulation* sim, int rb_idx, const vec2& vel) {
    assert((int)sim->rigid_bodies_.size() > rb_idx);
    const int start = sim->rigid_bodies_[rb_idx].start_pdata_idx;
    const int end = start + sim->rigid_bodies_[rb_idx].num_part;
	for (int i = start; i != end; ++i) {
        int idx = sim->rb_particles_data_[i].index;
        sim->particles_[idx].v += vel;
	}
}

void pbd_unified_sim_rb_set_density(struct PBDUnifiedSimulation* sim, int rb_idx, float density) {
    assert((int)sim->rigid_bodies_.size() > rb_idx);

    const float inv_mass = 1.0f / (2 * sim->particle_r_ * 2 * sim->particle_r_ * density);
    const int start = sim->rigid_bodies_[rb_idx].start_pdata_idx;
    const int end = start + sim->rigid_bodies_[rb_idx].num_part;
	for (int i = start; i != end; ++i) {
        int idx = sim->rb_particles_data_[i].index;

        sim->particles_[idx].inv_mass = inv_mass;
	}
}

void pbd_unified_sim_particle_set_params(struct PBDUnifiedSimulation* sim, int idx, float mu_s, float mu_k, float e) {
    assert((int)sim->particles_.size() > idx);
    sim->particles_[idx].mu_s = mu_s;
    sim->particles_[idx].mu_k = mu_k;
    sim->particles_[idx].e = e;
}

static int pbd_unified_sim_add_rb_particle_data(struct PBDUnifiedSimulation* sim, vec2 x0,
										 float sdf_value, vec2 sdf_grad,
										 int particle_index, bool b_is_boundary,
										 float density) {

	sim->rb_particles_data_.emplace_back(PBDRigidBodyParticleData{
		.x0 = x0,
        .sdf_value = sdf_value,
        .sdf_grad = sdf_grad,
        .index = particle_index,
        .b_is_boundary = b_is_boundary,
	});

    return (int)(sim->rb_particles_data_.size() - 1);
}

static int pbd_unified_sim_add_sb_particle_data(struct PBDUnifiedSimulation* sim, vec2 x0,
										 float sdf_value, vec2 sdf_grad,
										 int particle_index, bool b_is_boundary,
										 float density, int x, int y) {

	sim->sb_particles_data_.emplace_back(PBDSoftBodyParticleData{
		.base = PBDRigidBodyParticleData{.x0 = x0,
										 .sdf_value = sdf_value,
										 .sdf_grad = sdf_grad,
										 .index = particle_index,
										 .b_is_boundary = b_is_boundary},
		.x = x,
		.y = y,
		.goal_x = vec2(0.0f),
		.goal_R = mat2(0,0,0,0),
		.start_region_idx = kPBDInvalidIndex,
		.num_regions = 0,
	});

	return (int)(sim->sb_particles_data_.size() - 1);
}

void pbd_unified_sim_reset(PBDUnifiedSimulation* sim) {

    sim->collision_world_ = nullptr;

	sim->distance_c_.clear();
	sim->distance_c_idxs_.resize(0);
	sim->distance2_c_.clear();
	sim->distance2_c_idxs_.resize(0);

	sim->solid_collision_c_.resize(0);
	sim->solid_fluid_c_.resize(0);
	sim->shape_matching_c_.resize(0);
	sim->soft_shape_matching_c_.resize(0);
	sim->rb_collision_c_.resize(0);
	sim->particle_rb_collision_c_.resize(0);
	sim->box_boundary_c_.resize(0);
    
	sim->break_checks_.resize(0);

	sim->particles_.resize(0);
	sim->fluid_particle_idxs_.resize(0);
	sim->rb_particles_data_.resize(0);
	sim->sb_particles_data_.resize(0);
	sim->fluid_particles_data_.resize(0);  // do we need this?
	sim->rigid_bodies_.resize(0);
	sim->soft_bodies_.clear();
	sim->sb_idxs_.resize(0);
	sim->sb_regions_.resize(0);
	sim->sb_particles_per_region_.resize(0);
    sim->sb_regions_per_particle_.resize(0);
	sim->breakable_bodies_.clear();
	sim->fluid_models_.resize(0);
	sim->inv_scaled_mass_.resize(0);
	sim->dp_.resize(0);
	sim->num_constraints_.resize(0);
	sim->x_pred_.resize(0);
    sim->neigh_data_.neighbour_info_.resize(0);
	sim->contacts_info_.resize(0);
	sim->dbg_contacts_.resize(0);
	sim->dbg_friction_.resize(0);

	sim->box_boundary_c_.push_back(BoxBoundaryConstraint{
		.p_min = vec2(0),
		.p_max = sim->world_size_,
        .mu_s = 1.0f,
        .mu_k = 1.0f,
	});

}

float pbd_unified_sim_get_particle_radius(const PBDUnifiedSimulation* sim) {
    return sim->particle_r_;
}

int pbd_unified_sim_get_particle_count(const PBDUnifiedSimulation* sim) {
    return (int)sim->particles_.size();
}
const PBDParticle* pbd_unified_sim_get_particles(const PBDUnifiedSimulation* sim) {
    return sim->particles_.data();
}

static vec3 sdgBox2D(vec2 p, vec2 b )
{
    vec2 w = abs(p)-b;
    vec2 s = vec2(p.x<0.0?-1:1,p.y<0.0?-1:1);
    float g = max(w.x,w.y);
    vec2  q = max(w,vec2(0.0));
    float l = length(q);
	vec2 grad = s * ((g > 0.0) ? q / l : ((w.x > w.y) ? vec2(1, 0) : vec2(0, 1)));
	return vec3((g > 0.0) ? l : g, grad.x, grad.y);
}

int pbd_unified_sim_add_box_rigid_body(struct PBDUnifiedSimulation* sim, int size_x,
										int size_y, vec2 pos, float rot_angle, float density) {

	assert(size_x > 0 && size_y > 0);

	sim->rigid_bodies_.push_back(PBDRigidBody{
		.x = pos,
		.angle = rot_angle,
		.angle0 = rot_angle,
        .mu_s = 0.5f,
        .mu_k = 0.3f,
        .e = 0.5f,
        .Iinv = 0.0f,
		.start_pdata_idx = -1,
		.num_part = size_x * size_y,
		.flags = 0,
	});
    const int rb_idx = (int)(sim->rigid_bodies_.size() - 1);
    PBDRigidBody& rb = sim->rigid_bodies_[rb_idx];

	const float r = sim->particle_r_;
    const vec2 rb_size = 2.0f * r * vec2(size_x, size_y);
	const vec2 center = 0.5f * rb_size;

    int start_idx = -1;
    for(int y=0;y<size_y;++y) {
        for(int x=0;x<size_x;++x) {

			vec2 x0 = (vec2(r) + vec2(x, y) * 2 * r) - center;
			// in local coords (center is 0,0 no rotation, etc)
			vec3 dist_grad = sdgBox2D(x0, 0.5f * rb_size*vec2(1,0.98f));

            x0 = rotate2(rot_angle) * x0;
			int idx = pbd_unified_sim_add_particle(sim, pos + x0, density);

            // velocity pass does not work with rigid bodies for now... 
            // probably due to shape matching... only if all particles had collision
            // so have to somehow pass velocity to other particles, but to which ones?
            pbd_unified_sim_particle_set_params(sim, idx, rb.mu_s, rb.mu_k, rb.e);

            vec2 grad_norm = normalize(dist_grad.yz());
			bool b_is_boundary = y == 0 || x == 0 || y == size_y - 1 || x == size_x - 1;
			int data_idx = pbd_unified_sim_add_rb_particle_data(
				sim, x0, dist_grad.x, grad_norm, idx, b_is_boundary, density);

            sim->particles_[idx].flags |= PBDParticleFlags::kRigidBody;
            sim->particles_[idx].phase = rb_idx;
            sim->particles_[idx].rb_data_idx = data_idx;

			if(x==0 && y==0) {
                start_idx = data_idx;
            }
        }
    }

    rb.start_pdata_idx = start_idx;

    // calc inertia tensor
    const float mass = (2 * sim->particle_r_ * 2 * sim->particle_r_ * density);
    const float rb_mass = mass * size_x * size_y;
	const float I = (rb_mass / 12.0f) * lengthSqr(rb_size);
	rb.Iinv = 1.0f/I;

    sim->shape_matching_c_.push_back(ShapeMatchingConstraint{rb_idx});

    return rb_idx;
}

void debug_print_lattice(const PBDBodyLatticeData* lattice, int sx, int sy) {
#if 1
    for(i32 y=0;y<sy;++y) {
        for(i32 x=0;x<sx;++x) {
			const i32 i = x + y * sx;

#if INCLUDE_DIAG_NEIGHBOURS
			printf("%c%c%c%c[%c]%c%c%c%c ", 
#else
			printf("%c%c[%c]%c%c ", 
#endif
                   lattice[i].nflags & kL ? '<' : ' ',
				   lattice[i].nflags & kT ? '^' : ' ', 
#if INCLUDE_DIAG_NEIGHBOURS
                   lattice[i].nflags & kLT ? '\\' : ' ',
				   lattice[i].nflags & kLB ? '/' : ' ', 
#endif
                   lattice[i].nflags & kExists ? 'X' : 'O',
#if INCLUDE_DIAG_NEIGHBOURS
                   lattice[i].nflags & kRT ? '/' : ' ',
				   lattice[i].nflags & kRB ? '\\' : ' ', 
#endif
				   lattice[i].nflags & kB ? 'v' : ' ', 
                   lattice[i].nflags & kR ? '>' : ' ');
		}
        puts("\n");
    }
#endif
}

template<int SY>
static PBDRegion calc_square_region(int x, int y, int sx, int w, const u8 (*flags)[SY],
                                const uint32_t (*l2i)[SY], std::vector<uint32_t>& region_pidx, uint8_t mask) {

	const uint32_t s = (uint32_t)region_pidx.size();
	for (int j = -w; j <= w; ++j) {
		const int cy = y + j;
		for (int i = -w; i <= w; ++i) {
			const int cx = x + i;
            // TODO: move cy check up
			if (cx >= 0 && cx < sx && cy >= 0 && cy < SY && (flags[cy][cx]&mask)) {
				region_pidx.push_back(l2i[cy][cx]);
			}
		}
	}
	uint32_t count = (uint32_t)region_pidx.size() - s;
	return PBDRegion{s, count};
}

static PBDRegion calc_region(i32 x, i32 y, int sx, int sy, u32 w, PBDBodyLatticeData* lattice,
							 std::vector<u32>& region_pidx) {

	struct ptuple { 
        u32 w;
        int x, y;
        int sx;
        u16 nf;
        bool is_valid(i32 sy, u32 ww) const {
	        return x >= 0 && x < sx && y >= 0 && y < sy && w <= ww;
        }
        u32 i() const { return x + sx*y; }
    };

	assert(lattice[x + sx * y].nflags & kExists);

	const uint32_t s = (uint32_t)region_pidx.size();

    std::queue<ptuple> idxs;

    idxs.push(ptuple{0, x, y, sx});
    region_pidx.push_back(lattice[x + sx*y].particle_idx);

	lattice[x + sx * y].nflags |= kVisited;

	while(!idxs.empty()) {
        ptuple c = idxs.front();
        idxs.pop();
		if (c.w == w)
            continue;

        const int i = c.x + c.y*sx;
        const u32 cflags = lattice[i].nflags;

		ptuple nn[] = {
			{c.w + 1, c.x + 1, c.y, sx, kR}, {c.w + 1, c.x - 1, c.y, sx, kL},
			{c.w + 1, c.x, c.y - 1, sx, kT}, {c.w + 1, c.x, c.y + 1, sx, kB},
#if INCLUDE_DIAG_NEIGHBOURS
			{c.w + 1, c.x + 1, c.y + 1, sx, kRB}, {c.w + 1, c.x - 1, c.y - 1, sx, kLT},
			{c.w + 1, c.x + 1, c.y - 1, sx, kRT}, {c.w + 1, c.x - 1, c.y + 1, sx, kLB},
#endif
		};

        for (const ptuple& n: nn) {
			if (n.is_valid(sy, w) && (cflags & n.nf) && !(lattice[n.i()].nflags & kVisited)) {
				idxs.push(n);
				lattice[n.i()].nflags |= kVisited;
				region_pidx.push_back(lattice[n.i()].particle_idx);
			}
		}
	}
    // clear kVisited flags
	for (int dy = -(int)w; dy <= (int)w; ++dy) {
		const i32 cy = (int)y + dy;
		for (int dx = -(int)w; dx <= (int)w; ++dx) {
			const int cx = x + dx;
            const int i = cx + cy*sx;
			if (cx >= 0 && cx < sx && cy >= 0 && cy < sy) {
                lattice[i].nflags &= ~kVisited;
            }
        }
    }

	uint32_t count = (uint32_t)region_pidx.size() - s;
	return PBDRegion{s, count};
}

int pbd_unified_sim_add_box_soft_body(struct PBDUnifiedSimulation* sim, int size_x,
										int size_y, vec2 pos, float rot_angle, float density, uint32_t w, float alpha) {
    const constexpr int BOX_MAX_SIZE = 32;
	assert(size_x > 0 && size_y > 0);
	assert(size_x <= BOX_MAX_SIZE && size_y <= BOX_MAX_SIZE);

	const int sb_idx = sim->soft_bodies_.insert(PBDSoftBody{
		.base =
			PBDRigidBody{
				.x = pos,
				.angle = rot_angle,
				.angle0 = rot_angle,
				.mu_s = 0.5f,
				.mu_k = 0.3f,
				.e = 0.5f,
				.Iinv = 0.0f,
				.start_pdata_idx = -1,
				.num_part = size_x * size_y,
				.flags = 0,
			},
		.w = w,
        .alpha = alpha,
		.start_region_idx = kPBDInvalidIndex,
		.num_regions = 0,
        .parent_breakable_idx = kPBDInvalidIndex,
        .next_soft_body_idx = kPBDInvalidIndex,
        .island = -1,
	});
    sim->sb_idxs_.push_back(sb_idx);
    PBDSoftBody& sb = sim->soft_bodies_[sb_idx];

	const float r = sim->particle_r_;
    const vec2 sb_size = 2.0f * r * vec2(size_x, size_y);
	const vec2 center = 0.5f * sb_size;
    
    uint32_t lattice2idx[BOX_MAX_SIZE][BOX_MAX_SIZE] = {{0}};
    u8 flags[BOX_MAX_SIZE][BOX_MAX_SIZE] = {{0}};

    int start_idx = -1;
    for(int y=0;y<size_y;++y) {
        for(int x=0;x<size_x;++x) {

			vec2 x0 = (vec2(r) + vec2(x, y) * 2 * r) - center;
			// in local coords (center is 0,0 no rotation, etc)
            // !NB: why 0.98?
			vec3 dist_grad = sdgBox2D(x0, 0.5f * sb_size*vec2(1,0.98f));

            x0 = rotate2(rot_angle) * x0;
			int idx = pbd_unified_sim_add_particle(sim, pos + x0, density);

            // velocity pass does not work with rigid bodies for now... 
            // probably due to shape matching... only if all particles had collision
            // so have to somehow pass velocity to other particles, but to which ones?
            pbd_unified_sim_particle_set_params(sim, idx, sb.base.mu_s, sb.base.mu_k, sb.base.e);

            vec2 grad_norm = normalize(dist_grad.yz());
			bool b_is_boundary = y == 0 || x == 0 || y == size_y - 1 || x == size_x - 1;
			int data_idx = pbd_unified_sim_add_sb_particle_data(
				sim, x0, dist_grad.x, grad_norm, idx, b_is_boundary, density, x, y);

            sim->particles_[idx].flags |= PBDParticleFlags::kSoftBody;
            sim->particles_[idx].phase = sb_idx;
            sim->particles_[idx].sb_data_idx = data_idx;

			if(x==0 && y==0) {
                start_idx = data_idx;
            }

            lattice2idx[y][x] = idx;
            flags[y][x] = 1; // mark that particle exists here
        }
    }

    sb.base.start_pdata_idx = start_idx;

    // calc inertia tensor
    const float mass = (2 * sim->particle_r_ * 2 * sim->particle_r_ * density);
    const float rb_mass = mass * size_x * size_y;
	const float I = (rb_mass / 12.0f) * lengthSqr(sb_size);
	sb.base.Iinv = 1.0f/I;

    // calculate region data
    std::vector<uint32_t> region_pidx;
    std::vector<PBDRegion> regions;
    for(int y=0;y<size_y;++y) {
        for(int x=0;x<size_x;++x) {
            PBDRegion reg = calc_square_region(x, y, size_x, sb.w, flags, lattice2idx, region_pidx, 1);
            auto s_it = region_pidx.begin() + reg.start_part_idx;
            auto e_it = region_pidx.begin() + reg.start_part_idx + reg.num_part;
			std::sort(s_it, e_it);
			// TODO: also maybe remove regions which are not bringin any new information,
			// that is ones that are completely contained inside other region
			bool b_is_unique = true;
            for(const PBDRegion& ri : regions) {
                if(ri.num_part != reg.num_part) {
                    continue;
				}

                bool b_equal = true;
				for(uint32_t i=0;i<reg.num_part;++i) {
                    if(region_pidx[ri.start_part_idx + i]!= region_pidx[reg.start_part_idx + i]) {
                        b_equal = false;
                        break;
                    }
                }
                if(b_equal) {
                    b_is_unique = false;
                    break;
                }
            }

            if(b_is_unique) {
                const uint32_t off = reg.start_part_idx;
                reg.mass = reg.num_part;
                vec2 cm0 = vec2(0);
                for(uint32_t i=0;i<reg.num_part;++i) {
                    const uint32_t sb_data_idx = sim->particles_[region_pidx[i + off]].sb_data_idx;
                    const PBDSoftBodyParticleData& sd = sim->sb_particles_data_[sb_data_idx];
                    cm0 += sd.base.x0;
                }
                cm0 /= reg.num_part;
                reg.cm0 = cm0;
			    regions.push_back(reg);
            } else {
                region_pidx.erase(s_it, e_it);
            }
		}
    }

    // add new regions' particle indices
    uint32_t start_sb_p = (uint32_t)sim->sb_particles_per_region_.size();
    sim->sb_particles_per_region_.insert(sim->sb_particles_per_region_.end(), region_pidx.begin(), region_pidx.end());
    for(int i=0;i<(int)regions.size();++i) {
        regions[i].start_part_idx += start_sb_p;
    }
    // add regions themselves
    uint32_t start_sb_reg = (uint32_t)sim->sb_regions_.size();
    sim->sb_regions_.insert(sim->sb_regions_.end(), regions.begin(), regions.end());
	sb.start_region_idx = start_sb_reg;
    sb.num_regions = (uint32_t)regions.size();

    // for each particle in soft body
    for(int i=0;i<sb.base.num_part;++i) {
        PBDSoftBodyParticleData& pd = sim->sb_particles_data_[sb.base.start_pdata_idx + i];
		pd.num_regions = 0;
        pd.start_region_idx = -1;
		uint32_t cur_part_idx = pd.base.index;
        // check which regions contain it 
        for(int ri = 0; ri < (int)regions.size(); ++ri) {
            for(uint32_t pi = 0; pi<regions[ri].num_part; ++pi) {
                uint32_t rp = sim->sb_particles_per_region_[regions[ri].start_part_idx + pi];
                // and add if it is contained
                if(rp == cur_part_idx) {
                    if(0 == pd.num_regions) { 
                        pd.start_region_idx = (uint32_t)sim->sb_regions_per_particle_.size();
                    }
                    pd.num_regions++;
                    sim->sb_regions_per_particle_.push_back(start_sb_reg + ri);
                    break;
                }
            }
        }
    }

    sim->soft_shape_matching_c_.push_back(SoftShapeMatchingConstraint{sb_idx});

    return sb_idx;
}

void calc_regions_data(const u32 sx, const u32 sy, PBDBodyLatticeData* lattice, int island, u32 w,
					  const PBDUnifiedSimulation* sim, std::vector<uint32_t>& region_pidx,
					  std::vector<PBDRegion>& regions) {

	region_pidx.clear();
    regions.clear();

	for (u32 y = 0; y < sy; ++y) {
		for (u32 x = 0; x < sx; ++x) {

	        if (0 == (lattice[x + sx * y].nflags & kExists))
                continue;

	        if (island>0 && lattice[x + sx * y].island != island)
                continue;

			PBDRegion reg = calc_region((int)x, (int)y, sx, sy, w, lattice, region_pidx);
            if(reg.num_part == 0)
                continue;

            auto s_it = region_pidx.begin() + reg.start_part_idx;
            auto e_it = region_pidx.begin() + reg.start_part_idx + reg.num_part;
			std::sort(s_it, e_it);
			// TODO: also maybe remove regions which are not bringing any new information,
			// that is ones that are completely contained inside other region
			bool b_is_unique = true;
            for(const PBDRegion& ri : regions) {
                if(ri.num_part != reg.num_part) {
                    continue;
				}

                bool b_equal = true;
				for(uint32_t i=0;i<reg.num_part;++i) {
                    if(region_pidx[ri.start_part_idx + i]!= region_pidx[reg.start_part_idx + i]) {
                        b_equal = false;
                        break;
                    }
                }
                if(b_equal) {
                    b_is_unique = false;
                    break;
                }
            }

            if(b_is_unique) {
                // calc com
                const uint32_t off = reg.start_part_idx;
                reg.mass = reg.num_part;
                vec2 cm0 = vec2(0);
                for(uint32_t i=0;i<reg.num_part;++i) {
                    assert(i + off < region_pidx.size());
                    assert(region_pidx[i + off] < sim->particles_.size());
                    const uint32_t sb_data_idx = sim->particles_[region_pidx[i + off]].sb_data_idx;
                    assert(sb_data_idx < sim->sb_particles_data_.size());
                    const PBDSoftBodyParticleData& sd = sim->sb_particles_data_[sb_data_idx];
                    cm0 += sd.base.x0;
                }
                cm0 /= reg.num_part;
                reg.cm0 = cm0;

			    regions.push_back(reg);
            } else {
                region_pidx.erase(s_it, e_it);
            }
		}
	}
}

// index of island is starting from 1
// returns: number of islands
u32 enumerate_islands(i32 sx, i32 sy, PBDBodyLatticeData* lattice) {
    assert(sx>0 && sy>0);

	struct ptuple { 
        int x, y, sx;
        u16 nf;
        bool is_valid(i32 sy) const { return x >= 0 && x < sx && y >= 0 && y < sy; }
        u32 i() const { return x + sx*y; }
    };

    // get number of occupied particles
    // and also clear island lattice
    int num_occupied = 0;
    for(i32 i=0;i<sx*sy;i++) {
        num_occupied += (lattice[i].nflags&kExists)?1:0;
        lattice[i].island = 0;
    }

    u32 num_islands = 0;
    int num_visited = 0;
	int cur_island = 1;

	while (num_visited != num_occupied) {

		// find find first not visited
		int start_idx = -1;
		for (i32 i = 0; i < sx * sy; i++) {
			if ((lattice[i].nflags & kExists) && lattice[i].island == 0) {
				start_idx = (int)i;
				break;
			}
		}
        assert(start_idx!=-1);

		std::queue<ptuple> idxs;
		idxs.push(ptuple{start_idx % sx, start_idx / sx, sx});
		lattice[start_idx].island = cur_island;
        num_visited++;

		while (!idxs.empty()) {
			ptuple c = idxs.front();
			idxs.pop();

			const int i = c.x + c.y * sx;
			const u32 cflags = lattice[i].nflags;

			ptuple nn[] = {
				{c.x + 1, c.y, sx, kR}, {c.x - 1, c.y, sx, kL},
				{c.x, c.y - 1, sx, kT}, {c.x, c.y + 1, sx, kB},
#if INCLUDE_DIAG_NEIGHBOURS
    			{c.x + 1, c.y + 1, sx, kRB}, {c.x - 1, c.y - 1, sx, kLT},
    			{c.x + 1, c.y - 1, sx, kRT}, {c.x - 1, c.y + 1, sx, kLB},
#endif
			};

			for (const ptuple& n : nn) {
				if (n.is_valid(sy) && (cflags & n.nf) && (lattice[n.i()].island == 0)) {
					idxs.push(n);
					lattice[n.i()].island = cur_island;
					num_visited++;
				}
			}
		}
        
        num_islands++;
        cur_island++;

		for (i32 y = 0; y < sy; ++y) {
			for (i32 x = 0; x < sx; ++x) {
				printf("%d ", lattice[x + sx * y].island);
			}
			puts("\n");
		}
		puts("=======\n");
	}

	return num_islands;

}

u32 get_num_particle_in_island(i32 sx, i32 sy, PBDBodyLatticeData* lattice, int search_island) {

    int c=0;
	for (i32 i = 0; i < sx * sy; i++) {
		if(lattice[i].island == search_island)
            c++;
	}
    return c;
}

// similar to enumerate_islands but will only enumerate island passed as search_island, in
// case this island is broken into parts, it will create new islands with IDs starting
// from start_island_id
int enumerate_island(i32 sx, i32 sy, PBDBodyLatticeData* lattice, int search_island, int* start_island_id) {
    assert(sx>0 && sy>0);
    assert(start_island_id);

	struct ptuple { 
        int x, y, sx;
        u16 nf;
        bool is_valid(i32 sy) const { return x >= 0 && x < sx && y >= 0 && y < sy; }
        u32 i() const { return x + sx*y; }
    };

	// get number of occupied particles
	// and also clear island particles which are equal to search_island
	// NOTE: we call this function because we know that search_island is now split in
	// two(or more) parts, so part will have same id (search_island) and another will have
	// another id... which one? it should be current max island id + 1
	int num_occupied = 0;
    int max_island_id = 0;
	for (i32 i = 0; i < sx * sy; i++) {
        max_island_id = max_island_id > lattice[i].island ? max_island_id : lattice[i].island;
		bool exists = lattice[i].nflags & kExists;
		bool our_island = lattice[i].island == search_island;
		if(exists && our_island) {
            num_occupied++;
		    lattice[i].island = 0;
        }
	}

    int num_islands = 0;
    int num_visited = 0;
	int cur_island = max_island_id + 1;
    *start_island_id = cur_island;

	while (num_visited != num_occupied) {

		// find find first not visited
		int start_idx = -1;
		for (i32 i = 0; i < sx * sy; i++) {
			if ((lattice[i].nflags & kExists) && lattice[i].island == 0) {
				start_idx = (int)i;
				break;
			}
		}
        assert(start_idx!=-1);

		std::queue<ptuple> idxs;
		idxs.push(ptuple{start_idx % sx, start_idx / sx, sx});
		lattice[start_idx].island = cur_island;
        num_visited++;

		while (!idxs.empty()) {
			ptuple c = idxs.front();
			idxs.pop();

			const int i = c.x + c.y * sx;
			const u32 cflags = lattice[i].nflags;

			ptuple nn[] = {
				{c.x + 1, c.y, sx, kR}, {c.x - 1, c.y, sx, kL},
				{c.x, c.y - 1, sx, kT}, {c.x, c.y + 1, sx, kB},
#if INCLUDE_DIAG_NEIGHBOURS
    			{c.x + 1, c.y + 1, sx, kRB}, {c.x - 1, c.y - 1, sx, kLT},
    			{c.x + 1, c.y - 1, sx, kRT}, {c.x - 1, c.y + 1, sx, kLB},
#endif
			};

			for (const ptuple& n : nn) {
				if (n.is_valid(sy) && (cflags & n.nf) && (lattice[n.i()].island == 0)) {
					idxs.push(n);
					lattice[n.i()].island = cur_island;
					num_visited++;
				}
			}
		}
        
        num_islands++;
        cur_island++;

		for (i32 y = 0; y < sy; ++y) {
			for (i32 x = 0; x < sx; ++x) {
				printf("%d ", lattice[x + sx * y].island);
			}
			puts("\n");
		}
		puts("=======\n");
	}

    // otherwise why whould we call this function?
    assert(num_islands >= 2);
	return num_islands;

}


int construct_regions_for_body(PBDSoftBody& sb, u32 w, u32 sx, u32 sy, PBDBodyLatticeData* lattice, PBDUnifiedSimulation* sim) {

    assert(sb.start_region_idx == kPBDInvalidIndex);
    assert((sb.num_regions == kPBDInvalidIndex) || (sb.num_regions == 0));

    std::vector<uint32_t> region_pidx;
    std::vector<PBDRegion> regions;
    calc_regions_data(sx, sy, lattice, sb.island, w, sim, region_pidx, regions);

    // add new regions' particle indices
    uint32_t start_sb_p = (uint32_t)sim->sb_particles_per_region_.size();
    sim->sb_particles_per_region_.insert(sim->sb_particles_per_region_.end(), region_pidx.begin(), region_pidx.end());
    for(int i=0;i<(int)regions.size();++i) {
        regions[i].start_part_idx += start_sb_p;
    }
    // add regions themselves
    uint32_t start_sb_reg = (uint32_t)sim->sb_regions_.size();
    sim->sb_regions_.insert(sim->sb_regions_.end(), regions.begin(), regions.end());
	sb.start_region_idx = start_sb_reg;
    sb.num_regions = (uint32_t)regions.size();

    // for each particle in soft body
    for(int i=0;i<sb.base.num_part;++i) {
        PBDSoftBodyParticleData& pd = sim->sb_particles_data_[sb.base.start_pdata_idx + i];
		pd.num_regions = 0;
        pd.start_region_idx = kPBDInvalidIndex;
		uint32_t cur_part_idx = pd.base.index;
        // check which regions contain it 
        for(int ri = 0; ri < (int)regions.size(); ++ri) {
            for(uint32_t pi = 0; pi<regions[ri].num_part; ++pi) {
                uint32_t rp = sim->sb_particles_per_region_[regions[ri].start_part_idx + pi];
                // and add if it is contained
                if(rp == cur_part_idx) {
                    if(0 == pd.num_regions) { 
                        pd.start_region_idx = (uint32_t)sim->sb_regions_per_particle_.size();
                    }
                    pd.num_regions++;
                    sim->sb_regions_per_particle_.push_back(start_sb_reg + ri);
                    break;
                }
            }
        }
    }

    return (int)regions.size();
}

int remove_all_regions(PBDSoftBody& sb, PBDUnifiedSimulation* sim) {
	// !NB: nothing acually was removed! we just cleared all data to catch possible errors
	// of use-after-remove
	// TODO: later we can add all free indices to corresponding free lists (when we'll use
	// them), for now just keep memory leaks becaue we cannot shift data in arrays.

    int num_removed = 0;
	for (u32 ri = sb.start_region_idx; ri < sb.start_region_idx + sb.num_regions; ri++) {

		PBDRegion& reg = sim->sb_regions_[ri];
		uint32_t* const reg_pidxs = sim->sb_particles_per_region_.data();

		// clear all particle indices for this region
		for (u32 ppri = reg.start_part_idx; ppri < reg.start_part_idx + reg.num_part; ++ppri) {
            u32 pi = reg_pidxs[ppri];
			reg_pidxs[ppri] = kPBDInvalidIndex;
            const u32 pdata_idx = sim->particles_[pi].sb_data_idx;

            // clear particle data
			PBDSoftBodyParticleData& data = sim->sb_particles_data_[pdata_idx];
            // this particle data has been cleared by prev region
            if(data.start_region_idx==kPBDInvalidIndex) {
                continue;
            }
			// clear all region idices for this particle
			for (u32 pri = data.start_region_idx; pri < data.start_region_idx + data.num_regions; ++pri) {
				sim->sb_regions_per_particle_[pri] = kPBDInvalidIndex;
			}
			data.start_region_idx = kPBDInvalidIndex;
			data.num_regions = kPBDInvalidIndex;
		}
		reg.start_part_idx = kPBDInvalidIndex;
		reg.num_part = kPBDInvalidIndex;
        num_removed++;
	}

    // this is necessary in any case as sb is still valid just does not have any regions
	sb.start_region_idx = kPBDInvalidIndex;
    sb.num_regions = kPBDInvalidIndex;

	return num_removed;
}

int remove_all_regions(PBDBreakableSoftBody& bb, PBDUnifiedSimulation* sim) {

    int num_removed = 0;
	u32 sb_idx = bb.first_soft_body;
	while(sb_idx != kPBDInvalidIndex) {
		PBDSoftBody& sb = sim->soft_bodies_.at(sb_idx);
        num_removed += remove_all_regions(sb, sim);
        sb_idx = sb.next_soft_body_idx;
        sb.next_soft_body_idx = kPBDInvalidIndex;
	} 

    return num_removed;
}

void pbd_util_build_connectivity_lattice(u32 sx, u32 sy, const u8* blueprint,
										 PBDBodyLatticeData* lattice) {

	// separate clear loop because, in the next one nflags is modified by other indices
    // NOTE: maybe have a separate "init" function for lattice data?
	for (u32 y = 0; y < sy; ++y) {
		for (u32 x = 0; x < sx; ++x) {
			lattice[x + y*sx].nflags = 0;
        }
    }

	for (u32 y = 0; y < sy; ++y) {
		for (u32 x = 0; x < sx; ++x) {

			const int i = y * sx + x;

			lattice[i].particle_idx = kPBDInvalidIndex;
			lattice[i].pdata_idx = kPBDInvalidIndex;
			lattice[i].x = x;
			lattice[i].y = y;
			lattice[i].island = 0;

			if (0 == blueprint[i]) {
				continue;
			}

			lattice[i].nflags |= kExists;
			if (x + 1 < sx && blueprint[i + 1]) {
				lattice[i].nflags |= kR;
				lattice[i + 1].nflags |= kL;
			}
			if (y + 1 < sy && blueprint[i + sx]) {
				lattice[i].nflags |= kB;
				lattice[i + sx].nflags |= kT;
			}
#if INCLUDE_DIAG_NEIGHBOURS
			if (x + 1 < sx && y + 1 < sy && blueprint[i + 1 + sx]) {
				lattice[i].nflags |= kRB;
				lattice[i + 1 + sx].nflags |= kLT;
			}
			if (x + 1 < sx && (int)y - 1 >= 0 && blueprint[i + 1 - sx]) {
				lattice[i].nflags |= kRT;
				lattice[i + 1 - sx].nflags |= kLB;
			}
#endif
		}
	}
}

PBDSoftBody init_soft_body(vec2 pos, u32 w, float alpha, int island) {
    return PBDSoftBody{
		.base =
			PBDRigidBody{
				.x = pos,
				.angle = 0,
				.angle0 = 0,
				.mu_s = 0.5f,
				.mu_k = 0.3f,
				.e = 0.05f, // to not jump too much
				.Iinv = 0.0f,
				.start_pdata_idx = -1,
				.num_part = 0,
				.flags = 0,
			},
		.w = w,
        .alpha = alpha,
		.start_region_idx = kPBDInvalidIndex,
		.num_regions = 0,
        .parent_breakable_idx = kPBDInvalidIndex,
        .next_soft_body_idx = kPBDInvalidIndex,
        .island = island,
	};
}

int pbd_unified_sim_add_soft_body(struct PBDUnifiedSimulation* sim, uint32_t size_x,
								  uint32_t size_y, PBDBodyLatticeData* lattice, int island, vec2 pos,
								  float density, u32 w, float alpha) {

	const int sb_idx = sim->soft_bodies_.insert(init_soft_body(pos, w, alpha, island));
    sim->sb_idxs_.push_back(sb_idx);
    PBDSoftBody& sb = sim->soft_bodies_[sb_idx];

	const float r = sim->particle_r_;
    const vec2 sb_size = 2.0f * r * vec2((float)size_x, (float)size_y);
	const vec2 center = 0.5f * sb_size;

    u32 start_idx = kPBDInvalidIndex;
    u32 num_part = 0;
    for(uint32_t y=0;y<size_y;++y) {
        for(uint32_t x=0;x<size_x;++x) {

			const uint32_t i = x + y * size_x;
			if (!(lattice[i].nflags & kExists) ) 
                continue;

			if (island > 0 && lattice[i].island != island)
                continue;

			vec2 x0 = (vec2(r) + vec2(x, y) * 2 * r) - center;
			// in local coords (center is 0,0 no rotation, etc)
			// !NB: why 0.98?
			// NOTE: can't use distance field for now as have no code to generate it yet,
			// so just check particle radius and hope we will have no tunneling...
			//vec3 dist_grad = sdgBox2D(x0, 0.5f * sb_size*vec2(1,0.98f));
            vec2 grad_norm(1,0); // can try to fetch neighbours though
            float dist = -r;

            //x0 = rotate2(rot_angle) * x0;
			int idx = pbd_unified_sim_add_particle(sim, pos + x0, density);

            // velocity pass does not work with rigid bodies for now... 
            // probably due to shape matching... only if all particles had collision
            // so have to somehow pass velocity to other particles, but to which ones?
            pbd_unified_sim_particle_set_params(sim, idx, sb.base.mu_s, sb.base.mu_k, sb.base.e);

            //vec2 grad_norm = normalize(dist_grad.yz());
			bool b_is_boundary = y == 0 || x == 0 || y == size_y - 1 || x == size_x - 1;
			int data_idx = pbd_unified_sim_add_sb_particle_data(
				sim, x0, dist, grad_norm, idx, b_is_boundary, density, x, y);

            sim->particles_[idx].flags |= PBDParticleFlags::kSoftBody;
            sim->particles_[idx].phase = sb_idx;
            sim->particles_[idx].sb_data_idx = data_idx;

			if(start_idx == kPBDInvalidIndex) {
                start_idx = data_idx;
            }
            num_part++;

            lattice[i].particle_idx = idx;
            lattice[i].pdata_idx = data_idx;
        }
    }

    sb.base.start_pdata_idx = start_idx;
    sb.base.num_part = num_part;

    debug_print_lattice(lattice, size_x, size_y);

    // calc inertia tensor
    const float mass = (2 * sim->particle_r_ * 2 * sim->particle_r_ * density);
    const float rb_mass = mass * size_x * size_y;
    // !NB: only correct for box
	const float I = (rb_mass / 12.0f) * lengthSqr(sb_size);
	sb.base.Iinv = 1.0f/I;

    construct_regions_for_body(sb, w, size_x, size_y, lattice, sim);

    sim->soft_shape_matching_c_.push_back(SoftShapeMatchingConstraint{sb_idx});

    return sb_idx;
}

void on_soft_body_break(u32 bb_idx, u32 sb_idx, PBDUnifiedSimulation* sim) {
    PBDBreakableSoftBody& bb = sim->breakable_bodies_.at(bb_idx);
    PBDSoftBody& sb = sim->soft_bodies_[sb_idx];
    
    const u32 lsx = bb.lattice_sx;
    const u32 lsy = bb.lattice_sy;

    PBDBodyLatticeData* lattice = &sim->bb_lattice_[bb.lattice_idx];
    assert(sb.island>0);
    int start_island_id=0;
	const int num_islands = enumerate_island(lsx, lsy, lattice, sb.island, &start_island_id);
    assert(start_island_id>0);
    assert(num_islands>=2);
    sb.island = start_island_id;

    // sort particles to be arranged by owning island
    const auto begin = sim->sb_particles_data_.begin() + sb.base.start_pdata_idx;
    const auto end = begin + sb.base.num_part;
	std::sort(begin, end, [lattice, lsx](auto& pd0 , auto& pd1) {
		return lattice[pd0.x + pd0.y * lsx].island < lattice[pd1.x + pd1.y * lsx].island;
	});

	// update pdata indices, because they were re-sorted
    for(int i=0;i<sb.base.num_part; ++i) {
		const PBDSoftBodyParticleData& sb_pdata =
			sim->sb_particles_data_[sb.base.start_pdata_idx + i];

		u32 pidx = sb_pdata.base.index;
        sim->particles_[pidx].sb_data_idx = sb.base.start_pdata_idx + i;

        int lidx = sb_pdata.x + sb_pdata.y*lsx;
        lattice[lidx].pdata_idx =  sb.base.start_pdata_idx + i;
    }

	{
		remove_all_regions(sb, sim);
		sb.island = start_island_id;
		sb.base.num_part = get_num_particle_in_island(lsx, lsy, lattice, sb.island);
		int num_regions = construct_regions_for_body(sb, sb.w, lsx, lsy, lattice, sim);
		assert(num_regions);
	}

    // add new sbodies to the head 
    u32 part_data_offset = sb.base.num_part;
    // start from 1 because we already processed one sb just above
	for(int i=1;i<num_islands; ++i) {
		const int cur_sb_idx = sim->soft_bodies_.insert(
			init_soft_body(sb.base.x, sb.w, sb.alpha, start_island_id + i));
        sim->sb_idxs_.push_back(cur_sb_idx);
		PBDSoftBody& cur_sb = sim->soft_bodies_[cur_sb_idx];
        cur_sb.base.start_pdata_idx = sb.base.start_pdata_idx + part_data_offset;
        cur_sb.base.num_part = get_num_particle_in_island(lsx, lsy, lattice, cur_sb.island);

		// maybe add soft bodies earlier and update phase in the same loop as updating
		// pdata indices
		for(int j=0;j<(int)cur_sb.base.num_part; ++j) {
			const PBDSoftBodyParticleData& sb_pdata =
				sim->sb_particles_data_[cur_sb.base.start_pdata_idx + j];
			sim->particles_[sb_pdata.base.index].phase = cur_sb_idx;
        }

	    int num_regions = construct_regions_for_body(cur_sb, cur_sb.w, lsx, lsy, lattice, sim);
        assert(num_regions);
        sim->soft_shape_matching_c_.push_back(SoftShapeMatchingConstraint{cur_sb_idx});

        // inject into bb
        cur_sb.parent_breakable_idx = bb_idx;
        cur_sb.next_soft_body_idx = bb.first_soft_body;
        bb.first_soft_body = cur_sb_idx;

        part_data_offset += cur_sb.base.num_part;
    }

    // update first sbody ptr in bb
}

int pbd_unified_sim_add_breakable_soft_body(struct PBDUnifiedSimulation* sim, u32 sx,
											u32 sy, const u8* blueprint, vec2 pos,
											float density, u32 w, float compliance, float fr_len, float fr_angle) {

    const u32 lattice_idx = (u32)sim->bb_lattice_.size();
    sim->bb_lattice_.resize(sim->bb_lattice_.size() + sx*sy);
    PBDBodyLatticeData* lattice = sim->bb_lattice_.data() + lattice_idx;

    pbd_util_build_connectivity_lattice(sx, sy, blueprint, lattice);

    u32 num_islands = enumerate_islands((int)sx, (int)sy, lattice);
    printf("num islands: %d\n", num_islands);
    for(u32 y=0;y<sy;++y) {
        for(u32 x=0;x<sx;++x) {
            printf("%d ", lattice[x + sx*y].island);
        }
        puts("\n");
    }
    assert(num_islands == 1);

	const PBDBreakableSoftBody bb = {
		.w = w,
		.alpha = compliance,
        .fracture_len = fr_len,
        .fracture_cos_angle = cosf(fr_angle),
        .lattice_sx = sx,
        .lattice_sy = sy,
        .lattice_idx = lattice_idx,
		.first_soft_body = kPBDInvalidIndex,
	};
	const i32 bb_idx = sim->breakable_bodies_.insert(bb);
	sim->break_checks_.push_back(bb_idx);

    u32 first_sb_idx = kPBDInvalidIndex;
    PBDSoftBody* prev_sb = nullptr;
    for(u32 i=0;i<num_islands;++i) {
	    const i32 sb_idx = pbd_unified_sim_add_soft_body(sim, sx, sy, lattice, (int)i+1, pos, density, w, compliance);
        assert(sb_idx>=0);
        if(i==0)
            first_sb_idx = (u32)sb_idx;
        if(prev_sb) {
            prev_sb->next_soft_body_idx = sb_idx;
        }
        prev_sb = &sim->soft_bodies_.at(sb_idx);
        sim->soft_bodies_[sb_idx].parent_breakable_idx = bb_idx;
    }

    assert(first_sb_idx != kPBDInvalidIndex);
    sim->breakable_bodies_.at(bb_idx).first_soft_body = first_sb_idx;

    return bb_idx;
}

void pbd_unified_sim_remove_breakable_soft_body(struct PBDUnifiedSimulation* sim, int id) {
    assert(id>=0 && sim->breakable_bodies_.size()>id);

    // TODO: remove soft bodies belonging to breakable
    assert(0);

    sim->breakable_bodies_.release(id);

    for(int i=0; i<(int)sim->break_checks_.size(); ++i) {
        if(sim->break_checks_[i] == id) {
            sim->break_checks_.erase(sim->break_checks_.begin() + (u32)i);
        }
    }
}

int pbd_unified_sim_add_fluid_model(struct PBDUnifiedSimulation* sim, float viscosity, float desired_rest_density) {

    float mass = 1;
    float density0 = 0;

    // calculate rest density for the mass of 1 
    vec2 center(0);
    for(int y=-3; y< 3;++y) {
        for(int x=-3; x< 3;++x) {
            vec2 pos = center + vec2(x,y)*(2*sim->particle_r_);
            density0 += mass * sim->W(center - pos);
        }
    }

    // figure out mass based on desired deisity
    float particle_mass = desired_rest_density / density0;

	sim->fluid_models_.push_back(PBDFluidModel{
        .density0_ = desired_rest_density,
        .mass_ = particle_mass,
		.viscosity_ = viscosity,
        .debug_color_ = get_debug_colour(),
	});

    return (int)sim->fluid_models_.size() - 1;
}

int pbd_unified_sim_add_fluid_particle(struct PBDUnifiedSimulation* sim, vec2 pos, int fluid_model_idx) {

    assert((int)sim->fluid_models_.size() > fluid_model_idx);
    const PBDFluidModel& fm = sim->fluid_models_[fluid_model_idx];

    sim->fluid_particles_data_.push_back(PBDFluidParicleRuntimeData());
    const int fluid_particle_data_idx = (int)sim->fluid_particles_data_.size() - 1;

    float inv_mass = 1.0f/fm.mass_;
	sim->particles_.emplace_back(PBDParticle{
		.x = pos,
		.v = vec2(0),
		.inv_mass = inv_mass,
        .flags = PBDParticleFlags::kFluid,
        .phase = fluid_model_idx,
        .fluid_data_idx = fluid_particle_data_idx,
	});

	sim->inv_scaled_mass_.push_back(inv_mass);
	sim->dp_.push_back(vec2(0));
	sim->num_constraints_.push_back(0);
	sim->x_pred_.push_back(vec2(0));

    sim->neigh_data_.neighbour_info_.push_back(Pair{0,0});

    const int added_idx = (int)(sim->particles_.size() - 1);
    sim->fluid_particle_idxs_.push_back(added_idx);

    return added_idx;
}

int pbd_unified_sim_add_distance_constraint(struct PBDUnifiedSimulation* sim, int p0_idx, int p1_idx, float dist) {
    int idx = sim->distance_c_.insert(DistanceConstraint{p0_idx, p1_idx, dist});
    sim->distance_c_idxs_.push_back(idx);
    return idx;
}

int pbd_unified_sim_add_distance2_constraint(struct PBDUnifiedSimulation* sim, int p0_idx, vec2 pos, float dist) {
    int idx = sim->distance2_c_.insert(DistanceConstraint2{p0_idx, pos, dist});
    sim->distance2_c_idxs_.push_back(idx);
    return idx;
}

void pbd_unified_sim_update_distance2_constraint(struct PBDUnifiedSimulation* sim, int c_idx, vec2* pos, float dist) {
    DistanceConstraint2& c = sim->distance2_c_.at(c_idx);
    c.dist = dist >=0.0f ? dist : c.dist;
    c.pos = pos ? *pos : c.pos;
}

// TODO: unify DistanceConstraint & DistanceConstraint2 into same struct and just store indices in different arrays?
bool pbd_unified_sim_remove_distance_constraint(struct PBDUnifiedSimulation* sim, int c_idx) {
    sim->distance_c_.release(c_idx);
    bool b_found = false;
    for(auto it=sim->distance_c_idxs_.begin();it!=sim->distance_c_idxs_.end(); ++it) {
        if(*it == c_idx) {
            b_found = true;
            sim->distance_c_idxs_.erase(it);
        }
    }
    assert(b_found);
    return true;
}

bool pbd_unified_sim_remove_distance2_constraint(struct PBDUnifiedSimulation* sim, int c_idx) {

    sim->distance2_c_.release(c_idx);
    bool b_found = false;
    for(auto it=sim->distance2_c_idxs_.begin();it!=sim->distance2_c_idxs_.end(); ++it) {
        if(*it == c_idx) {
            b_found = true;
            sim->distance2_c_idxs_.erase(it);
            break;
        }
    }
    assert(b_found);
    return true;
}

// assume all masses are similar
vec2 rb_calc_com(const PBDRigidBody& rb, const vec2* pred_x, const PBDRigidBodyParticleData* rb_data) {
    
    vec2 com = vec2(0,0);
    for(int di = rb.start_pdata_idx; di < rb.start_pdata_idx + rb.num_part; ++di) {
        int pi = rb_data[di].index;
        com = com + pred_x[pi];
    }
	com *= 1.0f / (float)rb.num_part;
    return com;
}

vec2 calc_com(const vec2* pred_x, const uint32_t* const indices, const uint32_t count) {
    
    vec2 com = vec2(0,0);
    for(uint32_t i = 0; i < count; ++i) {
        com = com + pred_x[indices[i]];
    }
	com *= 1.0f / (float)count;
    return com;
}

mat2 calculateQ(const mat2 m, float* angle) {
	float theta = atan2(m[1][0] - m[0][1], m[0][0] + m[1][1]);

    float s, c;
#if PLATFORM_WINDOWS
	s = sinf(theta);
	c = cosf(theta);
#else
    sincosf(theta, &s, &c);
#endif
    *angle = theta;
    return mat2(c, -s, s, c);
}

vec2 calc_friction(const vec2 path, vec2 n, float d, float mu_s, float mu_k) {

	vec2 dxp = path - dot(path, n) * n;
	const float ldxp = length(dxp);

    // see 3.5 in Detailed RB Simulation ... paper
	if (ldxp >= mu_s * d) {
        // dynamic friction
		dxp = dxp * min(mu_k * d / ldxp, 1.0f);
        if(VELOCITY_UPDATE) {
            // then dynamic friction is calculated in velocity update
            dxp = vec2(0);
        }
	}
    // else static friction is returned
    return dxp;
}

void add_contact(const vec2 path, vec2 n, float d, int i0, int i1, vec2 v0, vec2 v1,
				 float mu_k, PBDUnifiedSimulation* sim) {
#if VELOCITY_UPDATE 
    ContactInfo ci;
    ci.n = n;
    ci.idx1 = i0;
    ci.v1_pre = v0;
    ci.idx2 = i1;
    ci.v2_pre = v1;
    ci.mu_d = mu_k;
    ci.d_lambda_n = dot(path, ci.n);

    // needs check:
    //vec2 pos = sim->x_pred_[particle_idx];
	//ci.r1 = pos - cc.n*sim->particle_r_;
	//ci.r2 = pos - cc.n*cc.dist;
    sim->contacts_info_.push_back(ci);
#endif
}

// particle vs. geometry
vec2 process_friction(const vec2 path, const vec2 n, const float d, const int i0,
				   const PBDParticle& p0, float contact_mu_s, float contact_mu_k, PBDUnifiedSimulation* sim) {

	const float mu_s = 0.5f*(p0.mu_s + contact_mu_s);
    const float mu_k = 0.5f*(p0.mu_k + contact_mu_k);
    const vec2 fd = calc_friction(path, n, d, mu_s, mu_k);

    add_contact(path, n, d, i0, -1, p0.v, vec2(0), mu_k, sim);

	add_dbg_friction_info(i0, n, path, mu_s, mu_k,
						  sim->particles_[i0].x,
						  sim->x_pred_[i0], sim);
    return fd;
}

// particle vs. particle
vec2 process_friction(const vec2 path, const vec2 n, const float d, const int i0,
				   const int i1, const PBDParticle& p0, const PBDParticle& p1,
				   PBDUnifiedSimulation* sim) {

	const float mu_s = 0.5f*(p0.mu_s + p1.mu_s);
    const float mu_k = 0.5f*(p0.mu_k + p1.mu_k);
    const vec2 fd = calc_friction(path, n, d, mu_s, mu_k);
    add_contact(path, n, d, i0, i1, p0.v, p1.v, mu_k, sim);

    return fd;
}

void update_positions(uint32_t i0, uint32_t i1, vec2 dp0, vec2 dp1, PBDUnifiedSimulation* sim) {
	if (GAUSS_SEIDEL_UPDATE()) {
		sim->x_pred_[i0] += dp0;
		sim->x_pred_[i1] += dp1;
	} else {
		sim->dp_[i0] += dp0;
		sim->dp_[i1] += dp1;

		sim->num_constraints_[i0]++;
		sim->num_constraints_[i1]++;
	}
}

void update_position(uint32_t i0, vec2 dp0, PBDUnifiedSimulation* sim) {
    if(GAUSS_SEIDEL_UPDATE()) {
		sim->x_pred_[i0] += dp0;
	} else {
		sim->dp_[i0] += dp0;
		sim->num_constraints_[i0]++;
	}
}

void update_collision(uint32_t i0, vec2 dp0, PBDUnifiedSimulation* sim) {
    if(GAUSS_SEIDEL_STATIC_COLLISION()) {
    //if(GAUSS_SEIDEL_UPDATE()) {
		sim->x_pred_[i0] += dp0;
	} else {
		sim->dp_[i0] += dp0;
		sim->num_constraints_[i0]++;
	}
}

void solve_distance_c(const DistanceConstraint& c, PBDUnifiedSimulation* sim, /*const*/ vec2* x_pred) {

	assert(c.idx0 >= 0 && c.idx0 < (int32_t)sim->particles_.size());
	assert(c.idx1 >= 0 && c.idx1 < (int32_t)sim->particles_.size());

    const PBDParticle& p0 = sim->particles_[c.idx0];
    const PBDParticle& p1 = sim->particles_[c.idx1];

	assert(p0.flags & PBDParticleFlags::kSolid);
	assert(p1.flags & PBDParticleFlags::kSolid);

    const float len01 = length(x_pred[c.idx0] - x_pred[c.idx1]);

	// len can be 0 often if initially particles are inside collision geometry and then get
	// adjusted same distance outwards
	vec2 gradC = len01==0 ? vec2(0) : (x_pred[c.idx0] - x_pred[c.idx1]) / len01;
    const float inv_mass_i = p0.inv_mass;
    const float inv_mass_j = p1.inv_mass;

	// TODO: add it to distance constraint instead?
	// we use compliance a !=0 to avoid selfpenetration when initially rope is inside
	// collision boundary (which usually should not happen, but one of our scenes has such
	// configuration on purpose)
	float h = 0.016/5;// dt/sim_iter_count
    float a = 0.000000001f; // compliance
	float a_ = a / (h * h);

	float C = len01 - c.dist;
    // can be useful
    //if(C<0) return;

    float s = C / (inv_mass_i + inv_mass_j + a_);
    vec2 dp0 = -s * inv_mass_i * gradC;
    vec2 dp1 = +s * inv_mass_j * gradC;

    update_positions(c.idx0, c.idx1, dp0, dp1, sim);
}

void solve_distance2_c(const DistanceConstraint2& c, PBDUnifiedSimulation* sim, const vec2* x_pred) {

	assert(c.idx0 >= 0 && c.idx0 < (int32_t)sim->particles_.size());

    const PBDParticle& p0 = sim->particles_[c.idx0];

	assert(p0.flags & PBDParticleFlags::kSolid);

    const float len01 = length(x_pred[c.idx0] - c.pos);

	vec2 gradC = (x_pred[c.idx0] - c.pos) / len01;
    const float inv_mass_i = p0.inv_mass;
    const float inv_mass_j = 0;

	// TODO: add it to distance constraint instead?
	// we use compliance a !=0 to avoid selfpenetration when initially rope is inside
	// collision boundary (which usually should not happen, but one of our scenes has such
	// configuration on purpose)
	float h = 0.016/5;// dt/sim_iter_count
    float a = 0.000000001f; // compliance
	float a_ = a / (h * h);

    float C = len01 - c.dist;
    // can be useful
    //if(C<0) return;

    float s = C / (inv_mass_i + inv_mass_j + a_);
    vec2 dp0 = -s * inv_mass_i * gradC;

    update_position(c.idx0, dp0, sim);
}

void solve_shape_matching_c(const ShapeMatchingConstraint& c,
						  const PBDRigidBodyParticleData* rb_data,
						  PBDUnifiedSimulation* sim) {

	const int rb_index = c.rb_index;
    PBDRigidBody& rb = sim->rigid_bodies_[rb_index];
    const vec2 com = rb_calc_com(rb, sim->x_pred_.data(), sim->rb_particles_data_.data());

    mat2 A(0,0,0,0);
    vec2* x_pred = sim->x_pred_.data();
	for (int di = rb.start_pdata_idx; di < rb.start_pdata_idx + rb.num_part; ++di) {
        int pi = rb_data[di].index;
        A = A + outer(x_pred[pi] - com, rb_data[di].x0);
	}

    float delta_angle;
    mat2 Q = calculateQ(A, &delta_angle);
#if 0 
    if(fabsf(delta_angle)< 1e-6) {
        delta_angle = 0.0f;
    }
#endif

    rb.angle = rb.angle0 + delta_angle;
    // test
    mat2 rot_m = rotate2(rb.angle);
    (void)rot_m;

    static float alpha = 1.0f;

	for (int di = rb.start_pdata_idx; di < rb.start_pdata_idx + rb.num_part; ++di) {
        int pi = rb_data[di].index;
        vec2 dx = vec2(0.0f);
        //if(delta_angle!=0.0f) {
            dx = (Q * rb_data[di].x0 + com) - x_pred[pi];
        //} else {
        //    dx = (rb_data[di].x0 + com) - x_pred[pi];
        //}

        //if(fabsf(dx.x)>1e-6 || fabsf(dx.y)>1e-6) {
            update_position(pi, alpha*dx, sim);
        //}
	}
}

void solve_soft_shape_matching_c(const SoftShapeMatchingConstraint& c,
						  const PBDSoftBodyParticleData* sb_data,
						  PBDUnifiedSimulation* sim) {

	const int sb_index = c.sb_index;
    const PBDSoftBody& sb = sim->soft_bodies_[sb_index];
    const PBDRigidBody& rb = sb.base;

	const vec2* const x_pred = sim->x_pred_.data();
	const uint32_t off = sb.start_region_idx;
	for (uint32_t ri = 0; ri < sb.num_regions; ri++) {
		PBDRegion& reg = sim->sb_regions_[off + ri];
        assert(reg.start_part_idx + reg.num_part <= sim->sb_particles_per_region_.size());
        const uint32_t* const reg_pidxs = &sim->sb_particles_per_region_[reg.start_part_idx];
	    const vec2 com_r = calc_com(sim->x_pred_.data(), reg_pidxs, reg.num_part);

        mat2 Ar(0,0,0,0);
        for (uint32_t i = 0; i < reg.num_part; ++i) {
            int pi = reg_pidxs[i];
            int di = sim->particles_[pi].sb_data_idx;
            Ar = Ar + outer(x_pred[pi] - com_r, sb_data[di].base.x0);
        }

        float delta_angle;
        mat2 Q = calculateQ(Ar, &delta_angle);

        reg.A = Ar;
        reg.R = Q;
        reg.cm = com_r;
        reg.angle = rb.angle0 + delta_angle;
	}

    const float alpha = sb.alpha;

	for (int di = rb.start_pdata_idx; di < rb.start_pdata_idx + rb.num_part; ++di) {
        /*const*/ PBDSoftBodyParticleData& sp_data = sim->sb_particles_data_[di];
        const int pi = sb_data[di].base.index;
        const uint32_t roff = sp_data.start_region_idx;

        vec2 x(0);
        sp_data.goal_R = mat2(0,0,0,0);
        for(uint32_t pri = roff; pri < roff + sp_data.num_regions; pri++) {
            uint32_t ri = sim->sb_regions_per_particle_[pri];

            vec2 x0r = sp_data.base.x0 - sim->sb_regions_[ri].cm0;
            mat2 rR = sim->sb_regions_[ri].R;
            vec2 r_cm = sim->sb_regions_[ri].cm;
			vec2 r_x = (rR * x0r + r_cm);
            x += r_x;
            sp_data.goal_R = sp_data.goal_R + rR;
		}

        x /= sp_data.num_regions;
        x = alpha*(x - x_pred[pi]) + x_pred[pi];
		sp_data.goal_x = x;
        // TODO: should we also lerp goal_R?
		sp_data.goal_R = sp_data.goal_R * (1.0f / sp_data.num_regions);

		vec2 dx = x - x_pred[pi];
        update_position(pi, dx, sim);
	}
}

void solve_soft_shape_matching_c2(const SoftShapeMatchingConstraint& c,
						  const PBDSoftBodyParticleData* sb_data,
						  PBDUnifiedSimulation* sim) {

	const int sb_index = c.sb_index;
    const PBDSoftBody& sb = sim->soft_bodies_[sb_index];
    const PBDRigidBody& rb = sb.base;

	const vec2* const x_pred = sim->x_pred_.data();
	const uint32_t off = sb.start_region_idx;

    // clear any prev reg values
	for (uint32_t ri = 0; ri < sb.num_regions; ri++) {
		PBDRegion& reg = sim->sb_regions_[off + ri];
		//reg.A = mat2(0, 0, 0, 0);
		//reg.R = mat2(0, 0, 0, 0);
		reg.cm = vec2(0);
	}

	for (int di = rb.start_pdata_idx; di < rb.start_pdata_idx + rb.num_part; ++di) {
        const PBDSoftBodyParticleData& sp_data = sim->sb_particles_data_[di];
        const uint32_t roff = sp_data.start_region_idx;

        for(uint32_t pri = roff; pri < roff + sp_data.num_regions; pri++) {
            uint32_t ri = sim->sb_regions_per_particle_[pri];
            PBDRegion& r = sim->sb_regions_[ri];
			// can multiply here or in a separate loop (*) after
			// this actually changes behaviour a bit :-/
			// if making it in a separate loop then result is same as in
			// solve_soft_shape_matching_c
			//
			// I think second loop variant is a bit more stable but may be a lower one

			// float oo_num_part = 1.0f/(float)r.num_part;
			r.cm += sim->x_pred_[sp_data.base.index];//*oo_num_part;
        }
    }

    // (*)
	for (uint32_t ri = 0; ri < sb.num_regions; ri++) {
		PBDRegion& reg = sim->sb_regions_[off + ri];
        float oo_num_part = 1.0f/(float)reg.num_part;
		reg.cm *= oo_num_part;
	}

	for (int di = rb.start_pdata_idx; di < rb.start_pdata_idx + rb.num_part; ++di) {
        /*const*/ PBDSoftBodyParticleData& sp_data = sim->sb_particles_data_[di];
        const uint32_t roff = sp_data.start_region_idx;

        for(uint32_t pri = roff; pri < roff + sp_data.num_regions; pri++) {
            uint32_t ri = sim->sb_regions_per_particle_[pri];
            PBDRegion& r = sim->sb_regions_[ri];
            r.A = r.A + outer(x_pred[sp_data.base.index] - r.cm, sp_data.base.x0);
        }
    }

	for (uint32_t ri = 0; ri < sb.num_regions; ri++) {
		PBDRegion& reg = sim->sb_regions_[off + ri];
        float delta_angle;
        mat2 Q = calculateQ(reg.A, &delta_angle);
        reg.R = Q;
        reg.angle = rb.angle0 + delta_angle;
        // clear values at the same time
		reg.A = mat2(0, 0, 0, 0);
    }

    const float alpha = sb.alpha;

	for (int di = rb.start_pdata_idx; di < rb.start_pdata_idx + rb.num_part; ++di) {
        /*const*/ PBDSoftBodyParticleData& sp_data = sim->sb_particles_data_[di];
        const int pi = sp_data.base.index;
        const uint32_t roff = sp_data.start_region_idx;

        vec2 x(0);
        sp_data.goal_R = mat2(0,0,0,0);
        for(uint32_t pri = roff; pri < roff + sp_data.num_regions; pri++) {
            uint32_t ri = sim->sb_regions_per_particle_[pri];

            vec2 x0r = sp_data.base.x0 - sim->sb_regions_[ri].cm0;
            mat2 rR = sim->sb_regions_[ri].R;
            vec2 r_cm = sim->sb_regions_[ri].cm;
			vec2 r_x = (rR * x0r + r_cm);
            x += r_x;
            sp_data.goal_R = sp_data.goal_R + rR;
		}

        x /= sp_data.num_regions;
        x = alpha*(x - x_pred[pi]) + x_pred[pi];
		sp_data.goal_x = x;
        // TODO: should we also lerp goal_R?
		sp_data.goal_R = sp_data.goal_R * (1.0f / sp_data.num_regions);

		vec2 dx = x - x_pred[pi];
        update_position(pi, dx, sim);
	}
}

void solve_solid_fluid_collision_c(const SolidParticlesCollisionConstraint& c, PBDUnifiedSimulation* sim) {

    assert(length(c.x_ij) <= 2*sim->particle_r_);

	const vec2 x_ij = sim->x_pred_[c.idx0] - sim->x_pred_[c.idx1];
    const float x_ij_len = length(x_ij);
    if(x_ij_len > 2*sim->particle_r_) {
        return;
    }

	assert(c.idx0 >= 0 && c.idx0 < (int32_t)sim->particles_.size());
	assert(c.idx1 >= 0 && c.idx1 < (int32_t)sim->particles_.size());

    const PBDParticle& p0 = sim->particles_[c.idx0];
    const PBDParticle& p1 = sim->particles_[c.idx1];

	assert(p0.flags & PBDParticleFlags::kSolid);
	assert(p1.flags & PBDParticleFlags::kFluid);

    const float d = x_ij_len - 2*sim->particle_r_;
    assert(d < 0);

	vec2 gradC = x_ij / max(x_ij_len, 1e-9f);
	float inv_mass_i = sim->inv_scaled_mass_[c.idx0];
    float inv_mass_j = sim->inv_scaled_mass_[c.idx1];

    float C = d;
    float s = C / (inv_mass_i + inv_mass_j);
    vec2 dp0 = -s * inv_mass_i * gradC;
    vec2 dp1 = +s * inv_mass_j * gradC;

    update_positions(c.idx0, c.idx1, dp0, dp1, sim);
}


void solve_solid_particle_collision_c(const SolidParticlesCollisionConstraint& c, PBDUnifiedSimulation* sim) {

    assert(length(c.x_ij) <= 2*sim->particle_r_);

	assert(c.idx0 >= 0 && c.idx0 < (int32_t)sim->particles_.size());
	assert(c.idx1 >= 0 && c.idx1 < (int32_t)sim->particles_.size());

    const PBDParticle& p0 = sim->particles_[c.idx0];
    const PBDParticle& p1 = sim->particles_[c.idx1];

	assert(p0.flags & (PBDParticleFlags::kSolid|PBDParticleFlags::kSoftBody));
	assert(p1.flags & (PBDParticleFlags::kSolid|PBDParticleFlags::kSoftBody));

	const vec2 x_ij = sim->x_pred_[c.idx0] - sim->x_pred_[c.idx1];
	const float x_ij_len = max(length(x_ij), 1e-9f);
    const float d = x_ij_len - 2*sim->particle_r_;
    if(d>=0) { // can happen if using gauss seidel style
        return;
    }

	vec2 gradC = x_ij / x_ij_len;
	float wi = sim->inv_scaled_mass_[c.idx0];
    float wj = sim->inv_scaled_mass_[c.idx1];

    float C = d;
    float s = C / (wi + wj);
    vec2 dp0 = -s * wi* gradC;
    vec2 dp1 = +s * wj* gradC;
     
    // friction
	const vec2 path = (sim->x_pred_[c.idx0] + dp0 - p0.x) -
			  (sim->x_pred_[c.idx1] + dp1 - p1.x);
    vec2 fd = process_friction(path, gradC, -d, c.idx0, c.idx1, p0, p1, sim); 
	dp0 -= wi / (wi + wj) * fd;
	dp1 += wj / (wi + wj) * fd;

    update_positions(c.idx0, c.idx1, dp0, dp1, sim);
}

void solve_particle_rb_collision_c(const ParticleRigidBodyCollisionConstraint& c, PBDUnifiedSimulation* sim) {

    assert(length(c.x_ij) <= 2*sim->particle_r_);

	assert(c.idx0 >= 0 && c.idx0 < (int32_t)sim->particles_.size());
	assert(c.idx1 >= 0 && c.idx1 < (int32_t)sim->particles_.size());

    const PBDParticle& p0 = sim->particles_[c.idx0];
    const PBDParticle& p1 = sim->particles_[c.idx1];

	assert(p0.flags & PBDParticleFlags::kRigidBody);
	assert(!(p1.flags & PBDParticleFlags::kRigidBody));

	int32_t rb_idx = sim->particles_[c.idx0].phase;
	assert(rb_idx >= 0 && rb_idx < (int32_t)sim->rigid_bodies_.size());

	const PBDRigidBody& rb = sim->rigid_bodies_[rb_idx];

	int32_t rb_data_idx = p0.rb_data_idx;
	assert(rb_data_idx >= 0 && rb_data_idx < (int32_t)sim->rb_particles_data_.size());

	PBDRigidBodyParticleData& rb_part_data = sim->rb_particles_data_[rb_data_idx];
	assert(rb_part_data.sdf_value <=
		   0); // otherwise not much sense to have a particle whoose center will be
			   // outside of the rigid body
    
    
    const vec2 x_ij = sim->x_pred_[c.idx0] - sim->x_pred_[c.idx1];
    const float x_ij_len = length(x_ij);
    const float d = x_ij_len - (sim->particle_r_ + (-rb_part_data.sdf_value));
    if(d >= 0) {
        return;
    }

    vec2 norm;
    // maybe makes sense to always use grad in case rb vs. single particle?
    if(fabsf(rb_part_data.sdf_value) < sim->particle_r_) {
        norm = rotate2(rb.angle) * rb_part_data.sdf_grad;
    } else {
		norm = -normalize(x_ij);
	}

	// Unified Particle Physics: one sided normals (Ch. 5 Rigid Bodies)
	if(rb_part_data.b_is_boundary) {
        if(dot(-x_ij, norm) < 0) {
            norm = normalize(reflect(-x_ij, norm));
        } else {
            norm = normalize(-x_ij);
        }
    }

#if MAKE_COLLISIONS_GREAT_AGAIN
    norm = normalize(-x_ij);
#endif

    // fluid will always be p1
    // looks like not using scaled mass works a bit better with fluid particles, so maybe use it 
	const bool b_with_fluid = p1.flags & PBDParticleFlags::kFluid;

    vec2 gradC = -norm;
    float w0 = (false&&b_with_fluid) ? p0.inv_mass : sim->inv_scaled_mass_[c.idx0];
    float w1 = (false&&b_with_fluid) ? p1.inv_mass : sim->inv_scaled_mass_[c.idx1];
    float s = d / (w0 + w1);
    vec2 dp0 = -s * w0 * gradC;
    vec2 dp1 = s * w1 * gradC;

    // no friction for collision with fluid particle (maybe enable later)
	if (!b_with_fluid) {
		vec2 path = (sim->x_pred_[c.idx0] + dp0 - p0.x) - (sim->x_pred_[c.idx1] + dp1 - p1.x);
        vec2 fd = process_friction(path, gradC, -d, c.idx0, c.idx1, p0, p1, sim); 

        dp0 += -w0 / (w0 + w1) * fd;
        dp1 += +w1 / (w0 + w1) * fd;
    }

    update_positions(c.idx0, c.idx1, dp0, dp1, sim);
}

void solve_rb_collision_c(const RigidBodyCollisionConstraint& c, PBDUnifiedSimulation* sim) {

    assert(length(c.x_ij) <= 2*sim->particle_r_);

    const PBDParticle& p0 = sim->particles_[c.idx0];
    const PBDParticle& p1 = sim->particles_[c.idx1];

	assert(c.idx0 >= 0 && c.idx0 < (int32_t)sim->particles_.size());
	assert(p0.flags & PBDParticleFlags::kRigidBody);

	assert(c.idx1 >= 0 && c.idx1 < (int32_t)sim->particles_.size());
	assert(p1.flags & PBDParticleFlags::kRigidBody);

	int32_t rb0_idx = p0.phase;
	int32_t rb1_idx = p1.phase;

	assert(rb0_idx >= 0 && rb0_idx < (int32_t)sim->rigid_bodies_.size());
	assert(rb1_idx >= 0 && rb1_idx < (int32_t)sim->rigid_bodies_.size());
    assert(rb0_idx != rb1_idx);

	const PBDRigidBody& rb0 = sim->rigid_bodies_[rb0_idx];
	const PBDRigidBody& rb1 = sim->rigid_bodies_[rb1_idx];

	int32_t rb_data_idx0 = p0.rb_data_idx;
	int32_t rb_data_idx1 = p1.rb_data_idx;

	assert(rb_data_idx0 >= 0 && rb_data_idx0 < (int32_t)sim->rb_particles_data_.size());
	assert(rb_data_idx1 >= 0 && rb_data_idx1 < (int32_t)sim->rb_particles_data_.size());

	PBDRigidBodyParticleData& rb_part_data0 = sim->rb_particles_data_[rb_data_idx0];
    PBDRigidBodyParticleData& rb_part_data1 = sim->rb_particles_data_[rb_data_idx1];

    const float d0 = -rb_part_data0.sdf_value;
    const float d1 = -rb_part_data1.sdf_value;

    vec2 x_ij = sim->x_pred_[c.idx0] - sim->x_pred_[c.idx1];
    float x_ij_len = length(x_ij);
	float d = x_ij_len - (d0 + d1);
	// no rb collision
    if(d >= 0) {
        return;
    }

    vec2 norm;
    if(fabsf(rb_part_data0.sdf_value) < fabsf(rb_part_data1.sdf_value)) {
        norm = rotate2(rb0.angle) * rb_part_data0.sdf_grad;
    } else {
		norm = -rotate2(rb1.angle) * rb_part_data1.sdf_grad;
	}

    const vec2 norm_x_ij = x_ij_len==0 ? vec2(0) : x_ij/x_ij_len;

	// Unified Particle Physics: one sided normals (Ch. 5 Rigid Bodies)
	if(rb_part_data0.b_is_boundary) {
        if(dot(-x_ij, norm) < 0) {
            norm = normalize(reflect(-norm_x_ij, norm));
        } else {
            norm = -norm_x_ij;
        }
    }

#if MAKE_COLLISIONS_GREAT_AGAIN
    norm = -norm_x_ij;
#endif

    vec2 gradC = -norm;
    float w0 = sim->inv_scaled_mass_[c.idx0];
    float w1 = sim->inv_scaled_mass_[c.idx1];
    float s = d / (w0 + w1);
    vec2 dp0 = -s * w0 * gradC;
    vec2 dp1 = s * w1 * gradC;

    add_dbg_info(c.idx0, c.idx1, gradC, dp0, dp1, sim->x_pred_[c.idx0], sim->x_pred_[c.idx1], sim);

	vec2 dx = (sim->x_pred_[c.idx0] + dp0 - p0.x) -
			  (sim->x_pred_[c.idx1] + dp1 - p1.x);
    vec2 fd = process_friction(dx, gradC, -d, c.idx0, c.idx1, p0, p1, sim);
	dp0 -= w0 / (w0 + w1) * fd;
	dp1 += w1 / (w0 + w1) * fd;

    update_positions(c.idx0, c.idx1, dp0, dp1, sim);
}

// resolves a contact of a particle and a static environment
// TODO: maybe pass all params about environment as a material inistead of separate function parameters
void resolve_contact(vec2 dx, const vec2 n, const int particle_idx,
					 const float cont_mu_s, const float cont_mu_k, PBDUnifiedSimulation* sim) {

    
    const vec2 path = (sim->x_pred_[particle_idx] + dx) - sim->particles_[particle_idx].x;

    const bool is_fluid = sim->particles_[particle_idx].flags & PBDParticleFlags::kFluid;
	if (!is_fluid) {
		float d = length(dx);
		vec2 fd = process_friction(path, n, d, particle_idx, sim->particles_[particle_idx],
								cont_mu_s, cont_mu_k, sim);
        dx -= fd;
	}

    update_collision(particle_idx, dx, sim);
}

void solve_box_boundary(const BoxBoundaryConstraint& c, vec2 pos, int particle_idx, PBDUnifiedSimulation* sim) {

	const float r = sim->particle_r_;
    vec2 dx = vec2(0);
    vec2 n = vec2(0);

    bool b_collided = false;

    // case when width or height is less than a particle radius is not handled
    // we can change 'else if' for just 'if' but then still it would not work
	if (pos.x - r < c.p_min.x) {

        dx.x = c.p_min.x - (pos.x - r);
        n = vec2(-1,0);
        b_collided = true;
        // TODO: those are part of a contact structure
		//ci.r1 = vec2(pos.x - r, pos.y);
		//ci.r2 = vec2(c.p_min.x, pos.y);

    } else if (pos.x + r > c.p_max.x) {

        dx.x = c.p_max.x - (pos.x + r);
        n = vec2(1,0);
        b_collided = true;
        // TODO: those are part of a contact structure
		//ci.r1 = vec2(pos.x + r, pos.y);
		//ci.r2 = vec2(c.p_max.x, pos.y);
    }

    if(b_collided) {
        // n - points to the gradient increase
        resolve_contact(dx, -n, particle_idx, c.mu_s, c.mu_k, sim);
        // re-read pos because we might update position immediately in case of gauss-seidel mode
        pos = sim->x_pred_[particle_idx];
    }

    b_collided = false;
    dx = vec2(0);

    if (pos.y - r < c.p_min.y) {

        dx.y = c.p_min.y - (pos.y - r);
        n = vec2(0,-1);
        b_collided = true;
        // TODO: those are part of a contact structure
		//ci.r1 = vec2(pos.x, pos.y - r);
		//ci.r2 = vec2(pos.x, c.p_min.y);

    } else if (pos.y + r > c.p_max.y) {

        dx.y = c.p_max.y - (pos.y + r);
        n = vec2(0,1);
        b_collided = true;
        // TODO: those are part of a contact structure
		//ci.r1 = vec2(pos.x, pos.y + r);
		//ci.r2 = vec2(pos.x, c.p_max.y);
    }

    if(b_collided) {
    // n - points to the gradient increase
        resolve_contact(dx, -n, particle_idx, c.mu_s, c.mu_k, sim);
    }
}

void solve_collision_constraint(CollisionContact& cc, PBDUnifiedSimulation* sim,
								const CollisionWorld* cword, const vec2* x_pred) {

    const int particle_idx = cc.idx;
    vec2 dx = cc.dist * cc.n;

	// friction
	const vec2 path = (sim->x_pred_[particle_idx] + dx) - sim->particles_[particle_idx].x;

	// TODO: add support for collision geometry friction coefficients
	// passing particle's mu instead of collision mu because a) we do not have it now b)
	// when averaging we do not need separate 'if' path, because 0.5(x + x) = x
	const float mu_s = sim->particles_[particle_idx].mu_s;
	const float mu_k = sim->particles_[particle_idx].mu_k;
	vec2 fd = process_friction(path, cc.n, cc.dist, particle_idx,
							sim->particles_[particle_idx], mu_s, mu_k, sim);
    dx -= fd;

    update_collision(particle_idx, dx, sim);
}


bool is_reachable(PBDBodyLatticeData* lattice, u32 sx, u32 sy, u32 i0, u32 i1) {
	struct dummy { 
        int sx;
        int sy;
        PBDBodyLatticeData* lattice;
        bool is_valid(ivec2 v) const {
	        return v.x >= 0 && v.x < sx && v.y >= 0 && v.y < sy;
        }
        ivec2 xy(int i) { return ivec2(i%sx, i/sx); }
        i32 i(ivec2 v) { return v.x + v.y*sx; }

        ~dummy() {
		    for (int cnt = 0; cnt < sx*sy; cnt++) {
                lattice[cnt].nflags &= ~kVisited;
            }
        }
    };

    dummy _{(i32)sx, (i32)sy, lattice};

    std::queue<u32> idxs;
    idxs.push(i0);
	lattice[i0].nflags |= kVisited;

	while(!idxs.empty()) {
        const u32 i = idxs.front();
        idxs.pop();

        const u32 cflags = lattice[i].nflags;
        i32 cx = i%sx;
        i32 cy = i/sx;

		ivec2 neigh[] = {
			ivec2(cx + 1, cy),	   ivec2(cx - 1, cy),	  ivec2(cx, cy - 1), ivec2(cx, cy + 1),	   
#if INCLUDE_DIAG_NEIGHBOURS
            ivec2(cx + 1, cy - 1), ivec2(cx + 1, cy + 1),
			ivec2(cx - 1, cy - 1), ivec2(cx - 1, cy + 1),
#endif
		};
		u32 nflags[] = {kR, kL, kT, kB,
#if INCLUDE_DIAG_NEIGHBOURS
            kRT, kRB, kLT, kLB
#endif
        };
        constexpr int arr_size = sizeof(neigh) / sizeof(neigh[0]);
		static_assert(arr_size == sizeof(nflags) / sizeof(nflags[0]));

		for (int cnt = 0; cnt < arr_size; cnt++) {
			i32 nidx = neigh[cnt].x + neigh[cnt].y * sx;

            bool valid = _.is_valid(neigh[cnt]);
			if (valid && (cflags & nflags[cnt]) && !(lattice[nidx].nflags & kVisited)) {

				if (nidx == (i32)i1) {
					return true;
				}

				idxs.push(nidx);
				lattice[nidx].nflags |= kVisited;
			}
		}
	}

    return false;
}

bool is_fractured(const PBDBreakableSoftBody& bb, const PBDSoftBodyParticleData& pdata0,
				  const PBDSoftBodyParticleData& pdata1) {

	float len = length(pdata1.goal_x - pdata0.goal_x);
	float len0 = length(pdata1.base.x0 - pdata0.base.x0);
    if(fabsf(len/len0) > bb.fracture_len)
        return true;

    vec2 axisA = normalize(vec2(pdata0.goal_R.e00, pdata0.goal_R.e10));
    vec2 axisB = normalize(vec2(pdata1.goal_R.e00, pdata1.goal_R.e10));
    if(dot(axisA, axisB) < bb.fracture_cos_angle)
        return true;

    return false;
}

void check_fracture(u32 bb_idx, PBDUnifiedSimulation* sim) {
    PBDBreakableSoftBody& bb = sim->breakable_bodies_.at(bb_idx);
    const u32 lsx = bb.lattice_sx;
    const u32 lsy = bb.lattice_sy;
    const u32 lattice_num = lsx*lsy;
    PBDBodyLatticeData* lattice = sim->bb_lattice_.data() + bb.lattice_idx;

    struct binfo { u32 bb; u32 sb_idx; bool island_broke;};
    std::vector<binfo> broken_info;

    u32 cur_sb_idx = bb.first_soft_body;

    // !NB: currently bodies will not break along diagonal links!

    do {
        assert(cur_sb_idx>=0);
        assert((u32)sim->soft_bodies_.size() > cur_sb_idx);

        PBDSoftBody& sb = sim->soft_bodies_[cur_sb_idx];
        bool b_was_break = false;
        bool b_did_island_broke = false;
        
        const u32 pdata_off = sb.base.start_pdata_idx;
        const u32 pdata_num = sb.base.num_part;
		for (u32 i = pdata_off; i < pdata_off + pdata_num; ++i) {
			PBDSoftBodyParticleData& pdata = sim->sb_particles_data_[i];
            const u32 lidx = pdata.x + pdata.y*lsx;
            if(lattice[lidx].nflags & kR) {
                u32 nidx = pdata.x + 1 + pdata.y*lsx;
                assert(lattice[nidx].nflags&kExists);
                assert(nidx < lattice_num);
			    PBDSoftBodyParticleData& pdata_n = sim->sb_particles_data_[lattice[nidx].pdata_idx];
                if(is_fractured(bb, pdata, pdata_n)) {
                    lattice[lidx].nflags &= ~kR;
                    lattice[nidx].nflags &= ~kL;
                    b_was_break = true;
					// NOTE: can only do reacheability check at the end of the for() loop
					// though need to probably use enumerate islands or something like
					// this
                    bool b_this_link_broke = !is_reachable(lattice, lsx, lsy, lidx, nidx);
                    b_did_island_broke |= b_this_link_broke;
                    printf("HorLink %d <-> %d broken, %s reachable\n", i, nidx, b_this_link_broke ? "and NOT" : "but");
                }
            }
            if(lattice[lidx].nflags & kB) {
                u32 nidx = pdata.x + (1 + pdata.y)*lsx;
                assert(lattice[nidx].nflags&kExists);
                assert(nidx < lattice_num);
			    PBDSoftBodyParticleData& pdata_n = sim->sb_particles_data_[lattice[nidx].pdata_idx];
                if(is_fractured(bb, pdata, pdata_n)) {
                    lattice[lidx].nflags &= ~kB;
                    lattice[nidx].nflags &= ~kT;
                    b_was_break = true;
                    bool b_this_link_broke = !is_reachable(lattice, lsx, lsy, lidx, nidx);
                    b_did_island_broke |= b_this_link_broke;
                    printf("VertLink %d <-> %d broken, %s reachable\n", i, nidx, b_this_link_broke ? "and NOT" : "but");
                }
            }
		} // for(pdata idx...

        // if at least one link broken
        if(b_was_break) {
			broken_info.push_back(
				binfo{bb_idx, cur_sb_idx, b_did_island_broke});
		}

        cur_sb_idx = sb.next_soft_body_idx;
	} while (cur_sb_idx != kPBDInvalidIndex);
#if 1
    for(int i=0;i<(int)broken_info.size(); ++i) {
        if(broken_info[i].island_broke) {
            on_soft_body_break(broken_info[i].bb, broken_info[i].sb_idx, sim);
        } else {
            PBDSoftBody& sb = sim->soft_bodies_[broken_info[i].sb_idx];
            remove_all_regions(sb, sim);
            construct_regions_for_body(sb, sb.w, lsx, lsy, lattice, sim);
        }
    }
#endif
}


////////////////////////////////////////////////////////////////////////////////
class PBDUnifiedTimestep {
	static const int stab_iter_count = 0;
	static const int sim_iter_count = 5;
	static const constexpr vec2 g = vec2(0.0f, -9.8f);
	static const constexpr float k = 1.5f;
	static const constexpr float eps = 1e-6f;
    // if movement less than 1% of particle radius
	static const constexpr float kSleepEps = 0.001f;
    // successive over-relaxation coefficient
	static const constexpr float kSOR = 1.0f; // [1,2]

	static const constexpr float kCFL = 0.4f;

	static float H(vec2 pos) { return pos.y; }

    void SimulateIteration(PBDUnifiedSimulation* sim, float dt, bool b_stabilization);
    void VelocityUpdate(PBDUnifiedSimulation* sim, const float h);
    void VelocityUpdateRigidBody(PBDUnifiedSimulation* sim, const float h);
    void fluidUpdate(PBDUnifiedSimulation* sim, float dt, std::vector<vec2>& pos_array);
    void fluidUpdate_GS(PBDUnifiedSimulation* sim, float dt, const std::vector<vec2>& pos_array);
    float getBoundaryDensity(PBDUnifiedSimulation* sim, int i, const std::vector<vec2>& pos_array, float boundary_mass);
    vec2 getBoundaryGrad(PBDUnifiedSimulation* sim, int i, const std::vector<vec2>& pos_array);

  public:
	void Simulate(PBDUnifiedSimulation* sim, float dt);
	void SimulateXPBD(PBDUnifiedSimulation* sim, const float dt);

};

void pbd_unified_timestep(PBDUnifiedSimulation* sim, float dt) {
    PBDUnifiedTimestep uts;
    dt = 0.016f;
    //uts.Simulate(sim, dt);
    uts.SimulateXPBD(sim, dt);
}

//#define SHAPE_MATCHING_FIX

void PBDUnifiedTimestep::SimulateIteration(PBDUnifiedSimulation* sim, float dt, bool b_stabilization) {
    SCOPED_ZONE_N(sim_iter, 0);

    const float r = sim->particle_r_;
	const int num_particles = (int)sim->particles_.size();
    //std::vector<float>& scaled_mass = sim->inv_scaled_mass_;
	std::vector<PBDParticle>& p = sim->particles_;
	std::vector<vec2>& dp = sim->dp_;
	std::vector<vec2>& x_pred = sim->x_pred_;

	sim->rb_collision_c_.resize(0);
	sim->particle_rb_collision_c_.resize(0);
	sim->solid_collision_c_.resize(0);
	sim->solid_fluid_c_.resize(0);
	sim->contacts_info_.resize(0);
	for (int i = 0; i < num_particles; i++) {
		sim->num_constraints_[i] = 0;
		sim->dp_[i] = vec2(0);
	}

	if (!b_stabilization) {
		fluidUpdate(sim, dt, x_pred);
	}

#if defined(SHAPE_MATCHING_FIX)
    for(auto c: sim->shape_matching_c_) {
        solve_shape_matching_c(c, sim->rb_particles_data_.data(), sim);

        // update immediately, helps with bouncing
        const PBDRigidBody& rb = sim->rigid_bodies_[c.rb_index];
	    for (int di = rb.start_part_idx; di < rb.start_part_idx + rb.num_part; ++di) {
            int i = sim->rb_particles_data_[di].index;
        
            if (sim->num_constraints_[i]) {
                vec2 dp_i = (kSOR / sim->num_constraints_[i]) * dp[i];
                if (b_stabilization) {
                    p[i].x += dp_i;
                }
                x_pred[i] += dp_i;
                sim->num_constraints_[i] = 0;
                sim->dp_[i] = vec2(0);
            }
        }
    }
#endif


	std::vector<CollisionContact> ccs;
	if (sim->collision_world_) {
		ccs = collision_world_check_collision(sim->collision_world_, x_pred.data(),
											  num_particles, r);
	}

    for(auto c: ccs) {
        solve_collision_constraint(c, sim, sim->collision_world_, x_pred.data());
    }

    for (int i = 0; i < num_particles; i++) {

        for(const BoxBoundaryConstraint& c: sim->box_boundary_c_) {
            if(!(p[i].flags & PBDParticleFlags::kFluid)) {
                solve_box_boundary(c, x_pred[i], i, sim); 
            }
        }

        const auto& neigh_list = sim->neigh_data_.neigh_idx_;
        const int n_start = sim->neigh_data_.neighbour_info_[i].start;
        const int n_cnt = sim->neigh_data_.neighbour_info_[i].num;

        for (int ni = 0; ni < n_cnt; ni++) {
            int j = neigh_list[n_start + ni];


            // because we project constraint for 2 particles
            if (j < i) {
                continue;
            }

            vec2 vec = x_pred[i] - x_pred[j];
            float dist_sqr = length(vec);

            float C = dist_sqr - 2*r;
            if (C < 0) {
                // rb - rb
                if(((p[i].flags&p[j].flags) & PBDParticleFlags::kRigidBody) ) {
                    if(p[i].phase!=p[j].phase)
                        sim->rb_collision_c_.push_back(RigidBodyCollisionConstraint{i,j, vec});
                // rb - solid/fluid
                } else if((p[i].flags & PBDParticleFlags::kRigidBody) /*&& (p[j].flags & PBDParticleFlags::kSolid)*/) {
                    sim->particle_rb_collision_c_.push_back(ParticleRigidBodyCollisionConstraint{i,j, vec});
                } else if((p[j].flags & PBDParticleFlags::kRigidBody) /*&& (p[i].flags & PBDParticleFlags::kSolid)*/) {
                    sim->particle_rb_collision_c_.push_back(ParticleRigidBodyCollisionConstraint{j,i, -vec});
                } else if(((p[i].flags&p[j].flags) & PBDParticleFlags::kSoftBody) ) {
                    if(p[i].phase!=p[j].phase) {
                        // check as if were individual particles for now
                        sim->solid_collision_c_.push_back(SolidParticlesCollisionConstraint{i,j, vec});
                    }
                // solid - solid
                } else if((p[j].flags & p[i].flags) & PBDParticleFlags::kSolid ) {
                    //if(p[j].phase != -1 && p[i].phase != -1)
                    //NOTE: for ropes, only check if not distance constraint between i & j?
                    sim->solid_collision_c_.push_back(SolidParticlesCollisionConstraint{i,j, vec});
                // solid - fluid
                } else if((p[i].flags & PBDParticleFlags::kSolid) && (p[j].flags & PBDParticleFlags::kFluid)) {
                    //sim->solid_fluid_c_.push_back(SolidParticlesCollisionConstraint{i,j, vec});
                } else if((p[j].flags & PBDParticleFlags::kSolid) && (p[i].flags & PBDParticleFlags::kFluid)) {
                    //sim->solid_fluid_c_.push_back(SolidParticlesCollisionConstraint{j,i, -vec});
                } else if((p[j].flags & p[i].flags) & PBDParticleFlags::kFluid) {
                    // handled separately in updateFluid
                } else {
                    assert(0 && "unsupported contact configuration");
                }
            }
        }
    }

    for(auto c: sim->solid_collision_c_) {
        solve_solid_particle_collision_c(c, sim);
    }

    for(auto c: sim->solid_fluid_c_) {
        solve_solid_fluid_collision_c(c, sim);
    }

    for(auto c: sim->rb_collision_c_) {
        solve_rb_collision_c(c, sim);
    }

    for(auto c: sim->particle_rb_collision_c_) {
        solve_particle_rb_collision_c(c, sim);
    }

    // NOTE: maybe use Jacobi style just for distance constraints...
    for(auto ci: sim->distance_c_idxs_) {
        DistanceConstraint& c = sim->distance_c_.at(ci);
        solve_distance_c(c, sim, x_pred.data());
    }

    for(auto ci: sim->distance2_c_idxs_) {
        DistanceConstraint2& c = sim->distance2_c_.at(ci);
        solve_distance2_c(c, sim, x_pred.data());
    }

#if !defined(SHAPE_MATCHING_FIX)
    for(auto c: sim->shape_matching_c_) {
        solve_shape_matching_c(c, sim->rb_particles_data_.data(), sim);
    }
#endif
#if 0
	for (int i = 0; i < num_particles; i++) {
        for(const BoxBoundaryConstraint& c: sim->box_boundary_c_) {
            if(!(p[i].flags & PBDParticleFlags::kFluid)) {
                solve_box_boundary(c, x_pred[i], i, sim); 
            }
        }
    }
#endif

    for(auto c: sim->soft_shape_matching_c_) {
        //solve_soft_shape_matching_c(c, sim->sb_particles_data_.data(), sim);
        solve_soft_shape_matching_c2(c, sim->sb_particles_data_.data(), sim);
    }

    for(auto b: sim->break_checks_) {
        check_fracture(b, sim);
    }

	//  adjust x* = x* + dx;
	// could set num_constraints to 1 during initialization to avoid branch
	if (b_stabilization) {
		for (int i = 0; i < num_particles; i++) {
			if (sim->num_constraints_[i]) {
				vec2 dp_i = (kSOR / sim->num_constraints_[i]) * dp[i];
				p[i].x += dp_i;
				x_pred[i] += dp_i;
			}
		}
	} else {
		for (int i = 0; i < num_particles; i++) {
			if (sim->num_constraints_[i]) {
				vec2 dp_i = (kSOR / sim->num_constraints_[i]) * dp[i];
				x_pred[i] += dp_i;
			}
		}
	}
}

constexpr const float very_small_float = 1e-12;

void PBDUnifiedTimestep::VelocityUpdate(PBDUnifiedSimulation* sim, const float h) {
    SCOPED_ZONE_N(VelocityUpdate, 0);

    //return VelocityUpdateRigidBody(sim, h);

    const int num_particles = (int)sim->particles_.size();
	std::vector<PBDParticle>& particles = sim->particles_;

	for (const ContactInfo& ci : sim->contacts_info_) {
        assert(ci.idx1 < num_particles);
        assert(ci.idx2==-1 || ci.idx2 < num_particles);
		assert(0 == (particles[ci.idx1].flags & PBDParticleFlags::kFluid));
		assert(ci.idx2==-1 || (0 == (particles[ci.idx2].flags & PBDParticleFlags::kFluid)));

		// these velocities are calculated after sim iteration, so we do not know them at
		// the moment of collision check, so cannot store in ContactInfo
		const vec2 v1 = particles[ci.idx1].v;
		const vec2 v2 = ci.idx2 != -1 ? particles[ci.idx2].v : vec2(0.0f);
		const vec2 v = v1 - v2;
		const float vn = dot(ci.n, v);
		const vec2 vt = v - vn * ci.n;

		// friction force (dynamic friction)
		const float vt_len = length(vt);
		const float fn = fabsf(ci.d_lambda_n / (h * h));
		// Eq. 30
		const vec2 dv = vt_len > very_small_float
							? -(vt / vt_len) * min(h * ci.mu_d * fn, vt_len)
							: vec2(0.0f);

		// restitution, Eq. 34
		const vec2 v_prev = ci.v1_pre - ci.v2_pre;
		const float v_prev_n = dot(v_prev, ci.n);
		const float e = fabsf(v_prev_n) < 2 * 9.81f * h ? 0.0f : particles[ci.idx1].e;
		// !NB: original paper has: min() in the last term, but probably it is an error or
		// their normal faces in other direction. Also maybe it makes sense to substitute
		// vn for min(vn,0) to not remove velocity if object moves avay from a contact
		vec2 restitution_dv1 = ci.n * (-vn + max(-e * v_prev_n, 0.0f));

		// apply delta velocity
		// NOTE: probably should first apply dv and then calculate restitution_dv and
		// apply it again?
		const float w1 = particles[ci.idx1].inv_mass;
		const float w2 = ci.idx2==-1 ? 0.0f : particles[ci.idx2].inv_mass;
		vec2 p = (dv + restitution_dv1) / (w1 + w2);
		particles[ci.idx1].v += p * w1;

		// NOTE: can use fake index to avoid 'if'
		if (ci.idx2 != -1) {
			float e2 = fabsf(v_prev_n)<2*9.81f*h ? 0.0f : particles[ci.idx2].e;
		    const vec2 restitution_dv2 = ci.n * (-vn + max(-e2 * v_prev_n, 0.0f));
			const vec2 p2 = (dv + restitution_dv2) / (w1 + w2);
			particles[ci.idx2].v -= p2 * w2;
		}
	}
}

void PBDUnifiedTimestep::VelocityUpdateRigidBody(PBDUnifiedSimulation* sim, const float h) {
    SCOPED_ZONE_N(VelocityUpdateRigidBody, 0);

#if 1
    vec2 rb_v = vec2(0.0f, 0.0f);
    float rb_omega= 0.0f;
    int count = 0;
    int rb_idx = -1;
    vec2 com;
    vec2 n;
    constexpr const int MAX_PART = 256;
    //int arr[MAX_PART] = {};
#endif

    const int num_particles = (int)sim->particles_.size();
	std::vector<PBDParticle>& particles = sim->particles_;

	for (const ContactInfo& ci : sim->contacts_info_) {
        assert(ci.idx1 < num_particles);
        assert(ci.idx2==-1 || ci.idx2 < num_particles);
		assert(0 == (particles[ci.idx1].flags & PBDParticleFlags::kFluid));
		assert(ci.idx2==-1 || (0 == (particles[ci.idx2].flags & PBDParticleFlags::kFluid)));
        bool b_is_static_boundary = ci.idx2==-1; 

		// these velocities are calculated after sim iteration, so we do not know them at
		// the moment of collision check, so cannot store in ContactInfo
		const vec2 v1 = particles[ci.idx1].v;
		const vec2 v2 = b_is_static_boundary ? vec2(0.0f) : particles[ci.idx2].v;
		const vec2 v = v1 - v2;
		const float vn = dot(ci.n, v);
		const vec2 vt = v - vn * ci.n;

        // maybe just do not nullify velocity (reflect: -vn) but still resolve contact?
        //if(vn>0)
         //   continue;

		// friction force (dynamic friction)
		const float vt_len = length(vt);
		const float fn = fabsf(ci.d_lambda_n / (h * h));
		// Eq. 30
		const vec2 dv = vt_len > very_small_float
							? -(vt / vt_len) * min(h * ci.mu_d * fn, vt_len)
							: vec2(0.0f);


		// restitution, Eq. 34
		const vec2 v_prev = ci.v1_pre - ci.v2_pre;
		const float v_prev_n = dot(v_prev, ci.n);
		const float e = particles[ci.idx1].e;
        // !NB: original paper has: min() in the first term, but probably it is an error
        // min() in the first term is added to not push particle back inside if it is already moving in the normal direction
        // though, probably will have to separately calculate restitution_dv2 
		vec2 restitution_dv1 = ci.n * (-min(vn,0.0f) - min(e * v_prev_n, 0.0f));

		// apply delta velocity
		// NOTE: probably should first apply dv and then calculate restitution_dv and
		// apply it again?
		const float w1 = particles[ci.idx1].inv_mass;
		const float w2 = b_is_static_boundary ? 0.0f : particles[ci.idx2].inv_mass;

        rb_idx = particles[ci.idx1].phase;
        const float Iinv = sim->rigid_bodies_[rb_idx].Iinv;
        com = rb_calc_com(sim->rigid_bodies_[rb_idx], sim->x_pred_.data(), sim->rb_particles_data_.data());
        vec2 r = sim->x_pred_[ci.idx1] - com;
        const int npart = sim->rigid_bodies_[rb_idx].num_part;
		const float rb_inv_mass = sim->particles_[0].inv_mass / npart;

        vec3 temp = cross(vec3(r, 0), vec3(ci.n,0));
        const float gw1 = rb_inv_mass + dot(temp, Iinv*temp);

		vec2 p = (dv + restitution_dv1) / (gw1 + w2);
		//particles[ci.idx1].v += p * w1;
#if 1

        rb_v += ((dv + restitution_dv1 + 0*vn*ci.n) / (gw1 + w2))*rb_inv_mass; // should be the whole mass of rb
        n = ci.n;
        assert(ci.idx1<MAX_PART);
        //arr[ci.idx1] = 1;
        //const PBDRigidBodyParticleData& data = sim->rb_particles_data_[particles[ci.idx1].rb_data_idx];
        vec3 c = cross(vec3(r,0), vec3(p, 0.0f));
        assert(c.x==0 && c.y == 0);
        rb_omega += Iinv * c.z;
        count++;
#endif
		// NOTE: can use fake index to avoid 'if'
		if (!b_is_static_boundary) {
			const float e2 = particles[ci.idx2].e;
			const vec2 restitution_dv2 = ci.n * (-vn + max(-e2 * v_prev_n, 0.0f));
			const vec2 p2 = (dv + restitution_dv2) / (w1 + w2);
			particles[ci.idx2].v -= p2 * w2;
		}
	}

	if (count) {
		// apply velocity to all rb particles according to impulse
		const int start = sim->rigid_bodies_[rb_idx].start_pdata_idx;
        const int npart = sim->rigid_bodies_[rb_idx].num_part;
		const int end = start + npart;
		for (int i = start; i != end; ++i) {
			int idx = sim->rb_particles_data_[i].index;
            vec2 r = sim->x_pred_[idx] - com;


			vec3 cr = cross(vec3(0, 0, rb_omega), vec3(r, 0));
            /*
			assert(idx < MAX_PART);
			if (arr[idx]) {
				const vec2 v = particles[idx].v;
				const float vn = dot(n, v);
				sim->particles_[idx].v += -n * vn;
			}
            */

			sim->particles_[idx].v += rb_v/count + cr.xy()/count;
		}
	}
}

void PBDUnifiedTimestep::Simulate(PBDUnifiedSimulation* sim, float dt) {

	const int num_particles = (int)sim->particles_.size();
	std::vector<PBDParticle>& p = sim->particles_;
	std::vector<float>& scaled_mass = sim->inv_scaled_mass_;
	std::vector<vec2>& x_pred = sim->x_pred_;

	// predict
	for (int i = 0; i < num_particles; i++) {
		vec2 pv = p[i].v + dt * g;
		x_pred[i] = p[i].x + dt * pv;
		scaled_mass[i] = p[i].inv_mass * exp(k * H(p[i].x)); // h(x_pred[i]) ?
	}

	findNeighboursNaiive(sim->x_pred_.data(), (int)sim->particles_.size(),
						 sim->support_r_, sim->particles_.data(), &sim->neigh_data_);


	for (int iter = 0; iter < stab_iter_count; iter++) {
        SimulateIteration(sim, dt, true);
    }

	for (int iter = 0; iter < sim_iter_count; iter++) {
        SimulateIteration(sim, dt, false);
	}

	for (int i = 0; i < num_particles; i++) {
        vec2 dpv = x_pred[i] - p[i].x;
        float dp_len = length(dpv);
#if ENABLE_CFL
        // CFL
        const float max_vel_ = kCFL * sim->particle_r_;
        if(dp_len > max_vel_) {
            dpv *= max_vel_ / dp_len;
            p[i].x = p[i].x + dpv;
        }
#endif

		// update velocity (first order)
		p[i].v = dpv / dt;

		// update position or sleep
        bool b_sleep = dp_len < kSleepEps;
        if(!b_sleep) {
		    p[i].x = x_pred[i];
            p[i].flags &= ~PBDParticleFlags::kSleep;
        } else {
            p[i].flags |= PBDParticleFlags::kSleep;
        }
	}
}

void PBDUnifiedTimestep::SimulateXPBD(PBDUnifiedSimulation* sim, const float dt) {

	const int num_particles = (int)sim->particles_.size();
	std::vector<PBDParticle>& p = sim->particles_;
	std::vector<float>& scaled_mass = sim->inv_scaled_mass_;
	std::vector<vec2>& x_pred = sim->x_pred_;

	sim->dbg_contacts_.resize(0);
	sim->dbg_friction_.resize(0);

    // expand neighbour search because we do it only once per iteration
    float neighbour_r = 2.0f*sim->support_r_;
	findNeighboursNaiive(sim->x_pred_.data(), (int)sim->particles_.size(),
						 neighbour_r, sim->particles_.data(), &sim->neigh_data_);

	const float h = dt / sim_iter_count;

	for (int iter = 0; iter < sim_iter_count; iter++) {

		// predict
		for (int i = 0; i < num_particles; i++) {
			vec2 pv = p[i].v + h * g;
			x_pred[i] = p[i].x + h * pv;
			scaled_mass[i] = p[i].inv_mass * exp(k * H(p[i].x)); // h(x_pred[i]) ?
			scaled_mass[i] = p[i].inv_mass * exp(k * H(p[i].x)); // h(x_pred[i]) ?
		}


		if (!VELOCITY_UPDATE && iter < 0) {
			SimulateIteration(sim, h, true);
		} else {
            SimulateIteration(sim, h, false);
		}

		for (int i = 0; i < num_particles; i++) {
			vec2 dpv = x_pred[i] - p[i].x;
			float dp_len = length(dpv);
#if ENABLE_CFL
			// CFL
			const float max_vel_ = kCFL * sim->particle_r_;
			if (dp_len > max_vel_) {
				dpv *= max_vel_ / dp_len;
				x_pred[i] = p[i].x + dpv;
			}
#endif

			// update velocity (first order)
			p[i].v = dpv / h;

			// update position or sleep (should probably take into account number of
			// frames as well) or remember pos at the beginning of iterations if using
			// dt/num_iter
			bool b_sleep = dp_len < 0.25f*kSleepEps;
			if (!b_sleep || true) {
				p[i].x = x_pred[i];
				p[i].flags &= ~PBDParticleFlags::kSleep;
			} else {
				p[i].flags |= PBDParticleFlags::kSleep;
			}
		}

        if(VELOCITY_UPDATE) {
            VelocityUpdate(sim, h);
        }

	}
}

float PBDUnifiedTimestep::getBoundaryDensity(PBDUnifiedSimulation* sim, int i, const std::vector<vec2>& pos_array, float boundary_mass) {

	//std::vector<PBDParticle>& p = sim->particles_;
	const std::vector<vec2>& x = pos_array;
    const float h = 3*sim->particle_r_;
    const float r = sim->particle_r_;
    const float m = boundary_mass;//1.0f/p[i].inv_mass;

    vec2 pos = pos_array[i];
    float density = 0;
	// if particle is in corner koeff will equal 2.0f and thus compensate for lack of
	// density of corner particles, still not mathematically correct, in practice need to
	// do density integration
	float koeff = 0.0f;
	for (const BoxBoundaryConstraint& c : sim->box_boundary_c_) {

		if (pos.x - h < c.p_min.x) {
            density += m * sim->W(x[i] - vec2(c.p_min.x - r, x[i].y));
            koeff += 1.0f;
		} else if (pos.x + h > c.p_max.x) {
            density += m * sim->W(x[i] - vec2(c.p_max.x + r, x[i].y));
            koeff += 1.0f;
		}

		if (pos.y - h < c.p_min.y) {
            density += m * sim->W(x[i] - vec2(x[i].x, c.p_min.y - r));
            koeff += 1.0f;
		} else if (pos.y + h > c.p_max.y) {
            density += m * sim->W(x[i] - vec2(x[i].x, c.p_max.y + r));
            koeff += 1.0f;
		}
	}

	return koeff * density;
}

vec2 PBDUnifiedTimestep::getBoundaryGrad(PBDUnifiedSimulation* sim, int i, const std::vector<vec2>& pos_array) {

	const std::vector<vec2>& x = pos_array;
    const float r_eps = 0.001f;
    const float h = 3*sim->particle_r_ + r_eps;
    const float r = sim->particle_r_;

    vec2 pos = pos_array[i];
    vec2 grad = vec2(0);
    float contacts = 0;
	for (const BoxBoundaryConstraint& c : sim->box_boundary_c_) {

		if (pos.x - h <= c.p_min.x) {
            grad += sim->gradW(x[i] - vec2(c.p_min.x - r, x[i].y));
            contacts++;
		} else if (pos.x + h >= c.p_max.x) {
            grad += sim->gradW(x[i] - vec2(c.p_max.x + r, x[i].y));
            contacts++;
		}

		if (pos.y - h <= c.p_min.y) {
            grad += sim->gradW(x[i] - vec2(x[i].x, c.p_min.y - r));
            contacts++;
		} else if (pos.y + h >= c.p_max.y) {
            grad += sim->gradW(x[i] - vec2(x[i].x, c.p_max.y + r));
            contacts++;
		}
	}

    //return contacts!=0 ? grad/contacts : vec2(0.0f);
    return grad;
}

void PBDUnifiedTimestep::fluidUpdate(PBDUnifiedSimulation* sim, float dt,
									 std::vector<vec2>& pos_array) {
    SCOPED_ZONE_N(fluidUpdate, 0);
	if (1)
	{
		return fluidUpdate_GS(sim, dt, pos_array);
	}

	const int num_fluid_particles = (int)sim->fluid_particle_idxs_.size();
	std::vector<PBDParticle>& p = sim->particles_;
	const std::vector<vec2>& x = pos_array;
	const auto& neigh_list = sim->neigh_data_.neigh_idx_;
	const float bs = 0.0f;

	for (int fi = 0; fi < num_fluid_particles; fi++) {
		const int i = sim->fluid_particle_idxs_[fi];
		for (const BoxBoundaryConstraint& c : sim->box_boundary_c_) {
			solve_box_boundary(c, x[i], i, sim);
		}
	}

	for (int fi = 0; fi < num_fluid_particles; fi++) {

		const int i = sim->fluid_particle_idxs_[fi];
		const int n_start = sim->neigh_data_.neighbour_info_[i].start;
		const int n_cnt = sim->neigh_data_.neighbour_info_[i].num;
		const PBDFluidModel& fm = sim->fluid_models_[p[i].phase];

		const float fmass = 1.0f / p[i].inv_mass;
		float density = sim->Wzero() * fmass;

		for (int ni = 0; ni < n_cnt; ni++) {
			int j = neigh_list[n_start + ni];

			float s = 1.0f;
			if (0 == (p[j].flags & PBDParticleFlags::kFluid)) {
				s = 1.0f;
			}

			// all particles participate in density calculations according to Eq. 27
			float m = 1.0f / p[j].inv_mass;
			//m = min(m, fmass);
			density += s * m * sim->W(x[i] - x[j]);
		}

		density += bs * getBoundaryDensity(sim, i, pos_array, fm.mass_);

		sim->fluid_particles_data_[p[i].fluid_data_idx].lambda_ = 0;
		const float mi = 1.0f / p[i].inv_mass;
		const float C = density / fm.density0_ - 1;
		if (C > 0)
        {
			float sum_grad_C_sq = 0;
			vec2 grad_Ci = vec2(0, 0); // first case when k = i

			for (int ni = 0; ni < n_cnt; ni++) {
				int j = neigh_list[n_start + ni];

				//if (0 == (p[j].flags & PBDParticleFlags::kFluid)) {
				//	continue;
				//}

				// looks like Eq. (7) and (8) miss mass multiplier
				float m = 1.0f / p[j].inv_mass;
				vec2 grad_Cj = -(m / fm.density0_) * sim->gradW(x[i] - x[j]);
				sum_grad_C_sq += lengthSqr(grad_Cj);
				grad_Ci += -grad_Cj;
			}

			if (1) {
				vec2 b_grad_Cj =
					-bs * (mi / fm.density0_) * getBoundaryGrad(sim, i, pos_array);
				sum_grad_C_sq += lengthSqr(b_grad_Cj);
				grad_Ci += -b_grad_Cj;
			}

			float sum_grad_sq = (sum_grad_C_sq + lengthSqr(grad_Ci));
			float lambda = -C / (sum_grad_sq + sim->e_);

			sim->fluid_particles_data_[p[i].fluid_data_idx].lambda_ = lambda;
		}
	}

	// solve fluid constraint
	for (int fi = 0; fi < num_fluid_particles; fi++) {
		const int i = sim->fluid_particle_idxs_[fi];
		const int n_start = sim->neigh_data_.neighbour_info_[i].start;
		const int n_cnt = sim->neigh_data_.neighbour_info_[i].num;
		const PBDFluidModel& fm = sim->fluid_models_[p[i].phase];

		vec2 dp = vec2(0, 0);
		const float lambda_i = sim->fluid_particles_data_[p[i].fluid_data_idx].lambda_;

		for (int ni = 0; ni < n_cnt; ni++) {
			int j = neigh_list[n_start + ni];

			if(0 == (p[j].flags & PBDParticleFlags::kFluid)) {
				// probably, should not influence fluid from rigid particles as fluid is
				// not influencing them as well
				//dp += (m / fm.density0_) * (lambda_i)*sim->gradW(x[i] - x[j]);
                continue;
            }

            assert(p[j].flags & PBDParticleFlags::kFluid);
            float lambda_j = sim->fluid_particles_data_[p[j].fluid_data_idx].lambda_;
            dp += (1.0f / (p[i].inv_mass * fm.density0_)) * (lambda_i + lambda_j) *
                sim->gradW(x[i] - x[j]);
		}

		// boundary
		float b_density = bs * getBoundaryDensity(sim, i, pos_array, fm.mass_);
		vec2 b_grad = bs * getBoundaryGrad(sim, i, pos_array);
		if (b_density) {
			dp += bs * (1.0f / (p[i].inv_mass * fm.density0_)) * (lambda_i)*b_grad;
		}

		sim->num_constraints_[i]++;
		sim->dp_[i] += dp;
	}

	for (int fi = 0; fi < num_fluid_particles; fi++) {
		const int i = sim->fluid_particle_idxs_[fi];
		assert(sim->num_constraints_[i]);
		pos_array[i] += (kSOR / sim->num_constraints_[i]) * sim->dp_[i];
		sim->num_constraints_[i] = 0;
		sim->dp_[i] = vec2(0);
	}
}

// so calles Gauss-Seidel style, when we update position right after calculating it, this
// make algorithm serial, but I think we can still parallelize this using atomics
//
// Note: for better buoyancy, we can scale down solid objects mass a bit 
void PBDUnifiedTimestep::fluidUpdate_GS(PBDUnifiedSimulation* sim, float dt,
									 const std::vector<vec2>& pos_array) {

    // precalculate denominator for lambda using min mass to make adjustments conservative
    vec2 center(0);
    float prec_sum_grad_C_sq = 0.0f;
    vec2 prec_grad_Ci(0.0f);

	if (0 == sim->fluid_models_.size()) return;

	float min_fluid_mass = sim->fluid_models_[0].mass_;
    float max_fluid_mass = sim->fluid_models_[0].mass_;
    for(const PBDFluidModel& fm:sim->fluid_models_) {
        min_fluid_mass = min(fm.mass_, min_fluid_mass);
        max_fluid_mass = max(fm.mass_, max_fluid_mass);
    }

    for(int y=-3; y< 3;++y) {
        for(int x=-3; x< 3;++x) {
            vec2 pos = center + vec2(x,y)*(2*sim->particle_r_);

            float m = sim->fluid_models_[0].mass_;
            float density = sim->fluid_models_[0].density0_;
			vec2 grad_Cj = -(m / density) * sim->gradW(center - pos);
			prec_sum_grad_C_sq += lengthSqr(grad_Cj);
            prec_grad_Ci += -grad_Cj;
        }
    }
	float prec_sum_grad_sq = (prec_sum_grad_C_sq + lengthSqr(prec_grad_Ci));
    (void)prec_sum_grad_sq;

	const int num_fluid_particles = (int)sim->fluid_particle_idxs_.size();
	std::vector<PBDParticle>& p = sim->particles_;
	const std::vector<vec2>& x = pos_array;
	const auto& neigh_list = sim->neigh_data_.neigh_idx_;

    // better for stability of corner particles than if using 1, anyway right now boundary density is very approx
	const float bs = .5f;

	for (int fi = 0; fi < num_fluid_particles; fi++) {

		const int i = sim->fluid_particle_idxs_[fi];

        solve_box_boundary(sim->box_boundary_c_[0], sim->x_pred_[i], i, sim);
        sim->x_pred_[i] += sim->dp_[i];
        sim->num_constraints_[i] = 0;
        sim->dp_[i] = vec2(0);

		const int n_start = sim->neigh_data_.neighbour_info_[i].start;
		const int n_cnt = sim->neigh_data_.neighbour_info_[i].num;
		const PBDFluidModel& fm = sim->fluid_models_[p[i].phase];

		const float fmass = 1.0f / p[i].inv_mass;
		float density = sim->Wzero() * fmass;

		for (int ni = 0; ni < n_cnt; ni++) {
			int j = neigh_list[n_start + ni];

			// all particles participate in density calculations according to Eq. 27
			float s = (p[j].flags & PBDParticleFlags::kFluid) ? 1.0f : 1.0f;
            
			float m = 1.0f / p[j].inv_mass;
			// do not select min mass as (instead use real) it prevents simulation to
			// converge (looks like)
			//m = min(fmass, m);
			density += s * m * sim->W(x[i] - x[j]);
		}

		density += bs * getBoundaryDensity(sim, i, pos_array, max_fluid_mass);

		sim->fluid_particles_data_[p[i].fluid_data_idx].lambda_ = 0;
		const float C = density / fm.density0_ - 1;
		const float mi = 1.0f / p[i].inv_mass;
		vec2 grad_Ci = vec2(0, 0); // first case when k = i
		if (C > 0)
        {
			float sum_grad_C_sq = 0;

			for (int ni = 0; ni < n_cnt; ni++) {
				int j = neigh_list[n_start + ni];

				float m = 1.0f / p[j].inv_mass;
				vec2 grad_Cj = -(m / fm.density0_) * sim->gradW(x[i] - x[j]);
				sum_grad_C_sq += lengthSqr(grad_Cj);
				grad_Ci += -grad_Cj;
			}

			if (1) {
				vec2 b_grad_Cj =
					-bs * (/*max_fluid_mass*/mi / fm.density0_) * getBoundaryGrad(sim, i, pos_array);
				sum_grad_C_sq += lengthSqr(b_grad_Cj);
				grad_Ci += -b_grad_Cj;
			}

			float sum_grad_sq = (sum_grad_C_sq + lengthSqr(grad_Ci));
			//sum_grad_sq = prec_sum_grad_sq;
			float lambda = -C / (sum_grad_sq + sim->e_);

			sim->fluid_particles_data_[p[i].fluid_data_idx].lambda_ = lambda;
		}

		const float lambda_i = sim->fluid_particles_data_[p[i].fluid_data_idx].lambda_;

		for (int ni = 0; ni < n_cnt; ni++) {
			int j = neigh_list[n_start + ni];

			if(0 == (p[j].flags & PBDParticleFlags::kFluid)) {
                // special behaviour for solid particles
                // would be better if actually use SDF for RBs normal calculation
                const float dist = length(x[i] - x[j]);
                const float d = max(0.0f, (2*sim->particle_r_ - dist));
                const float wi = 0 ? sim->inv_scaled_mass_[i] : p[i].inv_mass;
                const float wj = 0 ? sim->inv_scaled_mass_[j] : p[j].inv_mass;
			    sim->x_pred_[i] = sim->x_pred_[i] + (wi/(wi+wj)) * d * normalize(x[i] - x[j]);
			    sim->x_pred_[j] = sim->x_pred_[j] - (wj/(wi+wj)) * d * normalize(x[i] - x[j]);
            } 
			// TBH, having "else" is more correct, but adding pressure from fluid to all particles
			// (not only fluid ones) makes sim better
            //else
            {
				// for some precision reason using gr2 form instead of gr makes simulation
				// behave worse with a specially crafted scene
				// scene_fluid_simple_expose_bug, this could be fixed by multiplying gr by
				// some number close to 1.0 (e.g. 0.99999f) though. Anyhow it is sad that
				// difference in 0.0000001 can make a difference.
				vec2 gr =
					-(1.0f / (p[i].inv_mass * fm.density0_)) * sim->gradW(x[i] - x[j]);
#if 0
                vec2 gr2 = -(mi / fm.density0_) * sim->gradW(x[i] - x[j]);
                if (fabs(gr.x - gr2.x) > gr_eps) {
                    //printf("gr.x: %.8f gr2.x: %.8f\n", gr.x, gr2.x);
                }
                if (fabs(gr.y - gr2.y) > gr_eps) {
                    //printf("gr.y: %.8f gr2.y: %.8f\n", gr.y, gr2.y);
                }
#endif
                sim->x_pred_[j] = sim->x_pred_[j] + lambda_i * gr;
            }
		}

		sim->x_pred_[i] = sim->x_pred_[i] + lambda_i * grad_Ci;

		// boundary
		float b_density = bs * getBoundaryDensity(sim, i, pos_array, max_fluid_mass);
		vec2 b_grad = bs * getBoundaryGrad(sim, i, pos_array);
		if (b_density) {
            //const vec2 dp = bs * (1.0f / (p[i].inv_mass * fm.density0_)) * (lambda_i)*b_grad;
            const vec2 dp = bs * (/*max_fluid_mass*/mi / (fm.density0_)) * (lambda_i)*b_grad;
		    sim->x_pred_[i] = sim->x_pred_[i] + dp;
		}

        assert(sim->num_constraints_[i] == 0);
        assert(sim->dp_[i] == vec2(0));

	}

	for (int fi = 0; fi < num_fluid_particles; fi++) {
		const int i = sim->fluid_particle_idxs_[fi];
        solve_box_boundary(sim->box_boundary_c_[0], sim->x_pred_[i], i, sim);
        sim->x_pred_[i] += sim->dp_[i];
        sim->num_constraints_[i] = 0;
        sim->dp_[i] = vec2(0);
	}
}

const PBDRigidBodyParticleData* pbd_unified_sim_get_rb_particle_data(const struct PBDUnifiedSimulation* sim) {
    return sim->rb_particles_data_.data();
}

const PBDSoftBodyParticleData* pbd_unified_sim_get_sb_particle_data(const struct PBDUnifiedSimulation* sim) {
    return sim->sb_particles_data_.data();
}

uint32_t pbd_unified_sim_get_particle_flags(const struct PBDUnifiedSimulation* sim, int idx) {
    return sim->particles_[idx].flags;
}

const PBDFluidModel* pbd_unified_sim_get_fluid_particle_model(const struct PBDUnifiedSimulation* sim, int idx) {
    return &sim->fluid_models_[idx];
}

const PBDRigidBody* pbd_unified_sim_get_rigid_bodies(const struct PBDUnifiedSimulation* sim) {
    return sim->rigid_bodies_.data();
}

const PBDSoftBody& pbd_unified_sim_get_soft_body(int i, const struct PBDUnifiedSimulation* sim) {
    return sim->soft_bodies_.at(i);
}
const u32* pbd_unified_sim_get_soft_body_idxs(const struct PBDUnifiedSimulation* sim) {
    return sim->sb_idxs_.data();
}
const int pbd_unified_sim_get_soft_body_count(const struct PBDUnifiedSimulation* sim) {
    return (int)sim->sb_idxs_.size();
}
const PBDRegion* pbd_unified_sim_get_sb_regions(const struct PBDUnifiedSimulation* sim) {
    return sim->sb_regions_.data();
}
const u32* pbd_unified_sim_get_sb_regions_pidxs(const struct PBDUnifiedSimulation* sim) {
    return sim->sb_particles_per_region_.data();
}

const i32* pbd_unified_sim_get_distance_constraint_idxs(const struct PBDUnifiedSimulation* sim, int* count) {
    assert(count);
    *count = (int)sim->distance_c_idxs_.size();
    return sim->distance_c_idxs_.data();
}
const DistanceConstraint* pbd_unified_sim_get_distance_constraint(const struct PBDUnifiedSimulation* sim, int idx) {
    return &sim->distance_c_.at(idx);
}
const i32* pbd_unified_sim_get_distance2_constraint_idxs(const struct PBDUnifiedSimulation* sim, int* count) {
    assert(count);
    *count = (int)sim->distance2_c_idxs_.size();
    return sim->distance2_c_idxs_.data();
}
const DistanceConstraint2* pbd_unified_sim_get_distance2_constraint(const struct PBDUnifiedSimulation* sim, int idx) {
    return &sim->distance2_c_.at(idx);
}

vec2 pbd_unified_sim_get_world_bounds(const struct PBDUnifiedSimulation* sim) {
    return sim->world_size_;
}

const PBDSettings* pbd_unified_sim_settings(const struct PBDUnifiedSimulation* sim) {
    return &sim->settings;
}

static void add_dbg_info(int p0_idx, int p1_idx, vec2 norm, vec2 dp0, vec2 dp1, vec2 p0, vec2 p1, PBDUnifiedSimulation* sim) {
    sim->dbg_contacts_.emplace_back(DbgContactInfo{p0_idx, p1_idx, norm, dp0, dp1, p0, p1});
}

static void add_dbg_friction_info(int i, vec2 n, vec2 dx, float mu_s, float mu_k, vec2 s_path, vec2 d_path, PBDUnifiedSimulation* sim) {
    sim->dbg_friction_.emplace_back(DbgFrictionInfo{i, n, dx, mu_s, mu_k, s_path, d_path});
}

const DbgContactInfo* pbd_unified_sim_get_dbg_contacts(const struct PBDUnifiedSimulation* sim, int* count) {
    assert(count && sim);
    *count = (int)sim->dbg_contacts_.size();
    return sim->dbg_contacts_.data();
}

const DbgFrictionInfo* pbd_unified_sim_get_dbg_friction(const struct PBDUnifiedSimulation* sim, int* count) {
    assert(count && sim);
    *count = (int)sim->dbg_friction_.size();
    return sim->dbg_friction_.data();
}

