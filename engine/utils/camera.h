#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <memory.h>
#include "utils/vec.h"

struct camera
{
    camera();

	void set_projection(const float fov, const int width, const int height, const float near, const float far);
	void set_ortho_projection(const float l, const float r, const float t, const float b, const float near, const float far);
	void get_projection(mat4* proj) const { *proj = proj_; }
	const mat4& get_projection() const { return proj_; }
	const mat4& get_inv_projection() const { return inv_proj_; }
	void update(float dt);
	void get_pos(float (*p)[4] ) const; 
	void set_pos(const vec3& world_pos);
	void get_view_proj_inv(mat4* vpi) const { *vpi = view_proj_inv_; }
	void get_view(mat4* view) const { *view = view_; }
	const mat4& get_view() const { return view_; }
	const mat4& get_inv_view() const { return inv_view_; }
	
	void set_view(const mat4& view_mat);

    static void compose_view_matrix(mat4* view, const vec3& right, const vec3& up, const vec3& front, const vec3& world_pos);
	static void compose_view_matrix(mat4* view, const float (& mat)[3*4]);
	static void view_get_world_pos(const mat4& view, vec3* world_pos);

//private:

    float rot_x;
    float rot_y;
    float dist;
	float dx;
	float dy;
	float dz;
	float move_scale;

	float pos_[4];
	float lookat_vec[4];
	float right_vec[4];
	float up_vec[4];


    float get_fov() const { return fov_; }
    float get_aspect() const { return (float)width_/(float)height_; }
    float get_near() const { return near_; }
    float get_far() const { return far_; }

private:

	mat4 proj_;
	mat4 inv_proj_;
	mat4 view_;
	mat4 inv_view_;
	mat4 world_;
	mat4 view_proj_inv_;


	void set_projection(const mat4& proj);

    float fov_;
    float width_, height_;
    float left_, right_, top_, bottom_;
    float near_;
    float far_;
    bool is_perspective_;
};


#endif // __CAMERA_H__
