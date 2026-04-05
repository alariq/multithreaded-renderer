#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <memory.h>
#include "utils/vec.h"
#include "utils/imgui_property_list.h"

struct camera //: public imgui_props::IPolymorphicPropertyObject
{
    PROPERTY_SUPPORT(camera);
    //PROPERTY_POLYMORPHIC_DRAW_IMPL(camera);
    camera();

	void set_projection(const float fov, const int width, const int height, const float near, const float far);
	void set_fov(const float fov);
	void set_ortho_projection(const float l, const float r, const float t, const float b, const float near, const float far);
	void get_projection(mat4* proj) const { *proj = proj_; }
	const mat4& get_projection() const { return proj_; }
	const mat4& get_inv_projection() const { return inv_proj_; }
    static vec3 unproject_vec(const vec2& p, bool b_perspective, const mat4& inv_view, const mat4& inv_proj);
    static vec3 unproject(const vec2& p, float at_view_z, bool b_perspective, const mat4& inv_view, const mat4& inv_proj);
    // returns world-space position
    vec3 unproject(const vec2& p, float at_view_z) const {
        return unproject(p, at_view_z, is_perspective_, inv_view_, inv_proj_);
    }
    // returns world-space vector
    vec3 unproject_vec(const vec2& p) const {
        return unproject_vec(p, is_perspective_, inv_view_, inv_proj_);
    }
	void update(float dt);
	vec3 get_pos() const; 
	void set_pos(const vec3& world_pos);
	void get_view_proj_inv(mat4* vpi) const { *vpi = view_proj_inv_; }
	void get_view(mat4* view) const { *view = view_; }
	const mat4& get_view() const { return view_; }
	const mat4& get_inv_view() const { return inv_view_; }
	
	void set_view(const mat4& view_mat);
    void lookat(const vec3& eye, const vec3& target, const vec3& up_dir);

    vec3 right() const { return view_.getRightVec(); }
    vec3 fwd() const { return view_.getForwardVec(); }
    vec3 up() const { return view_.getUpVec(); }

    static void compose_view_matrix(mat4* view, const vec3& right, const vec3& up, const vec3& front, const vec3& world_pos);
	static void compose_view_matrix(mat4* view, const float (& mat)[3*4]);
	static void view_get_world_pos(const mat4& view, vec3* world_pos);

    static mat4 make_lookat(const vec3& eye, const vec3& target, const vec3& up_dir);

//private:

    float get_fov() const { return fov_; }
    float get_aspect() const { return (float)width_/(float)height_; }
    float get_near() const { return near_; }
    float get_far() const { return far_; }
    bool get_is_perspective() const { return is_perspective_; }

private:

	vec3 wpos_;

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

PROPERTY_LIST_DECLARE(camera);
// have it in the header because it becomes unresolved even though it is explicitly instanciated
// but because "engine" is linked as a static lib, it somehow lost (probably because not referenced anywhere
// in "engine". If I add PROPERTY_POLYMORPHIC_DRAW_IMPL() to camera class (have to derive from 
// imgui_props::IPolymorphicPropertyObject) then is it fine, because then GetPropertyList<camera> is
// actually used.
PROPERTY_LIST_BEGIN(camera)
    PROPERTY_READONLY_TEXT("Name", [](const camera& c) { return c.is_perspective_? "Persp" : "Ortho"; });
    PROPERTY_VEC3(wpos_, "WorldPos");
    PROPERTY_FLOAT(fov_, "FOV", 0, 30.0f, 120.0f, 1.0f, nullptr, [](camera& c, float fov){ c.set_fov(fov); });
    PROPERTY_BOOL(is_perspective_, "IsPerspective");
PROPERTY_LIST_END()


class fps_camera {

    mat4 view_;
	float move_scale;
	vec3 pos_;

    public:

    fps_camera():rot_x(0), rot_y(0)
    {
        dx = dy = dz = 0;
        move_scale = .1f;
        pos_ = vec3(0,0,0);

        view_ = mat4::identity();
    }

    void update(float dt);
    void set_pos(const vec3& pos) { pos_ = pos; }

    mat4& get_view() { return view_; }
    
    float rot_x;
    float rot_y;

	float dx;
	float dy;
	float dz;

};

class ortho_camera {

    mat4 view_;
    int proj_idx_;

    //vec3 init_pos_;
    vec3 pos_;

    public:

    static const int NUM_VIEWS = 3;
    static const constexpr vec3 la_targets[NUM_VIEWS] = { vec3(-1,0,0), vec3(0,0,-1), vec3(0,-1,0) };
    static const constexpr vec3 la_ups[NUM_VIEWS] = { vec3(0,1,0), vec3(0,1,0), vec3(0,0,1) };

    float dx, dy, dz;

    ortho_camera() { 
        view_ = mat4::identity();
        dx = dy = dz = 0;
        pos_ = vec3(0);
        proj_idx_ = 0;
    }

    void cycle_proj();

    void set_proj_idx(int i);
    void set_pos(const vec3& pos) { pos_ = pos; }
    void update(float dt);
    mat4 get_view() const { return view_ ; }
};


#endif // __CAMERA_H__
