#include <assert.h>

#include "utils/vec.h"
#include "utils/camera.h"
#include "utils/matrix.h"
#include "utils/math_utils.h"
#include "utils/logging.h"

//#include <GL/glew.h>
//#include <graphics/gl_utils.h>
#include "utils/matrix.h"

PROPERTY_LIST_BEGIN(camera)
    PROPERTY_READONLY_TEXT("Name", [](const camera& c) { return c.is_perspective_ ? "Persp" : "Ortho"; });
    PROPERTY_VEC3(wpos_, "WorldPos");
    PROPERTY_FLOAT(fov_, "FOV", 0, 30.0f, 120.0f, 1.0f, nullptr, [](camera& c, float fov){ c.set_fov(fov); });
    PROPERTY_BOOL(is_perspective_, "IsPerspective");
    PROPERTY_BOOL(b_rh, "RightHanded");
PROPERTY_LIST_END()

camera::camera()
{
	proj_ = mat4::identity();
	inv_proj_ = mat4::identity();
	view_ = mat4::identity();
	inv_view_ = mat4::identity();
	world_ = mat4::identity();
	view_proj_inv_ = mat4::identity();

    wpos_ = inv_view_.getTranslation();

    fov_ = 0;
    width_ = height_ = 0;
    left_ = right_ = top_ = bottom_ = 0;
    near_ = far_ = 0;
    is_perspective_ = false;

}

vec3 camera::get_pos() const { return wpos_; }

void camera::set_projection(const mat4& proj)
{
	proj_ = proj;
	float inv[16];
	glu_InvertMatrixf((const float*)proj_, inv);
	mat4 invProj( 
		inv[0], inv[1], inv[2], inv[3],
		inv[4], inv[5], inv[6], inv[7],	
		inv[8], inv[9], inv[10], inv[11],	
		inv[12], inv[13], inv[14], inv[15]
	);

	inv_proj_ = invProj;
}

void camera::set_projection(const float fov, const int w, const int h, const float near, const float far)
{
    width_ = (float)w;
    height_ = (float)h;
    fov_ = fov;
    near_ = near; far_ = far;
#if 1
    mat4 pm = perspectiveMatrix(fov * 3.1415f / 180.0f, w, h, near, far, b_rh, false);
#else
    const float aspectRatio = (float)h/w;
    const float fovy = aspectRatio * fov;
    const float DEG2RAD = M_PIf / 180.0f;
    float tangent = tan(fovy/2 * DEG2RAD);    // tangent of half fovY
    float top = near * tangent;              // half height of near plane
    float right = top * aspectRatio;          // half width of near plane
    mat4 pm = frustumProjMatrix(-right, right, -top, top, near, far);
#endif

    is_perspective_ = true;
    this->set_projection(pm);

}
void camera::set_ortho_projection(const float l, const float r, const float t, const float b, const float near, const float far)
{
    top_ = t; bottom_ = b; left_ = l; right_ = r;
    near_ = near; far_ = far;
    mat4 pm = orthoMatrix(l, r, t, b, near, far, false);
    is_perspective_ = false;
    this->set_projection(pm);
}

void camera::set_fov(const float fov)
{
    fov_ = fov;
    set_projection(fov_, width_, height_, near_, far_);
}

void camera::set_view(const mat4& view_mat)
{
	view_ = view_mat;

	vec3 wp;
	camera::view_get_world_pos(view_, &wp);
	
	inv_view_ = view_;

    // invert
    inv_view_.setCol3(vec4(0,0,0,1));
	inv_view_ = transpose(inv_view_);
    inv_view_.setCol3(vec4(wp.x,wp.y,wp.z, 1));
	
	view_proj_inv_ = inv_view_*inv_proj_;

	wpos_ = inv_view_.getTranslation();

}

void camera::compose_view_matrix(mat4* view, const float (& mat)[3*4])
{
	vec3 pos(mat[3], mat[7], mat[11]);

	vec3 right = vec3(mat[0], mat[1], mat[2]);
	vec3 up = vec3(mat[4], mat[5], mat[6]);
	vec3 front = vec3(mat[8], mat[9], mat[10]);

	view->setRow(0, vec4(right, dot(-pos, right)));
	view->setRow(1, vec4(up, dot(-pos, up)));
	view->setRow(2, vec4(front, dot(-pos, front)));
}

void camera::compose_view_matrix(mat4* view, const vec3& right, const vec3& up, const vec3& front, const vec3& world_pos)
{
	view->setRow(0, vec4(right, dot(-world_pos, right)));
	view->setRow(1, vec4(up, dot(-world_pos, up)));
	view->setRow(2, vec4(front, dot(-world_pos, front)));
}

void camera::lookat(const vec3& eye, const vec3& target, const vec3& up_dir)
{
    mat4 view = make_lookat(eye, target, up_dir);
    set_view(view);
}

mat4 camera::make_lookat(const vec3& eye, const vec3& target, const vec3& up_dir) {

    vec3 fwd = target - eye;
    float fwd_len = length(fwd);
    if(fwd_len < 1e-9) {
        fwd = vec3(0,0,1);
    } else {
        fwd = fwd / fwd_len;
    }

    vec3 up = up_dir;
    vec3 right = cross(up, fwd);
    float right_len = length(right);
    if( right_len < 1e-9) {
        calculate_basis(fwd, up, right);
    } else {
        right = normalize(right);
        up = cross(fwd, right);
    }

    mat4 view = mat4::identity();
    compose_view_matrix(&view, right, up, fwd, eye);
    return view;
}

// https://registry.khronos.org/OpenGL-Refpages/gl2.1/xhtml/gluLookAt.xml
// Because OpenGL uses RH, it means than with Identity camera matrix it will look at -Z 
// So if one sets eye = (0,0,0) and target = (0,0,-1) this will produce Identity matrix
mat4 camera::make_lookat_opengl(const vec3& eye, const vec3& target, const vec3& up_dir) {

    vec3 fwd = target - eye;
    float fwd_len = length(fwd);
    if(fwd_len < 1e-9) {
        fwd = vec3(0,0,1);
    } else {
        fwd = fwd / fwd_len;
    }

    vec3 up = up_dir;
    vec3 right = cross(fwd, up);
    float right_len = length(right);
    if( right_len < 1e-9) {
        calculate_basis(fwd, up, right);
    } else {
        right = normalize(right);
        up = cross(right, fwd);
    }

    mat4 view = mat4::identity();
    compose_view_matrix(&view, right, up, -fwd, eye);
    return view;
}

void camera::view_get_world_pos(const mat4& view, vec3* world_pos)
{
	vec3 view_p = view.getTranslation();
	vec3 rxuxfx = view.getCol0().getXYZ(); 
	vec3 ryuyfy = view.getCol1().getXYZ();
	vec3 rzuzfz = view.getCol2().getXYZ();

	world_pos->x = -dot(view_p, rxuxfx);
	world_pos->y = -dot(view_p, ryuyfy);
	world_pos->z = -dot(view_p, rzuzfz);
}

void camera::set_pos(const vec3& world_pos)
{
	vec3 right = view_.getRow(0).getXYZ();
	vec3 up = view_.getRow(1).getXYZ();
	vec3 front = view_.getRow(2).getXYZ();

	view_.setElem(3, 0, dot(-world_pos, right));
	view_.setElem(3, 1, dot(-world_pos, up));
	view_.setElem(3, 2, dot(-world_pos, front));

	wpos_ = world_pos;
}

vec3 camera::unproject_vec(const vec2& p, bool b_perspective, const mat4& inv_view, const mat4& inv_proj) {
    vec3 view_pos;
    if(b_perspective) {
        view_pos = normalize((inv_proj * vec4(p.x, p.y, 0, 1)).xyz());
    } else {
        // unproject_vec makes little sense in case fo parallel projection
        // just return forward vec?
        view_pos = normalize((inv_proj * vec4(0, 0, 1, 1)).xyz());
    }
    vec3 wpos = (inv_view * vec4(view_pos, 0.0f)).xyz();
    return wpos;
}

// returns world position
vec3 camera::unproject(const vec2& p, float at_view_z, bool b_perspective, const mat4& inv_view, const mat4& inv_proj) {
    vec3 view_pos;
    if(b_perspective) {
        view_pos = (inv_proj * vec4(p.x, p.y, 0, 1)).xyz();
        view_pos = at_view_z * normalize(view_pos);
    } else {
        view_pos = (inv_proj * vec4(p.x, p.y, at_view_z, 1)).xyz();
        view_pos.z = at_view_z;
    }
    vec3 wpos = (inv_view * vec4(view_pos, 1.0f)).xyz();
    return wpos;
}

float camera::getXrot(const mat4& view) {
    const vec2 fwd_hor = view.getForwardVec().xz();
    //const vec2 fwd_id(0,1); // 0, 0, 1 but with removed Y component
    //float angle = atan2(cross(fwd_id, fwd_hor), dot(fwd_id, fwd_hor));
    // or use angle_between_vecors()

    // this is basically same as above, we just rotate vector 90 degree, 
    // because identity vector is basicaly 1,0,0 rotated 90 around Y to be 0,0,-1
    vec2 v = vec2(fwd_hor.y, -fwd_hor.x); // rotate 90 
    return atan2(v.y, v.x);
}

void fps_camera::update(float dt) {

	mat4 rotX = rotateY4(rot_x);
	mat4 rotY = rotateX4(rot_y);

	mat4 matrot = rotY*rotX;

	pos_ += dx*matrot.getRightVec();
	pos_ += dy*matrot.getUpVec();
	pos_ += dz*matrot.getForwardVec();

	// update view matrix
	view_ = mat4::identity();
	view_.setRow(0, vec4(matrot.getRightVec(), dot(-pos_, matrot.getRightVec() )));
	view_.setRow(1, vec4(matrot.getUpVec(), dot(-pos_, matrot.getUpVec() )));
	view_.setRow(2, vec4(matrot.getForwardVec(), dot(-pos_,matrot.getForwardVec() )));

	dx = dy = dz = 0;
}



void ortho_camera::cycle_proj() {
    proj_idx_ = proj_idx_ + 1 % NUM_VIEWS;
}

void ortho_camera::set_proj_idx(int i) {
    proj_idx_ = clamp(i, 0, NUM_VIEWS-1);
}

void ortho_camera::update(float dt) {

    mat4 view = camera::make_lookat(vec3(0), targets[proj_idx_], ups[proj_idx_]);

    pos_ = project_vector_on_plane(pos_, vec4(view.getForwardVec(), 0)); 
    pos_ += dx * view.getRightVec();
    pos_ += dy * view.getUpVec();

    //if(WheelDelta) {
    //    cam_proj += view_.getForwardVec() * (WheelDelta>0 ? 20 : -20);
    //}

    // get far enough (TODO: use scroll instead of hardcoded value)
    pos_ += -200 * view.getForwardVec();

	// update view matrix
	view_ = mat4::identity();
	view_.setRow(0, vec4(view.getRightVec(), dot(-pos_, view.getRightVec() )));
	view_.setRow(1, vec4(view.getUpVec(), dot(-pos_, view.getUpVec() )));
	view_.setRow(2, vec4(view.getForwardVec(), dot(-pos_, view.getForwardVec() )));

    dx = dy = dz = 0;

}

