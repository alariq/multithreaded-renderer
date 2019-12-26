//#version 450
#define PREC highp

layout (location=0) out PREC vec4 FragColor;

uniform sampler2D tex1;
uniform sampler2D tex2;
uniform sampler2D tex3;

uniform vec4 lightdir_view_space_;
uniform mat4 proj_;

in PREC vec2 o_pos;
in PREC vec2 o_uv;
in PREC vec3 o_viewray;

// https://mynameismjp.wordpress.com/2010/09/05/position-from-depth-3/
vec3 unproject(float z_buf_depth, vec3 viewpos, mat4 proj) {
    // set view space position to the plane at z = 1
    // i.e. scale so that viewray has z equaled to 1, then we can just multiply
    // by real view z and get point coordinate
    vec3 viewray = vec3(viewpos.x / viewpos.z, viewpos.y / viewpos.z, 1.0f);

    // z_ndc same thing as z post projection, just a different name
    float z_ndc = 2.0*z_buf_depth - 1; // may be influenced by glDepthRange

    // post projection Z -> view Z
    // post projection Z is Zproj / Wproj , where Wproj is Zview
    float view_space_depth = proj[3][2] / (z_ndc - proj[2][2]);

    vec3 pos_view_space = viewray * view_space_depth;
    return pos_view_space;
}


void main(void)
{

	PREC vec3 albedo = texture(tex1, o_uv).rgb;
	PREC vec3 normal = texture(tex2, o_uv).rgb;
	PREC float depth = texture(tex3, o_uv).r;

    normal = 2.0*normal - vec3(1.0);

    float ndotl = max(0.2, dot(normal, -lightdir_view_space_.xyz));
    float intensity = 1.2;
    vec3 diffuse = albedo * ndotl * intensity;
    vec3 ambient = vec3(0.1, 0.05, 0.05);

    vec3 pixel_viewpos = unproject(depth, o_viewray, proj_);

	FragColor = vec4(diffuse + ambient, 1.0);
	//FragColor = vec4(pixel_viewpos.zzz/100, 1.0);
}

