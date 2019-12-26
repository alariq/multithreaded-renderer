//#version 450
#define PREC highp

layout (location=0) out PREC vec4 FragColor;


layout(location=0) in PsIn {
    PREC vec2 texcoord;
    PREC vec3 normal;
    PREC vec3 viewpos;
} Input;

uniform sampler2D tex1; // albedo
uniform sampler2D tex2; // normal
uniform sampler2D tex3; // depth

uniform vec4 color_;
uniform vec4 viewpos_radius_;
//uniform vec4 lightviewpos_;
uniform mat4 proj_;


in vec4 gl_FragCoord;

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

    vec2 tc = gl_FragCoord.xy;
    ivec2 tsize = textureSize(tex1, 0);
    tc = tc / vec2(tsize.x, tsize.y);

	PREC vec3 albedo = texture(tex1, tc).rgb;
	PREC vec3 normal = texture(tex2, tc).rgb;
	PREC float depth = texture(tex3, tc).r;

    normal = 2.0*normal - vec3(1.0);

    // reconstruct position
    vec3 pixel_viewpos = unproject(depth, Input.viewpos, proj_);
    vec3 light_viewpos = viewpos_radius_.xyz;

    vec3 lightdir = normalize(light_viewpos - pixel_viewpos);

    float dist = length(pixel_viewpos.xyz - light_viewpos.xyz);
    float r = viewpos_radius_.w;
    float fade = 1 - clamp(dist / r, 0.0, 1.0);

    float ndotl = max(0.0, dot(normal, lightdir));
    float intensity_scale = 1.0;
    vec3 diffuse = 
        fade *
        color_.xyz * 
        ndotl * (intensity_scale * color_.w);

    FragColor = vec4(diffuse, 1.0);
	//FragColor = vec4(fade.xxx*ndotl , 1);
	//FragColor = vec4(dist < viewpos_radius_.w?1:0, 0, 0 , 1);

}
