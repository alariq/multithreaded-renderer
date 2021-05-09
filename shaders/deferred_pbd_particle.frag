//#version 450
#extension GL_ARB_explicit_uniform_location: enable
#extension GL_ARB_explicit_attrib_location: enable

#define PREC highp

layout (location=0) out PREC vec4 g_buffer_albedo;
layout (location=1) out PREC vec4 g_buffer_normal;

layout(location=0) in PsIn {
    PREC vec2 texcoord;
    PREC vec3 normal;
    PREC vec2 force;
    PREC float density;
    PREC float pressure;
    flat PREC uint flags;
} Input;

#define kPBDFlagRigidBody 0x01u
#define kPBDFlagSleep 0x10u

layout (binding = 0, location=0) uniform sampler2D tex1;
layout (location = 1) uniform mat4 wv_;

void main(void)
{
	PREC vec4 albedo = texture(tex1, vec2(Input.texcoord.x, 1-Input.texcoord.y));

    if(uint(Input.flags & kPBDFlagSleep) != 0u)
        albedo.rgb = vec3(1,1,0);

    if(uint(Input.flags & kPBDFlagRigidBody ) != 0u)
        albedo.rgb = vec3(0,0,1);

    g_buffer_albedo = albedo;
    PREC float scaler = 0.1;
    mat4 scale = mat4(
                scaler, 0.0, 0.0, 0.0, 
                0.0, scaler, 0.0, 0.0, 
                0.0, 0.0, scaler, 0.0, 
                0.0, 0.0, 0.0, 1.0); 
    //TODO: optimize
    mat4 normal_wv = transpose(inverse(wv_));
    g_buffer_normal.rgb = 0.5*normalize((normal_wv * vec4(Input.normal, 0.0)).xyz) + 0.5;
    g_buffer_normal.w = 1.0;

    //g_buffer_albedo = vec4(g_buffer_normal.rgb*0.5+0.5, 1.0);
}

