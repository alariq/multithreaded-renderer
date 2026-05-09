//#version 450

// material to draw debug lines, and other stuff
//

#define PREC highp

layout (location=0) out PREC vec4 FragColor;

layout(location=0) in PsIn {
    PREC vec2 texcoord;
    PREC vec3 normal;
} In;

uniform sampler2D tex1;

uniform mat4 wv_;
uniform mat4 wvp_;
uniform vec4 lightdir_view_space_;

void main(void)
{
    PREC vec4 albedo = texture(tex1, In.texcoord);
    mat4 normal_tr = transpose(inverse(wv_));

    vec3 normal = normalize((normal_tr * vec4(In.normal, 0.0)).xyz);

    float ndotl = max(0.2, dot(normal, -lightdir_view_space_.xyz));
    float intensity = 1.0;
    vec3 diffuse = albedo.rgb * ndotl * intensity;
    //TODO: move to a single place (duplicatd deferred.frag)
    vec3 ambient = vec3(0.1, 0.05, 0.05);

    vec3 c = albedo.rgb;// + ambient;
    FragColor = vec4(c*albedo.w, albedo.w);
}

