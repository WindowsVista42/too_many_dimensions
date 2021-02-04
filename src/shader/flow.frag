#version 460

// Filmic Tonemapping Operators http://filmicworlds.com/blog/filmic-tonemapping-operators/
// Gamma correction is built-in
vec3 filmic(vec3 x) {
    vec3 X = max(vec3(0.0), x - 0.004);
    vec3 result = (X * (6.2 * X + 0.5)) / (X * (6.2 * X + 1.7) + 0.06);
    return result;
}

/*
// Ref Uniforms in flow.rs
layout(set = 0, binding = 0) uniform Uniforms {
    float dt;
    uint  ct;

    float flow_ext;
    float flow_acc;
    float flow_max;
    float flow_scl;
    float flow_jit;
    vec2  flow_off;

    float coll_scl;

    uint  mani_ct;
    float mani_acc;
    float mani_spd;

    uint  spaw_ct;
    float spaw_rte;
    float spaw_scl;
    float spaw_var;
    vec3  spaw_col;

    uint  accu_ct;
    float accu_rte;
    float accu_scl;
};
*/

// Lol what you can just insert these at random
layout(location = 0) in vec3 v_col; // Linear space

layout(location = 0) out vec4 f_col; // Srgb space

void main() {
    vec3 col = filmic(v_col);
    f_col = vec4(col, 0.15);
}