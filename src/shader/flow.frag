#version 460

#define INV_GAMMA 1.0 / 2.2

// Filmic Tonemapping Operators http://filmicworlds.com/blog/filmic-tonemapping-operators/
vec3 filmic(vec3 x) {
    vec3 X = max(vec3(0.0), x - 0.004);
    vec3 result = (X * (6.2 * X + 0.5)) / (X * (6.2 * X + 1.7) + 0.06);
    return result;
}

// Lol what you can just insert these at random
layout(location = 0) in vec3 v_col; // Linear space

layout(location = 0) out vec4 f_col; // Srgb space

void main() {
    vec3 col = filmic(v_col);
    //col = pow(col, vec3(INV_GAMMA));
    f_col = vec4(col, 0.15);
}