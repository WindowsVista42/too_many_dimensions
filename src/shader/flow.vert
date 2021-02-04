#version 460

#define _PI 3.141592653589793238

layout(location = 0) in vec2 aflowpos; // Flow position
layout(location = 1) in vec2 aflowvel; // Flow velocity
layout(location = 2) in vec2 apos; // Vertex position

// Lol what you can just insert these at random
layout(location = 0) out vec3 v_col;

layout(set = 0, binding = 0)
uniform Uniforms {
    vec2 u_view_pos; // View position
    vec2 u_view_scl; // View scale
};

void main() {
    const float angle = -atan(aflowvel.x, aflowvel.y);
    const vec2 turn = vec2(
        apos.x * cos(angle) - apos.y * sin(angle),
        apos.x * sin(angle) + apos.y * cos(angle)
    );
    const vec2 scl = vec2(
        map(clamp(abs(aflowvel.x), 0.0, 1.0), 0.0, 1.0, 0.8, 1.0),
        map(clamp(abs(aflowvel.y), 0.0, 1.0), 0.0, 1.0, 0.8, 1.0)
    );
    vec2 vpos = (turn * scl) + aflowpos - u_view_pos;
    vpos *= u_view_scl;

    const vec3 col = vec3(
        pow((sin(angle) + 1.0) * 0.5, 16.0),
        (sin(angle) + 1.0) * 0.5 + (cos(angle) + 1.0) * 0.7,
        2.0
    );

    v_col = col.rgb;
    gl_Position = vec4(vpos, 0.0, 1.0);
}