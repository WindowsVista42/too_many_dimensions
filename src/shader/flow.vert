#version 460

#define PI 3.141592653589793238

layout(location = 0) in vec2 aflowpos; // Flow position
layout(location = 1) in vec2 aflowvel; // Flow velocity
layout(location = 2) in vec2 apos; // Vertex position

layout(set = 0, binding = 0)
uniform Uniforms {
    vec2 u_view_pos; // View position
    vec2 u_view_scl; // View scale
};

void main() {
    const float angle = -atan(aflowvel.x, aflowvel.y);
    const vec2 test = vec2(
        apos.x * cos(angle) - apos.y * sin(angle),
        apos.x * sin(angle) + apos.y * cos(angle)
    ) * 0.5 + 0.3;
    vec2 vpos = test + apos + aflowpos - u_view_pos;
    vpos *= u_view_scl;
    gl_Position = vec4(vpos, 0.0, 1.0);
}