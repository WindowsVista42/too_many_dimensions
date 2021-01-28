#version 460

layout(location = 0) in vec2 aflowpos;
layout(location = 1) in vec2 aflowvel;
layout(location = 2) in vec2 apos;

layout(set = 0, binding = 0)
uniform Uniforms {
    vec2 u_view_pos;
    vec2 u_view_scl;
};

void main() {
    vec2 vpos = apos + aflowpos - u_view_pos;
    vpos *= u_view_scl;
    gl_Position = vec4(vpos, 0, 1);
}