#version 450

layout(location = 0) in vec2 apos;
layout(location = 1) in vec3 acol;
layout(location = 0) out vec3 vcol;

layout(set = 0, binding = 0)
uniform Uniforms {
    vec2 u_view_pos;
    vec2 u_view_scl;
};

void main() {
    vcol = acol;

    vec2 vpos = apos - u_view_pos;
    vpos *= u_view_scl;
    gl_Position = vec4(vpos, 0.0, 1.0);
}
