#version 450

layout(location = 0) in vec3 vcol;
layout(location = 0) out vec4 fcol;

void main() {
    fcol = vec4(vcol, 1.0);
}
