#version 460

layout(location = 0) out vec4 f_col;

void main() {
    vec3 col = vec3(1.0, 1.0, 1.0);
    f_col = vec4(col, 0.1);
}