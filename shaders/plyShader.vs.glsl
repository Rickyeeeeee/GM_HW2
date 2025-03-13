#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;

uniform mat4 projection_matrix;
uniform mat4 view_matrix;
uniform mat4 model_matrix;

out vec3 fragPos;
out vec3 Normal;
out vec3 vertexColor;

void main() {
    Normal = mat3(transpose(inverse(model_matrix))) * aNormal;
    vec4 worldPos = model_matrix * vec4(aPos, 1.0);
    fragPos = worldPos.xyz;
    vertexColor = aColor;
    gl_Position = projection_matrix * view_matrix * worldPos;
}
