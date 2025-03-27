#version 330 core

in vec3 Normal;
in vec3 fragPos;
in vec3 vertexColor;

uniform vec3 color;
out vec4 FragColor;

uniform vec3 viewPos;


// Function to generate a rainbow color based on position
vec3 rainbowColor(float value) {
    value = fract(value); // Keep it between 0 and 1
    float r = abs(sin(value * 6.2831));  // Red channel
    float g = abs(sin((value + 0.333) * 6.2831));  // Green channel
    float b = abs(sin((value + 0.666) * 6.2831));  // Blue channel
    return vec3(r, g, b);
}

void main() {
    vec3 normal = normalize(Normal);
    vec3 lightDir = normalize(vec3(1.0, 1.0, 1.0));

    // **Position-based rainbow color**
    vec3 baseColor = rainbowColor(fragPos.y * 2.0 + fragPos.x * 2.0 + fragPos.z * 2.0);

    // **Ambient lighting**
    vec3 ambient = 0.2 * baseColor;

    // **Diffuse lighting**
    float diff = max(dot(lightDir, normal), 0.0);
    vec3 diffuse = diff * baseColor * 1.2;

    // **Iridescent Specular Highlights**
    vec3 viewDir = normalize(viewPos - fragPos);
    vec3 halfwayDir = normalize(lightDir + viewDir);
    float spec = pow(max(dot(normal, halfwayDir), 0.0), 64.0);
    vec3 specular = rainbowColor(dot(viewDir, normal) * 0.5 + 0.5) * spec * 0.9;

    // **Neon Rim Lighting**
    float rim = pow(1.0 - max(dot(viewDir, normal), 0.0), 2.0);
    vec3 rimLight = rainbowColor(fragPos.y * 0.1) * rim * 1.5;

    // **Final color mix**
    vec3 finalColor = ambient + diffuse + specular + rimLight;

    // **Gamma correction**
    finalColor = pow(finalColor, vec3(1.0 / 2.2));

    FragColor = vec4(finalColor, 1.0);
}