#version 330

in vec3 fragPosition;
in vec2 fragTexCoord;
in vec4 fragColor;
in vec3 fragNormal;

uniform sampler2D texture0;
uniform vec4 colDiffuse;
float ambient = 0.2f;
vec3 lightPos = vec3(40, 50, -60);

out vec4 finalColor;

void main()
{
    vec3 dir = normalize(lightPos - fragPosition);
    float diffuse = max(dot(fragNormal, dir), ambient);
    finalColor = vec4(vec3(0.8f,0.8f,0.8f) * diffuse, 1.0f);
    //finalColor = vec4(fragColor.xyz * diffuse, 1.0f);
    //finalColor = vec4(1.0f, 1.0f, 1.0f, 10.0f / 255); //overdraw visualization
}
