#version 450

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;
uniform mat4 modelViewMatrix;
uniform mat4 projectionMatrix;
uniform mat4 textureMatrix;
uniform mat4 modelViewProjectionMatrix;

uniform mat4 lightSpaceMatrix;

in vec4 position;
in vec4 color;
in vec4 normal;
in vec2 texcoord;

out vec4 eye;
out vec4 fragPos;
out vec4 fragPosLightSpace;

out vec4 normal_varying;
out vec4 color_varying;
out vec2 texcoord_varying;

void main()
{
    vec4 view_vert = modelViewMatrix * position;
    eye = -view_vert;

    fragPos = modelMatrix * position;
    fragPosLightSpace = lightSpaceMatrix * vec4(fragPos.xyz, 1.0);

	texcoord_varying = texcoord;

	mat3 normalMatrix = mat3(transpose(inverse(modelMatrix)));
	vec3 normal_transformed = normalMatrix * normalize(normal.xyz);
	normal_varying = vec4(normal_transformed, 0.0);

    gl_Position = modelViewProjectionMatrix * position;
}
