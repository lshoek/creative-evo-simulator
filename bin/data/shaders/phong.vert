#version 450

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;
uniform mat4 modelViewMatrix;
uniform mat4 projectionMatrix;
uniform mat4 textureMatrix;
uniform mat4 modelViewProjectionMatrix;

in vec4 position;
in vec4 color;
in vec4 normal;
in vec2 texcoord;

out vec4 eye;

out vec4 normal_varying;
out vec4 color_varying;
out vec2 texcoord_varying;

void main()
{
    vec4 view_vert = modelViewMatrix * position;
    eye = -view_vert;

	texcoord_varying = texcoord;
	normal_varying = vec4(normalize(normal.xyz), 0.0);

    gl_Position = modelViewProjectionMatrix * position;
}
