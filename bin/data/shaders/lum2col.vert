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

out vec2 texcoord_varying;

void main()
{
	texcoord_varying = texcoord;
	gl_Position = modelViewProjectionMatrix * position;
}
