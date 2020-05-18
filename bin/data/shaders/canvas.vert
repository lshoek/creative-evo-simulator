#version 450

uniform mat4 modelViewProjectionMatrix;

in vec4 position;
in vec2 texcoord;

out vec2 texcoord_varying;

void main()
{
	texcoord_varying = texcoord;
    gl_Position = modelViewProjectionMatrix * position;
}
