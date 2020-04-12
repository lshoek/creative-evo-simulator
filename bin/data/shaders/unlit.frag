#version 450

uniform sampler2D tex;

uniform vec4 color = vec4(1.0);
uniform float alpha = 1.0;

// in vec4 color_varying;
// in vec4 normal_varying;
in vec2 texcoord_varying;

out vec4 fragColor;

void main() 
{
	vec4 texcol = texture(tex, texcoord_varying) * color;
	fragColor = texcol;
}
