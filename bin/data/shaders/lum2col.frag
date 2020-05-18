#version 450

uniform sampler2D tex;
uniform vec4 brush_color = vec4(1.0);

in vec2 texcoord_varying;

out vec4 fragColor;

void main() 
{
	float lum = texture(tex, texcoord_varying).r;
	fragColor = vec4(brush_color.rgb, lum);
}
