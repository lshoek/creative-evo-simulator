#version 450

uniform sampler2D tex;
uniform vec4 brush_color = vec4(1.0);
uniform vec4 canvas_color = vec4(0.0);

in vec2 texcoord_varying;

out vec4 fragColor;

void main() 
{
	float lum = texture(tex, texcoord_varying).r;
	vec3 col = mix(canvas_color.rgb, brush_color.rgb, lum);
	fragColor = vec4(col, 1.0);
}
