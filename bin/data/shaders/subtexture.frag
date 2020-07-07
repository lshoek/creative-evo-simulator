#version 450

uniform sampler2D tex;
uniform vec2 patchLocation;
uniform float patchSize;

in vec2 texcoord_varying;
out vec4 fragColor;
 
void main()
{
	vec2 st = texcoord_varying;
	vec2 patch_st = patchLocation + (st * patchSize) - patchSize*0.5;
	fragColor = texture(tex, patch_st);
}
