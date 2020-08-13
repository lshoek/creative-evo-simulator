#version 450
#define PI 3.14159265359

uniform sampler2D tex;
uniform vec4 patchRotationMatrix;
uniform vec2 patchLocation;
uniform float patchSize;

in vec2 texcoord_varying;
out vec4 fragColor;
 
mat2 rotate2d(float _angle)
{
	return mat2(
		cos(_angle),-sin(_angle),
		sin(_angle),cos(_angle)
	);
}

void main()
{
	vec2 st = texcoord_varying;

	mat2 rot = mat2(
		patchRotationMatrix[0], 
		patchRotationMatrix[1],
		patchRotationMatrix[2],
		patchRotationMatrix[3]
	);

	//vec2 patch_st = patchLocation + ((st * patchSize) * rot) - patchSize*0.5;
	vec2 patch_st = patchLocation + st * patchSize;
	patch_st -= patchSize*0.5;
	patch_st *= rot;


	fragColor = texture(tex, patch_st);
}
