#version 450
#pragma include "material.glslinc" 
#pragma include "light.glslinc" 
#pragma include "phong.glslinc"

uniform float alpha = 1.0;

uniform Material mtl;
uniform Light light;

uniform sampler2D tex;
uniform vec4 color = vec4(1.0);
uniform vec3 eyePos;

in vec4 eye;
in vec4 fragPos;  

in vec4 color_varying;
in vec4 normal_varying;
in vec2 texcoord_varying;

out vec4 fragColor;

void main() 
{
	vec4 texcol = texture(tex, texcoord_varying) * color;

	//vec4 mtlcol = directional_light(mtl, light, eye.xyz, normal_varying.xyz, texcol);
	vec4 mtlcol = point_light(mtl, light, eyePos, fragPos.xyz, normal_varying.xyz);

	vec4 outcol = texcol * mtlcol;
	outcol.a *= alpha;

	fragColor = outcol;
}
