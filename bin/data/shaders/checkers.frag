#version 450
#pragma include "material.glslinc" 
#pragma include "light.glslinc" 
#pragma include "phong.glslinc"

#define inv(x) 1.0-x

uniform float alpha = 1.0;

uniform Material mtl;
uniform Light light;

uniform sampler2D tex;
uniform vec3 checkers_col0 = vec3(0.913, 0.309, 0.215);
uniform vec3 checkers_col1 = vec3(0.878, 0.878, 0.886);
uniform vec3 eyePos;

in vec4 eye;
in vec4 fragPos;

in vec4 color_varying;
in vec4 normal_varying;
in vec2 texcoord_varying;

out vec4 fragColor;

vec4 checkers(vec2 st)
{
	vec2 p = floor(st*32.0);
	float f = mod(p.x + p.y, 2.0);

	vec3 col = mix(checkers_col0, checkers_col1, f);
	return vec4(col.rgb, 1.0);
}

void main(void)
{
	vec4 texcol = texture(tex, texcoord_varying*32.0);
	texcol.rgb = mix(checkers_col0, checkers_col1, texcol.r);

	//vec4 mtlcol = directional_light(mtl, light, eye.xyz, normal_varying.xyz, texcol);
	vec4 mtlcol = point_light(mtl, light, eyePos, fragPos.xyz, normal_varying.xyz);

	vec4 outcol = texcol * mtlcol;
	
	// float gamma = 2.2;
	// outcol.rgb = pow(outcol.rgb, vec3(1.0/gamma));

	float alpha_radial = 0.5 - distance(vec2(0.5), texcoord_varying);
	outcol.a *= alpha * smoothstep(0.0, 0.2, alpha_radial);

	fragColor = outcol;
}
