#version 450
#pragma include "material.glslinc" 
#pragma include "light.glslinc" 
#pragma include "phong.glslinc"
#pragma include "noise.glslinc"

uniform float alpha = 1.0;

uniform Material mtl;
uniform Light light;

uniform sampler2D tex;
uniform sampler2DShadow shadowMap;

uniform vec4 checkers_pos = vec4(0.356, 0.521, 0.666, 1.0);
//uniform vec4 checkers_pos = vec4(0.913, 0.309, 0.215, 1.0);
uniform vec4 checkers_neg = vec4(0.878, 0.878, 0.886, 1.0);
uniform vec3 eyePos;

in vec4 eye;
in vec4 fragPos;
in vec4 fragPosLightSpace;

in vec4 color_varying;
in vec4 normal_varying;
in vec2 texcoord_varying;

vec2 poissonDisk4[4] = vec2[](
	vec2(-0.94201624, -0.39906216),
	vec2(0.94558609, -0.76890725),
	vec2(-0.094184101, -0.92938870),
	vec2(0.34495938, 0.29387760)
);
vec2 poissonDisk16[16] = vec2[](
	vec2( -0.94201624, -0.39906216 ),
	vec2( 0.94558609, -0.76890725 ),
	vec2( -0.094184101, -0.92938870 ),
	vec2( 0.34495938, 0.29387760 ),
	vec2( -0.91588581, 0.45771432 ),
	vec2( -0.81544232, -0.87912464 ),
	vec2( -0.38277543, 0.27676845 ),
	vec2( 0.97484398, 0.75648379 ),
	vec2( 0.44323325, -0.97511554 ),
	vec2( 0.53742981, -0.47373420 ),
	vec2( -0.26496911, -0.41893023 ),
	vec2( 0.79197514, 0.19090188 ),
	vec2( -0.24188840, 0.99706507 ),
	vec2( -0.81409955, 0.91437590 ),
	vec2( 0.19984126, 0.78641367 ),
	vec2( 0.14383161, -0.14100790 )
);

float poissonSpread = 768.0;

out vec4 fragColor;


vec4 checkers(vec2 st)
{
	vec2 p = floor(st*32.0);
	float f = mod(p.x + p.y, 2.0);

	vec3 col = mix(checkers_pos.rgb, checkers_neg.rgb, f);
	return vec4(col.rgb, 1.0);
}

float shadow_calc(vec4 fragPosLightSpace, vec3 lightDir)
{
	float bias = max(0.01 * (1.0 - dot(normal_varying.xyz, lightDir)), 0.005);
	float fragDepth = (fragPosLightSpace.z-bias)/fragPosLightSpace.w;
	float shadow = 0.0;

	for (int i=4; i<16; i++) {
		//int idx = int(16.0*rnd(gl_FragCoord.xy, i))%16;
		int idx = i;
		shadow += 1.0 - texture(shadowMap, 
			vec3(fragPosLightSpace.xy + poissonDisk16[idx]/poissonSpread, fragDepth)
		);
	}
	shadow /= 16.0;
	return shadow;
}

void main(void)
{
	vec4 texcol = texture(tex, texcoord_varying*32.0);
	texcol.rgb = mix(checkers_pos.rgb, checkers_neg.rgb, texcol.r);

	vec4 ambient = light.ambient * mtl.ambient;

	vec3 light_dir = normalize(light.position - fragPos.xyz);
	float diff = max(dot(normal_varying.xyz, light_dir), 0.0);
	vec4 diffuse = light.diffuse * (diff * mtl.diffuse);

	vec3 view_dir = normalize(eyePos - fragPos.xyz);
	vec3 reflect_dir = reflect(-light_dir, normal_varying.xyz);
	float spec = 1.0 * pow(max(dot(view_dir, reflect_dir), 0.0), mtl.shininess);
	vec4 specular = light.specular * (spec * mtl.specular);

	float shadow = shadow_calc(fragPosLightSpace, light_dir);
    vec3 lighting = (ambient.rgb + (1.0 - shadow) * (diffuse.rgb + specular.rgb)) * texcol.rgb;  

    vec4 outcol = vec4(lighting, 1.0);

	float alpha_radial = 0.5 - distance(vec2(0.5), texcoord_varying);
	outcol.a *= alpha * smoothstep(0.0, 0.2, alpha_radial);

	fragColor = outcol;
}
