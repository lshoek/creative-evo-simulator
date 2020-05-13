#version 450
#pragma include "material.glslinc" 
#pragma include "light.glslinc" 
#pragma include "phong.glslinc"

uniform float alpha = 1.0;

uniform Material mtl;
uniform Light light;

uniform sampler2D tex;
uniform sampler2D shadowMap;

uniform vec3 checkers_col0 = vec3(0.913, 0.309, 0.215);
uniform vec3 checkers_col1 = vec3(0.878, 0.878, 0.886);
uniform vec3 eyePos;

in vec4 eye;
in vec4 fragPos;
in vec4 fragPosLightSpace;

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

float shadow_calc(vec4 fragPosLightSpace, vec3 lightDir)
{
	vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
	projCoords = projCoords * 0.5 + 0.5;

	float closestDepth = texture(shadowMap, projCoords.xy).r; 
	float currentDepth = projCoords.z;

	float bias = max(0.05 * (1.0 - dot(normal_varying.xyz, lightDir)), 0.005);
	float shadow = currentDepth - bias > closestDepth  ? 1.0 : 0.0;     

	return shadow;
}

void main(void)
{
	vec4 texcol = texture(tex, texcoord_varying*32.0);
	texcol.rgb = mix(checkers_col0, checkers_col1, texcol.r);

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
