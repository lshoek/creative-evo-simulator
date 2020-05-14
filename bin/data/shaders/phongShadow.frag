#version 450
#pragma include "material.glslinc" 
#pragma include "light.glslinc" 
#pragma include "phong.glslinc"

uniform float alpha = 1.0;

uniform Material mtl;
uniform Light light;

uniform sampler2D tex;
uniform sampler2D shadowMap;

uniform vec4 color = vec4(1.0);
uniform vec3 eyePos;

in vec4 eye;
in vec4 fragPos;
in vec4 fragPosLightSpace;

in vec4 color_varying;
in vec4 normal_varying;
in vec2 texcoord_varying;

vec2 poissonDisk[4] = vec2[](
	vec2(-0.94201624, -0.39906216),
	vec2(0.94558609, -0.76890725),
	vec2(-0.094184101, -0.92938870),
	vec2(0.34495938, 0.29387760)
);
float poissonSpread = 1536.0;

out vec4 fragColor;


float shadow_calc(vec4 fragPosLightSpace, vec3 lightDir)
{
	// perform perspective divide
	vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;

	// transform to [0,1] range
	projCoords = projCoords * 0.5 + 0.5;

	// get closest depth value from light's perspective (using [0,1] range fragPosLight as coords)
	float closestDepth = texture(shadowMap, projCoords.xy).r; 

	// get depth of current fragment from light's perspective
	float currentDepth = projCoords.z;

	float bias = max(0.01 * (1.0 - dot(normal_varying.xyz, lightDir)), 0.005);
	float shadow = 0.0;

	// with sampling
	for (int i=0; i<4; i++) {
		float pcfDepth = texture(shadowMap, projCoords.xy + poissonDisk[i]/poissonSpread).z; 
		shadow += currentDepth - bias > pcfDepth ? 1.0 : 0.0;        
	}
	shadow /= 4.0;

	// check whether current frag pos is in shadow
	// without sampling 
	//shadow = currentDepth > closestDepth + bias ? 1.0 : 0.0;   
	
	if(projCoords.z > 1.0) 
		shadow = 0.0;

	return shadow;
}

void main() 
{
	vec4 texcol = texture(tex, texcoord_varying) * color;

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
	outcol.a = alpha;

	fragColor = outcol;
}
