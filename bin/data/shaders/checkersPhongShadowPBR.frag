#version 450
#pragma include "material.glslinc" 
#pragma include "light.glslinc" 
#pragma include "phong.glslinc"
#pragma include "noise.glslinc"
#pragma include "include/pbr.glslinc"

#define NUM_LIGHTS 1
#define inv(x) 1.0-x

uniform float alpha = 1.0;
uniform float lightIntensity = 1.0;
uniform float normalMapMult = 1.0;

uniform PBRMaterial mtl;
uniform Light light;

// 0-1
uniform sampler2D tex;
uniform sampler2DShadow shadowMap;

// 2-6
uniform sampler2D albedoMap;
uniform sampler2D normalMap;
uniform sampler2D metallicMap;
uniform sampler2D roughnessMap;
uniform sampler2D aoMap;

//uniform vec4 checkers_pos = vec4(0.356, 0.521, 0.666, 1.0);
uniform vec4 checkers_pos = vec4(0.913, 0.309, 0.215, 1.0);
uniform vec4 checkers_neg = vec4(0.878, 0.878, 0.886, 1.0);
uniform vec3 eyePos;

in vec4 eye;
in vec4 worldPos;
in vec4 worldPosLightSpace;

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

const float poissonSpread = 768.0;
const float eps = 0.001;

out vec4 fragColor;


vec4 checkers(vec2 st)
{
	vec2 p = floor(st*32.0);
	float f = mod(p.x + p.y, 2.0);

	vec3 col = mix(checkers_pos.rgb, checkers_neg.rgb, f);
	return vec4(col.rgb, 1.0);
}

float calcShadow(vec4 fragPosLightSpace, vec3 lightDir)
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
	vec2 uv = (texcoord_varying + vec2(0.5)) * 16.0;

	//float tile = texture(tex, uv).r;
	//vec3 texcol = mix(checkers_pos.rgb, checkers_neg.rgb, tile);

	vec3 albedo = pow(texture(albedoMap, uv).rgb, vec3(2.2));
	//albedo = mix(texcol, albedo, 0.5);

	float metallic = mtl.metallic; //texture(metallicMap, texcoord_varying).r;
	float roughness = texture(roughnessMap, uv).r;
	float ao = texture(aoMap, uv).r;

	vec3 norm = normalFromMap(normalMap, uv, normal_varying.xyz, worldPos.xyz);
	norm = ((norm - vec3(0.5)) * normalMapMult) + vec3(0.5);

	//vec3 N = normalize(normal_varying.xyz); 
	vec3 N = norm;
    vec3 V = normalize(eyePos - worldPos.xyz);

    // calculate reflectance at normal incidence; if dia-electric (like plastic) use F0 
    // of 0.04 and if it's a metal, use the albedo color as F0 (metallic workflow)    
    vec3 F0 = vec3(0.04); 
    F0 = mix(F0, albedo, metallic);

    vec3 Lo = vec3(0.0);
	for(int i = 0; i < 1; ++i) 
	{
		vec3 L = normalize(light.position - worldPos.xyz);
		vec3 H = normalize(V + L);

		// currently only implemented for a single light
		float shadow = calcShadow(worldPosLightSpace, L);

		float distance = length(light.position - worldPos.xyz);
		float attenuation = 1.0 / (distance * distance);
		vec3 radiance = (1.0 - shadow) * light.ambient.rgb * lightIntensity * attenuation; 

		// Cook-Torrance BRDF
        float NDF = DistributionGGX(N, H, roughness);   
        float G = GeometrySmith(N, V, L, roughness);      
        vec3 F = fresnelSchlick(max(dot(H, V), 0.0), F0);

		vec3 nominator = NDF * G * F; 
		float denominator = 1.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + eps;
		vec3 specular = nominator / denominator;

        vec3 kS = F; // kS is equal to Fresnel

        // for energy conservation, the diffuse and specular light can't
        // be above 1.0 (unless the surface emits light); to preserve this
        // relationship the diffuse component (kD) should equal 1.0 - kS.
        vec3 kD = vec3(1.0) - kS;

        // multiply kD by the inverse metalness such that only non-metals 
        // have diffuse lighting, or a linear blend if partly metal (pure metals
        // have no diffuse light).
        kD *= 1.0 - metallic;

        // scale light by NdotL
        float NdotL = max(dot(N, L), 0.0);        

        // add to outgoing radiance Lo
        // note that we already multiplied the BRDF by the Fresnel (kS) 
        // so we won't multiply by kS again
        Lo += (kD * albedo / PI + specular) * radiance * NdotL;
	}
	
	vec3 ambient = vec3(0.03) * albedo * ao;
	vec3 color = ambient + Lo;  

	// HDR tonemapping
    color = color / (color + vec3(1.0));
    
    // gamma correct
    color = pow(color, vec3(1.0/2.2)); 

    vec4 outcol = vec4(color, 1.0);

	float alpha_radial = 0.5 - distance(vec2(0.5), (texcoord_varying));
	outcol.a *= alpha * smoothstep(0.0, 0.5, alpha_radial);

	fragColor = outcol;
}
