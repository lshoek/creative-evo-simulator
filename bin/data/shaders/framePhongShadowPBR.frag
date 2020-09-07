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

// 0-1
uniform vec4 color;
uniform sampler2D tex;
uniform sampler2DShadow shadowMap;

// 2-6
uniform sampler2D albedoMap;
uniform sampler2D normalMap;
uniform sampler2D metallicMap;
uniform sampler2D roughnessMap;
uniform sampler2D aoMap;

uniform PBRMaterial mtl;
uniform Light light;

uniform vec3 eyePos;

in vec4 eye;
in vec4 worldPos;
in vec4 worldPosLightSpace;

in vec4 color_varying;
in vec4 normal_varying;
in vec2 texcoord_varying;

uniform vec4 brush_color = vec4(1.0);
uniform float brush = 0.0;
uniform float brush_active = 0.0;

out vec4 fragColor;

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

// frame
const float off = 0.08;
const float offd2 = off/2.0;
const float df = 0.0125;
const float df2 = df*2.0;

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

void main() 
{
	vec2 st = texcoord_varying; //mix(texcoord_varying, fract(texcoord_varying*2.0), brush);
	float frame = max(
		smoothstep(off, off-df, st.y) + smoothstep(off, off-df, inv(st.y)),
		smoothstep(off, off-df, st.x) + smoothstep(off, off-df, inv(st.x))
	);
	float diag = smoothstep(st.x+df2+offd2, st.x+offd2, st.y) - smoothstep(st.x-offd2, st.x-df2-offd2, st.y);
	frame = max(frame, diag*brush);

	vec4 texcol = mix(brush_color, color, step(brush_active, 0)) * (inv(frame)*0.8);
	texcol = max(texcol, texcol*brush);
	vec3 albedo = pow(texcol.rgb, vec3(2.2));

	float metallic = mtl.metallic; //texture(metallicMap, texcoord_varying).r;
	float roughness = texture(roughnessMap, texcoord_varying).r;
	float ao = texture(aoMap, texcoord_varying).r;

	vec3 norm = normalFromMap(normalMap, texcoord_varying, normal_varying.xyz, worldPos.xyz);
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
		float denominator = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + eps;
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
	vec3 finalColor = ambient + Lo;  

	// HDR tonemapping
    finalColor = finalColor / (finalColor + vec3(1.0));
    
    // gamma correct
    finalColor = pow(finalColor, vec3(1.0/2.2)); 

	fragColor =  vec4(finalColor, 1.0);
}
