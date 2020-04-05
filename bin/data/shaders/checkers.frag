#version 450

uniform float alpha = 1.0;

uniform vec4 mtl_ambient;
uniform vec4 mtl_diffuse;
uniform vec4 mtl_specular;
uniform vec4 mtl_emission;
uniform float mtl_shininess;

uniform vec3 light_dir = vec3(0, -1.0, 0.75);

//uniform sampler2D tex;

in vec4 eye;

in vec4 color_varying;
in vec4 normal_varying;
in vec2 texcoord_varying;

out vec4 fragColor;

vec4 directional_light(vec3 normal, vec4 texcol)
{
	vec4 lightcol = vec4(0.0);
	vec4 diffuse, ambient, specular = vec4(0.0);
	
	// mult material & light ambient
	ambient = mtl_ambient * texcol; 
	lightcol = ambient;

	vec3 light_normal = normalize(light_dir);
	float lambert_term = dot(normal, light_dir);

	if (lambert_term > 0.0)
	{
		vec3 eye_normal = normalize(eye.xyz);
		vec3 reflection = reflect(-light_normal, normal_varying.xyz);

		// mult material & light diffuse
		diffuse = mtl_diffuse * lambert_term * texcol; 
		lightcol += diffuse;

 		// mult material & light specular
		specular = pow(max(dot(reflection, eye_normal), 0.0), mtl_shininess) * mtl_specular;
		lightcol += specular;
	}
	lightcol.w = 1.0;
	return lightcol;
}

vec4 checkers(vec2 st)
{
	vec2 p = floor(st*64.0);
	float f = mod(p.x + p.y, 2.0);

	vec3 col = mix(vec3(1.0, 1.0, 1.0), vec3(1.0, 0.0, 0.5), f);
	return vec4(col.rgb, 1.0);
}

void main(void)
{
	vec4 texcol = checkers(texcoord_varying);
	vec4 outcol = mtl_emission * texcol + directional_light(normal_varying.xyz, texcol);
	outcol.a *= alpha;

	fragColor = outcol;
}
