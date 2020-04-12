#version 450

uniform float alpha = 1.0;

uniform mat4 viewMatrix;

uniform vec4 mtl_ambient;
uniform vec4 mtl_diffuse ;
uniform vec4 mtl_specular;
uniform vec4 mtl_emission ;
uniform float mtl_shininess;

uniform vec3 light_dir = vec3(0.5, -1.0, 0);
uniform vec3 light_pos;

uniform sampler2D tex;
uniform vec4 color = vec4(1.0);

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

void main() 
{
	vec4 texcol = texture(tex, texcoord_varying) * color;
	vec4 outcol = mtl_emission * texcol + directional_light(normal_varying.xyz, texcol);

	// float gamma = 2.2;
	// outcol.rgb = pow(outcol.rgb, vec3(1.0/gamma));
	outcol.a *= alpha;

	fragColor = outcol;
}
