#version 450

#define inv(x) 1.0-x
#define PI 3.14159265359
#define BRUSH_COORD_BUF_MAXSIZE 16

struct BrushCoord {
	vec2 coord;
	float pressure;
	float enabled;
};
layout(std430, binding=0) buffer brushCoordBuffer {
	BrushCoord brush_coords[];
};

uniform float alpha = 1.0;
uniform float use_brush_pressure = 0.0;
uniform int brush_coords_bufsize = 0;

const float thickness = 1/64.0;
const float fade = 1/512.0;

in vec2 texcoord_varying;
out float fragColor;



void main()
{
	vec2 st = texcoord_varying;
	float pct = 0.0;

	for (int i=0; i<BRUSH_COORD_BUF_MAXSIZE; i++) 
	{
		BrushCoord b = brush_coords[i];

		if (i < brush_coords_bufsize)
		{
			float y = 1.0-pow(sin(PI*((b.pressure+1.0)/2.0)), 2.0);
			float press = clamp(y, 0.0, 1.0) * thickness;

			float dist = distance(b.coord, st);
			float result = smoothstep(press+fade, press-fade, dist);
			pct = max(result, pct);
		}
	}
	fragColor = max(pct, 0.0);
}
