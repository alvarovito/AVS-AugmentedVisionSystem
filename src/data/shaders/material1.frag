#version 330

in vec3 Color;
in vec2 Texcoord;

out vec4 outColor;

uniform sampler2D tex;
uniform vec2 LensCenter;
uniform vec2 ScreenCenter;


void main()
{
    //outColor = texture(tex, Texcoord) * vec4(Color, 1.0);
	//outColor = texture(tex, Texcoord);
        //outColor = vec4(1.0,0.0,0.0,1.0);

    vec2  theta = (Texcoord - LensCenter) * vec2(2,2);
	float rSq = theta.x * theta.x + theta.y * theta.y;
	vec2  theta1 = theta * (1.0 + 0.22 * rSq + 0.24 * rSq * rSq + 0 * rSq * rSq * rSq);

	vec2 tc = LensCenter + vec2(0.32,0.35) * theta1;

	if (any(bvec2(clamp(tc,-vec2(0.0,0.0), vec2(1,1)) - tc)))
       outColor = vec4(0);
   else
       outColor = texture(tex, tc);

	//outColor = texture(tex, Texcoord);
}
