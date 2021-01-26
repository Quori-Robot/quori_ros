#version 330 core
out vec4 FragColor;

in vec2 TexCoord;
in vec4 gl_FragCoord;

uniform sampler2D lookup_table;
uniform sampler2D image;
uniform vec3 i_resolution;

vec2 rotateUV(vec2 uv, float rotation)
{
    float mid = 0.5;
    return vec2(
        cos(rotation) * (uv.x - mid) + sin(rotation) * (uv.y - mid) + mid,
        cos(rotation) * (uv.y - mid) - sin(rotation) * (uv.x - mid) + mid
    );
}

void main()
{
    float TAU = 6.28318530718;
    float DIRECTIONS = 4.0;
    float QUALITY = 3.0;
    float SIZE = 2.0;
    vec2 RADIUS = SIZE / i_resolution.xy;
    vec2 uv = gl_FragCoord.xy / i_resolution.xy;
    vec3 lookup = texture2D(lookup_table, TexCoord.xy).xyz;
    vec2 table = rotateUV(lookup.xy, -TAU / 4);
    vec4 color = texture2D(image, table) * (1.0 - lookup.z);
    FragColor = color;
} 