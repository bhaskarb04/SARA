uniform sampler2D glSample0;
varying vec3 lightDir,normal;

#define textureWidth 512.0
#define textureHeight 512.0	
#define texel_size_x 1.0 / textureWidth
#define texel_size_y 1.0 / textureHeight


void main()
{
    vec3 ct,cf;
    vec4 texel;
    float intensity,at,af;

    intensity = max(dot(lightDir,normalize(normal)),0.0);
    
    cf = intensity * (gl_FrontMaterial.diffuse).rgb + gl_FrontMaterial.ambient.rgb;
    af = gl_FrontMaterial.diffuse.a;

    // Combines the vertex color from lighting operation with the fragment color.
    gl_FragColor = gl_Color * texture2D(glSample0, gl_TexCoord[0].xy);

    // Provides a depth value that pushes the video into the background. 
    gl_FragDepth = 0.999;
}
