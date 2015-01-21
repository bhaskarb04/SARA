varying vec3 z;
varying vec3 lightDir;
varying vec3 normal;

void main()
{

	normal = normalize(gl_NormalMatrix * gl_Normal);
	lightDir = normalize(vec3(gl_LightSource[0].position));

	gl_FrontColor = gl_Color;
	gl_TexCoord[0] = gl_MultiTexCoord0;
	gl_Position = gl_Vertex;//ftransform();
}