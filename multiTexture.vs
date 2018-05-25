// muti-Texture Vertex Shader
// Author: Hongwei Lu
// 2016. Nov 7
#version 330

in vec4 vTexCoords;
in vec4 vVertex;

uniform mat4 mvpMatrix; //texture transposition Mat
uniform float  fIndex;

smooth out vec3 vBowlCoords;

// accept Bowl texture dimention
// apply texture transposition 
void main(void)
{
 vBowlCoords.st = vTexCoords.st;
 vBowlCoords.p = fIndex;
 
 gl_Position =  gl_ModelViewProjectionMatrix * gl_Vertex;
 
}