// muti-Texture Fragment Shader
// Author: Hongwei Lu
// 2016. Nov 7
#version 330
#extension GL_NV_texture_array : enable

out vec4 vFragColor;

uniform sampler2DArray bowlImage;

smooth in vec3 vBowlCoords;

// use bowl coords to apply texture
void main(void)
{
	vFragColor = texture2DArray(bowlImage, vBowlCoords.stp);
}