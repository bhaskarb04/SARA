/**************************************************************
 This class implements a video subload callback.
 It invokes the glSubLoad function and copies the texture
 into an already existing texture.

Autor: Rafael Radkowski
Stand: 3. Mai 2007
Version: 1.1
****************************************************************/

#include <osg/Notify>

#include <iostream>

#include "VideoTexCallback.h"

using namespace osg;

/*!************************************************************************************************************ 
 Constructor
 @param unsigned char* image: Pointer that refers to the video image
 @param int vw: Width of the video
 @param int vh: Height of the video
 @param int tw: Width of the texture
 @param int th: Height of the texture
 @param format: the OpenGL video color space like GL_RGB, GL_RGBA, etc
****************************************************************************************************************/
VideoTexCallback::VideoTexCallback(unsigned char* image,int vw, int vh, int tw, int th, int format):
	_vidWidth(vw),
	_vidHeight(vh),
	_texWidth(tw),
	_texHeight(th),
	_video(image),
	_format(format)

{

}

void VideoTexCallback::load(const osg::Texture2D&, osg::State&) const 
{
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, _texWidth, _texHeight, 0, _format, GL_UNSIGNED_BYTE, NULL);
}
	

void VideoTexCallback::subload(const osg::Texture2D&, osg::State&) const 
{
	unsigned char* frame = _video;

	if (frame == 0L) return;

	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, _vidWidth, _vidHeight, _format, GL_UNSIGNED_BYTE, frame);

}

