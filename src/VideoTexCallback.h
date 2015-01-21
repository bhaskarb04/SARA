/**************************************************************
This class implements a video subload callback.
It invokes the glSubLoad function and copies the texture 
into an already existing texture. 

Autor: Rafael Radkowski
Stand: 3. Mai 2007
Version: 1.1
****************************************************************/

#pragma once

#include <osg/Texture2D>
#include <osg/Image>
#include <osg/Referenced>



class  VideoTexCallback : public osg::Texture2D::SubloadCallback 
{
public:

	

	/*! Constructor 
		@param unsigned char* image: Pointer that refers to the video image
		@param int vw: Width of the video
		@param int vh: Height of the video
		@param int tw: Width of the texture
		@param int th: Height of the texture
		@param format: the OpenGL video color space like GL_RGB, GL_RGBA, etc
	*/
	VideoTexCallback(unsigned char* image,int vw, int vh, int tw, int th, int format = GL_RGBA);
	
	/*! Virtual function
     Must be replaced by own code. 
     OSG invokes this function the first time the texture is loaded
     */
	virtual void load(const osg::Texture2D& texture, osg::State& state) const;	

	/*! Virtual function
     Must be replaced by own code.
     OSG invokes this function each time the frame is rendered
     */
	virtual void subload(const osg::Texture2D& texture, osg::State& state) const;

protected:

	// Width and height of the video image
	int _vidWidth;
	int _vidHeight;

	// Witdth and height of the texture.
    // Must not be a power of two, BUT power of two works faster
	int _texWidth;
	int _texHeight;

	// Video color space
	int	 _format;
	
	// The pointer that refers to the video image in main memory
	unsigned char* _video;

	
private:
	

};


