// HCI571X-VideoRenderer.cpp : Defines the entry point for the console application.
//


#include <iostream>
#include <math.h>

#include <osg/Group>
#include <osg/Node>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/TexEnv>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/Vec3>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>

#ifdef WIN32
#include <windows.h>
#endif

//---------------------------------------------
// Includes the video capture device. 
#include "ARVideoCapture.h"
#include "VideoTexCallback.h"

//---------------------------------------------
// A shader loader object
#include "GLSL_ShaderLoader.h"

#include "imgprocesscallback.h"
//-------------------------------------------
// The geometry node for the video texture
osg::Geode* geode;
// and the geometry
osg::Geometry* geometry;

//-------------------------------------------
// Arrays for texture and vertex coordinates
osg::Vec3Array* vertices;
osg::Vec3Array*	texCoordArray;

//-------------------------------------------
// The video texture
osg::Texture2D* texture0;

//-------------------------------------------
// The size of the texture in texture coordinates
double sizeX;
double sizeY;

//-------------------------------------------
// This specifies the gridsize: the num of rows and columns 
int textureCoordGridX = 16;
int textureCoordGridY = 16;

//-------------------------------------------
// The video capture device
ARVideoCapture*	myVideoCaptureDevice;

//-------------------------------------------
// The callback that subloads the video to the graphics hardware
VideoTexCallback* myCallback;
imgprocesscallback* imageprocesscb;



/////////////////////////////////////////////////////////////////////////////
// Prototypes
osg::Vec2 getTextureCoordinateAt(int i, int j);
bool setTextureCoordinateAt(int i, int j, double tx, double ty);


/*!
This creates the background object for the video polygon. 
It calculates the size of each grind and generates the quads row by row
*/
osg::Geode* createBackgroundObject(void)
{

    geode = new osg::Geode();
    geometry = new osg::Geometry();
    
    // The size of the entire surface. 
    double width = 2.0;
    double height = 2.0;
    
	double startX = -width/2.0;
	double startY = height/2.0;

	// The size for each quad. 
    double deltaX = width / (textureCoordGridX-1);
    double deltaY = -height/ (textureCoordGridY-1);

	// The number of vertices to be generated
    vertices = new osg::Vec3Array( (textureCoordGridX -1) * (textureCoordGridY - 1) * 4);
    
	// The loob that navigates through the lines
    for(int j=0; j<textureCoordGridY-1; j++) // rows
    {
        for(int i=0; i<textureCoordGridX-1; i++) // columns
        {
			// This computes the array starting index of each grid-vertex
            int idx0 = ( j * (textureCoordGridX-1) * 4) + (i * 4);
            
			// The coordinates of the first vertex in x and y
            double cx0 = startX + ( i * deltaX);
            double cy0 = startY + ( j * deltaY);
            
			// The coordinates of all the vertices are stored in the vertex array
            (*vertices)[idx0 + 2] = osg::Vec3(cx0, cy0, 0.0);
            (*vertices)[idx0 + 3] = osg::Vec3(cx0 + deltaX, cy0, 0.0);
            (*vertices)[idx0 + 0] = osg::Vec3(cx0 + deltaX, cy0 + deltaY, 0.0);
            (*vertices)[idx0 + 1] = osg::Vec3(cx0, cy0 + deltaY, 0.0);
        }
    }
    
	// The vertices and their relations are assigned to the geometry
	geometry->setVertexArray(vertices);
    geometry->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, vertices->size()));
    
    // A vertex color is generated. 
	osg::Vec4Array* color = new osg::Vec4Array;
    color->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    geometry->setColorArray(color);
    geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
    
    // set the normal in the same way color.
    osg::Vec3Array* normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,1.0f,0.0f));
    geometry->setNormalArray(normals);
    geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
    
	// Light and Blending are switched off.
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::OFF);
    
	// Disable the culling, thus, the background will not disappear if out of view frustun. 
	geode->setCullingActive(false);

    // The geometry object is passed to the geode node. 
	geode->addDrawable(geometry);
    



    return geode;

}



double getRadius(double x, double y)
{
    double r;
    
    r =  ((x - 0.0)*(x - 0.0)) + ((y - 0.0)*(y - 0.0));
    
    r = sqrt((double)r);
    return r;
}

/*
 This function calibrates the video image according to a
 radial calibration model. 
 @param k0, k1; the calibration factors. 
 */
bool radialCalibration(double k0, double k1)
{
    

    for(int iRow = 0; iRow < textureCoordGridY;  iRow++)
	{
		for(int iColomn=0; iColomn < textureCoordGridX; iColomn++)
		{
			osg::Vec2 d = getTextureCoordinateAt( iColomn, iRow);
            
            
			// Moves the origin into the center of the texture
			d.x() = d.x() - sizeX/2.0;
			d.y() = d.y() - sizeY/2.0;
            
            // gets the radius
			osg::Vec2 result;
			double r = getRadius(d.x(), d.y());
			
			// Correct the radial distortion 
			result.x() = d.x() * ( 1 + k0* pow((double)r,2)  + k1 * pow((double)r,4));
			result.y() = d.y() * ( 1 + k0* pow((double)r,2)  + k1 * pow((double)r,4));
            
            
			// Move the coordinate system back 
			result.x() += sizeX/2.0;
			result.y() += sizeY/2.0;
            
            // write the new texture coordinates. 
            setTextureCoordinateAt(  iColomn, iRow, result.x(), result.y());
            
		}
	}
    return true;
}
#define PI 3.1415926535
bool modifyCalibration()
{
    
	int bump=0;
    for(int iRow = 0; iRow < textureCoordGridY;  iRow++)
	{
		for(int iColomn=0; iColomn < textureCoordGridX; iColomn++)
		{
			//get old texture coordinates
			osg::Vec2 d = getTextureCoordinateAt( iColomn, iRow);
			double r = 0.025;				//amplitude
				
			d.y() += r*cos(bump*PI/180);	
            bump+=45;						//factor by which the phase changes
            
            // write the new texture coordinates. 
            setTextureCoordinateAt(  iColomn, iRow, d.x(), d.y());
            
		}
	}
    return true;
}


#ifdef COCOA // Only for Mac OS X Cocoa users

void getScreenResolution (int& screenX, int& screenY) {
    
    NSArray *screenArray = [NSScreen screens];
    NSScreen *mainScreen = [NSScreen mainScreen];
    unsigned screenCount = [screenArray count];
    unsigned index  = 0;
    
    for (index; index < screenCount; index++)
    {
        NSScreen *screen = [screenArray objectAtIndex: index];
        NSRect screenRect = [screen visibleFrame];
        NSString *mString = ((mainScreen == screen) ? @"Main" : @"not-main");
        
        NSLog(@"Screen #%d (%@) Frame: %@", index, mString, NSStringFromRect(screenRect));
        
        if(index == 0)// Take the first screen
        {
            screenX = screenRect.size.width;
            screenY = screenRect.size.height;
        }
    }
    
    
}

#endif



/*!
This function creates the texture coordinates for the quads. 
The texture coordinates pose the undistorted set, which 
maps a texture either streched nor compressed onto the quad. 
*/
void setTextureToRect(osg::Texture2D* texture, int videoWidth, int videoHeight, int textureWidth, int textureHeight)
{
    

	sizeX = double(videoWidth)/double(textureWidth);
	sizeY = double(videoHeight)/double(textureHeight);

    
    // start coordinates for the video image
    double startX = -0.0;
	double startY = -0.0;
    
    // This function is not available in this code example
    // You will get it later. 
  //adaptTexCoordToFieldOfView(60.0f,50.0f,50.0,40.0f,videoWidth,videoHeight,sizeX,sizeY, startX, startY);
    
    
	// The delta between each coordinate. 
	// The size of the texture is 1.0 but we see only a subset
    double deltaX = sizeX / ( textureCoordGridX - 1);
    double deltaY = sizeY / ( textureCoordGridY -1 );
    

    
	// The vertex array is generated. 
    texCoordArray = new osg::Vec3Array( (textureCoordGridX -1) * (textureCoordGridY - 1) * 4 );
    
    for(int j=0; j<textureCoordGridY-1; j++) // rows
    {
        for(int i=0; i<textureCoordGridX-1; i++) // columns
        {
			// This calculates the first index of first coordinate
            int idx0 = (j * (textureCoordGridX-1) * 4) + (i * 4);
            
			// The coordinates
            double cx0 = startX + (i * deltaX);
            double cy0 = startY + (j * deltaY);
            
			// The coordinates are stored an a texture coordinate array.
            (*texCoordArray)[idx0 + 2] = osg::Vec3(cx0, cy0, 0.0);
            (*texCoordArray)[idx0 + 3] = osg::Vec3(cx0 + deltaX, cy0, 0.0);
            (*texCoordArray)[idx0 + 0] = osg::Vec3(cx0 + deltaX, cy0 + deltaY, 0.0);
            (*texCoordArray)[idx0 + 1] = osg::Vec3(cx0, cy0 + deltaY, 0.0);

        }
    }
    
	// Assigend to geometry. 
	geometry->setTexCoordArray(0,texCoordArray);
    geometry->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture);
	geometry->dirtyDisplayList();
    
}




/*!
 This function sets a new texture coordinate.
 The index cannot be larger than the specified number of coordinates.
 @param i the column (width);
 @param j the row (height)
 @param tx the x coordinate (width)
 @param ty the y coordinate (height)
 */
bool setTextureCoordinateAt(int i, int j, double tx, double ty)
{


	///////////////////////////////////
	///
	/// Will be explained in class 
	//////////////////////////////////


    if(i >= textureCoordGridX) return false;
    if(j >= textureCoordGridY) return false;
    
    
    int size = texCoordArray->size();
    
    int idx0 = (j * (textureCoordGridX-1) * 4) + (i*4)  + 2;
    int idx1 = (j * (textureCoordGridX-1) * 4) + ((i-1)*4) + 3;
    int idx2 = ((j-1) * (textureCoordGridX-1) * 4) + ((i-1)*4);
    int idx3 = ((j-1) * (textureCoordGridX-1) * 4) + (i*4) +1;
    
    
    if(i ==  textureCoordGridX - 1) // right border of the grid
    {
        if(idx1 >= 0 && idx1 < size )
        {
            (*texCoordArray)[idx1] = osg::Vec3(tx, ty, 0.0);
        }
        if(idx2 >= 0 && idx2 < size )
        {
            (*texCoordArray)[idx2] = osg::Vec3(tx, ty, 0.0);
        }
    }
    else if(i ==  0) // left border of the grid
    {
        if(idx0 >= 0 && idx0 < size )
        {
            (*texCoordArray)[idx0] = osg::Vec3(tx, ty, 0.0);
        }
        if(idx3 >= 0 && idx3 < size )
        {
            (*texCoordArray)[idx3] = osg::Vec3(tx, ty, 0.0);
        }
    }
    else // all others are valid, if the idx is not below 0 or larger than the array
    {
        if(idx1 >= 0 && idx1 < size )
        {
            (*texCoordArray)[idx1] = osg::Vec3(tx, ty, 0.0);
        }
        if(idx2 >= 0 && idx2 < size )
        {
            (*texCoordArray)[idx2] = osg::Vec3(tx, ty, 0.0);
        }
        if(idx0 >= 0 && idx0 < size )
        {
            (*texCoordArray)[idx0] = osg::Vec3(tx, ty, 0.0);
        }
        if(idx3 >= 0 && idx3 < size )
        {
            (*texCoordArray)[idx3] = osg::Vec3(tx, ty, 0.0);
        }
    }
        
    return true;
    
}


/*!
 This function returns the texture corrdinates of a specified grid point
 @param i - the COLUMN of the grid
 @param j - the ROW of the grid. 
 @return a vec2 containing the texture coordinates in x and y
 */
osg::Vec2 getTextureCoordinateAt(int i, int j)
{
    if(i >= textureCoordGridX) return osg::Vec2(-1, -1);
    if(j >= textureCoordGridY) return osg::Vec2(-1, -1);;
    
    
    if(i ==  textureCoordGridX - 1) // right border of the grid
    {
        if(j == (textureCoordGridY-1)) // max
        {
            int idx2 = ((j-1) * (textureCoordGridX-1) * 4) + ((i-1)*4) ;
            return osg::Vec2((*texCoordArray)[idx2].x(), (*texCoordArray)[idx2].y());
        }
        else
        {
            int idx1 = (j * (textureCoordGridX-1) * 4) + ((i-1)*4) + 3;
            return osg::Vec2((*texCoordArray)[idx1].x(), (*texCoordArray)[idx1].y());
        }
    }
    else if(i ==  0) // left border of the grid
    {
        if(j == (textureCoordGridY-1))
        {
            int idx3 = ((j-1) * (textureCoordGridX-1) * 4) + (i*4) +1;
            return osg::Vec2((*texCoordArray)[idx3].x(), (*texCoordArray)[idx3].y());
        }
        else
        {
            int idx0 = (j * (textureCoordGridX-1) * 4) + (i*4)  + 2;
            return osg::Vec2((*texCoordArray)[idx0].x(), (*texCoordArray)[idx0].y());
        }
        
    }
    else // all others are valid, if the idx is not below 0 or larger than the array
    {
        if(j == (textureCoordGridY-1))
        {
            int idx3 = ((j-1) * (textureCoordGridX-1) * 4) + (i*4) +1;
            return osg::Vec2((*texCoordArray)[idx3].x(), (*texCoordArray)[idx3].y());
        }
        else
        {
            int idx0 = (j * (textureCoordGridX-1) * 4) + (i*4)  + 2;
            return osg::Vec2((*texCoordArray)[idx0].x(), (*texCoordArray)[idx0].y());
        }
    }
    
}



// Factor to convert a logarithm with basis 10 into a logarithm with basis 2.
// Log(10)/Log(2)
#define LG2 3.32192809489

/*!
	Calculate the logarithm to the basis of two. 
	param double x:the input value
	return: Logartihm of basis two
	return = log2 x
*/
double log2(double x)
{
	return log10(x) * LG2;
}


/*!*
	Calculates the next power of two for the passed number x. 
	@param: the number, for which the value of two is seeked. 
	
	Procedure: the logarithm of the number x to the basis of 2 is calculated. 
	Next, a value of 1 is added. The value z complies with the next power for x now 
	The integer value is take (everythink before of the comma) and the value is converted back.

*/
int mathNextPowerOf2(double x)
{	
	int result = 0;
	int z = (log2(x)+1);
	result = pow(2.0,z);
	return result;
}


///////////////////////////////////////////////////////
// Create the scene graph by adding primitives
osg::Group *createSphere(int radius)
{
	
	// Create a sphere; this object creates only the geometry of a sphere.
	osg::Sphere* sphere = new osg::Sphere(osg::Vec3(0, 0, 0), radius);

	// Create a drawable; a object that merge the object to be rendered. 
	// It contains all addition information like the color, transparancy etc. 
	osg::ShapeDrawable* sphere_drawable = new osg::ShapeDrawable(sphere);
	sphere_drawable->setColor(osg::Vec4(1.0, 0.0, 0.0, 1.0));

	// The Geode is a leaf node. It can be attached to a scene graph.
	// It contains all object information about a model to render
	osg::Geode* sphere_geode = new osg::Geode();
	sphere_geode->addDrawable(sphere_drawable);

	// Create a transformation node;
	osg::MatrixTransform*	sphere_transform = new osg::MatrixTransform();

	// apply the transformation immediatelly
	sphere_transform->setMatrix(osg::Matrix::translate(0, 10, 0));

	// Assemble the scene graph
	sphere_transform->addChild(sphere_geode);

	return sphere_transform;
}


/*
This function assembles the scene. 
It loads the texture etc. 
*/
osg::Group* createSzene(void)
{
	// Create a scene node
    osg::Group* scene = new osg::Group();
    
	// create the quad for the texture
    scene->addChild(createBackgroundObject());
    
	/////////////////////////////////////////////////////////////////////
    ///// The video capture device
	// This create a video capture device and opens devuce number 0.
	myVideoCaptureDevice = new ARVideoCapture(cv::CAP_OPENNI);

	int videoWidth =  myVideoCaptureDevice->getWidth();
	int videoHeight =  myVideoCaptureDevice->getHeight(); 
    
    /////////////////////////////////////////////////////////////////////
    ///// Create a texture for the video image
    texture0 = new osg::Texture2D();
    texture0->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);
	texture0->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR);
    texture0->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_BORDER);
    texture0->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_BORDER);
    texture0->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_BORDER);

	// Calculate a texture size with a power of 2
	int texWidth = mathNextPowerOf2(videoWidth);
	int texHeight  = mathNextPowerOf2(videoHeight);
	std::cout << "[VideoPolygon_Impl] Scaling the texture width from " << videoWidth << " to " << texWidth << "." << std::endl;
	std::cout << "[VideoPolygon_Impl] Scaling the texture height from " << videoHeight << " to " << texHeight << "." << std::endl;

	// apply the texture size
	texture0->setTextureSize(texWidth,texHeight);

    // Create the texture coordinates and apply them to the background
    setTextureToRect(texture0, videoWidth, videoHeight,  texWidth, texHeight );

	// Create the callback object that subloads the video to the graphics hardware. 
	myCallback = new VideoTexCallback(myVideoCaptureDevice->getVideoStreamPtr(), 
		videoWidth, 
		videoHeight,
		texWidth, 
		texHeight,
		GL_BGR);

	imageprocesscb = new imgprocesscallback(myVideoCaptureDevice->getVideoStream(),myVideoCaptureDevice->getdepthStream(),scene);
	scene->addUpdateCallback(imageprocesscb);
	// Pass the callback to the texture. 
	texture0->setSubloadCallback(myCallback);
    

	////////////////////////////////////////////////////////////////////////////
	// Shader Program 
	// This class loads a shader object and attaches is as stateset to the geode that
	// carries the video background
	GLSL_ShaderLoader* myShaderLoader = new GLSL_ShaderLoader(geode->getOrCreateStateSet(),
		"videshader",
#ifdef WIN32
		"../src/shader/videobg.vs",
		"../src/shader/videobg.fs");
#else
        "../../shader/videobg.vs",
        "../../shader/videobg.fs");
#endif

	// Create auniform variable that grants access to the texture.
	geode->getOrCreateStateSet()->addUniform( new osg::Uniform( "glSample0",0));



	////////////////////////////////////////////////////////////////////////////////
	// Add a 3D object
	//scene->addChild(createSphere(2.0));

    return scene;
}

extern osg::Geode* createAxis(const osg::Vec3& corner,const osg::Vec3& xdir,const osg::Vec3& ydir,const osg::Vec3& zdir);

int main(int argc, const char * argv[])
{

 
	// Create the root node and add a scene
    osg::Group* root = new osg::Group();
    osg::Group* scene = createSzene();
    root->addChild(scene);
	root->addChild(createAxis(osg::Vec3(0,0,0),osg::Vec3(1,0,0),osg::Vec3(0,1,0),osg::Vec3(0,0,1)));
    // Create a viewer and add a manipulator
    osgViewer::Viewer* viewer = new osgViewer::Viewer();
	viewer->setSceneData( root );
	viewer->setUpViewOnSingleScreen(0);
    viewer->setCameraManipulator(new osgGA::TrackballManipulator());
    viewer->getCamera()->setClearColor(osg::Vec4(0.5, 0.5, 1.0, 1.0)) ;
    viewer->setUpViewInWindow(10,10,1000,700);
    // Calibrate the video image by applying a radial distortion model 
	// at the texture coordinates
    //radialCalibration( 0.1, -0.02);
	//modifyCalibration();
    
    
	// Start the camera thread
	myVideoCaptureDevice->startCapturing();

    // Run the viewer
    viewer->run();
    
	// Stop the camera thread
	myVideoCaptureDevice->stopCapturing();

	return 1;
    

}

