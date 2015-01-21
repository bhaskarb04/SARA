/*
 *  ARVideoCapture.cpp
 *  ArtisZee_App
 *
 *  Created by Dr.-Ing. Rafael Radkowski on 04.02.11.
 *  Copyright 2011 Heinz Nixdorf Institute. All rights reserved.
 *
 */

#include "ARVideoCapture.h"

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif


ARVideoCapture::ARVideoCapture(int deviceID):
	_deviceID( deviceID)
{

	_file = "";
	_useFile = false;
	init();
}
	



ARVideoCapture::ARVideoCapture(std::string file):
	_file(file)
{
	_deviceID= -1;
	_useFile = true;
	init();
}

int ARVideoCapture::init()
{
	// set defaults
	_running = false;
	_capture = 0;
	_hasStopped = false;
    _withSync = false;


	if(_useFile == false)
	{
		// Open the capture device
		_capture = new cv::VideoCapture(_deviceID);

		
	}
	else if(_useFile == true)
	{
		// Open a video from file 
		_capture = new cv::VideoCapture(_file.c_str());
	}
    
	// check whether the video is available. 
    if(_capture->isOpened())
	{
		// Query the frame size of the video image.
		_size = cv::Size((int) _capture->get(cv::CAP_PROP_FRAME_WIDTH), (int) _capture->get(cv::CAP_PROP_FRAME_HEIGHT));

		// Query the frame rate
		_framerate = ( int ) _capture->get(cv::CAP_PROP_FPS);

		// Prepare a video image as output
		_convertedImage = cv::Mat(_size,CV_8UC3); 

		std::cout << "[ARVideoCapture] Video device "<< _deviceID << " is open, " <<_size.width << " x " << _size.height << " at " <<  _framerate << "." <<  std::endl;
		

		// Fetch the first image and convert it to have something inside memory
        //(*_capture) >> _image;
		if( !_capture->grab() ){
			std::cout<<"Couldnt grab"<<std::endl;
			return -1;
		}
		_capture->retrieve(_convertedImage,cv::CAP_OPENNI_BGR_IMAGE);
		_capture->retrieve(_depthImage,cv::CAP_OPENNI_POINT_CLOUD_MAP);
        //cv::cvtColor(_image, _convertedImage, cv::COLOR_RGBA2RGB);
		//_image.copyTo(_convertedImage);
		return 1;
	}
	else
	{
		std::cout << "[ARVideoCapture] Video device "<< _deviceID << " could not be opened." <<  std::endl;

		return -1;
	}
   
  
	
}




//virtual
void ARVideoCapture::run()
{
	// Last check, if the device is not open, 
	// the thread cancels itself. 
	if(!_capture->isOpened())
	{
		stopCapturing();
	}


	// This loop is continued since _running is true.
    while (_running)
	{
		// Fetch the image and convert it 
        if( !_capture->grab() ){
			std::cout<<"Couldnt grab"<<std::endl;
			//return -1;
		}
		_capture->retrieve(_convertedImage,cv::CAP_OPENNI_BGR_IMAGE);
		_capture->retrieve(_depthImage,cv::CAP_OPENNI_POINT_CLOUD_MAP);
        //_image.copyTo(_convertedImage);
		//cvtColor(_image, _convertedImage, cv::COLOR_RGBA2RGB);


		// Sync the video device with something else.
		if(_withSync)
		{
			this->_barrier.block(2);
		}
		else
		{
#ifdef WIN32
			// If no sync should be used, pause the thread 
			Sleep(0.02);
#else
			usleep(2000);
#endif
		}
	}
    
    // Release the video camera
    _capture->release();
    
	// indicate that the capture device is closed'
    _hasStopped = true;
    
}

void ARVideoCapture::startCapturing(void)
{
	_running = true;

	// Start the thread
    OpenThreads::Thread::Init();
    OpenThreads::Thread::startThread();
}

		 
void ARVideoCapture::stopCapturing(void)
{
	_running = false;
    
    int antiDeadlock = 0;
    while (_hasStopped == false)
    {
        antiDeadlock++;
        if(antiDeadlock > 10000000)
            break;

#ifdef WIN32
			// If no sync should be used, pause the thread 
			Sleep(0.5);
#else
			usleep(50000);
#endif
    }
    
	// Destroy the thread
    OpenThreads::Thread::cancelCleanup();

}

int ARVideoCapture::getWidth(void)
{
	return this->_size.width;
}

int ARVideoCapture::getHeight(void)
{
	return this->_size.height;
}


unsigned char* ARVideoCapture::getVideoStreamPtr(void)
{
	return (unsigned char*)this->_convertedImage.ptr<uchar>(0);
}

cv::Mat* ARVideoCapture::getVideoStream(void)
{
    return &_convertedImage;
}

cv::Mat* ARVideoCapture::getdepthStream(void)
{
    return &_depthImage;
}

void ARVideoCapture::sync(void)
{
	if(_withSync)
		this->_barrier.block(2);
}

void ARVideoCapture::useSync(bool value)
{
	this->_withSync = value;
}


