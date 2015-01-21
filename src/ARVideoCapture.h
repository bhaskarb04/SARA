/*
 *  ARVideoCapture
 *
 *	This class realize a video camera or video file update function.
 *	It opens a video device / video file and fetches single video frames.
 *	The operation can be synced or unsynced. 
 *
 *  Created by Dr.-Ing. Rafael Radkowski on 04.02.11.
 *  Copyright 2011 Heinz Nixdorf Institute. All rights reserved.
 *
 */
#pragma once

#include <iostream>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <opencv2/opencv.hpp>
#include <OpenThreads/Thread>
#include <OpenThreads/Barrier>

#include <osg/Image>

class ARVideoCapture : public OpenThreads::Thread
{
private:
	/////////////////////////////////////////////////////////////////////////////
	// Data

	//----------------------------------------------------------
	// The OpenCV capture device
	cv::VideoCapture*	 _capture;
    
	//----------------------------------------------------------
	// Device ID
	int				_deviceID;
    
	//----------------------------------------------------------
	// The filename if a video file is the source
    std::string		_file;
	
	//----------------------------------------------------------
	// Indicate whether the thread is running 
	bool			_running;
    
    //----------------------------------------------------------
	// Indicate whether the caputre device has stop capturing frames
    bool			_hasStopped;

	//----------------------------------------------------------
	// Indicates that a file is the source for the video image
	bool			_useFile;

	//----------------------------------------------------------
	// Indicates whether the sync should be used or not
	bool			_withSync;
	
	//----------------------------------------------------------
	// Keeps the frame rate
	int				_framerate;

	//----------------------------------------------------------
	// Size of the Video Image
	cv::Size			_size ;
	
	//----------------------------------------------------------
	// The source image fetched from the video capture device
	cv::Mat				_image;

	//----------------------------------------------------------
	// A converted image. 
	cv::Mat				_convertedImage;
	cv::Mat				_depthImage;
    
    //----------------------------------------------------------
	// An OpenThreads barries used for synced reading
	OpenThreads::Barrier _barrier;
	
	/////////////////////////////////////////////////////////////////////////////
	// Operations

	//----------------------------------------------------------
	// Init operation to fire up the capture device
	int init();
	
public:
	/**!
	 Construktor
	 @param deviceID - the number of the video capture device
	 */
	ARVideoCapture(int deviceID = 0);
	
	/**!
	 Construktor
	 @param file - the filename and path of the video file.
	 */
	ARVideoCapture(std::string file);

	/**!
	Virtual running function; it fetches the frames from the 
	video camera device or video file.
	 */
	virtual void run();
    
	/**!
	 Start the thread
	 */
	void startCapturing(void);
	
	/**!
	Stop the Thread
	 */
	void stopCapturing(void);
	
	/**!
	 Provides a pointer that refers to the video stream mem. 
	 @return unsigned char* ptr 
	 */
	unsigned char* getVideoStreamPtr(void);

	/**!
	 Provides an OpenCV pointer that refers to the image
	 @return ucv::Mat ptr
	 */
    cv::Mat* getVideoStream(void);
	cv::Mat* getdepthStream(void);
	
	/*!
	Returns the width of the image in pixel
	@return width in pixel 
	*/
	int getWidth(void);
	
	/*!
	Returns the height of the image in pixel
	@return height in pixel 
	*/
	int getHeight(void);
    
	/*!
	Sync the video image. 
	If sync is used, this function must be called
	to fetch a new frame
	*/
	void sync(void);

	/*!
	Enable or disavble the video camera sync
	@param value - true= with sync, false = no sync.
	*/
	void useSync(bool value);
};