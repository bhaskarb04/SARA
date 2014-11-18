#pragma once

#pragma warning( disable : 4996 )

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl\io\pcd_io.h>
#include <pcl\filters\conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl\ModelCoefficients.h>
#include <pcl\point_types.h>

#include <pcl\filters\extract_indices.h>
#include <pcl\filters\voxel_grid.h>
#include <pcl\features\normal_3d.h>
#include <pcl\kdtree\kdtree.h>
#include <pcl\segmentation\extract_clusters.h>

#include <opencv\highgui.h>
#include <opencv\cv.h>
#include <opencv\ml.h>


class skinfunction{
public:
	float mskin;
	float mnonskin;
	float mboth;
	bool trained;
	
	virtual void skintrain(vector<cv::Point3f> skinvalues)=0;
	virtual bool skincompare(float r, float g, float b)=0;
	virtual void save_function()=0;
	virtual void load_function()=0;
};

class skinfunction_1: public skinfunction{
	double rmean;
	cv::Point3d _mean;
	cv::Point3d _stddev;
public:
	void skintrain(vector<cv::Point3f> skinvalues);
	bool skincompare(float r, float g, float b);
	void save_function();
	void load_function();
};

class skinfunction_2: public skinfunction{
	cv::EM *gmm;
public:
	void skintrain(vector<cv::Point3f> skinvalues);
	bool skincompare(float r, float g, float b);
	void save_function();
	void load_function();
};