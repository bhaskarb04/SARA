#pragma once
#ifndef _HAND_MODEL_
#define _HAND_MODEL_
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

#include "skinfunction.h"

class HandModel
{
	bool trained;
	vector<skinfunction*> model;
	int skincount;

	pcl::ModelCoefficients::Ptr removeplane(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);
	void remove_noise(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::ModelCoefficients::Ptr coefficients);
	
public:
	HandModel(std::string base,int start, int end,bool show);
	HandModel(vector<std::string> filelist);
	HandModel();
	~HandModel(void);
	void save_model();
	void load_model();
	void removeHand(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud);
	bool isTrained(){return trained;}
	bool isHandPresent(){return (skincount > 20000);}
};

#endif